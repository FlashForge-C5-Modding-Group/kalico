# Virtual SDCard print stat tracking
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import bisect
import functools


class PrintStats:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.reactor = self.printer.get_reactor()
        # Retain the diagnostic session independently of ordinary stats reset.
        self._timing_session_id = 0
        self._timing_session = None
        self.reset()
        # Register commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "SET_PRINT_STATS_INFO",
            self.cmd_SET_PRINT_STATS_INFO,
            desc=self.cmd_SET_PRINT_STATS_INFO_help,
        )
        self.gcode.register_command(
            "GET_TIMER_TOO_CLOSE_STATS",
            self.cmd_GET_TIMER_TOO_CLOSE_STATS,
            when_not_ready=True,
            desc=self.cmd_GET_TIMER_TOO_CLOSE_STATS_help,
        )
        self.printer.register_event_handler(
            "mcu:timer_too_close", self._handle_timer_too_close
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_timing_shutdown
        )
        self.printer.register_event_handler(
            "extruder:activate_extruder", self._handle_activate_extruder
        )

    def _handle_activate_extruder(self):
        gc_status = self.gcode_move.get_status()
        self.last_epos = gc_status["position"].e

    def _update_filament_usage(self, eventtime):
        gc_status = self.gcode_move.get_status(eventtime)
        cur_epos = gc_status["position"].e
        self.filament_used += (cur_epos - self.last_epos) / gc_status[
            "extrude_factor"
        ]
        self.last_epos = cur_epos

    def set_current_file(self, filename):
        self.reset()
        self.filename = filename

    def note_start(self):
        curtime = self.reactor.monotonic()
        if self.print_start_time is None:
            self._start_timing_report(curtime)
            self.print_start_time = curtime
        elif self.last_pause_time is not None:
            # Update pause time duration
            pause_duration = curtime - self.last_pause_time
            self.prev_pause_duration += pause_duration
            self.last_pause_time = None
        # Reset last e-position
        gc_status = self.gcode_move.get_status(curtime)
        self.last_epos = gc_status["position"].e
        self.state = "printing"
        if self._timing_session is not None:
            if self._timing_session["end"] is None:
                self._timing_session["state"] = "printing"
        self.printer.send_event("print_stats:start_printing")
        self.error_message = ""

    def note_pause(self):
        if self.last_pause_time is None:
            curtime = self.reactor.monotonic()
            self.last_pause_time = curtime
            # update filament usage
            self._update_filament_usage(curtime)
        if self.state != "error":
            self.state = "paused"
            if self._timing_session is not None:
                if self._timing_session["end"] is None:
                    self._timing_session["state"] = "paused"
            self.printer.send_event("print_stats:paused_printing")

    def note_complete(self):
        self._note_finish("complete")
        self.printer.send_event("print_stats:complete_printing")

    def note_error(self, message):
        self._note_finish("error", message)
        self.printer.send_event("print_stats:error_printing")

    def note_cancel(self):
        self._note_finish("cancelled")
        self.printer.send_event("print_stats:cancelled_printing")

    def _note_finish(self, state, error_message=""):
        eventtime = self.reactor.monotonic()
        self._finalize_timing_report(state, eventtime)
        if self.print_start_time is None:
            return
        self.state = state
        self.error_message = error_message
        self.total_duration = eventtime - self.print_start_time
        if self.filament_used < 0.0000001:
            # No positive extusion detected during print
            self.init_duration = self.total_duration - self.prev_pause_duration
        self.print_start_time = None

    cmd_SET_PRINT_STATS_INFO_help = (
        "Pass slicer info like layer act and total to klipper"
    )

    def cmd_SET_PRINT_STATS_INFO(self, gcmd):
        previous_layer = self.info_current_layer
        total_layer = gcmd.get_int(
            "TOTAL_LAYER", self.info_total_layer, minval=0
        )
        current_layer = gcmd.get_int(
            "CURRENT_LAYER", self.info_current_layer, minval=0
        )
        if total_layer == 0:
            self.info_total_layer = None
            self.info_current_layer = None
        elif total_layer != self.info_total_layer:
            self.info_total_layer = total_layer
            self.info_current_layer = 0

        if (
            self.info_total_layer is not None
            and current_layer is not None
            and current_layer != self.info_current_layer
        ):
            self.info_current_layer = min(current_layer, self.info_total_layer)
        session = self._timing_session
        if session is None or session["end"] is not None:
            return
        layer = self.info_current_layer
        if not session["metadata_seen"] and self.info_total_layer is not None:
            session["metadata_seen"] = True
            # Startup metadata describes the current receipt-time baseline,
            # not the end of queued motion (or an empty-lookahead prime).
            mcu = self.printer.lookup_object("mcu")
            print_time = mcu.estimated_print_time(self.reactor.monotonic())
            self._record_timing_layer(session["id"], 0, print_time)
            previous_layer = 0
        if layer != previous_layer:
            marker_id = session["next_marker"]
            session["next_marker"] += 1
            session["pending_markers"].add(marker_id)
            callback = functools.partial(
                self._record_timing_layer, session["id"], layer,
                marker_id=marker_id,
            )
            self.printer.lookup_object("toolhead").register_lookahead_callback(
                callback
            )

    def reset(self):
        self._finalize_timing_report("cancelled", self.reactor.monotonic())
        self.filename = self.error_message = ""
        self.state = "standby"
        self.prev_pause_duration = self.last_epos = 0.0
        self.filament_used = self.total_duration = 0.0
        self.print_start_time = self.last_pause_time = None
        self.init_duration = 0.0
        self.info_total_layer = None
        self.info_current_layer = None
        self.printer.send_event("print_stats:reset")

    def get_status(self, eventtime):
        time_paused = self.prev_pause_duration
        if self.print_start_time is not None:
            if self.last_pause_time is not None:
                # Calculate the total time spent paused during the print
                time_paused += eventtime - self.last_pause_time
            else:
                # Accumulate filament if not paused
                self._update_filament_usage(eventtime)
            self.total_duration = eventtime - self.print_start_time
            if self.filament_used < 0.0000001:
                # Track duration prior to extrusion
                self.init_duration = self.total_duration - time_paused
        print_duration = self.total_duration - self.init_duration - time_paused
        return {
            "filename": self.filename,
            "total_duration": self.total_duration,
            "print_duration": print_duration,
            "filament_used": self.filament_used,
            "state": self.state,
            "message": self.error_message,
            "info": {
                "total_layer": self.info_total_layer,
                "current_layer": self.info_current_layer,
            },
            "timer_too_close": self._timing_snapshot(),
        }

    def _start_timing_report(self, eventtime):
        self._timing_session_id += 1
        print_time = self.printer.lookup_object("mcu").estimated_print_time(
            eventtime
        )
        self._timing_session = {
            "id": self._timing_session_id,
            "filename": self.filename,
            "state": "printing",
            "start": eventtime,
            "end": None,
            "total": 0,
            "metadata_seen": self.info_total_layer is not None,
            "boundaries": [(print_time, 0, self.info_current_layer)],
            "next_marker": 1,
            "pending_markers": set(),
            "pending_warnings": [],
            "buckets": {},
            "report_pending": False,
            "reported_total": None,
        }

    def _record_timing_layer(
        self, session_id, layer, print_time, marker_id=None
    ):
        session = self._timing_session
        if (
            session is None
            or session["id"] != session_id
            or session["end"] is not None
        ):
            return
        if marker_id is None:
            marker_id = session["next_marker"]
            session["next_marker"] += 1
        else:
            session["pending_markers"].discard(marker_id)
        # Command order breaks ties, even if callbacks resolve out of order.
        bisect.insort(session["boundaries"], (print_time, marker_id, layer))
        self._resolve_timing_warnings(session)

    def _bucket_timing_warning(self, session, warning, buckets):
        mcu_name, receive_time, print_time = warning
        boundaries = session["boundaries"]
        index = bisect.bisect_right(boundaries, (print_time, float("inf"))) - 1
        layer = boundaries[index][2] if index >= 0 else None
        elapsed = receive_time - session["start"]
        key = (layer, mcu_name)
        bucket = buckets.get(key)
        if bucket is None:
            buckets[key] = [1, elapsed, elapsed]
        else:
            bucket[0] += 1
            bucket[1] = min(bucket[1], elapsed)
            bucket[2] = max(bucket[2], elapsed)

    def _resolve_timing_warnings(self, session):
        if session["pending_markers"]:
            return
        for warning in session["pending_warnings"]:
            self._bucket_timing_warning(session, warning, session["buckets"])
        session["pending_warnings"].clear()

    def _handle_timer_too_close(self, mcu_name, receive_time):
        session = self._timing_session
        if session is None or receive_time < session["start"]:
            return
        if session["end"] is not None and receive_time > session["end"]:
            return
        # Convert the original receipt timestamp, never callback delivery time.
        print_time = self.printer.lookup_object("mcu").estimated_print_time(
            receive_time
        )
        warning = (mcu_name, receive_time, print_time)
        session["total"] += 1
        if session["pending_markers"]:
            session["pending_warnings"].append(warning)
        else:
            self._bucket_timing_warning(session, warning, session["buckets"])
        if session["end"] is not None:
            self._queue_timing_report(session)

    def _finalize_timing_report(self, state, eventtime):
        session = self._timing_session
        if session is None or session["end"] is not None:
            return
        session["end"] = eventtime
        session["state"] = state
        # Unresolved lookahead cannot describe executed motion at termination.
        # Do not flush it, and do not let later callbacks rewrite this report.
        session["pending_markers"].clear()
        self._resolve_timing_warnings(session)
        self._queue_timing_report(session)

    def _handle_timing_shutdown(self):
        self._finalize_timing_report("error", self.reactor.monotonic())

    def _queue_timing_report(self, session):
        if session["report_pending"]:
            return
        session["report_pending"] = True
        # Share the asynchronous FIFO with serial warnings already handed off
        # by the worker. Capture the session, not the next selected print.
        self.reactor.register_async_callback(
            functools.partial(self._emit_timing_report, session=session)
        )

    def _emit_timing_report(self, eventtime, session):
        session["report_pending"] = False
        if session["reported_total"] == session["total"]:
            return
        text = self._format_timing_report(
            session, updated=session["reported_total"] is not None
        )
        session["reported_total"] = session["total"]
        self.gcode.respond_info(text)

    def _timing_snapshot(self, session=None):
        if session is None:
            session = self._timing_session
        snapshot = {
            "filename": "" if session is None else session["filename"],
            "state": "standby" if session is None else session["state"],
            "total": 0 if session is None else session["total"],
            "attribution": "receive_time_scheduled_layer",
            "by_layer": [],
        }
        if session is None:
            return snapshot
        self._resolve_timing_warnings(session)
        buckets = {key: list(value) for key, value in session["buckets"].items()}
        # Pending warnings are provisional in snapshots only. Their stored
        # receipts can be rebucketed once all scheduled boundaries are known.
        for warning in session["pending_warnings"]:
            self._bucket_timing_warning(session, warning, buckets)
        for (layer, mcu_name), (count, first, last) in sorted(
            buckets.items(),
            key=lambda item: (
                item[0][0] is None,
                item[0][0] if item[0][0] is not None else 0,
                item[0][1],
            ),
        ):
            snapshot["by_layer"].append({
                "mcu": mcu_name,
                "layer": layer,
                "count": count,
                "first_elapsed": first,
                "last_elapsed": last,
            })
        return snapshot

    def _format_timing_report(self, session=None, updated=False):
        if session is None:
            session = self._timing_session
        if session is None:
            return "Timer too close: no print recorded"
        snapshot = self._timing_snapshot(session)
        lines = [
            "Timer too close: %d warning(s) - %s [%s]%s"
            % (
                snapshot["total"], snapshot["filename"], snapshot["state"],
                " updated" if updated else "",
            )
        ]
        for bucket in snapshot["by_layer"]:
            layer = bucket["layer"]
            layer_name = "unknown" if layer is None else (
                "startup" if layer == 0 else str(layer)
            )
            lines.append(
                "MCU=%s layer=%s count=%d first=%.6fs last=%.6fs"
                % (
                    bucket["mcu"], layer_name, bucket["count"],
                    bucket["first_elapsed"], bucket["last_elapsed"],
                )
            )
        return "\n".join(lines)

    cmd_GET_TIMER_TOO_CLOSE_STATS_help = (
        "Report retained per-print Timer too close warnings by MCU and "
        "scheduled layer at host receipt time, not physical position; "
        "query does not reset counts"
    )

    def cmd_GET_TIMER_TOO_CLOSE_STATS(self, gcmd):
        text = self._format_timing_report()
        self.gcode.respond_info(text)


def load_config(config):
    return PrintStats(config)
