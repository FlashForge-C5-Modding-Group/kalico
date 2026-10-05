# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import io
import logging
import os
import re
import sys

VALID_GCODE_EXTS = ["gcode", "g", "gco"]


DEFAULT_ERROR_GCODE = """
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
"""


def creator5_start_command(header):
    if re.search(r"^\s*C5_PRINT_START\b", header, re.I | re.M):
        return None
    tool = 0
    tool_seen = False
    hotend = bed = preheat = first_layer = None
    for raw in header.splitlines():
        meta = re.match(r"^\s*;\s*first_layer_height\s*=\s*([0-9.]+)", raw, re.I)
        if meta and first_layer is None:
            first_layer = float(meta.group(1))
        line = raw.split(";", 1)[0].strip()
        match = re.match(r"^T([0-3])(?:\s|$)", line, re.I)
        if match and not tool_seen:
            tool, tool_seen = int(match.group(1)), True
        match = re.match(r"^AFC_SELECT_TOOL\s+TOOL=extruder([0-3]?)\b", line, re.I)
        if match and not tool_seen:
            tool, tool_seen = int(match.group(1) or 0), True
        match = re.match(r"^M(104|109|140|190)\b.*?\bS\s*([0-9.]+)", line, re.I)
        if match and match.group(1) in ("104", "109"):
            value = float(match.group(2))
            if value > 0:
                if match.group(1) == "109" and hotend is None:
                    hotend = value
                elif match.group(1) == "104" and preheat is None:
                    preheat = value
        elif match and bed is None and match.group(1) in ("140", "190"):
            value = float(match.group(2))
            if value > 0:
                bed = value
    if hotend is None:
        hotend = preheat
    if hotend is None:
        raise ValueError("No hotend temperature found in G-code header; "
                         "add C5_PRINT_START TOOL=n HOTEND=n BED=n")
    command = "C5_PRINT_START TOOL=%d HOTEND=%g BED=%g" % (tool, hotend, bed or 0.)
    if first_layer is not None:
        command += " FIRST_LAYER_HEIGHT=%g" % first_layer
    return command


def creator5_slicer_z_offset(line):
    command = line.lstrip()
    if not command:
        return False
    if command[0] in "Gg":
        if command[:3].upper() != "G92":
            return False
    elif command[0] in "Ss":
        if command[:16].upper() != "SET_GCODE_OFFSET":
            return False
    else:
        return False
    command = command.split(";", 1)[0].strip()
    if (re.match(r"^SET_GCODE_OFFSET\s+", command, re.I)
            and re.search(r"\bZ(?:_ADJUST)?\s*=", command, re.I)):
        return True
    return bool(re.match(r"^G92\b", command, re.I)
                and re.search(r"\bZ\s*=?\s*[-+]?(?:\d|\.)", command, re.I))


def creator5_object_definitions(header):
    definitions = []
    offset = 0
    for raw in header.splitlines(True):
        if len(header) >= 65536 and not raw.endswith(("\n", "\r")):
            break
        command = raw.split(";", 1)[0].strip()
        if re.match(r"^EXCLUDE_OBJECT_DEFINE\s+", command, re.I):
            definitions.append((offset, command))
        offset += len(raw.encode())
    return definitions


class VirtualSD:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler(
            "klippy:shutdown", self.handle_shutdown
        )
        # sdcard state
        sd = config.get("path")
        self.with_subdirs = config.getboolean("with_subdirs", False)
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        self.auto_creator5_start = config.getboolean("auto_creator5_start", False)
        self.creator5_start_done = False
        self.creator5_stop_done = False
        self.creator5_preloaded_definitions = set()
        # Print Stat Tracking
        self.print_stats = self.printer.load_object(config, "print_stats")
        # Work timer
        self.reactor = self.printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.cancelled_work = False
        self.next_file_position = 0
        self.work_timer = None
        # Error handling
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.on_error_gcode = gcode_macro.load_template(
            config, "on_error_gcode", DEFAULT_ERROR_GCODE
        )
        # Register commands
        self.gcode = self.printer.lookup_object("gcode")
        for cmd in ["M20", "M21", "M23", "M24", "M25", "M26", "M27"]:
            self.gcode.register_command(cmd, getattr(self, "cmd_" + cmd))
        for cmd in ["M28", "M29", "M30"]:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command(
            "SDCARD_RESET_FILE",
            self.cmd_SDCARD_RESET_FILE,
            desc=self.cmd_SDCARD_RESET_FILE_help,
        )
        self.gcode.register_command(
            "SDCARD_PRINT_FILE",
            self.cmd_SDCARD_PRINT_FILE,
            desc=self.cmd_SDCARD_PRINT_FILE_help,
        )

    def handle_shutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                logging.exception("virtual_sdcard shutdown read")
                return
            logging.info(
                "Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                readpos,
                repr(data[:readcount]),
                self.file_position,
                repr(data[readcount:]),
            )

    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)

    def get_file_list(self, check_subdirs=False):
        if check_subdirs:
            flist = []
            for root, dirs, files in os.walk(
                self.sdcard_dirname, followlinks=True
            ):
                for name in files:
                    ext = name[name.rfind(".") + 1 :]
                    if ext not in VALID_GCODE_EXTS:
                        continue
                    full_path = os.path.join(root, name)
                    r_path = full_path[len(self.sdcard_dirname) + 1 :]
                    size = os.path.getsize(full_path)
                    flist.append((r_path, size))
            return sorted(flist, key=lambda f: f[0].lower())
        else:
            dname = self.sdcard_dirname
            try:
                filenames = os.listdir(self.sdcard_dirname)
                return [
                    (fname, os.path.getsize(os.path.join(dname, fname)))
                    for fname in sorted(filenames, key=str.lower)
                    if not fname.startswith(".")
                    and os.path.isfile((os.path.join(dname, fname)))
                ]
            except:
                logging.exception("virtual_sdcard get_file_list")
                raise self.gcode.error("Unable to get file list")

    def get_status(self, eventtime):
        return {
            "file_path": self.file_path(),
            "progress": self.progress(),
            "is_active": self.is_active(),
            "file_position": self.file_position,
            "file_size": self.file_size,
        }

    def file_path(self):
        if self.current_file:
            return self.current_file.name
        return None

    def progress(self):
        if self.file_size:
            return float(self.file_position) / self.file_size
        else:
            return 0.0

    def is_active(self):
        return self.work_timer is not None

    def do_pause(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            while self.work_timer is not None and not self.cmd_from_sd:
                self.reactor.pause(self.reactor.monotonic() + 0.001)

    def do_resume(self):
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW
        )

    def do_cancel(self):
        if self.current_file is not None:
            self.do_pause()
            self.cancelled_work = True
            self.current_file.close()
            self.current_file = None
            self.print_stats.note_cancel()
        self.file_position = self.file_size = 0

    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("SD write not supported")

    def _reset_file(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
        self.file_position = self.file_size = 0
        self.creator5_start_done = False
        self.creator5_stop_done = False
        self.cancelled_work = False
        self.creator5_preloaded_definitions = set()
        self.print_stats.reset()
        self.printer.send_event("virtual_sdcard:reset_file")

    cmd_SDCARD_RESET_FILE_help = (
        "Clears a loaded SD File. Stops the print if necessary"
    )

    def cmd_SDCARD_RESET_FILE(self, gcmd):
        if self.cmd_from_sd:
            raise gcmd.error("SDCARD_RESET_FILE cannot be run from the sdcard")
        self._reset_file()

    cmd_SDCARD_PRINT_FILE_help = (
        "Loads a SD file and starts the print.  May "
        "include files in subdirectories."
    )

    def cmd_SDCARD_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get("FILENAME")
        if filename[0] == "/":
            filename = filename[1:]
        self._load_file(gcmd, filename, check_subdirs=True)
        self.do_resume()

    def cmd_M20(self, gcmd):
        # List SD card
        files = self.get_file_list(self.with_subdirs)
        gcmd.respond_raw("Begin file list")
        for fname, fsize in files:
            gcmd.respond_raw("%s %d" % (fname, fsize))
        gcmd.respond_raw("End file list")

    def cmd_M21(self, gcmd):
        # Initialize SD card
        gcmd.respond_raw("SD card ok")

    def cmd_M23(self, gcmd):
        # Select SD file
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get_raw_command_parameters().strip()
        if filename.startswith("/"):
            filename = filename[1:]
        self._load_file(gcmd, filename, self.with_subdirs)

    def _load_file(self, gcmd, filename, check_subdirs=False):
        files = self.get_file_list(check_subdirs)
        flist = [f[0] for f in files]
        files_by_lower = {fname.lower(): fname for fname, fsize in files}
        fname = filename
        try:
            if fname not in flist:
                fname = files_by_lower[fname.lower()]
            fname = os.path.join(self.sdcard_dirname, fname)
            f = io.open(fname, "r", newline="")
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
        except:
            logging.exception("virtual_sdcard file open")
            raise gcmd.error("Unable to open file")
        gcmd.respond_raw("File opened: %s Size: %d" % (filename, fsize))
        gcmd.respond_raw("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.creator5_start_done = False
        self.creator5_stop_done = False
        self.cancelled_work = False
        self.creator5_preloaded_definitions = set()
        self.print_stats.set_current_file(filename)
        self.printer.send_event("virtual_sdcard:load_file")

    def cmd_M24(self, gcmd):
        # Start/resume SD print
        self.do_resume()

    def cmd_M25(self, gcmd):
        # Pause SD print
        self.do_pause()

    def cmd_M26(self, gcmd):
        # Set SD position
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        pos = gcmd.get_int("S", minval=0)
        self.file_position = pos

    def cmd_M27(self, gcmd):
        # Report SD print status
        if self.current_file is None:
            gcmd.respond_raw("Not SD printing.")
            return
        gcmd.respond_raw(
            "SD printing byte %d/%d" % (self.file_position, self.file_size)
        )

    def get_file_position(self):
        return self.next_file_position

    def set_file_position(self, pos):
        self.next_file_position = pos

    def is_cmd_from_sd(self):
        return self.cmd_from_sd

    # Background work timer
    def work_handler(self, eventtime):
        logging.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard seek")
            self.work_timer = None
            return self.reactor.NEVER
        self.print_stats.note_start()
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        final_line = False
        error_message = None
        next_yield = self.reactor.monotonic() + 0.010
        if (self.auto_creator5_start and not self.creator5_start_done
                and not self.file_position):
            try:
                header = self.current_file.read(65536)
                self.current_file.seek(0)
                definitions = creator5_object_definitions(header)
                if definitions:
                    self.gcode.run_script("EXCLUDE_OBJECT_DEFINE RESET=1")
                    for offset, definition in definitions:
                        self.gcode.run_script(definition)
                        self.creator5_preloaded_definitions.add(offset)
                start_command = creator5_start_command(header)
                self.creator5_start_done = True
                if start_command is not None:
                    self.gcode.run_script(start_command)
            except (ValueError, self.gcode.error) as e:
                error_message = str(e)
                self.must_pause_work = True
            except:
                logging.exception("virtual_sdcard Creator 5 print start")
                error_message = "Creator 5 print start failed"
                self.must_pause_work = True
            if error_message is not None:
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("virtual_sdcard on_error")
        while not self.must_pause_work:
            if not lines:
                # Read more data
                try:
                    data = self.current_file.read(8192)
                except:
                    logging.exception("virtual_sdcard read")
                    break
                if not data:
                    if partial_input:
                        lines = [partial_input]
                        partial_input = ""
                        final_line = True
                        continue
                    # End of file
                    if (self.auto_creator5_start and self.creator5_start_done
                            and not self.creator5_stop_done):
                        try:
                            self.gcode.run_script("C5_PRINT_STOP")
                            self.creator5_stop_done = True
                        except self.gcode.error as e:
                            error_message = str(e)
                            try:
                                self.gcode.run_script(self.on_error_gcode.render())
                            except:
                                logging.exception("virtual_sdcard on_error")
                        except:
                            logging.exception("virtual_sdcard Creator 5 print stop")
                            error_message = "Creator 5 print stop failed"
                            try:
                                self.gcode.run_script(self.on_error_gcode.render())
                            except:
                                logging.exception("virtual_sdcard on_error")
                    self.current_file.close()
                    self.current_file = None
                    if error_message is None:
                        logging.info("Finished SD card print")
                        self.gcode.respond_raw("Done printing file")
                    break
                lines = data.split("\n")
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                self.reactor.pause(self.reactor.NOW)
                continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            line = lines.pop()
            if sys.version_info.major >= 3:
                line_length = len(line) if line.isascii() else len(line.encode())
            else:
                line_length = len(line)
            next_file_position = (self.file_position + line_length
                                  + (0 if final_line else 1))
            self.next_file_position = next_file_position
            try:
                if self.file_position in self.creator5_preloaded_definitions:
                    pass
                elif self.auto_creator5_start and creator5_slicer_z_offset(line):
                    logging.warning("Ignoring slicer Z offset in Creator 5 "
                                    "print file: %s", line.strip())
                else:
                    self.gcode.run_script(line)
                    stripped = line.lstrip()
                    if (stripped[:1] in ("C", "c")
                            and re.match(r"^C5_PRINT_STOP(?:\s|$)",
                                         stripped.split(";", 1)[0], re.I)):
                        self.creator5_stop_done = True
            except self.gcode.error as e:
                error_message = str(e)
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("virtual_sdcard on_error")
                break
            except:
                logging.exception("virtual_sdcard dispatch")
                break
            self.cmd_from_sd = False
            self.file_position = self.next_file_position
            if self.reactor.monotonic() >= next_yield:
                self.reactor.pause(self.reactor.NOW)
                next_yield = self.reactor.monotonic() + 0.010
            # Do we need to skip around?
            if self.next_file_position != next_file_position:
                try:
                    self.current_file.seek(self.file_position)
                except:
                    logging.exception("virtual_sdcard seek")
                    self.work_timer = None
                    return self.reactor.NEVER
                lines = []
                partial_input = ""
                final_line = False
        logging.info("Exiting SD card print (position %d)", self.file_position)
        self.work_timer = None
        self.cmd_from_sd = False
        if error_message is not None:
            self.print_stats.note_error(error_message)
        elif self.cancelled_work:
            pass
        elif self.current_file is not None:
            self.print_stats.note_pause()
        else:
            self.print_stats.note_complete()
        return self.reactor.NEVER


def load_config(config):
    return VirtualSD(config)
