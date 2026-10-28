# Creator 5 skipped step and stall detection for the mclib XYZ motors
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

AXES = ('stepper_x', 'stepper_y', 'stepper_z')
ACTIONS = ('warn', 'pause')
# When the firmware follows the angle drift for skipped steps. Prints always
# use moving: holding and always count false slips while printing, so they
# only apply in test mode (pulling the carriage by hand).
SLIP_WATCH = {'moving': 0, 'holding': 1, 'always': 2}


class C5MotionGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.enabled = config.getboolean('enabled', True)
        self.action = config.getchoice('action', {a: a for a in ACTIONS},
                                       'warn')
        self.poll_time = config.getfloat('poll_time', .5, minval=.1)
        self.slip_watch = config.getchoice(
            'slip_watch', {k: k for k in SLIP_WATCH}, 'moving')
        # Net electrical cycles (4 full steps each) lost on one axis in one
        # print before it counts as skipped steps.
        self.slip_limit = config.getint('slip_limit', 1, minval=1)
        stall_axes = config.getlist('stall_axes', [])
        self.stall_axes = ['stepper_' + a.strip().lower() for a in stall_axes]
        for a in self.stall_axes:
            if a not in AXES:
                raise config.error("stall_axes: unknown axis '%s'" % (a,))
        self.motors = {}
        self.print_totals = {}
        self.recording = None
        self.tripped = None
        self.was_busy = self.was_printing = self.settle = False
        self.test_mode = False
        self.sent_mode, self.sent_ok = None, False
        self.timer = None
        self.printer.register_event_handler('klippy:ready', self._ready)
        # Homing drives into the endstops and probe; skip those windows too
        self.homing = set()
        for event, kind, on in (
                ('homing:home_rails_begin', 'rails', True),
                ('homing:home_rails_end', 'rails', False),
                ('homing:homing_move_begin', 'move', True),
                ('homing:homing_move_end', 'move', False)):
            self.printer.register_event_handler(
                event, lambda *a, kind=kind, on=on: self._homing(kind, on))
        for name, func, desc in (
                ('C5_MOTION_STATUS', self.cmd_STATUS, self.cmd_STATUS_help),
                ('C5_MOTION_RECORD', self.cmd_RECORD, self.cmd_RECORD_help),
                ('C5_MOTION_GUARD', self.cmd_GUARD, self.cmd_GUARD_help)):
            self.gcode.register_command(name, func, desc=desc)

    def _ready(self):
        for axis in AXES:
            motor = self.printer.lookup_object('mclib ' + axis, None)
            if motor is not None and motor.query_status_cmd is not None:
                self.motors[axis] = motor
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.toolchanger = self.printer.lookup_object('creator5_toolchanger',
                                                      None)
        self._reset_totals()
        self._send_slip_watch()
        if self.motors:
            self.timer = self.reactor.register_timer(
                self._poll, self.reactor.NOW)

    def _homing(self, kind, on):
        (self.homing.add if on else self.homing.discard)(kind)

    def _send_slip_watch(self, real_print=False):
        mode = 0
        if self.test_mode and not real_print:
            mode = SLIP_WATCH[self.slip_watch]
        if mode == self.sent_mode:
            return self.sent_ok
        self.sent_mode = mode
        self.sent_ok = all([motor.set_slip_mode(mode)
                            for motor in self.motors.values()])
        return self.sent_ok

    def _reset_totals(self):
        self.print_totals = {a: {'slips': 0, 'stalls': 0, 'ignored': 0}
                             for a in self.motors}
        self.reported = {}
        self.tripped = None

    def _printing(self):
        if self.print_stats is None:
            return False
        return self.print_stats.get_status(
            self.reactor.monotonic())['state'] == 'printing'

    def _start_window(self):
        # The MCU counts since its last query; throw away what came before
        # (the query sent here answers before the next poll).
        for motor in self.motors.values():
            motor.take_health()
            motor.query_health()
        self.settle = True

    def _poll(self, eventtime):
        real_print = self._printing()
        self._send_slip_watch(real_print)
        printing = self.enabled and (self.test_mode or real_print)
        if printing and not self.was_printing:
            self._reset_totals()
            self._start_window()
        self.was_printing = printing
        if not printing and self.recording is None:
            return eventtime + self.poll_time
        if self.settle:
            self.settle = False
            for motor in self.motors.values():
                motor.take_health()
                motor.query_health()
            return eventtime + self.poll_time
        # Docking pushes the carriage into the dock; skip any window that
        # overlaps a toolchange or homing.
        busy = bool(getattr(self.toolchanger, 'busy', False)) or bool(
            self.homing)
        skip = busy or self.was_busy
        self.was_busy = busy
        for axis, motor in self.motors.items():
            h = motor.take_health()
            motor.query_health()
            if self.recording is not None:
                self._record(axis, h)
            if not printing:
                continue
            totals = self.print_totals[axis]
            if skip:
                totals['ignored'] += abs(h['slips']) + h['stalls']
                continue
            totals['slips'] += h['slips']
            if axis in self.stall_axes:
                totals['stalls'] += h['stalls']
            self._check(axis, totals)
        return eventtime + self.poll_time

    def _record(self, axis, h):
        r = self.recording.setdefault(axis, {'samples': 0, 'stalls': 0,
                                             'slips': 0, 'torque_min': None,
                                             'torque_max': None,
                                             'err_min': None, 'err_max': None})
        r['samples'] += h['samples']
        r['stalls'] += h['stalls']
        r['slips'] += h['slips']
        for key, pick in (('torque_min', min), ('torque_max', max),
                          ('err_min', min), ('err_max', max)):
            if h[key] is not None:
                r[key] = h[key] if r[key] is None else pick(r[key], h[key])

    def _check(self, axis, totals):
        # Report every time an axis gets worse; pause only the first time
        slips, stalls = abs(totals['slips']), totals['stalls']
        reported = self.reported.setdefault(axis, (0, 0))
        reasons = []
        if slips >= self.slip_limit and slips != reported[0]:
            reasons.append("%s skipped steps: %d electrical cycles (%d full "
                           "steps) so far" % (axis, slips, 4 * slips))
        if stalls and stalls != reported[1]:
            reasons.append("%s stalled (%d so far)" % (axis, stalls))
        if not reasons:
            return
        self.reported[axis] = (slips, stalls)
        reason = "; ".join(reasons)
        first = self.tripped is None
        self.tripped = reason
        if self.action == 'pause' and first:
            self.reactor.register_callback(lambda e: self._pause(reason))
        else:
            self.gcode.respond_raw("!! Motion guard: %s" % (reason,))

    def _pause(self, reason):
        self.gcode.respond_raw("!! Motion guard: %s, pausing" % (reason,))
        try:
            self.gcode.run_script("PAUSE")
        except self.gcode.error as e:
            self.gcode.respond_raw("!! Motion guard: pause failed: %s" % (e,))

    def get_status(self, eventtime):
        return {'enabled': self.enabled, 'action': self.action,
                'slip_watch': self.slip_watch,
                'available': bool(self.motors), 'tripped': self.tripped,
                'recording': self.recording is not None,
                'test_mode': self.test_mode,
                'axes': {a: dict(t) for a, t in self.print_totals.items()}}

    def _need_firmware(self, gcmd):
        if not self.motors:
            raise gcmd.error("Motion guard needs the mainboard firmware with "
                             "mclib_query_status; flash the newer MCU update")

    cmd_STATUS_help = "Show skipped steps, stalls and motor torque per axis"
    def cmd_STATUS(self, gcmd):
        self._need_firmware(gcmd)
        lines = ["Motion guard: %s, action %s, watching %s, slip limit %d%s"
                 % ("on" if self.enabled else "off", self.action,
                    self.slip_watch, self.slip_limit,
            ", stall axes " + ", ".join(self.stall_axes)
            if self.stall_axes else "")]
        for axis, motor in self.motors.items():
            t = self.print_totals.get(axis, {})
            last = motor.last_status or {}
            lines.append(
                "%s: this print %d slips, %d stalls (%d ignored in toolchanges"
                " and homing);"
                " stall flag %s, stall_threshold %.3f" % (
                    axis, t.get('slips', 0), t.get('stalls', 0),
                    t.get('ignored', 0), last.get('stall', '?'),
                    motor.get_stall_threshold()))
        if self.tripped:
            lines.append("Tripped: " + self.tripped)
        gcmd.respond_info("\n".join(lines))

    cmd_RECORD_help = ("Record motor torque to calibrate stall thresholds; "
                       "run during normal moves, then STOP=1")
    def cmd_RECORD(self, gcmd):
        self._need_firmware(gcmd)
        if not gcmd.get_int('STOP', 0):
            self._start_window()
            self.recording = {}
            gcmd.respond_info("Recording motor torque. Print or move normally, "
                              "then run C5_MOTION_RECORD STOP=1")
            return
        if self.recording is None:
            raise gcmd.error("Not recording; run C5_MOTION_RECORD first")
        rec, self.recording = self.recording, None
        lines = []
        for axis in self.motors:
            r = rec.get(axis)
            if not r or not r['samples'] or r['torque_min'] is None:
                lines.append("%s: no fast moves recorded" % (axis,))
                continue
            lines.append(
                "%s: torque %.3f to %.3f, drift %.3f to %.3f rad over "
                "%d samples, %d stall flags, %d slips; a stall_threshold of "
                "about %.3f stays clear of normal moves" % (
                    axis, r['torque_min'], r['torque_max'], r['err_min'],
                    r['err_max'], r['samples'], r['stalls'], r['slips'],
                    r['torque_min'] * .5))
        gcmd.respond_info("\n".join(lines))

    cmd_GUARD_help = "Turn the motion guard on or off, or set its action"
    def cmd_GUARD(self, gcmd):
        self.enabled = bool(gcmd.get_int('ENABLE', int(self.enabled),
                                         minval=0, maxval=1))
        action = gcmd.get('ACTION', self.action).lower()
        if action not in ACTIONS:
            raise gcmd.error("ACTION must be one of: " + ", ".join(ACTIONS))
        self.action = action
        watch = gcmd.get('WATCH', self.slip_watch).lower()
        if watch not in SLIP_WATCH:
            raise gcmd.error("WATCH must be one of: " + ", ".join(SLIP_WATCH))
        if gcmd.get_int('RESET', 0):
            self._reset_totals()
        # TEST=1 watches outside a print too, to try it with manual moves
        test_mode = bool(gcmd.get_int('TEST', int(self.test_mode),
                                      minval=0, maxval=1))
        if (watch, test_mode) != (self.slip_watch, self.test_mode):
            self.slip_watch, self.test_mode = watch, test_mode
            if (self.motors and not self._send_slip_watch(self._printing())
                    and test_mode and watch != 'moving'):
                gcmd.respond_info("This mainboard firmware only watches fast "
                                  "moves; flash the newer MCU update")
        gcmd.respond_info("Motion guard %s, action %s%s"
                          % ("on" if self.enabled else "off", self.action,
                             ", test mode (watching outside prints, %s)"
                             % (self.slip_watch,) if self.test_mode else ""))


def load_config(config):
    return C5MotionGuard(config)
