# Torque ripple calibration for the Creator 5 mclib X/Y motors
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import cmath, logging, math

import numpy as np

from . import mclib as mclib_mod

# corexy: stepper_x is X+Y, stepper_y is X-Y
MOTOR_PATH = {'stepper_x': (1., 1.), 'stepper_y': (1., -1.)}
MAX_HZ = 700.
MIN_MOTOR_SPEED, MAX_MOTOR_SPEED = 5., 150.
# Good corrections on the Creator 5 stay under 0.05 A; anything larger is a
# bad fit, and too much current at a high harmonic grinds the motor
MAX_AMP = 0.06
# A probe must change the vibration by this share to trust the fit, and a
# point is only kept if it takes off this share of the vibration power
MIN_PROBE_EFFECT = 0.1
MIN_GAIN = 0.36  # power, so 20% less vibration


class MCLibRipple:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.accel_chip = config.get('accel_chip', 'lis2dw')
        self.center = config.getfloatlist('center', (130., 130.), count=2)
        self.accel = config.getfloat('accel', 5000., above=0.)
        # Motor speeds (mm/s of motor travel) each harmonic is calibrated
        # at; the correction is interpolated between them
        self.speeds = config.getfloatlist('speeds', (8., 15., 30., 60., 120.))
        self.probe = config.getfloat('probe', 0.015, above=0., maxval=MAX_AMP)
        self.cruise_time = config.getfloat('cruise_time', .5, above=.1)
        # Board fans held at one speed while measuring, so their vibration
        # doesn't change under the measurement (as for input shaper and VFA)
        self.fan_hold = config.getfloat('fan_hold', .5, minval=0., maxval=1.)
        self.fan_settle = config.getfloat('fan_settle', 3., minval=0.)
        self.gcode.register_command('MCLIB_RIPPLE_CALIBRATE',
                                    self.cmd_CALIBRATE,
                                    desc=self.cmd_CALIBRATE_help)
        self.gcode.register_command('MCLIB_RIPPLE_MEASURE',
                                    self.cmd_MEASURE,
                                    desc=self.cmd_MEASURE_help)

    # Lookups
    def _motor(self, gcmd):
        name = gcmd.get('STEPPER').lower()
        if name not in MOTOR_PATH:
            raise gcmd.error("STEPPER must be stepper_x or stepper_y")
        mclib = self.printer.lookup_object('mclib ' + name, None)
        if mclib is None or not mclib.has_ripple():
            raise gcmd.error("%s: ripple correction needs [mclib %s] and "
                             "newer mainboard firmware" % (name, name))
        return name, mclib

    def _cycles_per_mm(self, mclib):
        rd, steps = mclib.stepper.get_rotation_distance()
        full_steps = steps / float(2 ** mclib.microstep)
        return full_steps / 4. / rd

    def _forward_is_positive(self, mclib):
        # Motor forward (mclib dir 1) is the dir pin high
        return not mclib.stepper.get_dir_inverted()[0]

    # Board fans
    def _hold_fans(self):
        """Hold the board fans for a measurement; returns the hold to put
        back afterwards (an outer _FANS_HOLD is kept)."""
        fans = self.printer.lookup_object('c5_board_fans', None)
        if fans is None:
            return None
        previous = fans.hold
        if previous != self.fan_hold:
            fans.hold = self.fan_hold
            # Let them reach the held speed before measuring
            self.printer.lookup_object('toolhead').dwell(self.fan_settle)
        return (fans, previous)

    @staticmethod
    def _release_fans(held):
        if held is not None:
            held[0].hold = held[1]

    # Measurement
    def _move(self, x, y, speed):
        self.gcode.run_script_from_command(
            "G1 X%.3f Y%.3f F%.1f" % (x, y, speed * 60.))

    def _measure(self, name, mclib, motor_speed, freqs):
        """One move out and back at motor_speed (mm/s of motor travel).
        Returns {direction: [power at each freq]}, direction 1 forward."""
        toolhead = self.printer.lookup_object('toolhead')
        chip = self.printer.lookup_object(self.accel_chip)
        px, py = MOTOR_PATH[name]
        ramp = motor_speed / self.accel
        motor_dist = motor_speed * (self.cruise_time + 2. * ramp) + 1.
        half = motor_dist / 4.  # each axis moves half the motor distance
        cx, cy = self.center
        start = (cx - px * half, cy - py * half)
        end = (cx + px * half, cy + py * half)
        speed = motor_speed / math.sqrt(2.)
        self.gcode.run_script_from_command(
            "SET_VELOCITY_LIMIT ACCEL=%.1f" % (self.accel,))
        self._move(start[0], start[1], 100.)
        self.gcode.run_script_from_command("M400")
        client = chip.start_internal_client()
        toolhead.dwell(.1)
        t0 = toolhead.get_last_move_time()
        self._move(end[0], end[1], speed)
        t1 = toolhead.get_last_move_time()
        self._move(start[0], start[1], speed)
        t2 = toolhead.get_last_move_time()
        toolhead.dwell(.1)
        self.gcode.run_script_from_command("M400")
        client.finish_measurements()
        data = np.array([(s.time, s.accel_x, s.accel_y, s.accel_z)
                         for s in client.get_samples()])
        if len(data) < 100:
            raise self.gcode.error("No accelerometer data from %s"
                                   % (self.accel_chip,))
        margin = ramp + .03
        positive = (t0 + margin, t1 - margin)
        negative = (t1 + margin, t2 - margin)
        fwd_pos = self._forward_is_positive(mclib)
        return {1: self._powers(data, positive if fwd_pos else negative,
                                freqs),
                0: self._powers(data, negative if fwd_pos else positive,
                                freqs)}

    @staticmethod
    def _powers(data, window, freqs):
        sel = (data[:, 0] >= window[0]) & (data[:, 0] <= window[1])
        d = data[sel]
        if len(d) < 50:
            raise ValueError("cruise window too short")
        t = d[:, 0] - d[0, 0]
        acc = d[:, 1:4] - d[:, 1:4].mean(axis=0)
        w = np.hanning(len(t))
        out = []
        for f in freqs:
            ph = np.exp(-2j * np.pi * f * t) * w
            x = (acc * ph[:, None]).sum(axis=0) / w.sum()
            out.append(float((np.abs(x) ** 2).sum()))
        return out

    # Calibration
    @staticmethod
    def _set(mclib, k, c):
        mclib.set_ripple_table(k, [(mclib_mod.RIPPLE_FLAT_HZ, abs(c[1]),
                                    cmath.phase(c[1]), abs(c[0]),
                                    cmath.phase(c[0]))])

    def _calibrate_point(self, name, mclib, k, motor_speed, c, rounds):
        """Correction for harmonic k at one speed, starting from c
        ({direction: complex}); returns (c, before, after) powers."""
        freq = k * self._cycles_per_mm(mclib) * motor_speed

        def measure(cs):
            self._set(mclib, k, cs)
            p = self._measure(name, mclib, motor_speed, [freq])
            return {d: p[d][0] for d in (0, 1)}

        c = dict(c)
        delta = self.probe
        start = measure(c)
        best = dict(start)
        for rnd in range(rounds):
            p0 = measure(c)
            pp = [measure({d: c[d] + q for d in (0, 1)})
                  for q in (delta, 1j * delta, -delta)]
            new_c = {}
            for d in (0, 1):
                P0, P1, P2, P3 = p0[d], pp[0][d], pp[1][d], pp[2][d]
                g = (P1 + P3 - 2. * P0) / (2. * delta * delta)
                if g * delta * delta < MIN_PROBE_EFFECT * P0:
                    # The probes barely moved it: noise, not a fit
                    new_c[d] = c[d]
                    continue
                w = complex((P1 - P3) / (4. * delta),
                            (P0 + g * delta * delta - P2) / (2. * delta))
                cand = c[d] - w.conjugate() / g
                if abs(cand) > MAX_AMP:
                    cand = cand / abs(cand) * MAX_AMP
                new_c[d] = cand
            p_new = measure(new_c)
            for d in (0, 1):
                if p_new[d] < best[d]:
                    best[d] = p_new[d]
                    c[d] = new_c[d]
            delta *= .5
        ok = {d: best[d] <= (1. - MIN_GAIN) * start[d] for d in (0, 1)}
        for d in (0, 1):
            if not ok[d]:
                c[d] = 0j
        return c, start, best, ok

    def _calibrate_harmonic(self, gcmd, name, mclib, k, rounds, speeds):
        cpm = self._cycles_per_mm(mclib)
        c = {0: 0j, 1: 0j}
        points = []
        kept = 0
        for v in speeds:
            freq = k * cpm * v
            if freq < 20. or freq > MAX_HZ:
                continue
            c, before, after, ok = self._calibrate_point(name, mclib, k, v,
                                                         c, rounds)
            points.append((cpm * v, abs(c[1]), cmath.phase(c[1]),
                           abs(c[0]), cmath.phase(c[0])))
            kept += ok[0] + ok[1]

            def result(d):
                return ("%.0f -> %.0f" % (math.sqrt(before[d]),
                                          math.sqrt(after[d]))
                        if ok[d] else "%.0f, no gain, left off"
                        % (math.sqrt(before[d]),))
            gcmd.respond_info(
                "%s harmonic %d at %.0f Hz (%.0f mm/s motor): forward %s, "
                "reverse %s mm/s^2" % (name, k, freq, v, result(1),
                                       result(0)))
        mclib.set_ripple_table(k, points if kept else [])
        return kept

    cmd_CALIBRATE_help = ("Calibrate torque ripple correction for an X/Y "
                          "motor with the accelerometer (STEPPER, "
                          "HARMONICS=2,4,6,8, SPEEDS, ROUNDS=2, SAVE=1)")
    def cmd_CALIBRATE(self, gcmd):
        name, mclib = self._motor(gcmd)
        harmonics = [int(h) for h in
                     gcmd.get('HARMONICS', '2,4,6,8').split(',') if h.strip()]
        for k in harmonics:
            if k < 1 or k > 8:
                raise gcmd.error("HARMONICS must be 1 to 8")
        rounds = gcmd.get_int('ROUNDS', 2, minval=1, maxval=5)
        speeds = sorted(float(v) for v in gcmd.get(
            'SPEEDS', ",".join("%g" % v for v in self.speeds)).split(",")
            if v.strip())
        if len(speeds) > 6 or any(v < MIN_MOTOR_SPEED or v > MAX_MOTOR_SPEED
                                  for v in speeds):
            raise gcmd.error("SPEEDS: at most 6, each %g to %g mm/s"
                             % (MIN_MOTOR_SPEED, MAX_MOTOR_SPEED))
        toolhead = self.printer.lookup_object('toolhead')
        if 'xy' not in toolhead.get_status(
                self.printer.get_reactor().monotonic())['homed_axes']:
            raise gcmd.error("Home X and Y first")
        saved = toolhead.get_max_velocity()
        previous = mclib.get_ripple()

        def restore():
            for k in harmonics:
                mclib.set_ripple_table(k, previous.get(k, []))
        fans = self._hold_fans()
        kept = 0
        try:
            for k in harmonics:
                kept += self._calibrate_harmonic(gcmd, name, mclib, k, rounds,
                                                 speeds)
        except Exception:
            restore()
            raise
        finally:
            self._release_fans(fans)
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.1f" % (saved[1],))
        if not kept:
            restore()
            raise gcmd.error(
                "%s ripple calibration didn't reduce the vibration anywhere; "
                "kept the previous correction and saved nothing (check the "
                "accelerometer and that the board fans are held)" % (name,))
        if gcmd.get_int('SAVE', 1):
            configfile = self.printer.lookup_object('configfile')
            for k in harmonics:
                val = mclib.ripple_config_value(k)
                if val is not None:
                    configfile.set('mclib ' + name, 'ripple_%d' % (k,), val)
            gcmd.respond_info("Run SAVE_CONFIG to keep the %s ripple "
                              "correction" % (name,))

    cmd_MEASURE_help = ("Report the vibration at the electrical harmonics "
                        "of an X/Y motor (STEPPER, SPEED in mm/s of motor "
                        "travel)")
    def cmd_MEASURE(self, gcmd):
        name, mclib = self._motor(gcmd)
        motor_speed = gcmd.get_float('SPEED', 20., minval=MIN_MOTOR_SPEED,
                                     maxval=MAX_MOTOR_SPEED)
        toolhead = self.printer.lookup_object('toolhead')
        saved = toolhead.get_max_velocity()
        cpm = self._cycles_per_mm(mclib)
        ks = [k for k in range(1, 9) if k * cpm * motor_speed <= MAX_HZ]
        freqs = [k * cpm * motor_speed for k in ks]
        fans = self._hold_fans()
        try:
            p = self._measure(name, mclib, motor_speed, freqs)
        finally:
            self._release_fans(fans)
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.1f" % (saved[1],))
        for d in (1, 0):
            gcmd.respond_info("%s %s at %.1f mm/s: %s" % (
                name, "forward" if d else "reverse", motor_speed,
                ", ".join("h%d %.0fHz %.0f" % (k, f, math.sqrt(v))
                          for k, f, v in zip(ks, freqs, p[d]))))


def load_config(config):
    return MCLibRipple(config)
