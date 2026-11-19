# Torque ripple calibration for the Creator 5 mclib X/Y motors
#
# The coherent multi-pass measurement, the noise level at half orders, the
# speed sweep and the choice of calibration speeds are adapted from
# phase_calibrate.py and motor_harmonics.py in justinh-rahb/u1-klipper
# (Copyright (C) 2026  Asmund Collin <aakjaergaard@gmail.com>, GPLv3).
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import cmath, logging, math, os, time

import numpy as np

from . import mclib as mclib_mod

# corexy: stepper_x is X+Y, stepper_y is X-Y
MOTOR_PATH = {'stepper_x': (1., 1.), 'stepper_y': (1., -1.)}
MAX_HZ = 700.
# Only harmonics well below Nyquist: the accelerometer filters near it
MAX_FREQ_RATIO = .4
MIN_WINDOW_CYCLES = 5
MIN_MOTOR_SPEED, MAX_MOTOR_SPEED = 5., 150.
# Good corrections on the Creator 5 stay under 0.05 A; anything larger is a
# bad fit, and too much current at a high harmonic grinds the motor
MAX_AMP = 0.06
# A point is only kept if it takes off this share of the vibration
MIN_GAIN = 0.2
# A probe must move the vibration this far above the noise to trust the fit
MIN_PROBE_SNR = 2.
# Sweep: a harmonic shows where it is this many times its noise level
MIN_SNR = 3.
# Calibration speeds: clearly above noise and above this displacement (mm);
# the Creator 5's harmonics are tens of nm
PICK_SNR, PICK_MIN_DISP = 5., 5e-6
MAX_POINTS = 5
HARMONIC_COUNT = 8


def window_slice(times, begin, end):
    return slice(np.searchsorted(times, begin, side='left'),
                 np.searchsorted(times, end, side='right'))


def phase_amplitude(times, values, phase):
    """Complex amplitude per axis of values (samples x axes) along a known
    phase, with a Hann window weighted by the time each sample covers."""
    t = times - times[0]
    intervals = np.diff(t)
    covered = np.minimum(intervals, np.median(intervals))
    spacing = .5 * (np.append(covered, 0.) + np.insert(covered, 0, 0.))
    window = (.5 - .5 * np.cos(2. * math.pi * t / t[-1])) * spacing
    total = window.sum()
    centered = values - (window[:, None] * values).sum(axis=0) / total
    return 2. * (window[:, None] * centered
                 * np.exp(-1j * phase)[:, None]).sum(axis=0) / total


def pick_speeds(rows, harmonic, count):
    """Up to count speeds where the harmonic vibrates most in both
    directions, clearly above noise, spread over its peaks."""
    by_speed = {}
    for speed, d, k, disp, noise in rows:
        if k == harmonic and disp >= max(PICK_MIN_DISP, PICK_SNR * noise):
            by_speed.setdefault(speed, {})[d] = disp
    usable = {s: min(v.values()) for s, v in by_speed.items() if len(v) == 2}
    if not usable:
        return []
    gap = (max(usable) - min(usable)) / (2. * count)
    picked = []
    for s in sorted(usable, key=lambda s: (-usable[s], s)):
        if len(picked) == count:
            break
        if all(abs(s - p) >= gap for p in picked):
            picked.append(s)
    return sorted(picked)


def _clamp(c):
    return c / abs(c) * MAX_AMP if abs(c) > MAX_AMP else c


def _step(c, vib, gain):
    """Correction that cancels vib (a complex vector per axis) given the
    response gain to a change of c: least squares over the axes."""
    return complex(_clamp(c - np.vdot(gain, vib) / np.vdot(gain, gain)))


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
        # Moves averaged per measurement; each starts at the same motor
        # angle, so the vibration adds up and the noise averages out
        self.passes = config.getint('passes', 2, minval=1, maxval=10)
        self.sweep = config.getfloatlist('sweep', (10., 150., 10.), count=3)
        # Board fans held at one speed while measuring, so their vibration
        # doesn't change under the measurement (as for input shaper and VFA)
        self.fan_hold = config.getfloat('fan_hold', .5, minval=0., maxval=1.)
        self.fan_settle = config.getfloat('fan_settle', 3., minval=0.)
        for cmd, func, desc in (
                ('MCLIB_RIPPLE_CALIBRATE', self.cmd_CALIBRATE,
                 self.cmd_CALIBRATE_help),
                ('MCLIB_RIPPLE_MEASURE', self.cmd_MEASURE,
                 self.cmd_MEASURE_help),
                ('MCLIB_RIPPLE_SWEEP', self.cmd_SWEEP, self.cmd_SWEEP_help),
                ('MCLIB_OBSERVER_CHECK', self.cmd_OBSERVER_CHECK,
                 self.cmd_OBSERVER_CHECK_help)):
            self.gcode.register_command(cmd, func, desc=desc)

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

    def _check_homed(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if 'xy' not in toolhead.get_status(
                self.printer.get_reactor().monotonic())['homed_axes']:
            raise gcmd.error("Home X and Y first")

    def _cycles_per_mm(self, mclib):
        rd, steps = mclib.stepper.get_rotation_distance()
        full_steps = steps / float(2 ** mclib.microstep)
        return full_steps / 4. / rd

    def _forward_is_positive(self, mclib):
        # Motor forward (mclib dir 1) is the dir pin high
        return not mclib.stepper.get_dir_inverted()[0]

    def _harmonics(self, gcmd, default):
        try:
            ks = [int(h) for h in gcmd.get('HARMONICS', default).split(',')
                  if h.strip()]
        except ValueError:
            ks = []
        if not ks or any(k < 1 or k > HARMONIC_COUNT for k in ks):
            raise gcmd.error("HARMONICS must be 1 to %d" % (HARMONIC_COUNT,))
        return ks

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

    def _capture(self, name, mclib, motor_speed):
        """One move out and back at motor_speed (mm/s of motor travel), from
        the same start every time. Returns (times, accel samples x 3,
        {direction: (begin, end)}), direction 1 forward."""
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
        fwd = self._forward_is_positive(mclib)
        return data[:, 0], data[:, 1:4], {
            1: positive if fwd else negative, 0: negative if fwd else positive}

    def _measure(self, name, mclib, motor_speed, harmonics, passes):
        """Displacement (mm, complex per axis, in phase with the motor's
        electrical angle) of each harmonic, averaged over passes, and its
        noise level from the half orders either side, which no harmonic
        falls on. Returns {direction: {harmonic: (vector, noise)}};
        harmonics the window can't resolve are left out."""
        base = self._cycles_per_mm(mclib) * motor_speed
        sums = {}
        for _ in range(passes):
            times, values, windows = self._capture(name, mclib, motor_speed)
            for d, (begin, end) in windows.items():
                sl = window_slice(times, begin, end)
                t, v = times[sl], values[sl]
                if len(t) < 64:
                    raise self.gcode.error("cruise window too short")
                span = t[-1] - t[0]
                top = min(MAX_HZ, MAX_FREQ_RATIO * (len(t) - 1) / span)
                # The electrical angle runs backwards in reverse; following
                # it keeps the response linear in the correction
                sign = 1. if d == 1 else -1.
                tr = sign * 2. * math.pi * base * (t - begin)
                for k in harmonics:
                    freq = k * base
                    if freq > top or freq * span < MIN_WINDOW_CYCLES:
                        continue
                    acc = sums.setdefault(d, {}).setdefault(
                        k, [np.zeros(3, complex), np.zeros(3, complex),
                            np.zeros(3, complex)])
                    for i, order in enumerate((k, k - .5, k + .5)):
                        acc[i] += phase_amplitude(t, v, order * tr)
        out = {}
        for d, by_k in sums.items():
            for k, (main, lo, hi) in by_k.items():
                scale = 1. / ((2. * math.pi * k * base) ** 2 * passes)
                noise = .5 * (np.linalg.norm(lo) + np.linalg.norm(hi)) * scale
                out.setdefault(d, {})[k] = (main * scale, noise)
        return out

    def _sweep(self, name, mclib, speeds, harmonics, passes):
        """Rows (speed, direction, harmonic, disp mm, noise mm)."""
        rows = []
        for v in speeds:
            m = self._measure(name, mclib, v, harmonics, passes)
            for d in (1, 0):
                for k, (vec, noise) in sorted(m.get(d, {}).items()):
                    rows.append((v, d, k, float(np.linalg.norm(vec)), noise))
        return rows

    @staticmethod
    def _plan(speed_min, speed_max, step):
        n = int(math.floor((speed_max - speed_min) / step + 1e-9)) + 1
        return [speed_min + i * step for i in range(n)]

    def _sweep_speeds(self, gcmd):
        lo, hi, step = self.sweep
        lo = gcmd.get_float('SPEED_MIN', lo, minval=MIN_MOTOR_SPEED,
                            maxval=MAX_MOTOR_SPEED)
        hi = gcmd.get_float('SPEED_MAX', hi, minval=lo, maxval=MAX_MOTOR_SPEED)
        step = gcmd.get_float('SPEED_STEP', step, above=0.)
        speeds = self._plan(lo, hi, step)
        if len(speeds) > 40:
            raise gcmd.error("SPEED_STEP: at most 40 speeds")
        return speeds

    def _write_csv(self, name, rows):
        path = os.path.join('/tmp', 'mclib_ripple_%s_%s.csv'
                            % (name, time.strftime('%Y%m%d_%H%M%S')))
        try:
            with open(path, 'w') as f:
                f.write("speed,direction,harmonic,disp_nm,noise_nm\n")
                for v, d, k, disp, noise in rows:
                    f.write("%.1f,%s,%d,%.2f,%.2f\n" % (
                        v, "forward" if d else "reverse", k, disp * 1e6,
                        noise * 1e6))
        except IOError:
            logging.exception("mclib_ripple: can't write %s", path)
            return None
        return path

    def _with_fans(self, func):
        toolhead = self.printer.lookup_object('toolhead')
        saved = toolhead.get_max_velocity()
        fans = self._hold_fans()
        self.gcode.run_script_from_command(
            "SET_VELOCITY_LIMIT ACCEL=%.1f" % (self.accel,))
        try:
            return func()
        finally:
            self._release_fans(fans)
            self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.1f" % (saved[1],))

    # Calibration
    @staticmethod
    def _set(mclib, k, c):
        mclib.set_ripple_table(k, [(mclib_mod.RIPPLE_FLAT_HZ, abs(c[1]),
                                    cmath.phase(c[1]), abs(c[0]),
                                    cmath.phase(c[0]))])

    def _calibrate_point(self, name, mclib, k, motor_speed, c, rounds):
        """Correction for harmonic k at one speed, starting from c
        ({direction: complex A}). The vibration responds linearly to the
        correction, so two probes give the response and the correction that
        cancels it. Returns (c, before, after, ok), vibration in mm."""
        def measure(cs):
            self._set(mclib, k, cs)
            m = self._measure(name, mclib, motor_speed, [k], self.passes)
            try:
                return {d: m[d][k] for d in (0, 1)}
            except KeyError:
                raise self.gcode.error("harmonic %d not measurable at %g mm/s"
                                       % (k, motor_speed))

        c = dict(c)
        delta = self.probe
        start = measure(c)
        probes = (measure({d: c[d] + delta for d in (0, 1)}),
                  measure({d: c[d] + 1j * delta for d in (0, 1)}))
        best = {d: (c[d], start[d][0]) for d in (0, 1)}
        gains, new = {}, {}
        for d in (0, 1):
            r0 = start[d][0]
            gain = .5 * ((probes[0][d][0] - r0) / delta
                         + (probes[1][d][0] - r0) / (1j * delta))
            noise = max(start[d][1], probes[0][d][1], probes[1][d][1])
            if np.linalg.norm(gain) * delta < MIN_PROBE_SNR * noise:
                continue  # the probes didn't move it out of the noise
            gains[d] = gain
            new[d] = _step(c[d], r0, gain)
        for _ in range(rounds):
            if not gains:
                break
            trial = dict(c)
            trial.update(new)
            vib = measure(trial)
            for d in gains:
                if np.linalg.norm(vib[d][0]) < np.linalg.norm(best[d][1]):
                    best[d] = (new[d], vib[d][0])
                new[d] = _step(best[d][0], best[d][1], gains[d])
        before = {d: float(np.linalg.norm(start[d][0])) for d in (0, 1)}
        after = {d: float(np.linalg.norm(best[d][1])) for d in (0, 1)}
        c = {d: best[d][0] for d in (0, 1)}
        ok = {d: after[d] <= (1. - MIN_GAIN) * before[d] for d in (0, 1)}
        if not all(ok.values()) and any(c[d] for d in (0, 1) if not ok[d]):
            # No real gain over the correction carried from the last speed:
            # keep it unless running uncorrected is quieter here
            zero = measure({0: 0j, 1: 0j})
            for d in (0, 1):
                z = float(np.linalg.norm(zero[d][0]))
                if not ok[d] and z < after[d]:
                    c[d] = 0j
                    after[d] = z
        return c, before, after, ok

    def _total(self, name, mclib, k, speeds, table):
        """Vibration of harmonic k summed over speeds and directions (mm)
        with table in place."""
        mclib.set_ripple_table(k, table)
        total = 0.
        for v in speeds:
            m = self._measure(name, mclib, v, [k], self.passes)
            total += sum(float(np.linalg.norm(m[d][k][0]))
                         for d in m if k in m[d])
        return total

    def _calibrate_harmonic(self, gcmd, name, mclib, k, rounds, speeds,
                            previous):
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
                b, a = before[d] * 1e6, after[d] * 1e6
                if ok[d]:
                    return "%.1f -> %.1f" % (b, a)
                return "%.1f, no gain, %s" % (
                    b, "kept the slower speed's correction" if c[d]
                    else "left off")
            gcmd.respond_info(
                "%s harmonic %d at %.0f Hz (%.0f mm/s motor): forward %s, "
                "reverse %s nm" % (name, k, freq, v, result(1), result(0)))
        if not kept:
            mclib.set_ripple_table(k, previous)
            return 0
        # A point left uncorrected in one direction also turns off the stock
        # damping there, so check the whole table against what it replaces
        # and keep whichever leaves less
        used = [v for v in speeds if 20. <= k * cpm * v <= MAX_HZ]
        new_total = self._total(name, mclib, k, used, points)
        old_total = self._total(name, mclib, k, used, previous)
        better = new_total < (1. - MIN_GAIN / 2.) * old_total
        gcmd.respond_info(
            "%s harmonic %d over %s mm/s: new %.1f nm, previous %.1f nm; %s"
            % (name, k, ", ".join("%g" % v for v in used), new_total * 1e6,
               old_total * 1e6, "keeping the new correction" if better
               else "keeping the previous correction"))
        mclib.set_ripple_table(k, points if better else previous)
        return kept if better else 0

    cmd_CALIBRATE_help = ("Calibrate torque ripple correction for an X/Y "
                          "motor with the accelerometer (STEPPER, "
                          "HARMONICS, SPEEDS or SPEEDS=AUTO, ROUNDS, "
                          "PASSES, SAVE)")
    def cmd_CALIBRATE(self, gcmd):
        name, mclib = self._motor(gcmd)
        harmonics = self._harmonics(gcmd, '2,4,6,8')
        rounds = gcmd.get_int('ROUNDS', 2, minval=1, maxval=5)
        passes = gcmd.get_int('PASSES', self.passes, minval=1, maxval=10)
        raw = gcmd.get('SPEEDS', ",".join("%g" % v for v in self.speeds))
        auto = raw.strip().lower() == 'auto'
        if not auto:
            try:
                fixed = sorted(float(v) for v in raw.split(",") if v.strip())
            except ValueError:
                fixed = []
            if not fixed or len(fixed) > 6 or any(
                    v < MIN_MOTOR_SPEED or v > MAX_MOTOR_SPEED for v in fixed):
                raise gcmd.error("SPEEDS: AUTO, or at most 6, each %g to %g"
                                 " mm/s" % (MIN_MOTOR_SPEED, MAX_MOTOR_SPEED))
        self._check_homed(gcmd)
        sweep_speeds = self._sweep_speeds(gcmd) if auto else None
        previous = mclib.get_ripple()

        def restore():
            for k in harmonics:
                mclib.set_ripple_table(k, previous.get(k, []))

        def run():
            saved_passes, self.passes = self.passes, passes
            try:
                plan = {k: fixed for k in harmonics} if not auto else {}
                if auto:
                    # Pick speeds from the motor's own vibration, with the
                    # harmonics being calibrated left uncorrected
                    for k in harmonics:
                        mclib.set_ripple_table(k, [])
                    gcmd.respond_info("Finding calibration speeds for %s"
                                      % (name,))
                    rows = self._sweep(name, mclib, sweep_speeds, harmonics, 1)
                    for k in harmonics:
                        plan[k] = pick_speeds(rows, k, MAX_POINTS)
                        if not plan[k]:
                            gcmd.respond_info(
                                "%s harmonic %d: not clearly above noise at"
                                " any speed, left as it was" % (name, k))
                            mclib.set_ripple_table(k, previous.get(k, []))
                kept = 0
                for k in harmonics:
                    if plan[k]:
                        if auto:
                            gcmd.respond_info(
                                "%s harmonic %d at %s mm/s" % (
                                    name, k, ", ".join("%g" % v
                                                       for v in plan[k])))
                        kept += self._calibrate_harmonic(
                            gcmd, name, mclib, k, rounds, plan[k],
                            previous.get(k, []))
                return kept, plan
            finally:
                self.passes = saved_passes

        try:
            kept, plan = self._with_fans(run)
        except Exception:
            restore()
            raise
        if not kept:
            restore()
            raise gcmd.error(
                "%s ripple calibration didn't reduce the vibration anywhere; "
                "kept the previous correction and saved nothing (check the "
                "accelerometer and that the board fans are held)" % (name,))
        if gcmd.get_int('SAVE', 1):
            configfile = self.printer.lookup_object('configfile')
            for k in harmonics:
                if not plan.get(k):
                    continue
                val = mclib.ripple_config_value(k)
                if val is not None:
                    configfile.set('mclib ' + name, 'ripple_%d' % (k,), val)
            gcmd.respond_info("Run SAVE_CONFIG to keep the %s ripple "
                              "correction" % (name,))

    cmd_MEASURE_help = ("Report the vibration at the electrical harmonics "
                        "of an X/Y motor in nm (STEPPER, SPEED in mm/s of "
                        "motor travel, PASSES)")
    def cmd_MEASURE(self, gcmd):
        name, mclib = self._motor(gcmd)
        motor_speed = gcmd.get_float('SPEED', 20., minval=MIN_MOTOR_SPEED,
                                     maxval=MAX_MOTOR_SPEED)
        passes = gcmd.get_int('PASSES', self.passes, minval=1, maxval=10)
        self._check_homed(gcmd)
        ks = list(range(1, HARMONIC_COUNT + 1))
        m = self._with_fans(
            lambda: self._measure(name, mclib, motor_speed, ks, passes))
        cpm = self._cycles_per_mm(mclib)
        for d in (1, 0):
            gcmd.respond_info("%s %s at %.1f mm/s (nm, noise): %s" % (
                name, "forward" if d else "reverse", motor_speed,
                ", ".join("h%d %.0fHz %.1f (%.1f)" % (
                    k, k * cpm * motor_speed, np.linalg.norm(vec) * 1e6,
                    noise * 1e6)
                    for k, (vec, noise) in sorted(m.get(d, {}).items()))))

    cmd_SWEEP_help = ("Measure each electrical harmonic of an X/Y motor over "
                      "a range of speeds (STEPPER, SPEED_MIN, SPEED_MAX, "
                      "SPEED_STEP, HARMONICS, PASSES); writes a CSV to /tmp")
    def cmd_SWEEP(self, gcmd):
        name, mclib = self._motor(gcmd)
        harmonics = self._harmonics(gcmd, ",".join(
            str(k) for k in range(1, HARMONIC_COUNT + 1)))
        passes = gcmd.get_int('PASSES', 1, minval=1, maxval=10)
        speeds = self._sweep_speeds(gcmd)
        self._check_homed(gcmd)
        rows = self._with_fans(
            lambda: self._sweep(name, mclib, speeds, harmonics, passes))
        lines = ["%s: peak vibration per harmonic, %g-%g mm/s (nm at mm/s"
                 " motor speed)" % (name, speeds[0], speeds[-1])]
        for k in harmonics:
            parts = []
            for d in (1, 0):
                seen = [r for r in rows if r[1] == d and r[2] == k
                        and r[3] >= MIN_SNR * r[4]]
                if seen:
                    top = max(seen, key=lambda r: r[3])
                    parts.append("%s %.1f at %g" % (
                        "forward" if d else "reverse", top[3] * 1e6, top[0]))
                else:
                    parts.append("%s below noise"
                                 % ("forward" if d else "reverse",))
            lines.append("h%d: %s" % (k, ", ".join(parts)))
        path = self._write_csv(name, rows)
        if path:
            lines.append("Every speed: %s" % (path,))
        gcmd.respond_info("\n".join(lines))

    # Does the firmware's own load angle see the ripple?
    def _observer_point(self, name, mclib, speed, k, c, passes):
        """Accelerometer (complex vector, mm) and load angle (complex,
        electrical rad) of harmonic k per direction, with correction c."""
        self._set(mclib, k, c)
        mclib.arm_harmonics()
        m = self._measure(name, mclib, speed, [k], passes)
        h = mclib.read_harmonics()
        return {d: (m[d][k][0], h[(d, k)][0]) for d in (0, 1)}

    cmd_OBSERVER_CHECK_help = ("Compare the firmware's load angle with the "
                               "accelerometer at one harmonic, with and "
                               "without a correction probe (STEPPER, SPEED, "
                               "HARMONIC, PROBE in A)")
    def cmd_OBSERVER_CHECK(self, gcmd):
        name, mclib = self._motor(gcmd)
        if not mclib.has_harmonics():
            raise gcmd.error("%s: firmware has no load angle analysis"
                             % (name,))
        speed = gcmd.get_float('SPEED', 40., minval=MIN_MOTOR_SPEED,
                               maxval=MAX_MOTOR_SPEED)
        k = gcmd.get_int('HARMONIC', 4, minval=1, maxval=HARMONIC_COUNT)
        probe = gcmd.get_float('PROBE', .03, above=0., maxval=MAX_AMP)
        passes = gcmd.get_int('PASSES', self.passes, minval=1, maxval=10)
        self._check_homed(gcmd)
        previous = mclib.get_ripple()
        cpm = self._cycles_per_mm(mclib)

        def run():
            out = []
            for c in (0j, probe, 1j * probe):
                out.append(self._observer_point(
                    name, mclib, speed, k, {0: c, 1: c}, passes))
            return out
        try:
            base, pr, pi = self._with_fans(run)
        finally:
            mclib.set_ripple_table(k, previous.get(k, []))
        # load angle to motor displacement: one electrical period is
        # 1/cpm mm of belt
        to_nm = 1e6 / (2. * math.pi * cpm)
        lines = ["%s h%d at %g mm/s, probe %.3f A (nm):" % (
            name, k, speed, probe)]
        for d in (1, 0):
            acc = [np.linalg.norm(x[d][0]) * 1e6 for x in (base, pr, pi)]
            obs = [abs(x[d][1]) * to_nm for x in (base, pr, pi)]
            # response to the probes: how each sensor's reading moves
            dacc = [np.linalg.norm(x[d][0] - base[d][0]) * 1e6
                    for x in (pr, pi)]
            dobs = [abs(x[d][1] - base[d][1]) * to_nm for x in (pr, pi)]
            lines.append(
                "%s: accelerometer %.1f / %.1f / %.1f, moved %.1f %.1f;"
                " load angle %.1f / %.1f / %.1f, moved %.1f %.1f" % (
                    "forward" if d else "reverse", acc[0], acc[1], acc[2],
                    dacc[0], dacc[1], obs[0], obs[1], obs[2], dobs[0],
                    dobs[1]))
        gcmd.respond_info("\n".join(lines))


def load_config(config):
    return MCLibRipple(config)
