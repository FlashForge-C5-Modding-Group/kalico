# GD32 mcu motor control library configuration
#
# Copyright (C) 2024  Dongzhi Yu <dongzhi.yu@gigadevice.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from klippy import stepper

######################################################################
# MCLIB printer object
######################################################################

MAX_CURRENT = 4.0
RIPPLE_HARMONICS = 8
RIPPLE_POINTS = 6
# Calibrated corrections stay well under this; more grinds the motor
RIPPLE_MAX_AMP = 0.1
# A single-point correction is held at every speed below this frequency
RIPPLE_FLAT_HZ = 10000.
TIMING_RUN_FORMAT = (
    "mclib_timing_run oid=%c run_max=%u run_avg=%u run_samples=%u"
    " gap_max=%u missed=%u timer_max=%u own_max=%u serial_max=%u"
    " timer_steal=%u tick_steal=%u")
steps = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7, 256: 8}
# How the motor angle moves between steps (firmware without quadratic treats
# it as linear)
INTERPOLATION_MODES = {'linear': 1, 'quadratic': 2, 'quadratic_angle': 3}
stepper_id = {'stepper_x':0, 'stepper_y':1, 'stepper_z':2, 'extruder':3}

class MCLIB:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.stepper_name = " ".join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        if not config.has_section(self.stepper_name):
            raise config.error(
                "Could not find config section '[%s]' required by MCLIB"
                % (self.stepper_name,))
        self.stepper = None
        # Get config parameters
        sconfig = config.getsection(self.stepper_name)

        self.microstep = sconfig.getchoice('microsteps', steps)
        self.motor_rs  = int(config.getfloat('motor_rs', above=0.) * 1000)
        self.motor_ls  = int(config.getfloat('motor_ls', above=0.) * 1000000)
        self.motor_km  = int(config.getfloat('motor_km', above=0.) * 1000000)
        self.bus_voltage  = int(config.getfloat('bus_voltage', above=0.)* 1000)
        run_current = config.getfloat('run_current', 1., minval=0., maxval=4.)
        self.run_current = int(run_current * 1000)
        self.hold_current = int(config.getfloat('hold_current', run_current, minval=0., maxval=run_current)* 1000)
        self.interpolate = config.getboolean('interpolate', True)
        self.interpolation_mode = config.getchoice(
            'interpolation_mode', INTERPOLATION_MODES, 'linear')
        # How early a compressed step may come (Kalico's default is 25 us);
        # less makes the step times the interpolation follows more exact
        self.step_error = config.getfloat('step_error', None, minval=0.000001,
                                          maxval=0.000025)
        self.set_interpolation_cmd = None
        self.stall_threshold = int(config.getfloat('stall_threshold', self.bus_voltage, above=0.)* 1000)
        self.feedforward = config.getint('feedforward', 0, minval=0, maxval=100)
        # current PI bandwidth in percent of stock (1 kHz)
        self.pi_scale = config.getint('current_bandwidth', 100, minval=25,
                                      maxval=400)
        self.td1_amp = int(config.getfloat('td1_amp', 0., minval=0., maxval=run_current)* 1000)
        self.td1_phase1 = int(config.getfloat('td1_phase1', 0.)* 1000)
        self.td1_phase2 = int(config.getfloat('td1_phase2', 0.)* 1000)
        self.td2_amp = int(config.getfloat('td2_amp', 0., minval=0., maxval=run_current)* 1000)
        self.td2_phase1 = int(config.getfloat('td2_phase1', 0.)* 1000)
        self.td2_phase2 = int(config.getfloat('td2_phase2', 0.)* 1000)
        self.td4_amp = int(config.getfloat('td4_amp', 0., minval=0., maxval=run_current)* 1000)
        self.td4_phase1 = int(config.getfloat('td4_phase1', 0.)* 1000)
        self.td4_phase2 = int(config.getfloat('td4_phase2', 0.)* 1000)  
        # Torque ripple correction for harmonic k of the electrical angle:
        # ripple_<k> = groups of electrical Hz, amp_fwd, phase_fwd, amp_rev,
        # phase_rev (A, rad), in rising frequency
        self.ripple = {}
        for k in range(1, RIPPLE_HARMONICS + 1):
            vals = config.getfloatlist('ripple_%d' % (k,), None)
            if vals is None:
                continue
            if not vals or len(vals) % 5 or len(vals) > 5 * RIPPLE_POINTS:
                raise config.error(
                    "ripple_%d needs groups of 5 values (Hz, amp_fwd, "
                    "phase_fwd, amp_rev, phase_rev), at most %d"
                    % (k, RIPPLE_POINTS))
            points = [tuple(vals[i:i + 5]) for i in range(0, len(vals), 5)]
            if any(abs(p[1]) > RIPPLE_MAX_AMP or abs(p[3]) > RIPPLE_MAX_AMP
                   for p in points):
                raise config.error("ripple_%d: amplitudes over %.2f A"
                                   % (k, RIPPLE_MAX_AMP))
            self.ripple[k] = points
        self.set_ripple_cmd = None
        self.calibrated_damping = {
            1: (self.td1_amp, self.td1_phase1, self.td1_phase2),
            2: (self.td2_amp, self.td2_phase1, self.td2_phase2),
            4: (self.td4_amp, self.td4_phase1, self.td4_phase2),
        }

        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.set_current_cmd = None
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("MCLIB_SET_CURRENT", "STEPPER", self.name,
                                   self.cmd_MCLIB_SET_CURRENT,
                                   desc=self.cmd_MCLIB_SET_CURRENT_help)
        gcode.register_mux_command("MCLIB_SET_INTERPOLATION", "STEPPER",
                                   self.name,
                                   self.cmd_MCLIB_SET_INTERPOLATION,
                                   desc=self.cmd_MCLIB_SET_INTERPOLATION_help)

        # Motion health (c5_motion_guard): totals since the last take_health()
        self.query_status_cmd = None
        self.set_stall_cmd = None
        self.set_slip_cmd = None
        self.query_timing_cmd = None
        self.last_timing = self.last_timing_run = self.last_quality = None
        self.has_timing_run = self.has_quality = False
        self.health = self._empty_health()
        self.last_status = None
        gcode.register_mux_command("MCLIB_SET_STALL", "STEPPER", self.name,
                                   self.cmd_MCLIB_SET_STALL,
                                   desc=self.cmd_MCLIB_SET_STALL_help)
        gcode.register_mux_command("MCLIB_TIMING", "STEPPER", self.name,
                                   self.cmd_MCLIB_TIMING,
                                   desc=self.cmd_MCLIB_TIMING_help)
        self.query_slip_cmd = self.clear_slip_cmd = None
        self.slip_events = {}
        gcode.register_mux_command("MCLIB_SLIP_LOG", "STEPPER", self.name,
                                   self.cmd_MCLIB_SLIP_LOG,
                                   desc=self.cmd_MCLIB_SLIP_LOG_help)

        self.set_injection_cmd = None
        self.set_feedforward_cmd = None
        self.set_pi_scale_cmd = None
        self.arm_harmonics_cmd = self.query_harmonics_cmd = None
        self.harmonics = {}
        gcode.register_mux_command("MCLIB_SET_BANDWIDTH", "STEPPER",
                                   self.name, self.cmd_MCLIB_SET_BANDWIDTH,
                                   desc=self.cmd_MCLIB_SET_BANDWIDTH_help)
        gcode.register_mux_command("MCLIB_SET_FEEDFORWARD", "STEPPER",
                                   self.name, self.cmd_MCLIB_SET_FEEDFORWARD,
                                   desc=self.cmd_MCLIB_SET_FEEDFORWARD_help)
        gcode.register_mux_command("MCLIB_SET_INJECTION", "STEPPER", self.name,
                                   self.cmd_MCLIB_SET_INJECTION,
                                   desc=self.cmd_MCLIB_SET_INJECTION_help)

        self.set_resonance_damp_cmd = None
        if self.name != 'extruder':
            gcode.register_mux_command("MCLIB_SET_RESONANCE_DAMP", "STEPPER", self.name,
                                   self.cmd_MCLIB_SET_RESONANCE_DAMP,
                                   desc=self.cmd_MCLIB_SET_RESONANCE_DAMP_help)
            gcode.register_mux_command("MCLIB_SET_RIPPLE", "STEPPER",
                                       self.name, self.cmd_MCLIB_SET_RIPPLE,
                                       desc=self.cmd_MCLIB_SET_RIPPLE_help)
            gcode.register_mux_command("MCLIB_APPLY_CALIBRATION", "STEPPER",
                                       self.name, self.cmd_MCLIB_APPLY_CALIBRATION,
                                       desc=self.cmd_MCLIB_APPLY_CALIBRATION_help)

    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        self.mcu = self.stepper.get_mcu()
        # Firmware that counts both step pin edges takes a step as one timer
        # event instead of a step and an unstep
        if self.mcu.get_constants().get("C5_MAINBOARD_STEP_BOTH_EDGE"):
            self.stepper.setup_default_pulse_duration(0.000000100, True)
        if self.step_error is not None:
            self.stepper.set_max_step_error(self.step_error)
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.oid = self.mcu.create_oid()

        self.mcu.add_config_cmd("config_mclib oid=%d stepper=%s rs=%u ls=%u km=%u" % (
            self.oid, self.stepper_name, self.motor_rs, self.motor_ls, self.motor_km))

        self.mcu.add_config_cmd("mclib_config_microstep oid=%d interpolate=%d mstep=%u" % (
            self.oid, self.interpolation_mode if self.interpolate else 0,
            self.microstep))
            
        self.mcu.add_config_cmd("mclib_set_current oid=%d run_current=%u hold_current=%u"% (
            self.oid, self.run_current, self.hold_current))
            
        self.mcu.add_config_cmd("mclib_config_stalldetect oid=%d stallthrs=%u" % (
            self.oid, self.stall_threshold))

        self.set_current_cmd = self.mcu.lookup_command(
            "mclib_set_current oid=%c run_current=%u hold_current=%u")
        self.set_stall_cmd = self.mcu.lookup_command(
            "mclib_config_stalldetect oid=%c stallthrs=%u")
        # Older mainboard firmware has no status query; the guard reports that
        self.query_status_cmd = self.mcu.try_lookup_command(
            "mclib_query_status oid=%c")
        if self.query_status_cmd is not None:
            self.mcu.register_response(self._handle_status, "mclib_status",
                                       self.oid)
        self.set_slip_cmd = self.mcu.try_lookup_command(
            "mclib_config_slip oid=%c mode=%c")
        # Control-loop duration counts; absent on older firmware
        self.query_timing_cmd = self.mcu.try_lookup_command(
            "mclib_query_timing oid=%c")
        if self.query_timing_cmd is not None:
            self.mcu.register_response(self._handle_timing, "mclib_timing",
                                       self.oid)
            # Moving-only timing and preemption; absent on older firmware
            self.has_timing_run = self.mcu.try_lookup_command(
                TIMING_RUN_FORMAT) is not None
            if self.has_timing_run:
                self.mcu.register_response(self._handle_timing_run,
                                           "mclib_timing_run", self.oid)
            self.has_quality = self.mcu.try_lookup_command(
                "mclib_quality oid=%c ripple=%u ripple_samples=%u"
            ) is not None
            if self.has_quality:
                self.mcu.register_response(self._handle_quality,
                                           "mclib_quality", self.oid)
        # Interpolation mode at run time; absent on older firmware
        self.set_interpolation_cmd = self.mcu.try_lookup_command(
            "mclib_set_interpolation oid=%c mode=%c")
        # Slip log; absent on older firmware
        self.query_slip_cmd = self.mcu.try_lookup_command(
            "mclib_query_slip_event oid=%c index=%c")
        self.clear_slip_cmd = self.mcu.try_lookup_command(
            "mclib_clear_slip_log oid=%c")
        if self.query_slip_cmd is not None:
            self.mcu.register_response(self._handle_slip_event,
                                       "mclib_slip_event", self.oid)
        # Observer injection scale; absent on older firmware
        self.set_injection_cmd = self.mcu.try_lookup_command(
            "mclib_set_injection oid=%c scale=%u")
        self.set_feedforward_cmd = self.mcu.try_lookup_command(
            "mclib_set_feedforward oid=%c gain=%u")
        if self.set_feedforward_cmd is not None and self.feedforward:
            self.mcu.add_config_cmd("mclib_set_feedforward oid=%d gain=%u"
                                    % (self.oid, self.feedforward))
        # Current bandwidth and the load angle analysis; absent on older
        # firmware
        self.set_pi_scale_cmd = self.mcu.try_lookup_command(
            "mclib_set_pi_scale oid=%c percent=%u")
        if self.set_pi_scale_cmd is not None and self.pi_scale != 100:
            self.mcu.add_config_cmd("mclib_set_pi_scale oid=%d percent=%u"
                                    % (self.oid, self.pi_scale))
        self.arm_harmonics_cmd = self.mcu.try_lookup_command(
            "mclib_arm_harmonics oid=%c")
        self.query_harmonics_cmd = self.mcu.try_lookup_command(
            "mclib_query_harmonics oid=%c dir=%c harmonic=%c")
        if self.query_harmonics_cmd is not None:
            self.mcu.register_response(self._handle_harmonics,
                                       "mclib_harmonics", self.oid)
        if self.pi_scale != 100 and self.set_pi_scale_cmd is None:
            logging.warning("%s: current_bandwidth needs newer mainboard"
                            " firmware", self.name)

        if self.name != 'extruder':
            # Older mainboard firmware has no ripple correction
            self.set_ripple_cmd = self.mcu.try_lookup_command(
                "mclib_set_ripple oid=%c harmonic=%c dir=%c index=%c count=%c"
                " freq=%u amp=%i phase=%i")
            for tdx, (amp, phase1, phase2) in sorted(
                    self._stock_damping().items()):
                self.mcu.add_config_cmd(
                    "mclib_set_resonance_damp oid=%d tdx=%d amp=%u"
                    " phase1=%u phase2=%u"
                    % (self.oid, tdx, amp, phase1, phase2))

        if self.name != 'extruder':
            self.set_resonance_damp_cmd = self.mcu.lookup_command(
                "mclib_set_resonance_damp oid=%c tdx=%c amp=%u phase1=%u phase2=%u")
            if self.set_ripple_cmd is not None:
                for k in sorted(self.ripple):
                    for args in self._ripple_args(k):
                        self.mcu.add_config_cmd(
                            "mclib_set_ripple oid=%d harmonic=%d dir=%d"
                            " index=%d count=%d freq=%d amp=%d phase=%d"
                            % tuple([self.oid] + args))
            elif self.ripple:
                logging.warning("%s: ripple correction needs newer mainboard"
                                " firmware", self.name)

    def _ripple_args(self, harmonic):
        points = self.ripple.get(harmonic, [])
        if not points:
            return [[harmonic, d, 0, 0, 0, 0, 0] for d in (0, 1)]
        args = []
        for d in (0, 1):
            for i, p in enumerate(points):
                amp, phase = (p[1], p[2]) if d else (p[3], p[4])
                args.append([harmonic, d, i, len(points),
                             int(round(p[0] * 1000.)),
                             int(round(amp * 1000.)),
                             int(round(phase * 1000.))])
        return args

    def _stock_damping(self):
        # Where a ripple table covers the speed, the mainboard uses it in
        # place of the stock value for that harmonic
        return dict(self.calibrated_damping)

    def has_ripple(self):
        return self.set_ripple_cmd is not None

    def get_ripple(self):
        return {k: list(v) for k, v in self.ripple.items()}

    def set_ripple_table(self, harmonic, points):
        """points: (electrical Hz, amp_fwd, phase_fwd, amp_rev, phase_rev)
        in rising frequency; empty clears it. Until restart unless saved."""
        if points:
            self.ripple[harmonic] = [tuple(p) for p in points]
        else:
            self.ripple.pop(harmonic, None)
        for args in self._ripple_args(harmonic):
            self.set_ripple_cmd.send([self.oid] + args)

    def set_ripple(self, harmonic, direction, amp, phase):
        """One correction for a direction (motor forward is 1) at every
        speed, keeping the other direction's single value."""
        old = self.ripple.get(harmonic)
        other = (0., 0.)
        if old and len(old) == 1:
            other = (old[0][3], old[0][4]) if direction else (old[0][1],
                                                              old[0][2])
        fwd = (amp, phase) if direction else other
        rev = other if direction else (amp, phase)
        if not fwd[0] and not rev[0]:
            self.set_ripple_table(harmonic, [])
        else:
            self.set_ripple_table(
                harmonic, [(RIPPLE_FLAT_HZ, fwd[0], fwd[1], rev[0], rev[1])])

    def ripple_config_value(self, harmonic):
        points = self.ripple.get(harmonic)
        if not points:
            return None
        return ", ".join("%.1f, %.4f, %.3f, %.4f, %.3f" % p
                             for p in points)

    cmd_MCLIB_SET_CURRENT_help = "Sets the current of stepper in mclib "
    def cmd_MCLIB_SET_CURRENT(self, gcmd):
        run_current = int(gcmd.get_float('CURRENT', None, minval=0., maxval=MAX_CURRENT)* 1000)
        hold_current = int(gcmd.get_float('HOLDCURRENT', None, above=0., maxval=run_current)* 1000)

        gcmd.respond_info("set run_current=%u hold_current=%u" % (run_current, hold_current))

        if self.set_current_cmd is None:
            # Send setup message via mcu initialization
            self.mcu.add_config_cmd("mclib_set_current oid=%d run_current=%u hold_current=%u"
                                    % (self.oid, run_current, hold_current))
            return
        self.set_current_cmd.send([self.oid, run_current, hold_current])

    @staticmethod
    def _empty_health():
        return {'stalls': 0, 'slips': 0, 'slip_total': 0, 'samples': 0,
                'torque_min': None, 'torque_max': None,
                'err_min': None, 'err_max': None}

    def _handle_status(self, params):
        self.last_status = params
        h = self.health
        h['stalls'] += params['stalls']
        h['slips'] += params['slips']
        h['slip_total'] += params['slip_total']
        if not params['samples']:
            return
        h['samples'] += params['samples']
        for key, pick in (('torque_min', min), ('torque_max', max),
                          ('err_min', min), ('err_max', max)):
            value = params[key] / 1000.
            h[key] = value if h[key] is None else pick(h[key], value)

    def query_health(self):
        """Ask the MCU for its counters; the reply adds to self.health."""
        if self.query_status_cmd is None:
            return False
        self.query_status_cmd.send([self.oid])
        return True

    def query_timing(self):
        """Ask the MCU for control-loop cycle counts since the last query.
        The reply lands in self.last_timing."""
        if self.query_timing_cmd is None:
            return False
        self.query_timing_cmd.send([self.oid])
        return True

    def _handle_timing(self, params):
        self.last_timing = params

    def _handle_timing_run(self, params):
        self.last_timing_run = params

    def _handle_quality(self, params):
        self.last_quality = params

    def _handle_slip_event(self, params):
        self.slip_events[params['index']] = params

    cmd_MCLIB_SLIP_LOG_help = ("List the last 16 counted slips with the motor"
                               " state at each (CLEAR=1 empties it)")
    def cmd_MCLIB_SLIP_LOG(self, gcmd):
        if self.query_slip_cmd is None:
            raise gcmd.error("%s: firmware has no slip log" % (self.name,))
        if gcmd.get_int('CLEAR', 0):
            self.clear_slip_cmd.send([self.oid])
            gcmd.respond_info("%s: slip log cleared" % (self.name,))
            return
        lines = []
        for index in range(16):
            self.slip_events.pop(index, None)
            self.query_slip_cmd.send([self.oid, index])
            for _ in range(20):
                if index in self.slip_events:
                    break
                self.reactor.pause(self.reactor.monotonic() + 0.02)
            e = self.slip_events.get(index)
            if e is None or not e['valid']:
                break
            state = e['state']
            mode = {0: 'off', 1: 'hold', 2: 'run'}.get(state & 3, '?')
            lines.append(
                "%s %s src=%s %s%s%s %s level=%d after %d samples"
                " step_time=%u period=%u error=%.3f step=%.3f drift=%.3f"
                " observer=%d cmd=%d rad/s"
                % ('slip' if not e['kind'] else 'ended counted',
                   'fwd' if state & 16 else 'rev',
                   {1: 'fast', 2: 'low'}.get(e['source'], e['source']),
                   mode, ' fast_mode' if state & 4 else '',
                   ' slow' if state & 8 else '', '', e['level'],
                   e['segment'], e['step_time'], e['period'],
                   e['error'] / 1000., e['step'] / 1000.,
                   e['drift'] / 1000., e['speed'], e['omega']))
        gcmd.respond_info("%s: %d slip events\n%s" % (
            self.name, len(lines), "\n".join(lines)))

    # Load angle harmonics
    def has_harmonics(self):
        return self.query_harmonics_cmd is not None

    def arm_harmonics(self):
        self.arm_harmonics_cmd.send([self.oid])

    def _handle_harmonics(self, params):
        self.harmonics[(params['dir'], params['harmonic'])] = params

    def read_harmonics(self):
        """{(direction, harmonic): (complex electrical rad, samples)} of
        the load angle since arm_harmonics(); direction 1 forward."""
        self.harmonics = {}
        for d in (0, 1):
            for k in range(1, 9):
                self.query_harmonics_cmd.send([self.oid, d, k])
                for _ in range(20):
                    if (d, k) in self.harmonics:
                        break
                    self.reactor.pause(self.reactor.monotonic() + 0.02)
                else:
                    raise self.printer.command_error(
                        "%s: no load angle harmonic %d from the MCU"
                        % (self.name, k))
        return {key: (complex(p['c'], p['s']) * 1e-7, p['count'])
                for key, p in self.harmonics.items()}

    cmd_MCLIB_SET_BANDWIDTH_help = ("Set the current controller bandwidth in"
                                    " percent of stock (PERCENT, 100 = 1 kHz)")
    def cmd_MCLIB_SET_BANDWIDTH(self, gcmd):
        if self.set_pi_scale_cmd is None:
            raise gcmd.error("%s: firmware has no mclib_set_pi_scale"
                             % (self.name,))
        percent = gcmd.get_int('PERCENT', minval=25, maxval=400)
        self.set_pi_scale_cmd.send([self.oid, percent])
        self.pi_scale = percent
        gcmd.respond_info("%s: current bandwidth %d%% (until restart; set"
                          " current_bandwidth in [mclib %s] to keep it)"
                          % (self.name, percent, self.name))

    cmd_MCLIB_TIMING_help = ("Report control loop duration in CPU cycles"
                             " (max and average) since the last query")
    def cmd_MCLIB_TIMING(self, gcmd):
        if self.query_timing_cmd is None:
            raise gcmd.error("%s: firmware has no mclib_query_timing"
                             % (self.name,))
        self.last_timing = self.last_timing_run = self.last_quality = None
        self.query_timing_cmd.send([self.oid])
        for _ in range(20):
            if self.last_timing is not None and (
                    self.last_timing_run is not None
                    or not self.has_timing_run) and (
                    self.last_quality is not None or not self.has_quality):
                break
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        t = self.last_timing
        if t is None:
            raise gcmd.error("%s: no timing response" % (self.name,))
        average = (float(t['total']) / t['samples']) if t['samples'] else 0.
        msg = ("%s: max %u cycles, avg %.0f cycles over %u control updates"
               % (self.name, t['max'], average, t['samples']))
        if t.get('d_samples'):
            msg += ("\n%s: d-axis PI integral avg %.3f V over %u moving"
                    " updates (feedforward %d%%)"
                    % (self.name, t['d_integral'] / 1000., t['d_samples'],
                       self.feedforward))
        if t.get('hold_samples'):
            msg += ("\n%s: current noise while holding %.1f mA RMS over %u"
                    " updates" % (self.name, t['hold_noise'] / 1000.,
                                  t['hold_samples']))
        r = self.last_timing_run
        if r is not None:
            t = r
            mhz = self.mcu.seconds_to_clock(1.) / 1000000.
            msg += ("\n%s: while moving max %u cycles (%.1f us), avg %u"
                    " cycles over %u updates"
                    % (self.name, t['run_max'], t['run_max'] / mhz,
                       t['run_avg'], t['run_samples']))
            msg += ("\n%s: longest gap between updates %.1f us, %u late"
                    " updates (a period or more lost); longest Klipper timer"
                    " interrupt %.1f us"
                    % (self.name, t['gap_max'] / 100., t['missed'],
                       t['timer_max'] / mhz))
            msg += ("\n%s: longest update without interrupts %.1f us; most"
                    " one update lost to serial %.1f us, Klipper timer %.1f"
                    " us, stall tick %.1f us"
                    % (self.name, t['own_max'] / mhz, t['serial_max'] / mhz,
                       t['timer_steal'] / mhz, t['tick_steal'] / mhz))
        q = self.last_quality
        if q is not None and q['ripple_samples']:
            msg += ("\n%s: current ripple above 100 Hz while moving %.2f mA"
                    " RMS over %u updates"
                    % (self.name, q['ripple'] / 1000., q['ripple_samples']))
        gcmd.respond_info(msg)

    cmd_MCLIB_SET_INTERPOLATION_help = ("Set how the motor angle moves"
                                        " between steps (MODE=linear or"
                                        " quadratic)")
    def cmd_MCLIB_SET_INTERPOLATION(self, gcmd):
        if self.set_interpolation_cmd is None:
            raise gcmd.error("%s: firmware has no mclib_set_interpolation"
                             % (self.name,))
        mode = gcmd.get('MODE').lower()
        # off: no interpolation at all, for comparisons
        modes = dict(INTERPOLATION_MODES, off=0)
        if mode not in modes:
            raise gcmd.error("MODE must be one of %s"
                             % (", ".join(sorted(modes)),))
        if not self.interpolate:
            raise gcmd.error("%s: interpolate is off in [mclib %s]"
                             % (self.name, self.name))
        self.set_interpolation_cmd.send([self.oid, modes[mode]])
        self.interpolation_mode = modes[mode]
        gcmd.respond_info("%s: %s interpolation (until restart; set"
                          " interpolation_mode in [mclib %s] to keep it)"
                          % (self.name, mode, self.name))

    cmd_MCLIB_SET_FEEDFORWARD_help = ("Set the d-axis back-EMF and"
                                      " cross-coupling feedforward"
                                      " (percent, 0 = off)")
    def cmd_MCLIB_SET_FEEDFORWARD(self, gcmd):
        if self.set_feedforward_cmd is None:
            raise gcmd.error("%s: firmware has no mclib_set_feedforward"
                             % (self.name,))
        gain = gcmd.get_int('GAIN', minval=0, maxval=100)
        self.set_feedforward_cmd.send([self.oid, gain])
        self.feedforward = gain
        gcmd.respond_info("%s: feedforward %d%% (until restart; set"
                          " feedforward in [mclib %s] to keep it)"
                          % (self.name, gain, self.name))

    cmd_MCLIB_SET_INJECTION_help = ("Scale the observer's high-frequency"
                                    " injection (percent, 100 = stock)"
                                    " for bench experiments")
    def cmd_MCLIB_SET_INJECTION(self, gcmd):
        if self.set_injection_cmd is None:
            raise gcmd.error("%s: firmware has no mclib_set_injection"
                             % (self.name,))
        scale = gcmd.get_int('SCALE', 100, minval=10, maxval=100)
        self.set_injection_cmd.send([self.oid, scale])
        gcmd.respond_info("%s: injection scale %d%% (until restart)"
                          % (self.name, scale))

    def take_health(self):
        h, self.health = self.health, self._empty_health()
        return h

    def set_slip_mode(self, mode):
        """Slip tracking: 0 fast moves only, 1 also holding still, 2 whenever
        enabled. False on firmware without it."""
        if self.set_slip_cmd is None:
            return False
        self.set_slip_cmd.send([self.oid, mode])
        return True

    def get_stall_threshold(self):
        return self.stall_threshold / 1000.

    cmd_MCLIB_SET_STALL_help = "Set the stall detect threshold of a stepper in mclib"
    def cmd_MCLIB_SET_STALL(self, gcmd):
        threshold = gcmd.get_float('THRESHOLD', above=0.)
        self.stall_threshold = int(threshold * 1000)
        if self.set_stall_cmd is None:
            self.mcu.add_config_cmd("mclib_config_stalldetect oid=%d stallthrs=%u"
                                    % (self.oid, self.stall_threshold))
        else:
            self.set_stall_cmd.send([self.oid, self.stall_threshold])
        gcmd.respond_info("%s stall_threshold=%.3f (until restart; set "
                          "stall_threshold in [mclib %s] to keep it)"
                          % (self.name, threshold, self.name))

    cmd_MCLIB_SET_RESONANCE_DAMP_help = "Sets the stepper resonance damp paramters in mclib"
    def cmd_MCLIB_SET_RESONANCE_DAMP(self, gcmd):
        if self.name == 'extruder':
            raise gcmd.error("Extruder don't support resonance damping")
        tdx = gcmd.get_int('TDX', 1, minval=1, maxval=4) # 1, 2, 4
        if tdx not in [1, 2, 4]:
            raise gcmd.error("Invalid tdx, only 1, 2 or 4 supported!")
        amp = int(gcmd.get_float('AMP', None, minval=0., maxval=MAX_CURRENT)* 1000)
        phase1 = int(gcmd.get_float('PHASE1', None)* 1000)
        phase2 = int(gcmd.get_float('PHASE2', None)* 1000)

        if self.set_resonance_damp_cmd is None:
            # Send setup message via mcu initialization
            self.mcu.add_config_cmd(
                "mclib_set_resonance_damp oid=%d tdx=%d amp=%u phase1=%u phase2=%u"
                % (self.oid, tdx, amp, phase1, phase2))
            return
        self.set_resonance_damp_cmd.send([self.oid, tdx, amp, phase1, phase2])

    cmd_MCLIB_APPLY_CALIBRATION_help = (
        "Restore saved MCLib resonance damping for this stepper")
    def cmd_MCLIB_APPLY_CALIBRATION(self, gcmd):
        # Values are read from [mclib stepper_*] at startup, including values
        # written by SAVE_CONFIG after VFA calibration.
        for tdx, (amp, phase1, phase2) in sorted(
                self._stock_damping().items()):
            if self.set_resonance_damp_cmd is None:
                self.mcu.add_config_cmd(
                    "mclib_set_resonance_damp oid=%d tdx=%d amp=%u phase1=%u phase2=%u"
                    % (self.oid, tdx, amp, phase1, phase2))
            else:
                self.set_resonance_damp_cmd.send(
                    [self.oid, tdx, amp, phase1, phase2])
        gcmd.respond_info("Restored MCLib calibration for %s" % self.name)

    cmd_MCLIB_SET_RIPPLE_help = ("Set or show torque ripple correction "
                                 "(HARMONIC, DIR=1 forward or 0 reverse, "
                                 "AMP in A, PHASE in rad; SAVE=1 to save the "
                                 "tables)")
    def cmd_MCLIB_SET_RIPPLE(self, gcmd):
        if self.set_ripple_cmd is None:
            raise gcmd.error("Ripple correction needs newer mainboard "
                             "firmware")
        harmonic = gcmd.get_int('HARMONIC', None, minval=1,
                                maxval=RIPPLE_HARMONICS)
        if gcmd.get_int('SAVE', 0):
            # Store the live tables for SAVE_CONFIG
            configfile = self.printer.lookup_object('configfile')
            for k in range(1, RIPPLE_HARMONICS + 1):
                val = self.ripple_config_value(k)
                if val is not None:
                    configfile.set('mclib ' + self.name, 'ripple_%d' % (k,),
                                   val)
            gcmd.respond_info("Run SAVE_CONFIG to keep the %s ripple "
                              "correction" % (self.name,))
            return
        if harmonic is None:
            lines = ["%s ripple_%d: %s" % (self.name, k,
                                           self.ripple_config_value(k))
                     for k in sorted(self.ripple)]
            gcmd.respond_info("\n".join(lines) or
                              "%s: no ripple correction" % (self.name,))
            return
        direction = gcmd.get_int('DIR', minval=0, maxval=1)
        amp = gcmd.get_float('AMP', minval=0., maxval=RIPPLE_MAX_AMP)
        phase = gcmd.get_float('PHASE', 0.)
        self.set_ripple(harmonic, direction, amp, phase)

def load_config_prefix(config):
    return MCLIB(config)
