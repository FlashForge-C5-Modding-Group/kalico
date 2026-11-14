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
# A single-point correction is held at every speed below this frequency
RIPPLE_FLAT_HZ = 10000.
steps = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7, 256: 8}
stepper_id = {'stepper_x':0, 'stepper_y':1, 'stepper_z':2, 'extruder':3}

class MCLIB:
    def __init__(self, config):
        self.printer = config.get_printer()
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
        self.stall_threshold = int(config.getfloat('stall_threshold', self.bus_voltage, above=0.)* 1000)
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
            self.ripple[k] = [tuple(vals[i:i + 5])
                              for i in range(0, len(vals), 5)]
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

        # Motion health (c5_motion_guard): totals since the last take_health()
        self.query_status_cmd = None
        self.set_stall_cmd = None
        self.set_slip_cmd = None
        self.health = self._empty_health()
        self.last_status = None
        gcode.register_mux_command("MCLIB_SET_STALL", "STEPPER", self.name,
                                   self.cmd_MCLIB_SET_STALL,
                                   desc=self.cmd_MCLIB_SET_STALL_help)

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
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.oid = self.mcu.create_oid()

        self.mcu.add_config_cmd("config_mclib oid=%d stepper=%s rs=%u ls=%u km=%u" % (
            self.oid, self.stepper_name, self.motor_rs, self.motor_ls, self.motor_km))

        self.mcu.add_config_cmd("mclib_config_microstep oid=%d interpolate=%d mstep=%u" % (
            self.oid, self.interpolate, self.microstep))
            
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

        if self.name != 'extruder':
            for tdx, (amp, phase1, phase2) in sorted(
                    self._stock_damping().items()):
                self.mcu.add_config_cmd(
                    "mclib_set_resonance_damp oid=%d tdx=%d amp=%u"
                    " phase1=%u phase2=%u"
                    % (self.oid, tdx, amp, phase1, phase2))

        if self.name != 'extruder':
            self.set_resonance_damp_cmd = self.mcu.lookup_command(
                "mclib_set_resonance_damp oid=%c tdx=%c amp=%u phase1=%u phase2=%u")
            # Older mainboard firmware has no ripple correction
            self.set_ripple_cmd = self.mcu.try_lookup_command(
                "mclib_set_ripple oid=%c harmonic=%c dir=%c index=%c count=%c"
                " freq=%u amp=%i phase=%i")
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
        # A harmonic with a ripple table is corrected by that instead (the
        # ripple calibration runs without stock damping on it)
        ripple = self.ripple if self.set_ripple_cmd is not None else {}
        return {tdx: ((0, 0, 0) if tdx in ripple else vals)
                for tdx, vals in self.calibrated_damping.items()}

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
                                 "AMP in A, PHASE in rad)")
    def cmd_MCLIB_SET_RIPPLE(self, gcmd):
        if self.set_ripple_cmd is None:
            raise gcmd.error("Ripple correction needs newer mainboard "
                             "firmware")
        harmonic = gcmd.get_int('HARMONIC', None, minval=1,
                                maxval=RIPPLE_HARMONICS)
        if harmonic is None:
            lines = ["%s ripple_%d: %s" % (self.name, k,
                                           self.ripple_config_value(k))
                     for k in sorted(self.ripple)]
            gcmd.respond_info("\n".join(lines) or
                              "%s: no ripple correction" % (self.name,))
            return
        direction = gcmd.get_int('DIR', minval=0, maxval=1)
        amp = gcmd.get_float('AMP', minval=-1., maxval=1.)
        phase = gcmd.get_float('PHASE', 0.)
        self.set_ripple(harmonic, direction, amp, phase)

def load_config_prefix(config):
    return MCLIB(config)
