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

        if self.name != 'extruder':
            self.mcu.add_config_cmd("mclib_set_resonance_damp oid=%d tdx=1 amp=%u phase1=%u phase2=%u"% (
                self.oid, self.td1_amp, self.td1_phase1, self.td1_phase2))
            self.mcu.add_config_cmd("mclib_set_resonance_damp oid=%d tdx=2 amp=%u phase1=%u phase2=%u"% (
                self.oid, self.td2_amp, self.td2_phase1, self.td2_phase2))
            self.mcu.add_config_cmd("mclib_set_resonance_damp oid=%d tdx=4 amp=%u phase1=%u phase2=%u"% (
                self.oid, self.td4_amp, self.td4_phase1, self.td4_phase2))

        if self.name != 'extruder':
            self.set_resonance_damp_cmd = self.mcu.lookup_command(
                "mclib_set_resonance_damp oid=%c tdx=%c amp=%u phase1=%u phase2=%u")

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
                self.calibrated_damping.items()):
            if self.set_resonance_damp_cmd is None:
                self.mcu.add_config_cmd(
                    "mclib_set_resonance_damp oid=%d tdx=%d amp=%u phase1=%u phase2=%u"
                    % (self.oid, tdx, amp, phase1, phase2))
            else:
                self.set_resonance_damp_cmd.send(
                    [self.oid, tdx, amp, phase1, phase2])
        gcmd.respond_info("Restored MCLib calibration for %s" % self.name)

def load_config_prefix(config):
    return MCLIB(config)
