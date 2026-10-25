# Creator 5 toolchanger, levelboard calibration, and motion interlocks.
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import logging
import math
import os
import shutil
import tempfile
from contextlib import contextmanager
from datetime import datetime


# Config::initExtruderConfig() in the factory firmwareExe binary.  These are
# nominal positions; saved measurements in the printer's config take priority.
FACTORY_DOCKS = ((298.219, 56.472), (298.605, 107.100),
                 (298.200, 156.920), (298.238, 207.140))
FACTORY_MEASUREMENTS = ((15.800, 215.280, 1.460),
                        (15.820, 215.280, 1.467),
                        (15.550, 216.085, 1.406),
                        (15.864, 215.340, 1.437))
# CommMgr::cmdXYCalibrationManualFrontHome() in firmwareExe.i64 uses this
# temporary coordinate after the operator aligns the carriage by hand.
HOLDER_REFERENCE_X = 295.
HOLDER_REFERENCE_Y_BASE = 80.
HOLDER_REFERENCE_Y_PITCH = 50.


class Creator5MiscSwitch:
    def __init__(self, printer, gcode, name, enabled, on_change=None):
        self.printer = printer
        self.gcode = gcode
        self.name = name
        self.saved_key = 'c5_' + name
        self.enabled = bool(enabled)
        self.on_change = on_change
        printer.add_object('output_pin ' + name, self)
        printer.register_event_handler('klippy:ready', self._handle_ready)
        gcode.register_mux_command('SET_PIN', 'PIN',
                                   name, self.cmd_set)

    def _handle_ready(self):
        saved = self.printer.lookup_object('save_variables', None)
        if saved is not None:
            value = saved.allVariables.get(self.saved_key)
            if value in (0, 1):
                self.enabled = bool(value)
        if self.on_change is not None:
            self.on_change()

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled == self.enabled:
            return
        if self.printer.lookup_object('save_variables', None) is not None:
            self.gcode.run_script_from_command(
                'SAVE_VARIABLE VARIABLE=%s VALUE=%d'
                % (self.saved_key, int(enabled)))
        self.enabled = enabled
        if self.on_change is not None:
            self.on_change()

    def get_status(self, eventtime):
        return {'value': float(self.enabled)}

    def cmd_set(self, gcmd):
        self.set_enabled(gcmd.get_float('VALUE', 0., minval=0., maxval=1.) > 0.)
        gcmd.respond_info('Creator 5 %s %s' % (
            self.name, 'enabled' if self.enabled else 'disabled'))


class Creator5FlowSwitch(Creator5MiscSwitch):
    def __init__(self, printer, gcode, enabled):
        super().__init__(printer, gcode, 'flow_calibration', enabled)


class Creator5Toolchanger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.configfile = self.printer.lookup_object('configfile')
        self.section = config.get_name()
        self.docks = [[config.getfloat('t%d_dock_x' % i, value[0]),
                       config.getfloat('t%d_dock_y' % i, value[1])]
                      for i, value in enumerate(FACTORY_DOCKS)]
        self.measurements = [[config.getfloat('t%d_measure_x' % i, value[0]),
                              config.getfloat('t%d_measure_y' % i, value[1]),
                              config.getfloat('t%d_measure_z' % i, value[2])]
                             for i, value in enumerate(FACTORY_MEASUREMENTS)]
        self.station_x = config.getfloat('station_x', 28.815)
        self.station_y = config.getfloat('station_y', 215.097)
        self.station_z = config.getfloat('station_z', -1.078)
        self.extruder_json_path = os.path.expanduser(config.get(
            'extruder_json_path', '/usr/data/firmwareRes/config/extruder.json'))
        self._load_extruder_json()
        self.zoffset_json_path = os.path.expanduser(config.get(
            'zoffset_json_path', '/usr/data/firmwareRes/config/zoffset.json'))
        self.test_json_path = os.path.expanduser(config.get(
            'test_json_path', '/usr/data/firmwareRes/config/test.json'))
        self.z_adjustments = [0.] * 4
        self._load_zoffset_json()
        config_dir = os.path.dirname(os.path.abspath(
            self.printer.get_start_args().get('config_file', 'printer.cfg')))
        self.toolchange_count_path = os.path.expanduser(config.get(
            'toolchange_count_path', os.path.join(config_dir,
                                                  'toolchanges.txt')))
        self.toolchange_counts = self._load_toolchange_counts()
        self.print_z_baseline = [None] * 4
        self._button_objects = {}
        # UI status may be polled many times per second on a small MIPS host.
        # Motion interlocks always bypass this one-second status cache.
        self._ui_sensor_cache = None
        self.tool_fixture_shift_x = config.getfloat('tool_fixture_shift_x',
                                                    -12.5)
        self.scan_span = config.getfloat('scan_span', 7., above=0.)
        # Each edge has an independent reach; all default to the stock 7 mm.
        self.scan_spans = {
            ('X', 1): config.getfloat('scan_x_plus', self.scan_span,
                                     above=0., maxval=10.),
            ('Y', 1): config.getfloat('scan_y_plus', self.scan_span,
                                     above=0., maxval=10.),
            ('X', -1): config.getfloat('scan_x_minus', self.scan_span,
                                      above=0., maxval=10.),
            ('Y', -1): config.getfloat('scan_y_minus', self.scan_span,
                                      above=0., maxval=10.),
        }
        # Consume the retired setting so older printer configs still load;
        # it no longer changes negative-Y reach.
        config.getfloat('negative_y_scan_span', None)
        self.scan_height = config.getfloat('scan_height', None)
        self.reference_scan_height = config.getfloat('reference_scan_height',
                                                     None)
        self.z_probe_target = config.getfloat('z_probe_target', -8.)
        # CommMgr::checkPlatformRemove() probes these two locations with the
        # carriage's ordinary [probe] before touching the levelboard target.
        self.plate_check_x = config.getfloat('plate_check_x', 43.)
        self.plate_check_y = config.getfloat('plate_check_y', 226.)
        self.plate_check_min_delta = config.getfloat(
            'plate_check_min_delta', .8, above=0.)
        self.plate_check_travel_speed = config.getfloat(
            'plate_check_travel_speed', 100., above=0.)
        # Result of the last C5_CHECK_BUILD_PLATE (None until one runs).
        self.build_plate_detected = None
        self._stock_pin_approach_z = None
        self._offset_batch_active = False
        self.safe_z = config.getfloat('safe_z', 10., above=0.)
        self.offset_calibration_lift = config.getfloat(
            'offset_calibration_lift', 3., above=0.)
        self.offset_calibration_lift_speed = config.getfloat(
            'offset_calibration_lift_speed', 10., above=0.)
        self.max_mount_correction = config.getfloat(
            'max_mount_correction', 2., above=0.)
        self.max_offset_correction = config.getfloat(
            'max_offset_correction', 3., above=0.)
        self.pickup_accel = config.getfloat('pickup_accel', 8000., above=0.)
        # Toolchange moves are short: with the printer's minimum_cruise_ratio
        # they never reach their speed, so a lower ratio lets them use the
        # full pickup_accel. Unset keeps the printer's own.
        self.pickup_minimum_cruise_ratio = config.getfloat(
            'pickup_minimum_cruise_ratio', None, minval=0., below=1.)
        self.clear_travel_speed = config.getfloat(
            'clear_travel_speed', 80., above=0.)
        self.clearance_z_speed = config.getfloat(
            'clearance_z_speed', 20., above=0.)
        self.lower_bed_speed = config.getfloat(
            'lower_bed_speed', 10., above=0.)
        # Where C5_LOWER_BED stops: 5 mm above the bottom of Z travel (256).
        self.lower_bed_z = config.getfloat('lower_bed_z', 254., above=0.,
                                           maxval=256.)
        self.dock_approach_speed = config.getfloat(
            'dock_approach_speed', 20., above=0.)
        self.dock_predock_offset = config.getfloat(
            'dock_predock_offset', 20., above=10.)
        self.pickup_predock_speed = config.getfloat(
            'pickup_predock_speed', 40., above=0.)
        self.pickup_latch_speed = config.getfloat(
            'pickup_latch_speed', 20., above=0.)
        self.pullback_speed = config.getfloat(
            'pullback_speed', 80., above=0.)
        self.pullback_slow_distance = config.getfloat(
            'pullback_slow_distance', 20., above=0.)
        self.departure_speed = config.getfloat(
            'departure_speed', 80., above=0.)
        self.pickup_departure_speed = config.getfloat(
            'pickup_departure_speed', self.departure_speed, above=0.)
        self.post_select_accel = config.getfloat(
            'post_select_accel', None, above=0.)
        self.approach_x = config.getfloat('approach_x', 250.)
        self.pre_dock_x = config.getfloat('pre_dock_x', 280.)
        self.pullback = config.getfloat('pullback', 20., above=0.)
        self.pickup_latch_wait_ms = config.getint('pickup_latch_wait_ms',
                                                  1800, minval=0)
        self.release_latch_wait_ms = config.getint('release_latch_wait_ms',
                                                   800, minval=0)
        self.sensor_settle_ms = config.getint('sensor_settle_ms', 150,
                                              minval=0)
        self.toolchange_sensor_settle_ms = config.getint(
            'toolchange_sensor_settle_ms', 150, minval=0)
        self.position_calibration_timeout = config.getfloat(
            'position_calibration_timeout', 30., above=0.)
        self.position_calibration_release_wait_ms = config.getint(
            'position_calibration_release_wait_ms', 5000, minval=0)
        self.position_calibration_accel = config.getfloat(
            'position_calibration_accel', 1000., above=0.)
        self.prep_approach_x = config.getfloat('prep_approach_x', 250.)
        self.prep_approach_y = config.getfloat('prep_approach_y', 250.)
        self.purge_x = config.getfloat('purge_x', 276.5)
        self.purge_y = config.getfloat('purge_y', 250., maxval=260.)
        # The bucket sweep must stop short of the usable X280 edge. This
        # limit applies only to purge/flow, not to the docks beyond X280.
        self.purge_max_x = config.getfloat('purge_max_x', 279., maxval=279.)
        self.cooldown_x = config.getfloat('cooldown_x', 266.5)
        self.cooldown_y = config.getfloat('cooldown_y', 13.8)
        self.cooldown_wipe_z = config.getfloat('cooldown_wipe_z', 1.8,
                                               minval=1., maxval=5.)
        self.cooldown_lift_z = config.getfloat('cooldown_lift_z', 10.,
                                               above=self.cooldown_wipe_z)
        self.cooldown_fan_speed = config.getfloat('cooldown_fan_speed', .6,
                                                  minval=0., maxval=1.)
        self.purge_z = config.getfloat('purge_z', 8.0, minval=5.)
        self.purge_length = config.getfloat('purge_length', 12., minval=0.)
        self.purge_speed = config.getfloat('purge_speed', 3., above=0.)
        self.wheel_mm_per_pulse = config.getfloat('wheel_mm_per_pulse', None,
                                                  above=0.)
        self.rotation_cal_length = config.getfloat(
            'rotation_calibration_length', 100., above=10., maxval=300.)
        self.rotation_cal_speed = config.getfloat(
            'rotation_calibration_speed', 3., above=0., maxval=10.)
        self.rotation_cal_temp = config.getfloat(
            'rotation_calibration_temp', 245., minval=170., maxval=320.)
        self.rotation_measure_timeout = config.getfloat(
            'rotation_calibration_timeout', 30., above=0.)
        self.rotation_pending = None
        # Keep the nozzle high over the bucket during the flow test.
        self.flow_test_z = config.getfloat('flow_test_z', 8.,
                                           minval=5., maxval=15.)
        # A shorter stroke needs a slower velocity/acceleration profile so
        # the pressure transient lasts longer than PA smoothing.
        self.flow_sweep_x = config.getfloat('flow_sweep_x', 4.5,
                                            minval=4., maxval=12.)
        self._validate_purge_envelope(config.error)
        self.flow_test_accel = config.getfloat('flow_test_accel', 5000.,
                                               above=0., maxval=5000.)
        self.flow_slow_speed = config.getfloat('flow_slow_speed', 18.,
                                               above=0.)
        self.flow_fast_speed = config.getfloat('flow_fast_speed', 183.,
                                               above=self.flow_slow_speed)
        self.flow_verdict_settle_ms = config.getint(
            'flow_verdict_settle_ms', 50, minval=0, maxval=500)
        self.bed_soak_minutes = config.getfloat(
            'bed_soak_minutes', 10., minval=0., maxval=180.)
        # No soak below this bed target (C).
        self.bed_soak_min_temp = config.getfloat(
            'bed_soak_min_temp', 45., minval=0.)
        self.door_buttons = config.getlist('door_buttons', [])
        self.dock_buttons = config.getlist('dock_buttons',
            ['extruder_pos%d' % (i + 1) for i in range(4)])
        self.grab_buttons = config.getlist('grab_buttons',
            ['extruder_grab%d' % (i + 1) for i in range(4)])
        if len(self.dock_buttons) != 4 or len(self.grab_buttons) != 4:
            raise config.error('Creator 5 requires four dock and grab buttons')
        self.active = None
        self.busy = False
        self.flow_switch = Creator5FlowSwitch(
            self.printer, self.gcode,
            config.getboolean('flow_calibration_default', True))
        self.purge_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'purge',
            config.getboolean('purge_default', True))
        self.bed_leveling_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'bed_leveling',
            config.getboolean('bed_leveling_default', True))
        self.bed_soak_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'bed_soak',
            config.getboolean('bed_soak_default', False))
        self.clog_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'clog_detection',
            config.getboolean('clog_detection_default', False),
            self._sync_clog_sensors)
        self.lower_bed_on_end_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'lower_bed_on_end',
            config.getboolean('lower_bed_on_end_default', False))
        self.quiet_mode_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'quiet_mode',
            config.getboolean('quiet_mode_default', False))
        # Whether M191 (chamber heat-up) blocks until the chamber actually
        # reaches target, or just requests the target and lets the print
        # start heating in the background (M141-like). See M191 in
        # printer.macro.cfg.
        self.chamber_wait_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'chamber_wait',
            config.getboolean('chamber_wait_default', True))
        # Notify when a door is open during a print (see _OC_DOOR in
        # printer.misc.cfg).
        self.door_notify_switch = Creator5MiscSwitch(
            self.printer, self.gcode, 'door_notify',
            config.getboolean('door_notify_default', False))
        for name, handler in (
            ('C5_TOOL_STATUS', self.cmd_status),
            ('C5_TOOL_SELECT', self.cmd_select),
            ('C5_TOOL_DOCK', self.cmd_dock),
            ('C5_WHEEL_RUNOUT', self.cmd_wheel_runout),
            ('C5_WHEEL_CLOG', self.cmd_wheel_runout),
            ('C5_RUNOUT_SWITCH', self.cmd_runout_switch),
            ('C5_TOOL_PURGE', self.cmd_purge),
            ('C5_MOVE_TO_PURGE_BUCKET', self.cmd_move_to_purge_bucket),
            ('C5_CHECK_BUILD_PLATE', self.cmd_check_build_plate),
            ('C5_OFFSET_WIZARD', self.cmd_offset_wizard),
            ('C5_BED_SOAK', self.cmd_bed_soak),
            ('C5_LOWER_BED', self.cmd_lower_bed),
            ('C5_FLOW_STROKES', self.cmd_flow_strokes),
            ('C5_EXTRUDER_ROTATION_CALIBRATE',
             self.cmd_extruder_rotation_calibrate),
            ('C5_MISC_FLOW_ON', self.cmd_flow_on),
            ('C5_MISC_FLOW_OFF', self.cmd_flow_off),
            ('C5_COOL_FOR_DOCK', self.cmd_cool_for_dock),
            ('C5_CHECK_LOAD_TOOL', self.cmd_check_load_tool),
            ('C5_CALIBRATE_ALL_OFFSETS', self.cmd_calibrate_all_offsets),
            ('C5_HOME_FOR_PRINT', self.cmd_home_for_print),
            ('C5_RECOVER_ATTACHED', self.cmd_recover_attached),
            ('C5_TOOL_OFFSET_CALIBRATE', self.cmd_offset_calibrate),
            ('C5_LEVELBOARD_REFERENCE_CALIBRATE',
             self.cmd_reference_calibrate),
            ('C5_SAVE_OFFSETS_JSON', self.cmd_save_offsets_json),
            ('C5_SAVE_TOUCHSCREEN_Z_OFFSET',
             self.cmd_save_touchscreen_z_offset),
            ('C5_AUTO_NOZZLE_Z', self.cmd_auto_nozzle_z),
            ('C5_VERIFY_NOZZLE_Z', self.cmd_verify_nozzle_z),
            ('C5_CALIBRATE_ATTACHED', self.cmd_calibrate_attached),
            ('C5_MOUNT_COORDS', self.cmd_mount_coords),
            ('C5_MOUNT_CORRECT', self.cmd_mount_correct),
            ('C5_MOUNT_PROBE', self.cmd_mount_probe),
            ('EXTRUDER_POSITION_CALIBRATE',
             self.cmd_extruder_position_calibrate),
            ('C5_EXTRUDER_POSITION_CALIBRATE',
             self.cmd_extruder_position_calibrate)):
            self.gcode.register_command(name, handler)
        # Register after safe_z_home has installed its G28 handler, so partial
        # G28 commands can still delegate to Klipper's normal homing path.
        self.prev_G28 = None
        self._g28_passthrough = False
        self.prev_set_pressure_advance = None
        self._tool_pa_memory = {}
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

    def _handle_ready(self):
        # Match the C5 multi-MCU homing window without changing Kalico's
        # default for printers that do not load this toolchanger.
        danger = self.printer.lookup_object('danger_options', None)
        timeout = getattr(danger, 'multi_mcu_trsync_timeout', None)
        if isinstance(timeout, (int, float)):
            danger.multi_mcu_trsync_timeout = max(timeout, .1)
        self.prev_G28 = self.gcode.register_command('G28', None)
        self.gcode.register_command('G28', self.cmd_G28)
        self.prev_set_pressure_advance = self.gcode.register_command(
            'SET_PRESSURE_ADVANCE', None)
        if self.prev_set_pressure_advance is not None:
            self.gcode.register_command(
                'SET_PRESSURE_ADVANCE', self.cmd_set_pressure_advance)
        # Seed memory with each extruder's static config value so the first
        # pickup after startup (before anything has explicitly set PA) has
        # a sensible default rather than falling back to 0.
        for tool in range(4):
            name = 'extruder' if tool == 0 else 'extruder%d' % tool
            extruder = self.printer.lookup_object(name, None)
            stepper = getattr(extruder, 'extruder_stepper', None)
            if stepper is not None and stepper.config_pa > 0.:
                self._tool_pa_memory[name] = (
                    stepper.config_pa, stepper.config_smooth_time)

    def cmd_flow_on(self, gcmd):
        self.flow_switch.set_enabled(True)
        gcmd.respond_info('Creator 5 flow calibration enabled')

    def cmd_flow_off(self, gcmd):
        self.flow_switch.set_enabled(False)
        gcmd.respond_info('Creator 5 flow calibration disabled')

    def cmd_bed_soak(self, gcmd):
        """Wait after bed heat-up without queuing a long MCU dwell."""
        if not self.bed_soak_switch.enabled:
            return
        minutes = gcmd.get_float('MINUTES', self.bed_soak_minutes,
                                 minval=0., maxval=180.)
        if not minutes:
            return
        # A cool bed (PLA and the like) barely warps; soaking it only costs time.
        heaters = self.printer.lookup_object('heaters')
        bed_target = heaters.lookup_heater('heater_bed').get_status(
            self.printer.get_reactor().monotonic())['target']
        if bed_target < self.bed_soak_min_temp:
            gcmd.respond_info('Creator 5 bed soak skipped: bed target %.0f C '
                              'is below %.0f C' % (bed_target,
                                                   self.bed_soak_min_temp))
            return
        self.printer.lookup_object('toolhead').wait_moves()
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + minutes * 60.
        gcmd.respond_info('Creator 5 bed soak: %.1f minutes before mesh'
                          % (minutes,))
        while reactor.monotonic() < deadline:
            reactor.pause(min(deadline, reactor.monotonic() + 1.))
        gcmd.respond_info('Creator 5 bed soak complete')

    def cmd_lower_bed(self, gcmd):
        """Lower the bed to lower_bed_z, without G-code or nozzle offsets."""
        toolhead = self.printer.lookup_object('toolhead')
        status = toolhead.get_status(self.printer.get_reactor().monotonic())
        if 'z' not in status.get('homed_axes', ''):
            raise gcmd.error('Home Z before lowering the bed')
        z_max = 256
        current_z = toolhead.get_position()[2]
        if not math.isfinite(z_max) or not math.isfinite(current_z):
            raise gcmd.error('Cannot determine the physical Z travel limit')
        if current_z > z_max + .001:
            raise gcmd.error('Current Z position exceeds the travel limit')
        # Never raises the bed: a bed already lower than that stays put.
        if current_z < self.lower_bed_z - .001:
            self._move(z=self.lower_bed_z, feed=self.lower_bed_speed * 60.)

    def _home_off_docked_head(self, gcmd):
        # Every G28 first releases the carrier latch. A carrier resting
        # unlatched on a docked head reads docked and grabbed at once, which
        # blocks all motion, so it is homed in X (away from the docks, the
        # same way it leaves a dock) before the sensors are checked. Not
        # with a head reading attached: releasing would drop it, and the
        # normal path docks it instead.
        dock = [self._button(name, gcmd) for name in self.dock_buttons]
        grab = [self._button(name, gcmd) for name in self.grab_buttons]
        if any(grab[i] and not dock[i] for i in range(4)):
            return
        self._run('M400')
        self._run('MOTOR_RELEASE')
        self._pause_ms(self.release_latch_wait_ms)
        resting = [i for i in range(4) if dock[i] and grab[i]]
        if not resting:
            return
        gcmd.respond_info('Carrier is resting on docked T%s; homing X to move '
                          'it off' % ',T'.join(str(i) for i in resting))
        old_passthrough = self._g28_passthrough
        self._g28_passthrough = True
        try:
            self._run('G28 X')
        finally:
            self._g28_passthrough = old_passthrough
        dock = [self._button(name, gcmd) for name in self.dock_buttons]
        grab = [self._button(name, gcmd) for name in self.grab_buttons]
        still = [i for i in range(4) if dock[i] and grab[i]]
        if still:
            raise gcmd.error('T%s still reads docked and grabbed after the '
                             'carrier moved off; check its sensors'
                             % ',T'.join(str(i) for i in still))

    def cmd_G28(self, gcmd):
        if self._g28_passthrough:
            self.prev_G28(gcmd)
            return
        self._home_off_docked_head(gcmd)
        requested = [gcmd.get(axis, None) is not None for axis in 'XYZ']
        if not any(requested) or all(requested):
            self.cmd_home_for_print(gcmd)
            return
        if requested[2] and self.attached_tool_from_pins() is not None:
            raise gcmd.error('Dock the attached tool before Z homing; use G28')
        self.prev_G28(gcmd)

    def _read_stock_json(self, path):
        with open(path, 'r') as source:
            raw = source.read()
        data, end = json.JSONDecoder().raw_decode(raw.lstrip())
        if not isinstance(data, dict):
            raise ValueError('%s must contain a JSON object' % path)
        suffix = raw.lstrip()[end:].strip()
        if suffix and not (suffix.startswith('/*') and suffix.endswith('*/')):
            raise ValueError('%s has unexpected trailing content' % path)
        return data, suffix

    def _load_extruder_json(self):
        if not os.path.isfile(self.extruder_json_path):
            return
        try:
            data, suffix = self._read_stock_json(self.extruder_json_path)
            for tool in range(4):
                for axis, index in (('x', 0), ('y', 1), ('z', 2)):
                    key = 't%d_offset_%s' % (tool, axis)
                    if key in data:
                        value = float(data[key])
                        if not math.isfinite(value):
                            raise ValueError('%s is not finite' % key)
                        self.measurements[tool][index] = value
                xkey = 'x_check_pos' + (str(tool) if tool else '')
                ykey = 'y_check_pos' + (str(tool) if tool else '')
                for key, index in ((xkey, 0), (ykey, 1)):
                    if key in data:
                        value = float(data[key])
                        if not math.isfinite(value):
                            raise ValueError('%s is not finite' % key)
                        self.docks[tool][index] = value
            for key, attr in (('x_station_pos', 'station_x'),
                              ('y_station_pos', 'station_y'),
                              ('z_station_pos', 'station_z')):
                if key in data:
                    value = float(data[key])
                    if not math.isfinite(value):
                        raise ValueError('%s is not finite' % key)
                    setattr(self, attr, value)
        except (OSError, ValueError, TypeError) as exc:
            raise self.printer.config_error('Invalid %s: %s' % (
                self.extruder_json_path, exc))

    def _load_zoffset_json(self):
        adjustments = [0.] * 4
        if not os.path.isfile(self.zoffset_json_path):
            self.z_adjustments = adjustments
            return
        try:
            data, suffix = self._read_stock_json(self.zoffset_json_path)
            for tool in range(4):
                key = 'z_offset_t%d' % (tool + 1)
                value = float(data.get(key, 0.))
                if not math.isfinite(value):
                    raise ValueError('%s is not finite' % key)
                adjustments[tool] = value
        except (OSError, ValueError, TypeError) as exc:
            raise self.printer.config_error('Invalid %s: %s' % (
                self.zoffset_json_path, exc))
        self.z_adjustments = adjustments

    def cmd_save_touchscreen_z_offset(self, gcmd):
        # The touchscreen's SET_GCODE_OFFSET value is a total G-code Z
        # offset. Store only the manual part; the levelboard tool difference
        # is already included when a tool is selected.
        if self.busy:
            raise gcmd.error('Cannot save Z offset during a toolchange')
        dock, grab, tool = self._sensor_state(gcmd)
        self._validate_sensor_state(gcmd, dock, tool)
        if tool is None:
            raise gcmd.error('Attach a tool before saving its Z offset')
        origin = self.printer.lookup_object('gcode_move').get_status()[
            'homing_origin'][2]
        baseline = self.print_z_baseline[tool]
        if baseline is None:
            baseline = self.measurements[tool][2] - self.station_z
        adjustment = origin - baseline
        if not math.isfinite(adjustment) or abs(adjustment) > 5.:
            raise gcmd.error('Touchscreen Z adjustment is outside +/-5 mm')
        if not os.path.isfile(self.zoffset_json_path):
            raise gcmd.error('Missing stock %s; refusing to create it'
                             % self.zoffset_json_path)
        temp_path = None
        try:
            data, suffix = self._read_stock_json(self.zoffset_json_path)
            data['z_offset_t%d' % (tool + 1)] = round(adjustment, 4)
            directory = os.path.dirname(self.zoffset_json_path)
            backup = self.zoffset_json_path + '.bak-' + (
                datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            shutil.copy2(self.zoffset_json_path, backup)
            fd, temp_path = tempfile.mkstemp(prefix='.zoffset-',
                                             suffix='.tmp', dir=directory)
            os.chmod(temp_path, os.stat(self.zoffset_json_path).st_mode)
            with os.fdopen(fd, 'w') as target:
                json.dump(data, target, indent=3, sort_keys=True)
                target.write('\n')
                if suffix:
                    target.write(suffix + '\n')
                target.flush()
                os.fsync(target.fileno())
            os.replace(temp_path, self.zoffset_json_path)
            temp_path = None
        except (OSError, ValueError, TypeError) as exc:
            raise gcmd.error('Could not save touchscreen Z offset: %s' % exc)
        finally:
            if temp_path is not None and os.path.exists(temp_path):
                os.unlink(temp_path)
        self.z_adjustments[tool] = adjustment
        gcmd.respond_info('Saved T%d touchscreen Z adjustment %.4f mm'
                          % (tool, adjustment))

    def _load_toolchange_counts(self):
        counts = [0] * 4
        try:
            with open(self.toolchange_count_path) as source:
                for line in source:
                    key, _, value = line.partition(':')
                    key = key.strip().upper()
                    if key in ('T0', 'T1', 'T2', 'T3'):
                        counts[int(key[1])] = max(0, int(value.strip()))
        except (OSError, ValueError):
            pass
        return counts

    def _count_toolchange(self, tool):
        # A failed write must never stop a print.
        self.toolchange_counts[tool] += 1
        counts = self.toolchange_counts
        temp_path = None
        try:
            fd, temp_path = tempfile.mkstemp(
                prefix='.toolchanges-', suffix='.tmp',
                dir=os.path.dirname(self.toolchange_count_path) or '.')
            with os.fdopen(fd, 'w') as target:
                target.write('Total: %d\n' % sum(counts))
                for index, count in enumerate(counts):
                    target.write('T%d: %d\n' % (index, count))
            os.chmod(temp_path, 0o644)
            os.replace(temp_path, self.toolchange_count_path)
            temp_path = None
        except OSError:
            logging.exception('Could not save %s', self.toolchange_count_path)
        finally:
            if temp_path is not None and os.path.exists(temp_path):
                os.unlink(temp_path)

    def cmd_auto_nozzle_z(self, gcmd):
        tool = gcmd.get_int('T', minval=0, maxval=3)
        hotend = gcmd.get_float('HOTEND', 120., minval=0.)
        bed = gcmd.get_float('BED', 0., minval=0.)
        first_layer = gcmd.get_float('FIRST_LAYER_HEIGHT', 0.2, above=0.)
        dock, grab, attached = self._preflight(gcmd)
        if attached != tool:
            raise gcmd.error('Attach T%d before automatic nozzle Z' % tool)
        if not os.path.isfile(self.extruder_json_path):
            raise gcmd.error('Missing calibrated %s' % self.extruder_json_path)
        if not os.path.isfile(self.zoffset_json_path):
            raise gcmd.error('Missing touchscreen %s' % self.zoffset_json_path)
        if not os.path.isfile(self.test_json_path):
            raise gcmd.error('Missing factory test config %s' % self.test_json_path)
        try:
            data, suffix = self._read_stock_json(self.extruder_json_path)
            test_data, _ = self._read_stock_json(self.test_json_path)
            if test_data.get('generalFirmware', False):
                raise ValueError('factory generalFirmware mode has no nozzle '
                                 'offset application')
            temp_coefficient = float(test_data['tempOffset'])
            tool_z = float(data['t%d_offset_z' % tool])
            station_z = float(data['z_station_pos'])
            if not all(math.isfinite(v) for v in (
                    tool_z, station_z, temp_coefficient, hotend, bed,
                    first_layer)):
                raise ValueError('non-finite Z calibration')
            self._load_zoffset_json()
        except (OSError, ValueError, TypeError, KeyError) as exc:
            raise gcmd.error('Invalid automatic nozzle Z calibration: %s' % exc)
        baseline = tool_z - station_z
        correction = ((hotend - 120.) * temp_coefficient
                      - (0.08 if bed >= 100. else 0.)
                      - (0.06 if first_layer < 0.11 else 0.))
        offset = baseline + correction + self.z_adjustments[tool]
        if not 0.5 <= offset <= 5.:
            raise gcmd.error('Automatic T%d nozzle Z %.4f mm outside 0.5..5 mm'
                             % (tool, offset))
        # Factory BuildPage::startPrint applies this absolute offset with
        # MOVE=1 before print motion. The earlier crash path was the
        # preparation purge occurring before this command at all.
        self._run('SET_GCODE_OFFSET Z=%.4f MOVE=1 MOVE_SPEED=100' % offset)
        self.print_z_baseline[tool] = baseline + correction
        gcmd.respond_info('Automatic T%d nozzle Z %.4f mm '
                          '(tool %.4f - station %.4f + print %.4f + '
                          'touchscreen %.4f)'
                          % (tool, offset, tool_z, station_z, correction,
                             self.z_adjustments[tool]))

    def cmd_verify_nozzle_z(self, gcmd):
        dock, grab, tool = self._preflight(gcmd)
        if tool is None or self.print_z_baseline[tool] is None:
            raise gcmd.error('Automatic nozzle Z is not applied; refusing '
                             'low-Z print motion')
        expected = self.print_z_baseline[tool] + self.z_adjustments[tool]
        actual = self.printer.lookup_object('gcode_move').get_status()[
            'homing_origin'][2]
        if not math.isfinite(actual) or abs(actual - expected) > 0.02:
            raise gcmd.error('Nozzle Z offset changed or reset '
                             '(expected %.4f, got %.4f)' % (expected, actual))

    def cmd_save_offsets_json(self, gcmd):
        if self.busy:
            raise gcmd.error('Cannot save offsets while toolchanger is moving')
        if not os.path.isfile(self.extruder_json_path):
            raise gcmd.error('Missing stock %s; refusing to create it'
                             % self.extruder_json_path)
        try:
            data, suffix = self._read_stock_json(self.extruder_json_path)
            for tool in range(4):
                for axis, value in zip('xyz', self.measurements[tool]):
                    data['t%d_offset_%s' % (tool, axis)] = value
                data['x_check_pos' + (str(tool) if tool else '')] = (
                    self.docks[tool][0])
                data['y_check_pos' + (str(tool) if tool else '')] = (
                    self.docks[tool][1])
            data['x_station_pos'] = self.station_x
            data['y_station_pos'] = self.station_y
            data['z_station_pos'] = self.station_z
            directory = os.path.dirname(self.extruder_json_path)
            backup = self.extruder_json_path + '.bak-' + (
                datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            shutil.copy2(self.extruder_json_path, backup)
            fd, temp_path = tempfile.mkstemp(prefix='.extruder-',
                                             suffix='.tmp', dir=directory)
            try:
                os.chmod(temp_path, os.stat(self.extruder_json_path).st_mode)
                with os.fdopen(fd, 'w') as target:
                    json.dump(data, target, indent=3, sort_keys=True)
                    target.write('\n')
                    if suffix:
                        target.write(suffix + '\n')
                    target.flush()
                    os.fsync(target.fileno())
                os.replace(temp_path, self.extruder_json_path)
            finally:
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
        except (OSError, ValueError, TypeError) as exc:
            raise gcmd.error('Could not save extruder offsets: %s' % exc)
        gcmd.respond_info('Saved offsets to %s (backup: %s)' % (
            self.extruder_json_path, backup))

    def _save_holder_position(self, gcmd, tool, x, y):
        # Extruder Position Calibrate changes only x/y_check_pos. The t*_offset
        # keys belong to the separate Extruder Offset Calibration workflow.
        if not os.path.isfile(self.extruder_json_path):
            raise gcmd.error('Missing stock %s; refusing to create it'
                             % self.extruder_json_path)
        temp_path = None
        try:
            data, suffix = self._read_stock_json(self.extruder_json_path)
            key_suffix = str(tool) if tool else ''
            data['x_check_pos' + key_suffix] = x
            data['y_check_pos' + key_suffix] = y
            directory = os.path.dirname(self.extruder_json_path)
            backup = self.extruder_json_path + '.bak-' + (
                datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            shutil.copy2(self.extruder_json_path, backup)
            fd, temp_path = tempfile.mkstemp(prefix='.extruder-holder-',
                                             suffix='.tmp', dir=directory)
            with os.fdopen(fd, 'w') as target:
                os.chmod(temp_path, os.stat(self.extruder_json_path).st_mode)
                json.dump(data, target, indent=3, sort_keys=True)
                target.write('\n')
                if suffix:
                    target.write(suffix + '\n')
                target.flush()
                os.fsync(target.fileno())
            os.replace(temp_path, self.extruder_json_path)
            temp_path = None
        except (OSError, ValueError, TypeError) as exc:
            raise gcmd.error('Could not save holder position: %s' % exc)
        finally:
            if temp_path is not None and os.path.exists(temp_path):
                os.unlink(temp_path)
        gcmd.respond_info('Saved T%d holder X=%.4f Y=%.4f to %s '
                          '(backup: %s)' % (
                              tool, x, y, self.extruder_json_path, backup))

    def _button(self, name, gcmd):
        obj = self._button_objects.get(name)
        if obj is None:
            obj = self.printer.lookup_object('gcode_button ' + name, None)
            if obj is not None:
                self._button_objects[name] = obj
        if obj is None:
            raise self._sensor_error(gcmd, 'Missing gcode_button %s' % name)
        return obj.get_status().get('state') == 'PRESSED'

    def _sensor_error(self, gcmd, message):
        if gcmd is not None:
            return gcmd.error(message)
        return self.printer.command_error(message)

    def _sensor_state(self, gcmd=None):
        # A safety read must also make the next UI poll reflect fresh pins.
        self._ui_sensor_cache = None
        dock = [self._button(name, gcmd) for name in self.dock_buttons]
        grab = [self._button(name, gcmd) for name in self.grab_buttons]
        if any(dock[i] and grab[i] for i in range(4)):
            raise self._sensor_error(
                gcmd, 'Conflicting dock/grab sensors; motion blocked')
        attached = [i for i in range(4) if grab[i] and not dock[i]]
        if len(attached) > 1:
            raise self._sensor_error(
                gcmd, 'Multiple heads appear attached; motion blocked')
        return dock, grab, attached[0] if attached else None

    def _validate_sensor_state(self, gcmd, dock, attached):
        # All four Creator 5 heads are fixed.  A head that is neither docked
        # nor detected as attached is an invalid state, not an empty slot.
        if attached is None:
            missing = [i for i, value in enumerate(dock) if not value]
        else:
            missing = [i for i, value in enumerate(dock)
                       if i != attached and not value]
        if missing:
            names = ','.join('T%d' % i for i in missing)
            raise self._sensor_error(
                gcmd, 'Missing dock confirmation for %s; motion blocked'
                % names)

    def attached_tool_from_pins(self):
        # AFC and status clients may query this, but the physical Creator 5
        # dock/grab inputs remain the sole authority for tool presence.
        dock, _, attached = self._sensor_state()
        self._validate_sensor_state(None, dock, attached)
        return attached

    def validate_nozzle_z_offset(self, gcmd, offset):
        # The physical grab inputs, not AFC's logical lane state, decide
        # whether a nozzle is present.  Zero is permitted only with no tool.
        if offset >= 0.5:
            return
        dock, _, attached = self._sensor_state(gcmd)
        self._validate_sensor_state(gcmd, dock, attached)
        if attached is not None:
            raise gcmd.error('Refusing nozzle Z offset %.4f with T%d '
                             'attached; use C5_AUTO_NOZZLE_Z'
                             % (offset, attached))

    def _chamber_heater_active(self, gcmd):
        # The Creator 5 (non-Pro) has no chamber heater.
        if self.printer.lookup_object('heater_generic chamber_heater',
                                      None) is None:
            return False
        try:
            heater = self.printer.lookup_object('heaters').lookup_heater(
                'chamber_heater')
            status = heater.get_status(self.printer.get_reactor().monotonic())
        except Exception as exc:
            raise gcmd.error('Cannot verify chamber heater state: %s' % exc)
        return status.get('target', 0.) > 0. or status.get('power', 0.) > 0.

    def _check_chamber_doors(self, gcmd, message):
        # Open access is permitted with the chamber heater off.  Never
        # silently bypass a failed heater-state lookup.
        if not self.door_buttons:
            return
        if self._chamber_heater_active(gcmd):
            for name in self.door_buttons:
                # The door switches read PRESSED while closed (the door
                # pushes the switch shut) and released once opened, so an
                # open door is the *not* pressed case.
                if not self._button(name, gcmd):
                    raise gcmd.error(message % name)

    def _preflight(self, gcmd, axes='xyz'):
        if self.busy:
            raise gcmd.error('Creator 5 toolchanger is already moving')
        self._check_chamber_doors(
            gcmd, 'Door %s is open while chamber heater is enabled')
        status = self.printer.lookup_object('toolhead').get_status(
            self.printer.get_reactor().monotonic())
        homed = status.get('homed_axes', '')
        if any(axis not in homed for axis in axes):
            raise gcmd.error('Home %s before Creator 5 motion' % axes.upper())
        dock, grab, attached = self._sensor_state(gcmd)
        self._validate_sensor_state(gcmd, dock, attached)
        return dock, grab, attached

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _pause_ms(self, milliseconds):
        if not milliseconds:
            return
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + milliseconds / 1000.)

    def _enable_extruder_driver(self, extruder):
        # Enabling the extruder motor re-initialises every TMC register over
        # the UART (each write verified with an IFCNT read) and restarts the
        # driver checks. In eboard PA mode the eboard polls SG_RESULT on the
        # same UART and the replies collide ("Unable to read tmc uart
        # 'extruder' register IFCNT"), so enable it before PA mode starts.
        stepper_enable = self.printer.lookup_object('stepper_enable')
        known = stepper_enable.get_steppers()
        names = [n for n in dict.fromkeys(('extruder', extruder)) if n in known]
        if stepper_enable.set_motors_enable(names, True):
            self._run('M400')
            # The re-init runs from a reactor callback; let it finish.
            self._pause_ms(1000)

    def _pause_extruder_tmc_checks(self):
        paused = []
        for name, obj in self.printer.lookup_objects('tmc2209'):
            if not name.startswith('tmc2209 extruder'):
                continue
            cmdhelper = getattr(obj, 'cmdhelper', None)
            echeck = getattr(cmdhelper, 'echeck_helper', None)
            if echeck is None or echeck.check_timer is None:
                continue
            echeck.stop_checks()
            paused.append(echeck)
        return paused

    def _resume_tmc_checks(self, paused):
        for echeck in paused:
            echeck.start_checks()

    # Detach the XY steppers from the toolhead so XY moves are planned
    # (and pressure advance applies) but no XY steps are sent. The drivers
    # stay enabled and hold the carriage in place.
    def _lock_xy(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        steppers = [s for s in toolhead.get_kinematics().get_steppers()
                    if s.is_active_axis('x') or s.is_active_axis('y')]
        trapqs = [s.set_trapq(None) for s in steppers]
        eventtime = self.printer.get_reactor().monotonic()
        scv = toolhead.get_status(eventtime)['square_corner_velocity']
        return steppers, trapqs, toolhead.get_position(), scv

    def _unlock_xy(self, locked):
        if locked is None:
            return
        steppers, trapqs, position, scv = locked
        self._run('M400')
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        for stepper, trapq in zip(steppers, trapqs):
            stepper.set_trapq(trapq)
        current = toolhead.get_position()
        toolhead.set_position([position[0], position[1]] + current[2:])
        self.printer.lookup_object('gcode_move').reset_last_position()
        self._run('SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=%.3f' % scv)

    def _move(self, x=None, y=None, z=None, feed=4800, machine=True):
        values = [value for value in (x, y, z) if value is not None]
        if (not all(math.isfinite(value) for value in values)
                or not math.isfinite(feed) or feed <= 0):
            raise self.printer.command_error(
                'Motion coordinates and feed must be finite')
        if machine:
            # Dock and fixture coordinates are physical carriage positions,
            # independent of G92, nozzle offsets, mesh, and speed overrides.
            toolhead = self.printer.lookup_object('toolhead')
            try:
                toolhead.manual_move([x, y, z], feed / 60.)
            finally:
                # Rebase G-code's cached position through its current transform
                # without changing that transform or the caller's origin.
                self.printer.lookup_object('gcode_move').reset_last_position()
            return
        parts = ['G1']
        for axis, value in (('X', x), ('Y', y), ('Z', z)):
            if value is not None:
                parts.append('%s%.4f' % (axis, value))
        parts.append('F%d' % feed)
        self._run(' '.join(parts))

    def _travel_to_lane(self, y, feed):
        # Inboard of approach_x nothing sits between the head and the dock
        # lane, so go there in one diagonal move instead of X then Y.
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_position()[0] <= self.approach_x + .001:
            self._move(x=self.approach_x, y=y, feed=feed)
        else:
            self._move(x=self.approach_x, feed=feed)
            self._move(y=y, feed=feed)

    def _raise_z(self):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_position()[2] < self.safe_z:
            self._move(z=self.safe_z,
                       feed=self.clearance_z_speed * 60.)

    def _lift_after_offset_calibration(self, gcmd):
        # Leave the levelboard target vertically before AFC docks or swaps
        # the measured tool. Use physical Z so nozzle offsets cannot alter it.
        toolhead = self.printer.lookup_object('toolhead')
        status = toolhead.get_status(self.printer.get_reactor().monotonic())
        if 'z' not in status.get('homed_axes', ''):
            raise gcmd.error('Home Z before offset calibration tool lift')
        current_z = toolhead.get_position()[2]
        z_max = status['axis_maximum'].z
        if not math.isfinite(current_z) or not math.isfinite(z_max):
            raise gcmd.error('Cannot determine offset calibration Z clearance')
        if current_z > z_max + .001:
            raise gcmd.error('Current Z position exceeds the travel limit')
        # A short relative lift is not enough when the fixture scan finishes
        # near Z=2. Clear the nozzle to the dock-safe height before returning.
        target = min(max(current_z + self.offset_calibration_lift,
                         self.safe_z), z_max)
        if target > current_z + .001:
            self._move(z=target,
                       feed=self.offset_calibration_lift_speed * 60.)
            self._run('M400')

    def _move_to_prep_bucket(self):
        # Travel inboard before entering the rear-right preparation area.
        self._raise_z()
        self._move(x=self.prep_approach_x, feed=6000, machine=False)
        self._move(y=self.prep_approach_y, feed=6000, machine=False)
        self._move(x=self.purge_x, y=self.purge_y,
                   feed=3000, machine=False)

    def _validate_purge_envelope(self, error):
        if (not all(math.isfinite(value) for value in (
                self.purge_x, self.purge_y, self.flow_sweep_x,
                self.purge_max_x))
                or self.purge_x + self.flow_sweep_x / 2. > self.purge_max_x):
            raise error('Purge/flow X path exceeds purge_max_x')

    def _verify(self, gcmd, expected, parked=None):
        try:
            self._run('M400')
            self._pause_ms(self.toolchange_sensor_settle_ms)
            dock, _, attached = self._sensor_state(gcmd)
            self._validate_sensor_state(gcmd, dock, attached)
            if attached != expected:
                raise gcmd.error('Tool sensor verification failed; heaters disabled')
            if parked is not None and not dock[parked]:
                raise gcmd.error('T%d was not confirmed in its dock' % parked)
        except Exception:
            # A failed tool motion must fail closed even when the sensor
            # state is contradictory or the requested dock is not confirmed.
            try:
                self._run('TURN_OFF_HEATERS')
            except Exception:
                # Preserve the original verification error for diagnosis.
                logging.exception(
                    'Creator 5 heater shutdown failed after tool verification')
            raise
        self.active = attached
        return dock

    def _afc_extruder(self, physical_name, afc=None):
        if afc is None:
            afc = self.printer.lookup_object('AFC', None)
        return afc.tools.get(physical_name) if afc is not None else None

    def _afc_lane(self, afc, physical_name):
        # AFC registers tools by physical extruder, but standalone lanes by
        # the AFC_extruder section alias (for example extruder0 for extruder).
        extruder = self._afc_extruder(physical_name, afc)
        return afc.lanes.get(extruder.name) if extruder is not None else None

    def _dock(self, gcmd, tool, raise_z=True):
        x, y = self.docks[tool]
        clear_feed = self.clear_travel_speed * 60.
        # Do not clear nozzle Z while a tool is physically mounted.  A
        # previously selected tool may have been left at zero by a slicer.
        origin_z = self.printer.lookup_object('gcode_move').get_status()[
            'homing_origin'][2]
        if not math.isfinite(origin_z) or origin_z < 0.5:
            self._apply_offsets(tool)
        # Dock motion uses physical coordinates, independent of XY offsets.
        self._run('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
        if raise_z:
            self._raise_z()
        self._travel_to_lane(y, clear_feed)
        self._move(x=x - self.dock_predock_offset, feed=clear_feed)
        self._move(x=x - 10., feed=self.dock_approach_speed * 60.)
        self._move(x=x, feed=self.dock_approach_speed * 60.)
        self._run('M400')
        # Release only with the head seated in its holder and the carriage at
        # the mount. After lost steps the dock position is wrong, and an
        # unchecked release dropped the head; keep it locked on instead.
        self._pause_ms(self.toolchange_sensor_settle_ms)
        if not (self._button(self.dock_buttons[tool], gcmd)
                and self._button(self.grab_buttons[tool], gcmd)):
            try:
                self._run('TURN_OFF_HEATERS')
            except Exception:
                logging.exception('Creator 5 heater shutdown failed after dock check')
            raise gcmd.error(
                'T%d is not seated in its dock; not releasing it (heaters '
                'off). XY may have lost steps: re-home before docking' % tool)
        self._run('MOTOR_RELEASE')
        self._pause_ms(self.release_latch_wait_ms)
        self._move(x=self.approach_x,
                   feed=self.departure_speed * 60.)
        self._verify(gcmd, None, parked=tool)
        # _dock() is AFC's custom_unselect command (bypassing its own
        # unload flow entirely), so lane_loaded/status never got cleared
        # here -- AFC kept reporting the lane as loaded indefinitely after
        # a successful physical dock. set_tool_unloaded() is pure
        # bookkeeping/LED state (no physical motion), safe to call after
        # the dock move above has already completed.
        extruder_name = 'extruder' if tool == 0 else 'extruder%d' % tool
        extruder_obj = self._afc_extruder(extruder_name)
        if extruder_obj is not None and extruder_obj.lane_loaded:
            lane = extruder_obj.lanes.get(extruder_obj.lane_loaded)
            if lane is not None:
                lane.set_tool_unloaded(normal_toolchange=True)

    def cmd_set_pressure_advance(self, gcmd):
        # Slicers using Klipper's "Klipper" gcode flavor commonly emit a
        # bare "SET_PRESSURE_ADVANCE ADVANCE=0" (no EXTRUDER=) immediately
        # before every toolchange, to disable PA on whatever extruder is
        # currently active before it gets docked -- and never restore a
        # real value afterward anywhere in the file. Remember the last
        # *nonzero* value actually applied to each extruder here (covering
        # a static config value, a slicer's own per-filament EXTRUDER=
        # command, or an auto/adaptive PA routine's result -- whichever one
        # last legitimately set it) so _restore_pressure_advance() can put
        # the right number back on the next pickup instead of guessing.
        self.prev_set_pressure_advance(gcmd)
        extruder_name = gcmd.get('EXTRUDER', None)
        if extruder_name is None:
            extruder_name = self.printer.lookup_object(
                'toolhead').get_extruder().get_name()
        extruder = self.printer.lookup_object(extruder_name, None)
        stepper = getattr(extruder, 'extruder_stepper', None)
        if stepper is not None and stepper.pressure_advance > 0.:
            self._tool_pa_memory[extruder_name] = (
                stepper.pressure_advance, stepper.pressure_advance_smooth_time)

    def _restore_pressure_advance(self, extruder_name):
        # Re-apply whatever pressure_advance should actually be active for
        # this tool right after every pickup, so a toolchange can never
        # leave it zeroed (see cmd_set_pressure_advance above for why).
        extruder = self.printer.lookup_object(extruder_name, None)
        stepper = getattr(extruder, 'extruder_stepper', None)
        if stepper is None:
            return
        advance, smooth_time = self._tool_pa_memory.get(
            extruder_name, (stepper.config_pa, stepper.config_smooth_time))
        self._run('SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.5f '
                  'SMOOTH_TIME=%.5f'
                  % (extruder_name, advance, smooth_time))

    def _pickup(self, gcmd, tool, raise_z=True):
        x, y = self.docks[tool]
        clear_feed = self.clear_travel_speed * 60.
        # Establish a calibrated, positive nozzle clearance before the latch
        # can make the tool physically attached.  Print-specific thermal Z
        # compensation is applied later by C5_AUTO_NOZZLE_Z.
        self._apply_offsets(tool)
        if raise_z:
            self._raise_z()
        self._travel_to_lane(y, clear_feed)
        self._move(x=self.pre_dock_x,
                   feed=self.pickup_predock_speed * 60.)
        self._move(x=x, feed=self.pickup_latch_speed * 60.)
        self._run('M400')
        if not self._button(self.grab_buttons[tool], gcmd):
            raise gcmd.error('T%d grab sensor did not engage at mount' % tool)
        self._run('MOTOR_GRAB')
        slow_distance = min(self.pullback, self.pullback_slow_distance)
        self._move(x=x - slow_distance,
                   feed=self.pullback_speed * 60.)
        if self.pullback > slow_distance:
            self._move(x=x - self.pullback, feed=clear_feed)
        self._run('MOTOR_GRAB2')
        self._pause_ms(self.pickup_latch_wait_ms)
        # Verify the physical sensors at the clear pullback point before
        # activation. The final departure can flow into AFC's return move.
        self._verify(gcmd, tool)
        extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
        self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
        self._restore_pressure_advance(extruder)
        # A pullback longer than the dock-to-approach_x distance already
        # ends inboard of it; moving "out" to approach_x would step back
        # toward the docks.
        if x - self.pullback > self.approach_x:
            self._move(x=self.approach_x,
                       feed=self.pickup_departure_speed * 60.)

    def _with_motion(self, gcmd, operation, final_accel=None):
        toolhead = self.printer.lookup_object('toolhead')
        old_accel = toolhead.get_max_velocity()[1]
        old_mcr = toolhead.get_status(
            self.printer.get_reactor().monotonic()).get('minimum_cruise_ratio')
        gcode_move = self.printer.lookup_object('gcode_move')
        was_absolute = gcode_move.get_status().get('absolute_coordinates')
        self._ui_sensor_cache = None
        self.busy = True
        completed = False
        try:
            self._run('G90')
            self._run('SET_VELOCITY_LIMIT ACCEL=%.0f' % self.pickup_accel)
            if self.pickup_minimum_cruise_ratio is not None and old_mcr is not None:
                self._run('SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=%.3f'
                          % self.pickup_minimum_cruise_ratio)
            result = operation()
            completed = True
            return result
        finally:
            try:
                restore_accel = (final_accel if completed and
                                 final_accel is not None else old_accel)
                self._run('SET_VELOCITY_LIMIT ACCEL=%.4f'
                          % restore_accel)
                if self.pickup_minimum_cruise_ratio is not None and old_mcr is not None:
                    self._run('SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=%.6f'
                              % old_mcr)
                if not was_absolute:
                    self._run('G91')
            finally:
                self.busy = False
                self._ui_sensor_cache = None

    @contextmanager
    def _calibration_transaction(self):
        measurements = [list(values) for values in self.measurements]
        station = self.station_x, self.station_y, self.station_z
        origin = tuple(self.printer.lookup_object('gcode_move').get_status()[
            'homing_origin'][:3])
        try:
            yield
        except Exception:
            self.measurements = measurements
            self.station_x, self.station_y, self.station_z = station
            if origin[2] >= .5:
                self._run('SET_GCODE_OFFSET X=%.4f Y=%.4f Z=%.4f MOVE=0'
                          % origin)
            else:
                # A tool may have been picked up after this snapshot. Never
                # restore its old zero/near-zero nozzle Z and hide the actual
                # calibration failure behind the nozzle safety guard.
                self._run('SET_GCODE_OFFSET X=%.4f Y=%.4f MOVE=0'
                          % origin[:2])
            raise

    def cmd_home_for_print(self, gcmd):
        dock, grab, attached = self._preflight(gcmd, axes='')
        recovered = attached is not None
        if attached is not None:
            self.cmd_recover_attached(gcmd)
            dock, grab, attached = self._preflight(gcmd, axes='')
        if attached is not None:
            raise gcmd.error('Attached head remains after recovery; Z homing blocked')
        if not all(dock):
            raise gcmd.error('Confirm all heads are docked before print homing')
        if recovered:
            self._run('G28 Z')
        else:
            # Preserve safe_z_home's single-call all-axis sequence when
            # there was no head to recover.
            old_passthrough = self._g28_passthrough
            self._g28_passthrough = True
            try:
                self._run('G28')
            finally:
                self._g28_passthrough = old_passthrough

    def cmd_recover_attached(self, gcmd):
        dock, grab, attached = self._preflight(gcmd, axes='')
        if attached is None:
            return
        # safe_z_home supplies a relative clearance hop even when Z is not
        # referenced. G28 X Y does not home/probe Z. Docking then uses XY only.
        safe_home = self.printer.lookup_object('safe_z_home', None)
        if safe_home is None or safe_home.z_hop < self.safe_z:
            raise gcmd.error('Attached-head recovery requires safe_z_home '
                             'z_hop of at least %.1f mm' % self.safe_z)
        self._run('G28 X Y')
        dock, grab, current = self._preflight(gcmd, axes='xy')
        if current != attached:
            raise gcmd.error('Attached head changed during XY homing')
        self._with_motion(gcmd, lambda: self._dock(
            gcmd, attached, raise_z=False))
        # AFC's active-tool lookup reads the same physical pins; after this
        # verified dock it reports no selected head without a second dock call.

    def _prepare_tool_motion(self, gcmd):
        dock, grab, attached = self._preflight(gcmd, axes='xy')
        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(
            self.printer.get_reactor().monotonic()).get('homed_axes', '')
        if 'z' in homed:
            return dock, attached, True
        # SafeZHoming provides a relative clearance hop without claiming Z is
        # homed. Re-run XY homing to guarantee that hop before dock travel.
        safe_home = self.printer.lookup_object('safe_z_home', None)
        if safe_home is None or safe_home.z_hop < self.safe_z:
            raise gcmd.error('XY-only tool motion requires safe_z_home '
                             'z_hop of at least %.1f mm' % self.safe_z)
        self._run('G28 X Y')
        new_dock, _, current = self._preflight(gcmd, axes='xy')
        if current != attached or new_dock != dock:
            raise gcmd.error('Tool sensors changed during XY homing')
        return new_dock, current, False

    def cmd_status(self, gcmd):
        # A status query must report raw sensor bits even when they look
        # contradictory - that is the one case a diagnostic command exists
        # for. _sensor_state raises on conflicts, which is right for a
        # motion safety gate but wrong here: it left this command unable to
        # show anything precisely when there was something to show.
        dock = [self._button(name, gcmd) for name in self.dock_buttons]
        grab = [self._button(name, gcmd) for name in self.grab_buttons]
        conflicts = [i for i in range(4) if dock[i] and grab[i]]
        attached = [i for i in range(4) if grab[i] and not dock[i]]
        if conflicts:
            label = 'conflict(%s)' % ','.join('T%d' % i for i in conflicts)
        elif len(attached) > 1:
            label = 'multiple(%s)' % ','.join('T%d' % i for i in attached)
        elif attached:
            label = 'T%d' % attached[0]
            self.active = attached[0]
        else:
            label = 'none'
            self.active = None
        gcmd.respond_info('attached=%s dock=%s grab=%s' % (
            label,
            ''.join('1' if v else '0' for v in dock),
            ''.join('1' if v else '0' for v in grab)))

    def cmd_wheel_runout(self, gcmd):
        """The stock fm_exN wheel failure is a clog, not spool runout."""
        if not self.clog_switch.enabled:
            return
        tool = gcmd.get_int('TOOL', minval=0, maxval=3)
        afc = self.printer.lookup_object('AFC')
        if (not afc.function.is_printing() or afc.in_toolchange
                or afc.error_state):
            return
        name = 'extruder' if tool == 0 else 'extruder%d' % tool
        lane = self._afc_lane(afc, name)
        if lane is None or afc.current is None:
            afc.error.AFC_error('T%d wheel failure: AFC active lane is unknown'
                                % tool)
            return
        if afc.current != lane.name:
            # A docked tool is not the printing tool. Its encoder must not
            # interrupt the active head or cause an unrelated tool swap.
            return
        try:
            attached = self.attached_tool_from_pins()
        except Exception as exc:
            afc.error.AFC_error('T%d wheel failure: mount sensors invalid: %s'
                                % (tool, exc))
            return
        if (attached != tool or lane.extruder_obj.creator5_tool_index != tool
                or not lane._is_normal_printing_state()):
            afc.error.AFC_error(
                'T%d wheel failure: active tool state is inconsistent; '
                'print paused' % tool)
            return
        if not lane.extruder_obj.tool_start_state:
            # The separate presence switch owns runout and infinite spool.
            # Its callback may be racing this encoder event.
            return
        afc.error.AFC_error(
            'E0163: Paused. T%d clog detected: filament is present but the '
            'wheel stopped moving. Clear the filament path before resuming.'
            % tool)

    def cmd_runout_switch(self, gcmd):
        """Continue a print paused by TOOL's runout on tool TO."""
        tool = gcmd.get_int('TOOL', minval=0, maxval=3)
        to = gcmd.get_int('TO', minval=0, maxval=3)
        temp = gcmd.get_float('TEMP', None, above=0.)
        if to == tool:
            raise gcmd.error('TO must be a different tool than TOOL')
        afc = self.printer.lookup_object('AFC')
        if not afc.function.is_paused():
            raise gcmd.error('The print is not paused')
        source = 'extruder' if tool == 0 else 'extruder%d' % tool
        target = 'extruder' if to == 0 else 'extruder%d' % to
        lane = self._afc_lane(afc, source)
        replacement = self._afc_lane(afc, target)
        if lane is None or replacement is None:
            raise gcmd.error('No AFC lane for T%d or T%d' % (tool, to))
        # Keep the position AFC saved when the runout paused the print:
        # reset_failure() clears position_saved, and restore_pos needs it.
        position_saved = afc.position_saved
        afc.error.reset_failure()
        afc.position_saved = position_saved
        lane._perform_creator5_infinite_runout(replacement, target_temp=temp)
        if afc.error_state:
            return
        # PAUSE marked the print paused; the switch resumed it directly.
        self.printer.lookup_object('pause_resume').is_paused = False
        self._run('SET_HEATER_TEMPERATURE HEATER=%s TARGET=0' % source)

    def _sync_clog_sensors(self, tool=None):
        # firmwareExe disables all four wheels, then resets and enables only
        # the selected wheel. Do this directly without its three-second wait.
        if tool is None:
            tool = self.active
        for index in range(4):
            sensor = self.printer.lookup_object(
                'filament_motion_sensor fm_ex%d' % index)
            sensor.runout_helper.sensor_enabled = False
        if not self.clog_switch.enabled or tool is None:
            return
        sensor = self.printer.lookup_object(
            'filament_motion_sensor fm_ex%d' % tool)
        sensor._update_filament_runout_pos()
        sensor.runout_helper.filament_present = True
        sensor.runout_helper.sensor_enabled = True

    def cmd_select(self, gcmd):
        tool = gcmd.get_int('T', minval=0, maxval=3)
        # The touchscreen may have saved a new per-tool Z value since Klippy
        # started. Refresh before moving or applying offsets for this pickup.
        self._load_zoffset_json()
        dock, grab, attached = self._preflight(gcmd, axes='')
        if attached == tool:
            # AFC may select a head that was already mounted at Klippy start.
            # Restore its active extruder and offsets even without pickup motion.
            self._apply_offsets(tool)
            extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
            self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
            self._restore_pressure_advance(extruder)
            if self.post_select_accel is not None:
                self._run('SET_VELOCITY_LIMIT ACCEL=%.0f'
                          % self.post_select_accel)
            self.active = tool
            self._sync_clog_sensors(tool)
            gcmd.respond_info('T%d is already attached' % tool)
            return
        dock, attached, z_homed = self._prepare_tool_motion(gcmd)
        if not dock[tool]:
            raise gcmd.error('T%d is not in its dock' % tool)
        def change():
            if attached is not None:
                self._dock(gcmd, attached, raise_z=z_homed)
            self._pickup(gcmd, tool, raise_z=z_homed)
        self._with_motion(gcmd, change,
                          final_accel=self.post_select_accel)
        self._count_toolchange(tool)
        self._sync_clog_sensors(tool)

    def cmd_dock(self, gcmd):
        dock, grab, attached = self._preflight(gcmd, axes='')
        if attached is not None:
            dock, attached, z_homed = self._prepare_tool_motion(gcmd)
            self._with_motion(gcmd, lambda: self._dock(
                gcmd, attached, raise_z=z_homed))
            self._sync_clog_sensors()

    def cmd_mount_coords(self, gcmd):
        for i, (x, y) in enumerate(self.docks):
            gcmd.respond_info('T%d mount X=%.4f Y=%.4f' % (i, x, y))

    def cmd_mount_correct(self, gcmd):
        tool = gcmd.get_int('T', minval=0, maxval=3)
        x = gcmd.get_float('X')
        y = gcmd.get_float('Y')
        if self.busy:
            raise gcmd.error('Cannot correct mounts while toolchanger is moving')
        if not all(math.isfinite(value) for value in (x, y)):
            raise gcmd.error('Mount coordinates must be finite')
        old = self.docks[tool]
        if (abs(x - old[0]) > self.max_mount_correction
                or abs(y - old[1]) > self.max_mount_correction):
            raise gcmd.error('Mount deviation exceeds configured limit')
        if gcmd.get_int('SAVE', 0, minval=0, maxval=1):
            self._save_holder_position(gcmd, tool, x, y)
        self.docks[tool] = [x, y]
        gcmd.respond_info('T%d mount correction DX=%+.4f DY=%+.4f'
                          % (tool, x - old[0], y - old[1]))

    def _estop(self, axis, target, gcmd):
        obj = self.printer.lookup_object('e_stop ' + axis, None)
        if obj is None:
            raise gcmd.error('Missing [e_stop %s] for levelboard calibration'
                             % axis)
        old_target = obj.position_offset
        try:
            obj.position_offset = target
            value = obj.run_probe(gcmd)
        finally:
            obj.position_offset = old_target
        if value is None or not math.isfinite(value):
            raise gcmd.error('Invalid levelboard %s probe result' % axis)
        return value

    def _scan_xy(self, gcmd, cx, cy, z):
        if z is None:
            raise gcmd.error('Set scan_height before levelboard XY scanning')
        if not math.isfinite(z):
            raise gcmd.error('Levelboard scan height must be finite')
        points = []
        for axis, direction in (('X', 1), ('Y', 1),
                                ('X', -1), ('Y', -1)):
            self._raise_z()
            # Stock calibrationPoint() starts at the cylinder center and
            # probes outward by 7 mm in each direction. Starting at an edge
            # can miss the target and produced off-fixture travel here.
            self._move(x=cx, y=cy, feed=1200)
            self._move(z=z, feed=600)
            center = cx if axis == 'X' else cy
            span = self.scan_spans[(axis, direction)]
            target = center + span * direction
            try:
                result = self._estop(axis, target, gcmd)
            except Exception as exc:
                raise gcmd.error(
                    'Levelboard %s%s probe failed: center=%.3f '
                    'target=%.3f Z=%.3f: %s' %
                    (axis, '+' if direction > 0 else '-', center,
                     target, z, exc)) from exc
            distance = (result - center) * direction
            if not .25 <= distance <= span:
                raise gcmd.error(
                    'Levelboard %s edge outside scan range: center=%.3f '
                    'target=%.3f contact=%.3f' %
                    (axis, center, target, result))
            points.append(result)
        self._raise_z()
        if (abs(points[0] - points[2]) < 0.25
                or abs(points[1] - points[3]) < 0.25):
            raise gcmd.error('Levelboard scan did not resolve both fixture edges')
        return ((points[0] + points[2]) / 2.,
                (points[1] + points[3]) / 2.)

    def _calibrate_tool_xy(self, gcmd, tool, scan_height=None):
        cx = self.station_x + self.tool_fixture_shift_x
        cy = self.station_y
        if scan_height is None:
            scan_height = self.scan_height
        x, y = self._scan_xy(gcmd, cx, cy, scan_height)
        old = self.measurements[tool]
        if (abs(x - old[0]) > self.max_offset_correction
                or abs(y - old[1]) > self.max_offset_correction):
            raise gcmd.error('T%d offset deviation exceeds configured limit'
                             % tool)
        old[0], old[1] = x, y
        gcmd.respond_info('T%d levelboard XY X=%.4f Y=%.4f' % (tool, x, y))

    def _probe_stock_pin(self, gcmd, x, y, runs=3):
        # runs: probe runs to compare (each is [probe] samples taps).
        # Kalico's probe.py has no get_probe_params()/start_probe_session()
        # (those are mainline klipper's ProbeSessionHelper API) -- its
        # PrinterProbe.run_probe(gcmd) already does its own multi-sample
        # averaging and returns a plain [x, y, z] list, so drive it
        # directly using its own sample_retract_dist/get_lift_speed.
        probe = self.printer.lookup_object('probe', None)
        if probe is None:
            raise gcmd.error('Stock [probe] pin is required for offset calibration')
        toolhead = self.printer.lookup_object('toolhead')
        sample_retract_dist = gcmd.get_float(
            'SAMPLE_RETRACT_DIST', probe.sample_retract_dist, above=0.)
        lift_speed = probe.get_lift_speed(gcmd)
        self._raise_z()
        self._move(x=x, y=y, feed=self.plate_check_travel_speed * 60.)
        probe.multi_probe_begin(always_restore_toolhead=True)
        try:
            values = []
            for i in range(runs):
                pos = probe.run_probe(gcmd)
                values.append(pos[2])
                if i < runs - 1:
                    # run_probe() only retracts between its OWN internal
                    # samples, never after the last one -- every other
                    # multi-probe caller in klipper (e.g. cmd_PROBE_ACCURACY)
                    # explicitly lifts between calls for exactly this
                    # reason. Without it, the next outer iteration here
                    # starts right at the previous contact point with the
                    # probe still physically triggered, which is what was
                    # producing "Probe triggered prior to movement"
                    # consistently on the 2nd/3rd outer probe.
                    lift_z = toolhead.get_position()[2] + sample_retract_dist
                    self._move(z=lift_z, feed=lift_speed * 60)
        finally:
            probe.multi_probe_end()
            self._raise_z()
        if (len(values) != runs or not all(math.isfinite(z) for z in values)
                or max(values) - min(values) >= .1):
            raise gcmd.error('Stock calibration pin is not repeatable within 0.1 mm')
        return sum(values) / len(values)

    def _measure_plate_pins(self, gcmd, runs=3):
        # Stock checkPlatformRemove(): compare the regular pin's contact
        # height at X43/Y226 with the cylinder position from test.json.
        if not os.path.isfile(self.test_json_path):
            raise gcmd.error('Missing factory test config %s' % self.test_json_path)
        try:
            data, _ = self._read_stock_json(self.test_json_path)
            cylinder_x = float(data['cylinder_x'])
            cylinder_y = float(data['cylinder_y'])
        except (OSError, ValueError, TypeError, KeyError) as exc:
            raise gcmd.error('Invalid factory cylinder position: %s' % exc)
        if not all(math.isfinite(v) for v in (cylinder_x, cylinder_y)):
            raise gcmd.error('Factory cylinder position must be finite')
        first = self._probe_stock_pin(gcmd, self.plate_check_x,
                                      self.plate_check_y, runs)
        second = self._probe_stock_pin(gcmd, cylinder_x, cylinder_y, runs)
        return first, second

    def _check_plate_removed(self, gcmd):
        first, second = self._measure_plate_pins(gcmd)
        if abs(first - second) < self.plate_check_min_delta:
            raise gcmd.error('Build plate may still be installed; ordinary '
                             'calibration pin heights differ by less than '
                             '%.2f mm' % self.plate_check_min_delta)
        gcmd.respond_info('Stock calibration pin Z=%.4f / %.4f; plate '
                          'removal verified' % (first, second))
        return first

    def _check_plate_present(self, gcmd, raise_error=True):
        # Inverse of _check_plate_removed: with the plate installed both
        # pins read within plate_check_min_delta of each other. A plate is
        # there or not (0.8 mm apart), so one probe run per pin will do.
        first, second = self._measure_plate_pins(gcmd, runs=1)
        if abs(first - second) >= self.plate_check_min_delta:
            msg = ('Build plate not detected; install the build plate and '
                   'restart the print (calibration pin heights differ by '
                   '%.2f mm)' % abs(first - second))
            self.build_plate_detected = False
            if raise_error:
                raise gcmd.error(msg)
            gcmd.respond_info(msg)
            return
        self.build_plate_detected = True
        gcmd.respond_info('Stock calibration pin Z=%.4f / %.4f; build '
                          'plate detected' % (first, second))

    def _probe_fixture_z(self, gcmd, x, y, target=None):
        self._move(z=self.safe_z, feed=1200)
        self._move(x=x, y=y, feed=1200)
        if target is None:
            target = self.z_probe_target
            if self._stock_pin_approach_z is not None:
                # Never descend farther than the configured hard floor.
                target = max(target, self._stock_pin_approach_z)
        z = self._estop('Z', target, gcmd)
        self._raise_z()
        return z

    def _calibrate_tool_z(self, gcmd, tool):
        z = self._probe_fixture_z(gcmd,
                                  self.station_x + self.tool_fixture_shift_x,
                                  self.station_y)
        if abs(z - self.measurements[tool][2]) > self.max_offset_correction:
            raise gcmd.error('T%d Z deviation exceeds configured limit' % tool)
        self.measurements[tool][2] = z
        gcmd.respond_info('T%d levelboard Z=%.4f' % (tool, z))
        return z

    def _apply_offsets(self, tool):
        self.print_z_baseline[tool] = None
        offsets = [self.measurements[tool][axis]
                   - self.measurements[0][axis] for axis in range(2)]
        offsets.append(self.measurements[tool][2] - self.station_z
                       + self.z_adjustments[tool])
        if not all(math.isfinite(value) for value in offsets) or not (
                0.5 <= offsets[2] <= 5.):
            raise self.printer.command_error(
                'Unsafe T%d nozzle Z offset; refusing tool motion' % tool)
        self._run('SET_GCODE_OFFSET X=%.4f Y=%.4f Z=%.4f MOVE=0'
                  % tuple(offsets))

    def cmd_reference_calibrate(self, gcmd):
        if not gcmd.get_int('BUILDPLATE_REMOVED', 0, minval=0, maxval=1):
            raise gcmd.error('Remove the build plate and pass BUILDPLATE_REMOVED=1')
        scan_height = gcmd.get_float('SCAN_Z', self.reference_scan_height)
        if scan_height is not None and not math.isfinite(scan_height):
            raise gcmd.error('SCAN_Z must be finite')
        dock, grab, attached = self._preflight(gcmd)
        if attached is not None:
            raise gcmd.error('Dock the mounted head before reference scanning')
        def calibrate():
            self._run('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
            z = self._probe_fixture_z(gcmd, self.station_x, self.station_y)
            if abs(z - self.station_z) > self.max_offset_correction:
                raise gcmd.error('Levelboard reference Z deviation exceeds limit')
            if scan_height is None:
                actual_scan_height = z + 0.6
            else:
                actual_scan_height = scan_height
            x, y = self._scan_xy(gcmd, self.station_x, self.station_y,
                                 actual_scan_height)
            if (abs(x - self.station_x) > self.max_offset_correction
                    or abs(y - self.station_y) > self.max_offset_correction):
                raise gcmd.error('Levelboard reference deviation exceeds limit')
            self.station_x, self.station_y, self.station_z = x, y, z
            gcmd.respond_info('Levelboard reference X=%.4f Y=%.4f Z=%.4f'
                              % (x, y, z))
        with self._calibration_transaction():
            self._with_motion(gcmd, calibrate)
        if gcmd.get_int('SAVE', 0, minval=0, maxval=1):
            self.configfile.set(self.section, 'station_x',
                                '%.4f' % self.station_x)
            self.configfile.set(self.section, 'station_y',
                                '%.4f' % self.station_y)
            self.configfile.set(self.section, 'station_z',
                                '%.4f' % self.station_z)

    def _dock_for_offset_reference(self, gcmd, attached):
        if attached is None:
            return
        # The stock probe needs a bare carriage. Clear the attached nozzle
        # to safe_z (10 mm in the Creator 5 config) before either dock path.
        self._raise_z()
        # Keep AFC's logical tool state aligned when it owns the attached
        # head. A recovered head with no AFC selection can dock directly.
        afc = self.printer.lookup_object('AFC', None)
        name = 'extruder' if attached == 0 else 'extruder%d' % attached
        lane = self._afc_lane(afc, name) if afc is not None else None
        self._run('AFC_UNSELECT_TOOL' if lane is not None and
                  afc.current == lane.name else 'C5_TOOL_DOCK')
        _, _, still_attached = self._preflight(gcmd)
        if still_attached is not None:
            raise gcmd.error('Dock T%d before loadcell reference' % attached)

    def _calibrate_offset_reference(self, gcmd, levelboard=True):
        # This ordinary stock probe check also verifies that the plate was
        # removed. It must precede any levelboard contact, for a single head
        # just as it does for a full four-head calibration.
        pin_z = self._with_motion(gcmd,
                                  lambda: self._check_plate_removed(gcmd))
        self._stock_pin_approach_z = pin_z - 5.
        if levelboard:
            self._run('C5_LEVELBOARD_REFERENCE_CALIBRATE SAVE=0 '
                      'BUILDPLATE_REMOVED=1')

    def cmd_calibrate_all_offsets(self, gcmd):
        if not gcmd.get_command_parameters():
            self._offset_wizard_step(gcmd, 'plate', 'ALL')
            return
        if not gcmd.get_int('BUILDPLATE_REMOVED', 0, minval=0, maxval=1):
            raise gcmd.error('Remove the build plate and pass BUILDPLATE_REMOVED=1')
        save = gcmd.get_int('SAVE', 1, minval=0, maxval=1)
        _, _, attached = self._preflight(gcmd)
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(
                self.printer.get_reactor().monotonic()).get('state') in (
                    'printing', 'paused'):
            raise gcmd.error('Finish or cancel the print before calibration')
        # One transaction spans reference, all four heads, and the atomic JSON
        # replacement. Never stage individual tools in SAVE_CONFIG: JSON is the
        # authoritative set loaded at startup, and a later failure must leave
        # neither a partial runtime set nor partial pending config values.
        with self._calibration_transaction():
            self._dock_for_offset_reference(gcmd, attached)
            try:
                self._calibrate_offset_reference(gcmd)
                self._offset_batch_active = True
                for tool in range(4):
                    heater = 'extruder' if tool == 0 else 'extruder%d' % tool
                    self._run('AFC_SELECT_TOOL TOOL=%s' % heater)
                    self._run('C5_TOOL_OFFSET_CALIBRATE T=%d Z=1 SAVE=0 '
                              'BUILDPLATE_REMOVED=1' % tool)
                    self._run('AFC_UNSELECT_TOOL')
                if save:
                    self.cmd_save_offsets_json(gcmd)
            finally:
                self._offset_batch_active = False
                self._stock_pin_approach_z = None

    def cmd_check_load_tool(self, gcmd):
        _, _, attached = self._preflight(gcmd, axes='')
        requested = gcmd.get_int('T', attached, minval=0, maxval=3)
        if attached is None or requested != attached:
            raise gcmd.error('Loading requires the requested tool to be attached')
        heater = 'extruder' if attached == 0 else 'extruder%d' % attached
        active = self.printer.lookup_object('toolhead').get_extruder().get_name()
        if active != heater:
            raise gcmd.error('Active hotend does not match the attached tool')

    def cmd_offset_calibrate(self, gcmd):
        if not gcmd.get_command_parameters():
            self._offset_wizard_step(gcmd, 'tool', '')
            return
        if not gcmd.get_int('BUILDPLATE_REMOVED', 0, minval=0, maxval=1):
            raise gcmd.error('Remove the build plate and pass BUILDPLATE_REMOVED=1')
        tool = gcmd.get_int('T', minval=0, maxval=3)
        include_z = gcmd.get_int('Z', 0, minval=0, maxval=1)
        levelboard = gcmd.get_int('LEVELBOARD', 1, minval=0, maxval=1)
        scan_height = gcmd.get_float('SCAN_Z', self.scan_height)
        if scan_height is not None and not math.isfinite(scan_height):
            raise gcmd.error('SCAN_Z must be finite')
        if scan_height is None and not include_z:
            raise gcmd.error('Set scan_height or provide SCAN_Z before scanning')
        def calibrate():
            self._run('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
            try:
                if include_z:
                    z = self._calibrate_tool_z(gcmd, tool)
                    actual_scan_height = (z + 0.6 if scan_height is None
                                          else scan_height)
                else:
                    actual_scan_height = scan_height
                self._calibrate_tool_xy(gcmd, tool, actual_scan_height)
                self._apply_offsets(tool)
            finally:
                # Also clear a failed edge scan before reporting its error.
                self._lift_after_offset_calibration(gcmd)
        with self._calibration_transaction():
            single = not self._offset_batch_active
            if single:
                _, _, attached = self._preflight(gcmd)
                stats = self.printer.lookup_object('print_stats', None)
                if stats is not None and stats.get_status(
                        self.printer.get_reactor().monotonic()).get('state') in (
                            'printing', 'paused'):
                    raise gcmd.error('Finish or cancel the print before calibration')
                self._dock_for_offset_reference(gcmd, attached)
            try:
                if single:
                    self._calibrate_offset_reference(gcmd, levelboard=levelboard)
                    heater = 'extruder' if tool == 0 else 'extruder%d' % tool
                    self._run('AFC_SELECT_TOOL TOOL=%s' % heater)
                _, _, attached = self._preflight(gcmd)
                if attached != tool:
                    raise gcmd.error('Attach T%d before offset calibration' % tool)
                self._with_motion(gcmd, calibrate)
            finally:
                if single:
                    self._stock_pin_approach_z = None
        if gcmd.get_int('SAVE', 0, minval=0, maxval=1):
            axes = 'xyz' if include_z else 'xy'
            for axis, value in zip(axes, self.measurements[tool]):
                self.configfile.set(self.section, 't%d_measure_%s' % (tool, axis),
                                    '%.4f' % value)
        if single:
            self._dock(gcmd, tool)

    def cmd_calibrate_attached(self, gcmd):
        if not gcmd.get_int('BUILDPLATE_REMOVED', 0, minval=0, maxval=1):
            raise gcmd.error('Remove the build plate and pass BUILDPLATE_REMOVED=1')
        dock, grab, attached = self._preflight(gcmd)
        if attached is None:
            gcmd.respond_info('No head attached')
            return
        def calibrate_then_park():
            with self._calibration_transaction():
                self._run('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
                self._calibrate_tool_xy(gcmd, attached)
                self._apply_offsets(attached)
                self._dock(gcmd, attached)
        self._with_motion(gcmd, calibrate_then_park)

    def cmd_mount_probe(self, gcmd):
        tool = gcmd.get_int('T', minval=0, maxval=3)
        dock, grab, attached = self._preflight(gcmd)
        if attached is not None or not dock[tool]:
            raise gcmd.error('Park all heads before mount probe')
        x = gcmd.get_float('CENTER_X')
        y = gcmd.get_float('CENTER_Y')
        z = gcmd.get_float('SCAN_Z')
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise gcmd.error('Mount probe coordinates must be finite')
        def measure():
            measured_x, measured_y = self._scan_xy(gcmd, x, y, z)
            dx, dy = measured_x - x, measured_y - y
            if (abs(dx) > self.max_mount_correction
                    or abs(dy) > self.max_mount_correction):
                raise gcmd.error('Mount probe deviation exceeds limit')
            if gcmd.get_int('SAVE', 0, minval=0, maxval=1):
                self._save_holder_position(gcmd, tool,
                                           self.docks[tool][0] + dx,
                                           self.docks[tool][1] + dy)
            self.cmd_mount_correct_values(gcmd, tool, dx, dy)
        self._with_motion(gcmd, measure)

    def cmd_mount_correct_values(self, gcmd, tool, dx, dy):
        self.docks[tool][0] += dx
        self.docks[tool][1] += dy
        gcmd.respond_info('T%d mount measured DX=%+.4f DY=%+.4f; corrected X=%.4f Y=%.4f'
                          % (tool, dx, dy, *self.docks[tool]))

    def _wait_for_holder_contact(self, gcmd, tool):
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + self.position_calibration_timeout
        while reactor.monotonic() < deadline:
            if (self._button(self.dock_buttons[tool], gcmd)
                    and self._button(self.grab_buttons[tool], gcmd)):
                if any(self._button(name, gcmd) for i, name in
                       enumerate(self.grab_buttons) if i != tool):
                    raise gcmd.error('Another toolhead also reports attached')
                return
            reactor.pause(min(reactor.monotonic() + .05, deadline))
        raise gcmd.error('Timed out waiting for T%d holder contact; '
                         'home XY before other motion' % tool)

    def _measure_holder_position(self, gcmd, tool):
        x_home = self.printer.lookup_object('hd_home X', None)
        y_home = self.printer.lookup_object('hd_home Y', None)
        if x_home is None or y_home is None:
            raise gcmd.error('Extruder Position Calibrate needs '
                             '[hd_home X] and [hd_home Y]')
        y_reference = (HOLDER_REFERENCE_Y_BASE
                       + HOLDER_REFERENCE_Y_PITCH * tool)
        # The operator moved the carriage while XY motors were disabled.
        # Contact with both holder and grab sensors is the only permitted
        # reference before re-establishing a temporary XY coordinate.
        self._run('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1')
        self._run('SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=1')
        self._run('SET_KINEMATIC_POSITION X=%.3f Y=%.3f SET_HOMED=xy' % (
            HOLDER_REFERENCE_X, y_reference))
        # Contact alone does not latch the tool. Grab it the same way
        # _pickup does, or the carriage pulls away while the tool stays
        # behind on the dock. 5mm was not enough clearance to reliably
        # release the dock sensor; reuse _pickup's own tuned slow-pullback
        # distance/speed instead of a smaller ad hoc value.
        self._run('MOTOR_GRAB')
        self._move(x=HOLDER_REFERENCE_X - self.pullback_slow_distance,
                  feed=self.pullback_speed * 60.)
        self._run('MOTOR_GRAB2')
        self._pause_ms(self.pickup_latch_wait_ms)
        self._run('M400')
        self._pause_ms(self.sensor_settle_ms)
        if (self._button(self.dock_buttons[tool], gcmd)
                or not self._button(self.grab_buttons[tool], gcmd)):
            raise gcmd.error('T%d did not leave its holder with the '
                             'carriage; calibration stopped' % tool)
        # Travel fast to just short of where the saved dock position puts
        # each endstop, then measure slowly from there. The margin stays
        # wider than the correction limit, so a holder that is still within
        # it can never be driven into at travel speed.
        old_x, old_y = self.docks[tool]
        approach = max(10., self.max_mount_correction + 5.)
        self._move(x=240., feed=12000)
        self._move(x=min(240., HOLDER_REFERENCE_X - old_x + approach),
                   feed=12000)
        x_result = x_home.measure_axis(-5.)
        self._move(y=min(y_reference, y_reference - old_y + approach),
                   feed=12000)
        y_result = y_home.measure_axis(0.)
        x = abs(HOLDER_REFERENCE_X - x_result)
        y = y_reference - y_result
        if not all(math.isfinite(value) for value in (x, y)):
            raise gcmd.error('Non-finite holder measurement')
        if (abs(x - old_x) > self.max_mount_correction
                or abs(y - old_y) > self.max_mount_correction):
            raise gcmd.error('T%d holder X=%.4f Y=%.4f exceeds the '
                             '%.2f mm correction limit; no automatic '
                             're-dock or save was attempted' % (
                                 tool, x, y, self.max_mount_correction))
        return x, y

    def cmd_extruder_position_calibrate(self, gcmd):
        tool = gcmd.get_int('T', None, minval=0, maxval=3)
        if tool is None:
            gcmd.respond_info('Select a holder: '
                              'EXTRUDER_POSITION_CALIBRATE T=0..3')
            return
        if not os.path.isfile(self.extruder_json_path):
            raise gcmd.error('Missing stock %s; refusing to calibrate'
                             % self.extruder_json_path)
        eventtime = self.printer.get_reactor().monotonic()
        print_stats = self.printer.lookup_object('print_stats', None)
        if (print_stats is not None and print_stats.get_status(eventtime).get(
                'state') in ('printing', 'paused')):
            raise gcmd.error('Extruder Position Calibrate is unavailable '
                             'during a print')
        for i in range(4):
            heater = self.printer.lookup_object(
                'extruder' if i == 0 else 'extruder%d' % i, None)
            if (heater is not None and heater.get_status(eventtime).get(
                    'temperature', 0.) > 50.):
                raise gcmd.error('Cool all toolheads below 50 C before '
                                 'manual holder calibration')
        dock, grab, attached = self._preflight(gcmd)
        if attached is not None or not all(dock) or any(grab):
            raise gcmd.error('Park all toolheads before holder calibration')
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_position()[2] < self.safe_z:
            raise gcmd.error('Raise Z above %.1f before holder calibration'
                             % self.safe_z)

        def calibrate():
            xy_disabled = False
            state_saved = False
            complete = False
            try:
                self._run('M400')
                self._run('SET_VELOCITY_LIMIT ACCEL=%.0f'
                          % self.position_calibration_accel)
                self._run('SAVE_GCODE_STATE NAME=C5_HOLDER_CALIBRATE')
                state_saved = True
                self._run('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
                xy_disabled = True
                self._run('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=0')
                self._run('SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=0')
                toolhead.get_kinematics().clear_homing_state('xy')
                gcmd.respond_info('Move the master carriage by hand to T%d '
                                  'until its holder and grab sensors both '
                                  'activate. Release your hand when asked.'
                                  % tool)
                self._wait_for_holder_contact(gcmd, tool)
                gcmd.respond_info('T%d detected. Release your hand; '
                                  'measurement starts in '
                                  '%.1f seconds.' % (
                                      tool,
                                      self.position_calibration_release_wait_ms
                                      / 1000.))
                self._pause_ms(self.position_calibration_release_wait_ms)
                if (not self._button(self.dock_buttons[tool], gcmd)
                        or not self._button(self.grab_buttons[tool], gcmd)):
                    raise gcmd.error('T%d holder contact was lost' % tool)
                if any(self._button(name, gcmd) for i, name in
                       enumerate(self.grab_buttons) if i != tool):
                    raise gcmd.error('Another toolhead also reports attached')
                self._check_chamber_doors(
                    gcmd, 'Close door %s before automatic holder '
                          'measurement while chamber heater is enabled')
                x, y = self._measure_holder_position(gcmd, tool)
                self._check_chamber_doors(
                    gcmd, 'Door %s opened during holder measurement; '
                          'no re-dock or save')
                # HDHOME returned measurements in the temporary frame.
                # Establish real XY coordinates before travelling to a dock.
                self._run('G28 X Y')
                previous_dock = self.docks[tool]
                self.docks[tool] = [x, y]
                try:
                    self._dock(gcmd, tool)
                    self._save_holder_position(gcmd, tool, x, y)
                except Exception:
                    self.docks[tool] = previous_dock
                    raise
                complete = True
            finally:
                try:
                    if xy_disabled and not complete:
                        try:
                            self._run('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1')
                        finally:
                            try:
                                self._run('SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=1')
                            finally:
                                toolhead.get_kinematics().clear_homing_state('xy')
                finally:
                    if state_saved and not complete:
                        self._run('RESTORE_GCODE_STATE NAME=C5_HOLDER_CALIBRATE MOVE=0')
        self._with_motion(gcmd, calibrate)

    # Fluidd/Mainsail prompt flow for offset calibration without arguments:
    # tool, then build plate removal, then (single tool) the levelboard
    # reference. Each answer is passed to the next step as a parameter.
    def _prompt(self, title, text, buttons=(), footer=()):
        lines = ['action:prompt_end', 'action:prompt_begin %s' % title]
        lines += ['action:prompt_text %s' % t for t in text]
        if buttons:
            lines.append('action:prompt_button_group_start')
            lines += ['action:prompt_button %s' % b for b in buttons]
            lines.append('action:prompt_button_group_end')
        lines += ['action:prompt_footer_button %s' % b for b in footer]
        lines.append('action:prompt_show')
        for line in lines:
            self.gcode.respond_raw('// ' + line)

    def _offset_wizard_step(self, gcmd, step, tool, levelboard=1):
        cancel = 'Cancel|C5_OFFSET_WIZARD STEP=cancel|error'
        if step == 'tool':
            self._prompt('Tool offset calibration',
                         ['Which tools should be calibrated?'],
                         ['All tools|C5_OFFSET_WIZARD STEP=plate TOOL=ALL|primary']
                         + ['T%d|C5_OFFSET_WIZARD STEP=plate TOOL=%d|secondary'
                            % (t, t) for t in range(4)],
                         [cancel])
            return
        if tool != 'ALL' and tool not in ('0', '1', '2', '3'):
            raise gcmd.error('TOOL must be ALL or 0..3')
        if step == 'home':
            self.gcode.respond_raw('// action:prompt_end')
            self._run('G28')
            step = 'plate'
        if step == 'plate':
            homed = self.printer.lookup_object('toolhead').get_status(
                self.printer.get_reactor().monotonic())['homed_axes']
            if tool == 'ALL' and not all(a in homed for a in 'xyz'):
                self._prompt('Home first',
                             ['All-tool calibration needs the printer homed. '
                              'Keep the build plate installed while it homes.'],
                             footer=[cancel, 'Home now|C5_OFFSET_WIZARD '
                                     'STEP=home TOOL=ALL|primary'])
                return
            label = 'all tools' if tool == 'ALL' else 'T' + tool
            nxt = ('STEP=run TOOL=ALL LEVELBOARD=1' if tool == 'ALL'
                   else 'STEP=probe TOOL=%s' % tool)
            self._prompt('Remove the build plate',
                         ['Calibrating %s. Remove the build plate now, then '
                          'confirm. The probe checks that it is really gone '
                          'before touching anything.' % label],
                         footer=[cancel, 'Plate removed|C5_OFFSET_WIZARD %s|primary'
                                 % nxt])
            return
        if step == 'probe':
            self._prompt('Probe offset',
                         ['Also recalibrate the probe (levelboard reference)? '
                          'Choose No to reuse the saved reference.'],
                         footer=['No|C5_OFFSET_WIZARD STEP=run TOOL=%s LEVELBOARD=0'
                                 % tool,
                                 'Yes|C5_OFFSET_WIZARD STEP=run TOOL=%s '
                                 'LEVELBOARD=1|primary' % tool])
            return
        if step == 'run':
            self.gcode.respond_raw('// action:prompt_end')
            if tool == 'ALL':
                self._run('C5_CALIBRATE_ALL_OFFSETS SAVE=1 BUILDPLATE_REMOVED=1')
            else:
                self._run('C5_TOOL_OFFSET_CALIBRATE T=%s Z=1 SAVE=1 '
                          'LEVELBOARD=%d BUILDPLATE_REMOVED=1'
                          % (tool, levelboard))
                self._run('C5_SAVE_OFFSETS_JSON')
            return
        raise gcmd.error('Unknown STEP %s' % step)

    def cmd_offset_wizard(self, gcmd):
        step = gcmd.get('STEP', 'tool').lower()
        if step == 'cancel':
            self.gcode.respond_raw('// action:prompt_end')
            gcmd.respond_info('Tool offset calibration cancelled')
            return
        self._offset_wizard_step(gcmd, step, gcmd.get('TOOL', '').upper(),
                                 gcmd.get_int('LEVELBOARD', 1, minval=0,
                                              maxval=1))

    def cmd_check_build_plate(self, gcmd):
        # ERROR=0 (print start): report a missing plate in the status
        # (build_plate_detected) instead of raising, so the print can be
        # cancelled rather than end in an error.
        raise_error = gcmd.get_int('ERROR', 1, minval=0, maxval=1)
        _, _, attached = self._preflight(gcmd)
        if attached is not None:
            raise gcmd.error('Dock T%d before the build plate check' % attached)
        self.build_plate_detected = None
        self._with_motion(gcmd, lambda: self._check_plate_present(
            gcmd, raise_error=bool(raise_error)))

    def cmd_move_to_purge_bucket(self, gcmd):
        # Called before heating to the full priming temperature so any ooze
        # during heat-soak lands in the purge bucket instead of wherever the
        # pickup left the toolhead (often over the bed).
        dock, grab, attached = self._preflight(gcmd)
        if attached is None:
            raise gcmd.error('No attached tool to move to the purge bucket')
        def move():
            self._raise_z()
            self._run('G90')
            self._move_to_prep_bucket()
        self._with_motion(gcmd, move)

    def cmd_purge(self, gcmd):
        length = gcmd.get_float('LENGTH', self.purge_length,
                                minval=0., maxval=80.)
        if not math.isfinite(length) or length <= 0.:
            raise gcmd.error('LENGTH must be finite and positive')
        dock, grab, attached = self._preflight(gcmd)
        if attached is None:
            raise gcmd.error('No attached tool for purge')
        self.cmd_verify_nozzle_z(gcmd)
        extruder = 'extruder' if attached == 0 else 'extruder%d' % attached
        heater = self.printer.lookup_object(extruder)
        if not heater.get_status(self.printer.get_reactor().monotonic()).get(
                'can_extrude'):
            raise gcmd.error('T%d is not hot enough to purge' % attached)
        def purge():
            # The carriage has one physical drive. These logical extruder
            # objects select the attached hotend's temperature/flow context.
            self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
            self._run('SAVE_GCODE_STATE NAME=C5_PURGE_PREP')
            self._raise_z()
            try:
                self._run('G90')
                self._move_to_prep_bucket()
                self._move(z=self.purge_z, feed=600, machine=False)
                self._run('M83')
                self._run('G1 E%.3f F%.0f' % (length,
                                              self.purge_speed * 60.))
                self._run('G1 E-5 F1200')
                self._run('M400')
            finally:
                try:
                    self._raise_z()
                finally:
                    self._run('RESTORE_GCODE_STATE NAME=C5_PURGE_PREP')
        gcmd.respond_info('T%d purging %.1f mm at X%.1f Y%.1f before cooldown'
                          % (attached, length, self.purge_x, self.purge_y))
        self._with_motion(gcmd, purge)

    def cmd_extruder_rotation_calibrate(self, gcmd):
        cancel = 'Cancel|C5_EXTRUDER_ROTATION_CALIBRATE CANCEL=1|error'
        if gcmd.get_int('CANCEL', 0, minval=0, maxval=1):
            pending = self.rotation_pending
            self.rotation_pending = None
            self.gcode.respond_raw('// action:prompt_end')
            gcmd.respond_info('Extruder rotation calibration cancelled')
            if pending is not None:
                self._end_rotation(pending['tool'], pending['temp'])
            return
        tool = gcmd.get_int('TOOL', None, minval=0, maxval=3)
        if tool is None:
            self._prompt('Extruder Rotation Calibration',
                         ['Which tool should be calibrated?'],
                         ['T%d|C5_EXTRUDER_ROTATION_CALIBRATE TOOL=%d|primary'
                          % (t, t) for t in range(4)],
                         [cancel])
            return
        remaining = gcmd.get_float('REMAINING', None, minval=0.)
        if remaining is not None:
            self._finish_manual_rotation(gcmd, tool, remaining)
            return
        if gcmd.get_int('MARKED', 0, minval=0, maxval=1):
            pending = self.rotation_pending
            if pending is None or pending['tool'] != tool:
                raise gcmd.error('Start with C5_EXTRUDER_ROTATION_CALIBRATE '
                                 'TOOL=%d' % tool)
            self.gcode.respond_raw('// action:prompt_end')
            try:
                pending['pulses'] = self._measure_rotation(
                    gcmd, tool, pending['length'], pending['speed'])
            except Exception:
                self.rotation_pending = None
                self._rotation_heater_off(tool)
                raise
            self._prompt('Extruder Rotation Calibration',
                         ['Measure from the inlet to the mark in mm, then run '
                          'C5_EXTRUDER_ROTATION_CALIBRATE TOOL=%d '
                          'REMAINING=<mm> within %.0f seconds'
                          % (tool, self.rotation_measure_timeout)],
                         footer=[cancel])
            self._start_rotation_timeout(pending)
            return
        length = gcmd.get_float('LENGTH', self.rotation_cal_length,
                                above=10., maxval=300.)
        speed = gcmd.get_float('SPEED', self.rotation_cal_speed,
                               above=0., maxval=10.)
        temp = gcmd.get_float('TEMP', self.rotation_cal_temp,
                              minval=170., maxval=320.)
        manual = (gcmd.get_int('MANUAL', 0, minval=0, maxval=1)
                  or self.wheel_mm_per_pulse is None)
        mark = gcmd.get_float('MARK', length + 20., above=length)
        self.rotation_pending = None
        try:
            self._prepare_rotation(gcmd, tool, temp)
        except Exception:
            self._rotation_heater_off(tool)
            raise
        if manual:
            self.rotation_pending = {'tool': tool, 'length': length,
                                     'speed': speed, 'mark': mark,
                                     'temp': temp}
            self._prompt('Extruder Rotation Calibration',
                         ['Mark the filament %.0f mm above where it enters '
                          'T%d, then press Extrude.' % (mark, tool)],
                         ['Extrude|C5_EXTRUDER_ROTATION_CALIBRATE TOOL=%d '
                          'MARKED=1|primary' % tool], [cancel])
            return
        try:
            pulses = self._measure_rotation(gcmd, tool, length, speed)
        except Exception:
            self._rotation_heater_off(tool)
            raise
        try:
            self._apply_rotation(gcmd, tool, length,
                                 pulses * self.wheel_mm_per_pulse, pulses)
        finally:
            self._end_rotation(tool, temp)

    def _end_rotation(self, tool, temp):
        try:
            self._run('C5_COOL_FOR_DOCK HOTEND=%.1f' % temp)
        finally:
            self._rotation_heater_off(tool)
        self._run('AFC_UNSELECT_TOOL')

    def _rotation_heater_off(self, tool):
        extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
        heater = self.printer.lookup_object(extruder).get_heater()
        self.printer.lookup_object('heaters').set_temperature(heater, 0.)

    def _start_rotation_timeout(self, pending):
        reactor = self.printer.get_reactor()

        def expire(eventtime):
            reactor.unregister_timer(timer)
            if self.rotation_pending is not pending:
                return reactor.NEVER
            self.rotation_pending = None
            tool = pending['tool']
            self._rotation_heater_off(tool)
            self.gcode.respond_raw('// action:prompt_end')
            self.gcode.respond_raw(
                '!! T%d extruder rotation calibration timed out waiting for '
                'REMAINING; heater turned off' % tool)
            return reactor.NEVER
        timer = reactor.register_timer(
            expire, reactor.monotonic() + self.rotation_measure_timeout)

    def _prepare_rotation(self, gcmd, tool, temp):
        homed = self.printer.lookup_object('toolhead').get_status(
            self.printer.get_reactor().monotonic()).get('homed_axes', '')
        if any(axis not in homed for axis in 'xyz'):
            self._run('G28')
        _, _, attached = self._preflight(gcmd)
        if attached != tool:
            self._run('T%d' % tool)
            _, _, attached = self._preflight(gcmd)
            if attached != tool:
                raise gcmd.error('T%d was not picked up' % tool)
        extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
        bed = self.printer.lookup_object('heater_bed').get_status(
            self.printer.get_reactor().monotonic())['target']
        self._run('C5_AUTO_NOZZLE_Z T=%d HOTEND=%.1f BED=%.1f '
                  'FIRST_LAYER_HEIGHT=0.2' % (tool, temp, bed))
        afc_extruder = self._afc_extruder(extruder)
        start = getattr(afc_extruder, 'fila_tool_start', None)
        if start is None:
            start = self.printer.lookup_object(
                'filament_switch_sensor %s_tool_start' % extruder, None)
        if start is not None and not start.get_status(None).get(
                'filament_detected', True):
            raise gcmd.error('No filament loaded in T%d' % tool)
        self.cmd_move_to_purge_bucket(gcmd)
        self._run('SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.0f'
                  % (extruder, temp))
        self._run('TEMPERATURE_WAIT SENSOR=%s MINIMUM=%.0f'
                  % (extruder, temp - 2.))

        def prime():
            self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
            self._run('SAVE_GCODE_STATE NAME=C5_ROTATION_PREP')
            try:
                self._move(z=self.purge_z, feed=600, machine=False)
                self._run('M83')
                self._run('G1 E5 F%.0f' % (self.purge_speed * 60.))
                self._run('M400')
            finally:
                self._run('RESTORE_GCODE_STATE NAME=C5_ROTATION_PREP MOVE=0')
        self._with_motion(gcmd, prime)

    def _measure_rotation(self, gcmd, tool, length, speed):
        _, _, attached = self._preflight(gcmd)
        if attached != tool:
            raise gcmd.error('T%d is no longer attached' % tool)
        extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
        if not self.printer.lookup_object(extruder).get_status(
                self.printer.get_reactor().monotonic()).get('can_extrude'):
            raise gcmd.error('T%d is not hot enough to extrude' % tool)
        wheel = self.printer.lookup_object(
            'filament_motion_sensor fm_ex%d' % tool)

        def measure():
            self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
            self._run('SAVE_GCODE_STATE NAME=C5_ROTATION_CAL')
            try:
                self._run('M83')
                self._run('M400')
                self._pause_ms(500)
                start = wheel.pulse_count
                self._run('G1 E%.3f F%.0f' % (length, speed * 60.))
                self._run('M400')
                self._pause_ms(500)
                return wheel.pulse_count - start
            finally:
                try:
                    self._raise_z()
                finally:
                    self._run('RESTORE_GCODE_STATE NAME=C5_ROTATION_CAL MOVE=0')
        gcmd.respond_info('T%d extruding %.1f mm at %.1f mm/s'
                          % (tool, length, speed))
        pulses = self._with_motion(gcmd, measure)
        if pulses < 10:
            raise gcmd.error('T%d wheel saw only %d pulses; check that the '
                             'filament is moving through the sensor'
                             % (tool, pulses))
        return pulses

    def _finish_manual_rotation(self, gcmd, tool, remaining):
        pending = self.rotation_pending
        if (pending is None or pending['tool'] != tool
                or 'pulses' not in pending):
            raise gcmd.error('No measured T%d extrusion to finish' % tool)
        actual = pending['mark'] - remaining
        if actual <= 0.:
            raise gcmd.error('REMAINING must be less than the %.1f mm mark'
                             % pending['mark'])
        self.gcode.respond_raw('// action:prompt_end')
        self.rotation_pending = None
        self.wheel_mm_per_pulse = actual / pending['pulses']
        self.configfile.set(self.section, 'wheel_mm_per_pulse',
                            '%.6f' % self.wheel_mm_per_pulse)
        gcmd.respond_info('Wheel: %.6f mm per pulse' % self.wheel_mm_per_pulse)
        try:
            self._apply_rotation(gcmd, tool, pending['length'], actual,
                                 pending['pulses'], all_tools=True)
        finally:
            self._end_rotation(tool, pending['temp'])

    def _apply_rotation(self, gcmd, tool, requested, measured, pulses,
                        all_tools=False):
        ratio = measured / requested
        if not .7 <= ratio <= 1.3:
            raise gcmd.error('T%d moved %.1f mm of %.1f mm requested; check '
                             'for slipping or a wrong wheel_mm_per_pulse'
                             % (tool, measured, requested))
        extruder = 'extruder' if tool == 0 else 'extruder%d' % tool
        stepper = self.printer.lookup_object(extruder).extruder_stepper.stepper
        old = stepper.get_rotation_distance()[0]
        new = old * ratio
        lines = ['T%d requested %.1f mm, wheel measured %.1f mm (%d pulses, '
                 '%.1f%%)' % (tool, requested, measured, pulses, ratio * 100.)]
        for index in (range(4) if all_tools else (tool,)):
            name = 'extruder' if index == 0 else 'extruder%d' % index
            previous = self.printer.lookup_object(
                name).extruder_stepper.stepper.get_rotation_distance()[0]
            self._run('SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=%s '
                      'DISTANCE=%.4f' % (name, new))
            self.configfile.set(name, 'rotation_distance', '%.4f' % new)
            lines.append('T%d rotation_distance %.4f -> %.4f'
                         % (index, previous, new))
        lines.append('Run SAVE_CONFIG to keep it')
        gcmd.respond_info('\n'.join(lines))

    def cmd_flow_strokes(self, gcmd):
        """Run the stock alternating-speed eboard PA/flow measurement."""
        _, _, attached = self._preflight(gcmd)
        if attached is None:
            raise gcmd.error('No attached tool for flow calibration')
        self.cmd_verify_nozzle_z(gcmd)
        extruder = 'extruder' if attached == 0 else 'extruder%d' % attached
        eventtime = self.printer.get_reactor().monotonic()
        heater = self.printer.lookup_object(extruder)
        heater_status = heater.get_status(eventtime)
        if not heater_status.get('can_extrude'):
            raise gcmd.error('T%d is not hot enough for flow calibration'
                             % attached)
        pa = self.printer.lookup_object('pa_adjust', None)
        if pa is None:
            raise gcmd.error('Flow calibration requires [pa_adjust] on eboard')
        gcmd.respond_info('Starting T%d flow calibration strokes'
                          % attached)
        old_pa = heater_status.get('pressure_advance', 0.)
        candidates = (.0100, .0200, .0150, .0350, .0250, .0300, .0400)
        minimum_candidate = min(candidates)
        # The eboard classifies the *motor-current waveform*, not a bead on
        # the plate.  Klipper only applies pressure advance to moves with
        # XY motion, so like the stock firmware the strokes are XY moves,
        # run with the XY steppers detached (held, not stepping) while the
        # nozzle stays over the bucket.
        slow_feed = self.flow_slow_speed * 60.
        fast_feed = self.flow_fast_speed * 60.
        segments = ((60., slow_feed, 1.13573), (100., fast_feed, 2.27146),
                    (120., slow_feed, 1.13573), (140., slow_feed, 1.13573),
                    (180., fast_feed, 2.27146), (200., slow_feed, 1.13573))

        def measure():
            self._run('ACTIVATE_EXTRUDER EXTRUDER=%s' % extruder)
            self._run('SAVE_GCODE_STATE NAME=C5_FLOW_TEST')
            picks = []
            calibrated = False
            paused = []
            locked = None
            try:
                self._run('G90')
                self._run('M83')
                self._run('G92 E0')
                self._move_to_prep_bucket()
                self._move(z=self.flow_test_z, feed=1200, machine=False)
                self._run('M400')
                self._enable_extruder_driver(extruder)
                paused = self._pause_extruder_tmc_checks()
                locked = self._lock_xy()
                self._run('SET_VELOCITY_LIMIT ACCEL=%.0f '
                          'SQUARE_CORNER_VELOCITY=9' % self.flow_test_accel)
                for repeat in range(5):
                    successful = []
                    for index, advance in enumerate(candidates):
                        self._run('SET_PRESSURE_ADVANCE EXTRUDER=%s '
                                  'ADVANCE=%.4f' % (extruder, advance))
                        self._run('G1 X40 Y%.1f F%.0f'
                                  % (50. + 5. * index, fast_feed))
                        pa.pa_action(11, 666)
                        try:
                            for x, feed, amount in segments:
                                self._run('G1 X%.1f E%.5f F%.0f'
                                          % (x, amount, feed))
                        finally:
                            self._run('M400')
                            pa.pa_action(0, 666)
                            # The eboard computes the verdict in a task
                            # after PA_ACTION=0; do not query in that same
                            # command turnaround.
                            self._pause_ms(self.flow_verdict_settle_ms)
                        if pa.pa_get_value() == 9:
                            successful.append(advance)
                            gcmd.respond_info(
                                'T%d flow valid: PA %.4f (pass %d)'
                                % (attached, advance, repeat + 1))
                            # The remaining candidates cannot improve this
                            # pass once the lowest PA has been accepted.
                            if advance == minimum_candidate:
                                break
                    if successful:
                        picks.append(min(successful))
                    if len(picks) >= 3:
                        break
                if len(picks) >= 3:
                    result = sum(picks[:3]) / 3.
                    self._run('SET_PRESSURE_ADVANCE EXTRUDER=%s '
                              'ADVANCE=%.5f' % (extruder, result))
                    calibrated = True
                    gcmd.respond_info('T%d bucket flow test: pressure advance '
                                      '%.5f' % (attached, result))
                else:
                    gcmd.respond_info('T%d bucket flow test: fewer than three '
                                      'valid eboard readings; pressure '
                                      'advance unchanged' % attached)
            finally:
                try:
                    pa.pa_action(0, 666)
                    self._unlock_xy(locked)
                    self._raise_z()
                    if not calibrated:
                        self._run('SET_PRESSURE_ADVANCE EXTRUDER=%s '
                                  'ADVANCE=%.5f' % (extruder, old_pa))
                finally:
                    try:
                        self._run('RESTORE_GCODE_STATE NAME=C5_FLOW_TEST '
                                  'MOVE=0')
                    finally:
                        self._resume_tmc_checks(paused)

        self._with_motion(gcmd, measure)

    def cmd_cool_for_dock(self, gcmd):
        hotend = gcmd.get_float('HOTEND', above=0.)
        if not math.isfinite(hotend):
            raise gcmd.error('HOTEND must be finite')
        _, _, attached = self._preflight(gcmd)
        if attached is None:
            raise gcmd.error('No attached tool to cool before docking')
        self.cmd_verify_nozzle_z(gcmd)
        extruder = 'extruder' if attached == 0 else 'extruder%d' % attached
        target = max(0., hotend - 100.)
        reactor = self.printer.get_reactor()
        heater = self.printer.lookup_object(extruder)
        # Leave the bucket through the inboard waypoint before moving to the
        # front cooldown position; do not sweep across parked toolheads.
        self._run('SAVE_GCODE_STATE NAME=C5_COOLDOWN_TRAVEL')
        at_wiper = False
        try:
            self._run('G90')
            self._raise_z()
            self._move(x=self.prep_approach_x,
                       y=self.prep_approach_y, feed=6000, machine=False)
            self._move(y=self.cooldown_y, feed=6000, machine=False)
            self._move(x=self.cooldown_x, feed=3000, machine=False)
            # Lower only at the wiper, never while crossing the bed or docks.
            at_wiper = True
            self._move(z=self.cooldown_wipe_z, feed=600, machine=False)
            # fanM106 is the eboard-connected toolhead part-cooling fan.
            self._run('SET_FAN_SPEED FAN=fanM106 SPEED=%.3f'
                      % self.cooldown_fan_speed)
            # Stock clearNozzlePrint cools to operation temperature minus
            # 100 C (220 C -> 120 C), with a 180-second timeout.
            self._run('SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.1f'
                      % (extruder, target))
            deadline = reactor.monotonic() + 180.
            while True:
                now = reactor.monotonic()
                actual = heater.get_status(now).get('temperature')
                if actual is None or not math.isfinite(actual):
                    raise gcmd.error('Cannot verify T%d temperature' % attached)
                if actual <= target + 3.:
                    break
                if now >= deadline:
                    self._run('TURN_OFF_HEATERS')
                    raise gcmd.error('T%d did not cool to %.1f C before '
                                     'docking' % (attached, target))
                reactor.pause(min(deadline, now + 1.))
        finally:
            try:
                if at_wiper:
                    self._move(z=self.cooldown_lift_z, feed=600,
                               machine=False)
            finally:
                try:
                    self._run('SET_FAN_SPEED FAN=fanM106 SPEED=0')
                finally:
                    self._run('RESTORE_GCODE_STATE NAME=C5_COOLDOWN_TRAVEL '
                              'MOVE=0')

    def get_status(self, eventtime=None):
        cached = self._ui_sensor_cache
        if (eventtime is not None and cached is not None
                and 0. <= eventtime - cached[0] < 1.):
            active_tool, sensor_error = cached[1:]
        else:
            try:
                active_tool = self.attached_tool_from_pins()
                sensor_error = None
            except Exception as exc:
                active_tool = None
                sensor_error = str(exc)
            if eventtime is not None:
                self._ui_sensor_cache = (eventtime, active_tool, sensor_error)
        return {'active_tool': active_tool, 'sensor_error': sensor_error,
                'busy': self.busy,
                'build_plate_detected': self.build_plate_detected,
                'dock_positions': [list(p) for p in self.docks],
                'tool_measurements': [list(p) for p in self.measurements]}


def load_config(config):
    return Creator5Toolchanger(config)
