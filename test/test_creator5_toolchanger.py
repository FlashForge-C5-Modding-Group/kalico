import importlib.util
import json
import os
import tempfile
import sys
import types
import unittest
from unittest import mock
from pathlib import Path

EXTRAS_DIR = Path(__file__).resolve().parents[1] / 'klippy' / 'extras'
extras_package = types.ModuleType('klippy.extras')
extras_package.__path__ = [str(EXTRAS_DIR)]
with mock.patch.dict(sys.modules, {
        'klippy': types.ModuleType('klippy'),
        'klippy.extras': extras_package,
        'klippy.extras.filament_switch_sensor': types.ModuleType(
            'klippy.extras.filament_switch_sensor')}):
    sensor_spec = importlib.util.spec_from_file_location(
        'klippy.extras.filament_motion_sensor',
        EXTRAS_DIR / 'filament_motion_sensor.py')
    filament_motion_sensor = importlib.util.module_from_spec(sensor_spec)
    sensor_spec.loader.exec_module(filament_motion_sensor)


MODULE_PATH = Path(__file__).resolve().parents[1] / 'klippy' / 'extras' / 'creator5_toolchanger.py'
SPEC = importlib.util.spec_from_file_location('creator5_toolchanger', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **params):
        self.params = params
        self.messages = []

    def get_int(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def get_float(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def get(self, name, default=None):
        return self.params.get(name, default)

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.message = message
        self.messages.append(message)


class Creator5OffsetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = os.path.join(self.temp.name, 'extruder.json')
        with open(self.path, 'w') as target:
            target.write('{"now_extruder": -1, "t0_offset_x": 16.25, '
                         '"z_station_pos": -1.5}\n/* stock comment */\n')
        self.board = MODULE.Creator5Toolchanger.__new__(
            MODULE.Creator5Toolchanger)
        self.board.extruder_json_path = self.path
        self.board.test_json_path = os.path.join(self.temp.name, 'test.json')
        with open(self.board.test_json_path, 'w') as target:
            target.write('{"generalFirmware": false, "tempOffset": 0.00045, '
                         '"cylinder_x": 28.5, "cylinder_y": 214.5}\n')
        self.board.measurements = [[15., 215., 1.] for _ in range(4)]
        self.board.docks = [[298., 50. + i * 50.] for i in range(4)]
        self.board.station_x = 29.
        self.board.station_y = 214.
        self.board.station_z = -1.
        self.board.plate_check_x = 43.
        self.board.plate_check_y = 226.
        self.board.plate_check_min_delta = .8
        self.board._stock_pin_approach_z = None
        self.board.print_z_baseline = [None] * 4
        self.board._button_objects = {}
        self.board._ui_sensor_cache = None
        self.board.busy = False
        self.board.toolchange_sensor_settle_ms = 0
        self.board.pullback_slow_distance = 20.
        self.board.pickup_departure_speed = 80.
        self.board.clear_travel_speed = 600.
        self.board._tool_pa_memory = {}
        self.board._restore_pressure_advance = mock.Mock()
        self.board.post_select_accel = None
        self.board.flow_verdict_settle_ms = 50
        self.board.lower_bed_speed = 10.
        self.board.offset_calibration_lift = 3.
        self.board.offset_calibration_lift_speed = 10.
        self.board.safe_z = 10.
        self.board.scan_spans = {
            ('X', 1): 7., ('Y', 1): 7.,
            ('X', -1): 7., ('Y', -1): 7.}
        self.board._g28_passthrough = False
        self.board._offset_batch_active = False
        self.board.clog_switch = mock.Mock(enabled=True)
        self.board.printer = mock.Mock()
        self.board.printer.lookup_object.return_value.get_status.return_value = {
            'homing_origin': (.1, .2, .3, 0.)}

    def test_wheel_clog_pauses_even_with_infinite_spool_replacement(self):
        afc = mock.Mock(current='extruder1', in_toolchange=False,
                        error_state=False)
        afc.function.is_printing.return_value = True
        lane = mock.Mock(runout_lane='extruder3')
        lane.name = 'extruder1'
        lane.extruder_obj.creator5_tool_index = 1
        lane.extruder_obj.tool_start_state = True
        lane._is_normal_printing_state.return_value = True
        lane.extruder_obj.is_standalone.return_value = True
        afc.lanes = {'extruder1': lane}
        self.board.printer.lookup_object.return_value = afc
        self.board.attached_tool_from_pins = mock.Mock(return_value=1)

        self.board.cmd_wheel_runout(GCmd(TOOL=1))

        lane._perform_infinite_runout.assert_not_called()
        self.assertIn('clog detected',
                      afc.error.AFC_error.call_args.args[0])

    def test_wheel_runout_pauses_without_selected_replacement(self):
        afc = mock.Mock(current='extruder1', in_toolchange=False,
                        error_state=False)
        afc.function.is_printing.return_value = True
        lane = mock.Mock(runout_lane=None)
        lane.name = 'extruder1'
        lane.extruder_obj.creator5_tool_index = 1
        lane.extruder_obj.tool_start_state = True
        lane._is_normal_printing_state.return_value = True
        afc.lanes = {'extruder1': lane}
        self.board.printer.lookup_object.return_value = afc
        self.board.attached_tool_from_pins = mock.Mock(return_value=1)

        self.board.cmd_wheel_runout(GCmd(TOOL=1))

        afc.error.AFC_error.assert_called_once()
        lane._perform_infinite_runout.assert_not_called()

    def test_inactive_wheel_cannot_interrupt_current_tool(self):
        afc = mock.Mock(current='extruder1', in_toolchange=False,
                        error_state=False)
        afc.function.is_printing.return_value = True
        lane = mock.Mock(runout_lane='extruder3')
        lane.name = 'extruder2'
        afc.lanes = {'extruder2': lane}
        self.board.printer.lookup_object.return_value = afc
        self.board.attached_tool_from_pins = mock.Mock(return_value=1)

        self.board.cmd_wheel_runout(GCmd(TOOL=2))

        lane._perform_infinite_runout.assert_not_called()
        afc.error.AFC_error.assert_not_called()

    def test_wheel_failure_with_absent_filament_defers_to_afc_runout(self):
        afc = mock.Mock(current='extruder1', in_toolchange=False,
                        error_state=False)
        afc.function.is_printing.return_value = True
        lane = mock.Mock(runout_lane='extruder3')
        lane.name = 'extruder1'
        lane.extruder_obj.creator5_tool_index = 1
        lane.extruder_obj.tool_start_state = False
        lane._is_normal_printing_state.return_value = True
        afc.lanes = {'extruder1': lane}
        self.board.printer.lookup_object.return_value = afc
        self.board.attached_tool_from_pins = mock.Mock(return_value=1)

        self.board.cmd_wheel_runout(GCmd(TOOL=1))

        afc.error.AFC_error.assert_not_called()
        lane._perform_infinite_runout.assert_not_called()

    def test_clog_detection_off_ignores_wheel_failure(self):
        self.board.clog_switch.enabled = False
        self.board.printer.lookup_object = mock.Mock()

        self.board.cmd_wheel_runout(GCmd(TOOL=1))

        self.board.printer.lookup_object.assert_not_called()

    def test_clog_arms_only_selected_wheel_and_resets_threshold(self):
        sensors = [mock.Mock() for _ in range(4)]
        for sensor in sensors:
            sensor.runout_helper.sensor_enabled = True
        self.board.printer.lookup_object.side_effect = (
            lambda name: sensors[int(name[-1])])

        self.board._sync_clog_sensors(2)

        self.assertEqual([sensor.runout_helper.sensor_enabled
                          for sensor in sensors], [False, False, True, False])
        sensors[2]._update_filament_runout_pos.assert_called_once_with()
        self.assertTrue(sensors[2].runout_helper.filament_present)
        self.board.clog_switch.enabled = False
        self.board._sync_clog_sensors(2)
        self.assertFalse(any(sensor.runout_helper.sensor_enabled
                             for sensor in sensors))

    def test_clog_misc_switch_persists_and_resyncs_sensors(self):
        printer, gcode = mock.Mock(), mock.Mock()
        printer.lookup_object.return_value = mock.Mock(
            allVariables={'c5_clog_detection': 1})
        on_change = mock.Mock()
        switch = MODULE.Creator5MiscSwitch(
            printer, gcode, 'clog_detection', False, on_change)
        switch._handle_ready()
        self.assertTrue(switch.enabled)
        on_change.assert_called_once_with()
        switch.cmd_set(GCmd(ENABLE=0))
        self.assertFalse(switch.enabled)
        self.assertEqual(on_change.call_count, 2)
        gcode.run_script_from_command.assert_called_once_with(
            'SAVE_VARIABLE VARIABLE=c5_clog_detection VALUE=0')

    def test_reset_filament_sensor_does_not_emit_insert_event(self):
        sensor = filament_motion_sensor.EncoderSensor.__new__(
            filament_motion_sensor.EncoderSensor)
        sensor.extruder = mock.Mock()
        sensor._update_filament_runout_pos = mock.Mock()
        sensor.runout_helper = mock.Mock(filament_present=False)

        sensor.cmd_RESET_FILAMENT_SENSOR(GCmd())

        sensor._update_filament_runout_pos.assert_called_once_with()
        self.assertTrue(sensor.runout_helper.filament_present)
        sensor.runout_helper.note_filament_present.assert_not_called()

    def test_wheel_runout_pauses_if_mount_state_is_invalid(self):
        afc = mock.Mock(current='extruder1', in_toolchange=False,
                        error_state=False)
        afc.function.is_printing.return_value = True
        lane = mock.Mock(runout_lane='extruder3')
        lane.name = 'extruder1'
        lane.extruder_obj.creator5_tool_index = 1
        lane._is_normal_printing_state.return_value = True
        afc.lanes = {'extruder1': lane}
        self.board.printer.lookup_object.return_value = afc
        self.board.attached_tool_from_pins = mock.Mock(
            side_effect=RuntimeError('grab switch conflict'))

        self.board.cmd_wheel_runout(GCmd(TOOL=1))

        afc.error.AFC_error.assert_called_once()
        lane._perform_infinite_runout.assert_not_called()

    def test_flow_calibration_misc_switch_controls_live_state(self):
        printer, gcode = mock.Mock(), mock.Mock()
        switch = MODULE.Creator5FlowSwitch(printer, gcode, True)
        printer.add_object.assert_called_once_with(
            'filament_switch_sensor flow_calibration', switch)
        self.assertTrue(switch.get_status(0.)['enabled'])
        switch.cmd_set(GCmd(ENABLE=0))
        self.assertFalse(switch.get_status(0.)['enabled'])
        self.assertFalse(switch.get_status(0.)['filament_detected'])
        switch.cmd_set(GCmd(ENABLE=1))
        self.assertTrue(switch.get_status(0.)['enabled'])
        self.assertEqual(gcode.register_mux_command.call_count, 2)

    def test_purge_misc_switch_controls_live_state(self):
        printer, gcode = mock.Mock(), mock.Mock()
        switch = MODULE.Creator5MiscSwitch(printer, gcode, 'purge', True)
        printer.add_object.assert_called_once_with(
            'filament_switch_sensor purge', switch)
        self.assertTrue(switch.get_status(0.)['enabled'])
        command = GCmd(ENABLE=0)
        switch.cmd_set(command)
        self.assertFalse(switch.get_status(0.)['enabled'])
        self.assertFalse(switch.get_status(0.)['filament_detected'])
        self.assertEqual(command.message, 'Creator 5 purge disabled')
        switch.cmd_query(command)
        self.assertEqual(command.message, 'Creator 5 purge disabled')
        self.assertEqual(gcode.register_mux_command.call_count, 2)

    def test_lower_bed_on_end_misc_switch_starts_disabled(self):
        printer, gcode = mock.Mock(), mock.Mock()
        switch = MODULE.Creator5MiscSwitch(
            printer, gcode, 'lower_bed_on_end', False)
        printer.add_object.assert_called_once_with(
            'filament_switch_sensor lower_bed_on_end', switch)
        self.assertFalse(switch.get_status(0.)['enabled'])
        switch.cmd_set(GCmd(ENABLE=1))
        self.assertTrue(switch.get_status(0.)['enabled'])

    def test_misc_switch_restores_and_saves_mainsail_changes(self):
        printer, gcode = mock.Mock(), mock.Mock()
        printer.lookup_object.return_value = mock.Mock(
            allVariables={'c5_purge': 0})
        switch = MODULE.Creator5MiscSwitch(printer, gcode, 'purge', True)
        switch._handle_ready()
        self.assertFalse(switch.enabled)
        switch.cmd_set(GCmd(ENABLE=1))
        self.assertTrue(switch.enabled)
        gcode.run_script_from_command.assert_called_once_with(
            'SAVE_VARIABLE VARIABLE=c5_purge VALUE=1')
        switch.cmd_set(GCmd(ENABLE=1))
        gcode.run_script_from_command.assert_called_once()

    def test_misc_switch_does_not_change_when_save_fails(self):
        printer, gcode = mock.Mock(), mock.Mock()
        printer.lookup_object.return_value = mock.Mock(allVariables={})
        gcode.run_script_from_command.side_effect = RuntimeError('disk full')
        switch = MODULE.Creator5MiscSwitch(
            printer, gcode, 'lower_bed_on_end', False)
        with self.assertRaisesRegex(RuntimeError, 'disk full'):
            switch.cmd_set(GCmd(ENABLE=1))
        self.assertFalse(switch.enabled)

    def test_flow_macro_commands_use_persistent_switch(self):
        self.board.flow_switch = mock.Mock()
        self.board.cmd_flow_on(GCmd())
        self.board.cmd_flow_off(GCmd())
        self.assertEqual(
            self.board.flow_switch.set_enabled.call_args_list,
            [mock.call(True), mock.call(False)])

    def test_lower_bed_uses_machine_z_limit(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xyz', 'axis_maximum': mock.Mock(z=270.)}
        toolhead.get_position.return_value = [10., 20., 100., 0.]
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()

        self.board.cmd_lower_bed(GCmd())

        self.board._move.assert_called_once_with(z=256., feed=600.)

    def test_lower_bed_requires_homed_z_and_skips_at_max(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xy', 'axis_maximum': mock.Mock(z=270.)}
        toolhead.get_position.return_value = [10., 20., 100., 0.]
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()

        with self.assertRaisesRegex(RuntimeError, 'Home Z'):
            self.board.cmd_lower_bed(GCmd())
        self.board._move.assert_not_called()

        toolhead.get_status.return_value['homed_axes'] = 'xyz'
        toolhead.get_position.return_value = [10., 20., 256., 0.]
        self.board.cmd_lower_bed(GCmd())
        self.board._move.assert_not_called()

    def test_open_door_blocks_only_with_chamber_heater_active(self):
        heater = mock.Mock()
        heaters = mock.Mock()
        heaters.lookup_heater.return_value = heater
        self.board.printer.lookup_object.return_value = heaters
        self.board.door_buttons = ['topDoor', 'frontDoor']
        # PRESSED means closed; topDoor reads not-pressed here, i.e. open.
        self.board._button = lambda name, gcmd: name != 'topDoor'
        heater.get_status.return_value = {'target': 0., 'power': 0.}
        self.board._check_chamber_doors(GCmd(), 'Door %s is open')
        heater.get_status.return_value = {'target': 40., 'power': 0.}
        with self.assertRaisesRegex(RuntimeError, 'Door topDoor is open'):
            self.board._check_chamber_doors(GCmd(), 'Door %s is open')
        heater.get_status.return_value = {'target': 0., 'power': 0.5}
        with self.assertRaisesRegex(RuntimeError, 'Door topDoor is open'):
            self.board._check_chamber_doors(GCmd(), 'Door %s is open')

    def test_unknown_chamber_heater_state_does_not_bypass_open_doors(self):
        self.board.door_buttons = ['topDoor']
        self.board.printer.lookup_object.return_value = None
        with self.assertRaisesRegex(RuntimeError,
                                    'Cannot verify chamber heater state'):
            self.board._check_chamber_doors(GCmd(), 'Door %s is open')

    def test_attached_tool_and_status_use_only_dock_grab_pins(self):
        states = {'dock0': 'PRESSED', 'dock1': 'PRESSED',
                  'dock2': 'RELEASED', 'dock3': 'PRESSED',
                  'grab0': 'RELEASED', 'grab1': 'RELEASED',
                  'grab2': 'PRESSED', 'grab3': 'RELEASED'}
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        self.board.printer.command_error = RuntimeError
        self.board.printer.lookup_object.side_effect = lambda name, default=None: (
            mock.Mock(get_status=lambda: {'state': states[name.split()[-1]]})
            if name.startswith('gcode_button ') else default)
        self.board.active = 1  # Cached state must not override the pins.
        self.assertEqual(self.board.attached_tool_from_pins(), 2)
        self.assertEqual(self.board.get_status()['active_tool'], 2)
        states['grab0'] = 'PRESSED'
        with self.assertRaisesRegex(RuntimeError, 'Conflicting dock/grab'):
            self.board.attached_tool_from_pins()
        self.assertIsNone(self.board.get_status()['active_tool'])
        self.assertIn('Conflicting dock/grab',
                      self.board.get_status()['sensor_error'])

    def test_ui_status_polling_is_bounded_but_safety_reads_live_pins(self):
        states = {'dock0': 'RELEASED', 'dock1': 'PRESSED',
                  'dock2': 'PRESSED', 'dock3': 'PRESSED',
                  'grab0': 'PRESSED', 'grab1': 'RELEASED',
                  'grab2': 'RELEASED', 'grab3': 'RELEASED'}
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        self.board.printer.command_error = RuntimeError
        self.board.printer.lookup_object.side_effect = (
            lambda name, default=None: mock.Mock(
                get_status=lambda: {'state': states[name.split()[-1]]})
            if name.startswith('gcode_button ') else default)
        self.assertEqual(self.board.get_status(1.)['active_tool'], 0)
        self.assertEqual(self.board.printer.lookup_object.call_count, 8)
        states['grab0'] = 'RELEASED'
        self.assertEqual(self.board.get_status(1.1)['active_tool'], 0)
        self.assertEqual(self.board.printer.lookup_object.call_count, 8)
        with self.assertRaisesRegex(RuntimeError, 'Missing dock'):
            self.board.attached_tool_from_pins()
        self.assertIsNone(self.board.get_status(2.1)['active_tool'])
        self.assertEqual(self.board.printer.lookup_object.call_count, 8)

    def test_dock_speeds_and_release_wait_come_from_config(self):
        board = self.board
        board.approach_x = 250.
        board.clear_travel_speed = 600.
        board.dock_approach_speed = 25.
        board.dock_predock_offset = 20.
        board.departure_speed = 90.
        board.release_latch_wait_ms = 50
        board._move = mock.Mock()
        board._run = mock.Mock()
        board._pause_ms = mock.Mock()
        board._verify = mock.Mock()
        board.z_adjustments = [0.] * 4

        board._dock(GCmd(), 0, raise_z=False)

        self.assertEqual(board._move.call_args_list, [
            mock.call(x=250., feed=36000.),
            mock.call(y=50., feed=36000.),
            mock.call(x=278., feed=36000.),
            mock.call(x=288., feed=1500.),
            mock.call(x=298., feed=1500.),
            mock.call(x=250., feed=5400.),
        ])
        board._pause_ms.assert_called_once_with(50)
        board._verify.assert_called_once_with(mock.ANY, None, parked=0)
        self.assertEqual(board._run.call_args_list[0], mock.call(
            'SET_GCODE_OFFSET X=0.0000 Y=0.0000 Z=2.0000 MOVE=0'))
        board._run.assert_any_call('SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
        self.assertFalse(any(' Z=0 ' in call.args[0] for call in
                             board._run.call_args_list))

    def test_pickup_speeds_come_from_config(self):
        board = self.board
        board.approach_x = 250.
        board.pre_dock_x = 280.
        board.pullback = 20.
        board.clear_travel_speed = 600.
        board.pickup_predock_speed = 45.
        board.pickup_latch_speed = 25.
        board.pullback_speed = 70.
        board.departure_speed = 90.
        board.pickup_departure_speed = 90.
        board.pickup_latch_wait_ms = 1800
        board.grab_buttons = ['grab0']
        board._button = mock.Mock(return_value=True)
        board._raise_z = mock.Mock()
        board._move = mock.Mock()
        board._run = mock.Mock()
        board._pause_ms = mock.Mock()
        board._verify = mock.Mock()
        board._apply_offsets = mock.Mock()

        board._pickup(GCmd(), 0)

        self.assertEqual(board._move.call_args_list, [
            mock.call(x=250., feed=36000.),
            mock.call(y=50., feed=36000.),
            mock.call(x=280., feed=2700.),
            mock.call(x=298., feed=1500.),
            mock.call(x=278., feed=4200.),
            mock.call(x=250., feed=5400.),
        ])
        board._pause_ms.assert_called_once_with(1800)

    def test_pickup_uses_fast_travel_only_after_latch_clearance(self):
        board = self.board
        board.approach_x, board.pre_dock_x = 250., 280.
        board.pullback, board.pullback_slow_distance = 60., 20.
        board.clear_travel_speed = 600.
        board.pickup_predock_speed = board.pickup_latch_speed = 60.
        board.pullback_speed = 80.
        board.pickup_departure_speed = 600.
        board.pickup_latch_wait_ms = 0
        board.grab_buttons = ['grab0']
        board._button = mock.Mock(return_value=True)
        board._raise_z = mock.Mock()
        board._move = mock.Mock()
        board._run = mock.Mock()
        board._pause_ms = mock.Mock()
        board._verify = mock.Mock()
        board._apply_offsets = mock.Mock()
        activation_move_counts = []
        board._run.side_effect = lambda command: (
            activation_move_counts.append(board._move.call_count)
            if command.startswith('ACTIVATE_EXTRUDER') else None)
        board._pickup(GCmd(), 0)
        self.assertEqual(board._move.call_args_list[-3:], [
            mock.call(x=278., feed=4800.),
            mock.call(x=238., feed=36000.),
            mock.call(x=250., feed=36000.)])
        board._verify.assert_called_once_with(mock.ANY, 0)
        self.assertEqual(activation_move_counts, [6])
        board._apply_offsets.assert_called_once_with(0)
        self.assertEqual(board._run.call_args_list[-1],
                         mock.call('ACTIVATE_EXTRUDER EXTRUDER=extruder'))

    def test_failed_pickup_verification_never_activates_or_departs(self):
        board = self.board
        board.approach_x, board.pre_dock_x = 250., 280.
        board.pullback, board.pullback_slow_distance = 60., 20.
        board.clear_travel_speed = 600.
        board.pickup_predock_speed = board.pickup_latch_speed = 60.
        board.pullback_speed = 80.
        board.pickup_latch_wait_ms = 0
        board.grab_buttons = ['grab0']
        board._button = mock.Mock(return_value=True)
        board._raise_z = mock.Mock()
        board._move = mock.Mock()
        board._run = mock.Mock()
        board._pause_ms = mock.Mock()
        board._verify = mock.Mock(side_effect=RuntimeError('sensor failed'))
        board._apply_offsets = mock.Mock()

        with self.assertRaisesRegex(RuntimeError, 'sensor failed'):
            board._pickup(GCmd(), 0)

        self.assertEqual(board._move.call_count, 6)
        self.assertFalse(any('ACTIVATE_EXTRUDER' in call.args[0]
                             for call in board._run.call_args_list))

    def test_select_sets_part_accel_only_after_success(self):
        board = self.board
        board.pickup_accel = 6000.
        board._run = mock.Mock()
        toolhead = mock.Mock()
        toolhead.get_max_velocity.return_value = (600., 1000.)
        gcode_move = mock.Mock()
        gcode_move.get_status.return_value = {'absolute_coordinates': True}
        board.printer.lookup_object.side_effect = lambda name: {
            'toolhead': toolhead, 'gcode_move': gcode_move}[name]
        board._with_motion(GCmd(), lambda: None, final_accel=30000.)
        self.assertEqual(board._run.call_args_list[-1],
                         mock.call('SET_VELOCITY_LIMIT ACCEL=30000.0000'))
        board._run.reset_mock()
        with self.assertRaisesRegex(RuntimeError, 'pickup failed'):
            board._with_motion(GCmd(),
                               lambda: (_ for _ in ()).throw(
                                   RuntimeError('pickup failed')),
                               final_accel=30000.)
        self.assertEqual(board._run.call_args_list[-1],
                         mock.call('SET_VELOCITY_LIMIT ACCEL=1000.0000'))
        self.assertFalse(board.busy)

    def test_xy_only_tool_motion_uses_safe_home_hop(self):
        board = self.board
        board.safe_z = 10.
        board._preflight = mock.Mock(return_value=([True] * 4, [], None))
        board._run = mock.Mock()
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {'homed_axes': 'xy'}
        safe_home = mock.Mock(z_hop=10.)
        board.printer.lookup_object.side_effect = lambda name, default=None: {
            'toolhead': toolhead, 'safe_z_home': safe_home}.get(name, default)
        self.assertEqual(board._prepare_tool_motion(GCmd()),
                         ([True] * 4, None, False))
        board._run.assert_called_once_with('G28 X Y')
        self.assertEqual(board._preflight.call_count, 2)

    def test_pickup_without_z_home_does_not_command_absolute_z(self):
        board = self.board
        board.approach_x = 250.
        board.pre_dock_x = 280.
        board.pullback = 20.
        board.clear_travel_speed = 600.
        board.pickup_predock_speed = 40.
        board.pickup_latch_speed = 20.
        board.pullback_speed = 80.
        board.departure_speed = 80.
        board.pickup_latch_wait_ms = 0
        board.grab_buttons = ['grab0']
        board._button = mock.Mock(return_value=True)
        board._raise_z = mock.Mock()
        board._move = mock.Mock()
        board._run = mock.Mock()
        board._pause_ms = mock.Mock()
        board._verify = mock.Mock()
        board._apply_offsets = mock.Mock()
        board._pickup(GCmd(), 0, raise_z=False)
        board._raise_z.assert_not_called()
        self.assertTrue(all('z' not in call.kwargs for call in
                            board._move.call_args_list))

    def test_xy_only_tool_motion_requires_clearance_hop(self):
        board = self.board
        board.safe_z = 10.
        board._preflight = mock.Mock(return_value=([True] * 4, [], None))
        board._run = mock.Mock()
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {'homed_axes': 'xy'}
        safe_home = mock.Mock(z_hop=5.)
        board.printer.lookup_object.side_effect = lambda name, default=None: {
            'toolhead': toolhead, 'safe_z_home': safe_home}.get(name, default)
        with self.assertRaisesRegex(RuntimeError, 'z_hop of at least'):
            board._prepare_tool_motion(GCmd())
        board._run.assert_not_called()

    def test_load_and_save_stock_schema(self):
        self.board._load_extruder_json()
        self.assertEqual(self.board.measurements[0][0], 16.25)
        self.assertEqual(self.board.station_z, -1.5)
        self.board.measurements[1][2] = 1.23
        self.board.cmd_save_offsets_json(GCmd())
        data, comment = self.board._read_stock_json(self.path)
        self.assertEqual(data['t1_offset_z'], 1.23)
        self.assertEqual(data['now_extruder'], -1)
        self.assertEqual(comment, '/* stock comment */')
        backups = list(Path(self.temp.name).glob('extruder.json.bak-*'))
        self.assertEqual(len(backups), 1)

    def test_applies_relative_xy_and_positive_absolute_nozzle_z(self):
        self.board.measurements[0] = [16.25, 214., 1.3]
        self.board.measurements[1] = [15.95, 213.5, 1.2]
        self.board.z_adjustments = [0., 0.04, 0., 0.]
        commands = []
        self.board._run = commands.append
        self.board._apply_offsets(1)
        self.assertEqual(commands, [
            'SET_GCODE_OFFSET X=-0.3000 Y=-0.5000 Z=2.2400 MOVE=0'])

    def test_attached_nozzle_rejects_zero_offset(self):
        self.board._sensor_state = lambda command: ([False] * 4, [], 0)
        self.board._validate_sensor_state = mock.Mock()
        with self.assertRaisesRegex(RuntimeError,
                                    'Refusing nozzle Z offset 0.0000'):
            self.board.validate_nozzle_z_offset(GCmd(), 0.)
        self.board.validate_nozzle_z_offset(GCmd(), 2.86)
        self.board._sensor_state = lambda command: ([True] * 4, [], None)
        self.board.validate_nozzle_z_offset(GCmd(), 0.)

    def test_reloads_touchscreen_zoffset_before_selecting_tool(self):
        zpath = os.path.join(self.temp.name, 'zoffset.json')
        self.board.zoffset_json_path = zpath
        self.board.z_adjustments = [0.] * 4
        self.board._preflight = mock.Mock(return_value=([True] * 4, [], 1))
        self.board._apply_offsets = mock.Mock()
        self.board._run = mock.Mock()
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t2": 0.04}\n/* stock comment */\n')
        self.board.cmd_select(GCmd(T=1))
        self.assertEqual(self.board.z_adjustments[1], 0.04)
        self.board._apply_offsets.assert_called_once_with(1)
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t2": 0.09}\n/* stock comment */\n')
        self.board.cmd_select(GCmd(T=1))
        self.assertEqual(self.board.z_adjustments[1], 0.09)

    def test_missing_touchscreen_zoffset_clears_stale_values(self):
        self.board.zoffset_json_path = os.path.join(
            self.temp.name, 'missing-zoffset.json')
        self.board.z_adjustments = [0., 0.2, 0., 0.]
        self.board._load_zoffset_json()
        self.assertEqual(self.board.z_adjustments, [0.] * 4)

    def test_save_touchscreen_offset_subtracts_levelboard_baseline(self):
        zpath = os.path.join(self.temp.name, 'zoffset.json')
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t2": 0.01}\n/* stock comment */\n')
        self.board.zoffset_json_path = zpath
        self.board.z_adjustments = [0.] * 4
        self.board.measurements[0][2] = 1.3
        self.board.measurements[1][2] = 1.2
        self.board._sensor_state = lambda command: ([True] * 4, [], 1)
        self.board._validate_sensor_state = lambda *args: None
        self.board.printer.lookup_object.return_value.get_status.return_value = {
            'homing_origin': (0., 0., 2.45, 0.)}
        self.board.cmd_save_touchscreen_z_offset(GCmd())
        data, suffix = self.board._read_stock_json(zpath)
        self.assertAlmostEqual(data['z_offset_t2'], 0.25)
        self.assertEqual(suffix, '/* stock comment */')
        self.assertAlmostEqual(self.board.z_adjustments[1], 0.25)
        self.assertEqual(len(list(Path(self.temp.name).glob(
            'zoffset.json.bak-*'))), 1)

    def test_factory_nozzle_z_from_tool_station_and_touchscreen(self):
        zpath = os.path.join(self.temp.name, 'zoffset.json')
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t2": 0.05}\n/* stock comment */\n')
        with open(self.path, 'w') as target:
            target.write('{"t1_offset_z": 1.3, '
                         '"z_station_pos": -1.5}\n')
        self.board.zoffset_json_path = zpath
        self.board.z_adjustments = [0.] * 4
        self.board._preflight = lambda command: ([True] * 4, [], 1)
        self.board._run = mock.Mock()
        self.board.cmd_auto_nozzle_z(GCmd(T=1))
        self.board._run.assert_called_once_with(
            'SET_GCODE_OFFSET Z=2.8500 MOVE=1 MOVE_SPEED=100')
        self.assertAlmostEqual(self.board.print_z_baseline[1], 2.8)

    def test_factory_print_temperature_bed_and_layer_z_compensation(self):
        zpath = os.path.join(self.temp.name, 'zoffset.json')
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t1": 0.0}\n')
        with open(self.path, 'w') as target:
            target.write('{"t0_offset_z": 1.325, '
                         '"z_station_pos": -1.538889}\n')
        self.board.zoffset_json_path = zpath
        self.board.z_adjustments = [0.] * 4
        self.board._preflight = lambda command: ([True] * 4, [], 0)
        self.board._run = mock.Mock()
        self.board.cmd_auto_nozzle_z(GCmd(
            T=0, HOTEND=220., BED=100., FIRST_LAYER_HEIGHT=0.08))
        self.board._run.assert_called_once_with(
            'SET_GCODE_OFFSET Z=2.7689 MOVE=1 MOVE_SPEED=100')

    def test_nozzle_z_guard_rejects_zero_or_missing_offset(self):
        self.board._preflight = lambda command: ([True] * 4, [], 0)
        self.board.print_z_baseline[0] = 2.85
        self.board.z_adjustments = [0.] * 4
        self.board.printer.lookup_object.return_value.get_status.return_value = {
            'homing_origin': (0., 0., 0., 0.)}
        with self.assertRaisesRegex(RuntimeError, 'changed or reset'):
            self.board.cmd_verify_nozzle_z(GCmd())
        self.board.print_z_baseline[0] = None
        with self.assertRaisesRegex(RuntimeError, 'not applied'):
            self.board.cmd_verify_nozzle_z(GCmd())

    def test_factory_nozzle_z_rejects_out_of_range_value(self):
        zpath = os.path.join(self.temp.name, 'zoffset.json')
        with open(zpath, 'w') as target:
            target.write('{"z_offset_t1": 0.0}\n')
        with open(self.path, 'w') as target:
            target.write('{"t0_offset_z": 1.3, '
                         '"z_station_pos": 1.3}\n')
        self.board.zoffset_json_path = zpath
        self.board._preflight = lambda command: ([True] * 4, [], 0)
        self.board._run = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'outside 0.5..5 mm'):
            self.board.cmd_auto_nozzle_z(GCmd(T=0))
        self.board._run.assert_not_called()

    def test_factory_nozzle_z_requires_matching_physical_tool(self):
        self.board._preflight = lambda command: ([True] * 4, [], None)
        self.board._run = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'Attach T1'):
            self.board.cmd_auto_nozzle_z(GCmd(T=1))
        self.board._run.assert_not_called()

    def test_no_levelboard_motion_before_plate_confirmation(self):
        self.board.reference_scan_height = None
        self.board._preflight = lambda command: self.fail('preflight reached')
        with self.assertRaisesRegex(RuntimeError, 'Remove the build plate'):
            self.board.cmd_reference_calibrate(GCmd())
        with self.assertRaisesRegex(RuntimeError, 'Remove the build plate'):
            self.board.cmd_offset_calibrate(GCmd(T=0))

    def test_levelboard_xy_scan_starts_at_center_and_probes_outward(self):
        self.board.scan_span = 7.
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._estop = mock.Mock(side_effect=[32., 219., 26., 213.])

        self.assertEqual(self.board._scan_xy(GCmd(), 29., 216., 2.),
                         (29., 216.))
        self.assertEqual(self.board._estop.call_args_list, [
            mock.call('X', 36., mock.ANY),
            mock.call('Y', 223., mock.ANY),
            mock.call('X', 22., mock.ANY),
            mock.call('Y', 209., mock.ANY)])
        self.assertEqual(self.board._move.call_args_list[0],
                         mock.call(x=29., y=216., feed=1200))
        self.assertEqual(self.board._move.call_args_list[2],
                         mock.call(x=29., y=216., feed=1200))

    def test_levelboard_scan_reaches_are_independent(self):
        self.board.scan_spans = {
            ('X', 1): 6., ('Y', 1): 7.,
            ('X', -1): 8., ('Y', -1): 9.}
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._estop = mock.Mock(side_effect=[32., 219., 26., 210.])

        self.assertEqual(self.board._scan_xy(GCmd(), 29., 216., 2.),
                         (29., 214.5))
        self.assertEqual(self.board._estop.call_args_list, [
            mock.call('X', 35., mock.ANY),
            mock.call('Y', 223., mock.ANY),
            mock.call('X', 21., mock.ANY),
            mock.call('Y', 207., mock.ANY)])

    def test_single_offset_docks_checks_plate_references_then_selects(self):
        self.board.scan_height = None
        self.board._preflight = mock.Mock(side_effect=[
            ([], [], 1), ([], [], 1)])
        steps = []
        self.board._dock_for_offset_reference = (
            lambda gcmd, attached: steps.append(('dock', attached)))
        self.board._dock = lambda gcmd, tool: steps.append(('dock', tool))
        self.board._calibrate_offset_reference = (
            lambda gcmd, levelboard=True:
                steps.append(('plate-and-reference', levelboard)))
        def run(command):
            steps.append(command)
        self.board._run = run
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._calibrate_tool_z = lambda gcmd, tool: (
            steps.append(('z', tool)) or 1.3)
        self.board._calibrate_tool_xy = lambda gcmd, tool, height: (
            steps.append(('xy', tool, height)))
        self.board._apply_offsets = lambda tool: steps.append(('apply', tool))
        self.board._lift_after_offset_calibration = (
            lambda gcmd: steps.append('lift'))
        self.board.printer.lookup_object.side_effect = None
        self.board.printer.lookup_object.return_value = mock.Mock(
            get_status=mock.Mock(return_value={
                'homing_origin': (.1, .2, .3, 0.), 'state': 'standby'}))

        self.board.cmd_offset_calibrate(GCmd(
            BUILDPLATE_REMOVED=1, T=1, Z=1, SAVE=0))

        self.assertEqual(steps[:3], [
            ('dock', 1), ('plate-and-reference', 1),
            'AFC_SELECT_TOOL TOOL=extruder1'])
        self.assertEqual(steps[4:6], [('z', 1), ('xy', 1, 1.9)])
        self.assertIsNone(self.board._stock_pin_approach_z)

    def test_single_offset_plate_check_failure_prevents_pickup_and_fixture(self):
        self.board.scan_height = None
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(
            side_effect=RuntimeError('Build plate may still be installed'))
        self.board._probe_fixture_z = mock.Mock()
        self.board._run = mock.Mock()
        self.board.printer.lookup_object.return_value.get_status.return_value = {
            'homing_origin': (.1, .2, .3, 0.), 'state': 'standby'}

        with self.assertRaisesRegex(RuntimeError, 'Build plate may'):
            self.board.cmd_offset_calibrate(GCmd(
                BUILDPLATE_REMOVED=1, T=0, Z=1, SAVE=0))

        self.board._check_plate_removed.assert_called_once()
        self.board._probe_fixture_z.assert_not_called()
        self.assertFalse(any('AFC_SELECT_TOOL' in call.args[0]
                             for call in self.board._run.call_args_list))

    def test_single_reference_uses_stock_pin_before_levelboard(self):
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(return_value=1.25)
        self.board._run = mock.Mock()

        self.board._calibrate_offset_reference(GCmd())

        self.board._check_plate_removed.assert_called_once()
        self.assertEqual(self.board._stock_pin_approach_z, -3.75)
        self.board._run.assert_called_once_with(
            'C5_LEVELBOARD_REFERENCE_CALIBRATE SAVE=0 '
            'BUILDPLATE_REMOVED=1')

    def test_single_reference_can_skip_levelboard_but_not_plate_check(self):
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(return_value=1.25)
        self.board._run = mock.Mock()

        self.board._calibrate_offset_reference(GCmd(), levelboard=0)

        self.board._check_plate_removed.assert_called_once()
        self.board._run.assert_not_called()
        self.assertEqual(self.board._stock_pin_approach_z, -3.75)

    def test_automatic_z_precedes_xy_and_sets_scan_height(self):
        calls = []
        self.board._offset_batch_active = True
        self.board.scan_height = None
        self.board._preflight = lambda command: ([], [], 1)
        self.board._with_motion = lambda command, operation: operation()
        self.board._run = calls.append
        self.board._calibrate_tool_z = lambda command, tool: (
            calls.append(('z', tool)) or 1.23)
        self.board._calibrate_tool_xy = lambda command, tool, height: (
            calls.append(('xy', tool, height)))
        self.board._apply_offsets = lambda tool: calls.append(('apply', tool))
        self.board._lift_after_offset_calibration = (
            lambda command: calls.append('lift'))
        self.board.cmd_offset_calibrate(GCmd(
            BUILDPLATE_REMOVED=1, T=1, Z=1, SAVE=0))
        self.assertEqual(calls[0],
                         'SET_GCODE_OFFSET X=0 Y=0 MOVE=0')
        self.assertEqual(calls[1], ('z', 1))
        self.assertEqual(calls[2][0:2], ('xy', 1))
        self.assertAlmostEqual(calls[2][2], 1.83)
        self.assertEqual(calls[3], ('apply', 1))
        self.assertEqual(calls[4], 'lift')

    def test_offset_calibration_lift_uses_physical_z_and_waits(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xyz', 'axis_maximum': mock.Mock(z=270.)}
        toolhead.get_position.return_value = [10., 20., 10., 0.]
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()

        self.board._lift_after_offset_calibration(GCmd())

        self.board._move.assert_called_once_with(z=13., feed=600.)
        self.board._run.assert_called_once_with('M400')

    def test_offset_calibration_lift_clears_fixture_to_safe_z(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xyz', 'axis_maximum': mock.Mock(z=270.)}
        toolhead.get_position.return_value = [29., 214., 2., 0.]
        self.board.safe_z = 10.
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()

        self.board._lift_after_offset_calibration(GCmd())

        self.board._move.assert_called_once_with(z=10., feed=600.)
        self.board._run.assert_called_once_with('M400')

    def test_offset_calibration_lift_is_bounded_by_z_max(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xyz', 'axis_maximum': mock.Mock(z=270.)}
        toolhead.get_position.return_value = [10., 20., 269., 0.]
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()

        self.board._lift_after_offset_calibration(GCmd())

        self.board._move.assert_called_once_with(z=270., feed=600.)
        self.board._run.assert_called_once_with('M400')

    def test_offset_calibration_lift_requires_homed_z(self):
        toolhead = mock.Mock()
        toolhead.get_status.return_value = {
            'homed_axes': 'xy', 'axis_maximum': mock.Mock(z=270.)}
        self.board.printer.lookup_object.return_value = toolhead
        self.board._move = mock.Mock()

        with self.assertRaisesRegex(RuntimeError, 'Home Z'):
            self.board._lift_after_offset_calibration(GCmd())
        self.board._move.assert_not_called()

    def test_xy_failure_restores_measurements_and_runtime_offsets(self):
        self.board._offset_batch_active = True
        self.board.scan_height = None
        self.board._preflight = lambda command: ([], [], 1)
        self.board._with_motion = lambda command, operation: operation()
        self.board._run = mock.Mock()
        original = [list(v) for v in self.board.measurements]
        def probe_z(command, tool):
            self.board.measurements[tool][2] = 2.
            return 2.
        self.board._calibrate_tool_z = probe_z
        self.board._calibrate_tool_xy = mock.Mock(
            side_effect=RuntimeError('XY probe failed'))
        self.board._lift_after_offset_calibration = mock.Mock()
        self.board.configfile = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'XY probe failed'):
            self.board.cmd_offset_calibrate(GCmd(
                BUILDPLATE_REMOVED=1, T=1, Z=1, SAVE=1))
        self.assertEqual(self.board.measurements, original)
        self.board._lift_after_offset_calibration.assert_called_once()
        self.board.configfile.set.assert_not_called()
        self.board._run.assert_called_with(
            'SET_GCODE_OFFSET X=0.1000 Y=0.2000 MOVE=0')

    def test_manual_mount_save_survives_json_reload(self):
        self.board.max_mount_correction = 2.
        self.board.cmd_save_offsets_json(GCmd())
        self.board.cmd_mount_correct(GCmd(T=0, X=299., Y=51., SAVE=1))
        self.board._load_extruder_json()
        self.assertEqual(self.board.docks[0], [299., 51.])

    def test_purge_selects_attached_hotend_for_shared_drive(self):
        self.board.purge_length = 12.
        self.board.purge_x, self.board.purge_y, self.board.purge_z = 276.5, 260., 8.
        self.board.prep_approach_x = self.board.prep_approach_y = 250.
        self.board.purge_speed = 3.
        self.board.flow_test_z = 8.
        self.board._preflight = lambda command: ([], [], 1)
        self.board.cmd_verify_nozzle_z = mock.Mock()
        toolhead, heater, other = mock.Mock(), mock.Mock(), mock.Mock()
        toolhead.get_extruder.return_value = other
        heater.get_status.return_value = {'can_extrude': True}
        self.board.printer.lookup_object.side_effect = (
            lambda name: {'toolhead': toolhead, 'extruder1': heater}[name])
        self.board._with_motion = lambda command, operation: operation()
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()
        self.board._pause_ms = mock.Mock()
        self.board.cmd_purge(GCmd())
        self.assertEqual(self.board._run.call_args_list[0],
                         mock.call('ACTIVATE_EXTRUDER EXTRUDER=extruder1'))
        self.board._run.assert_any_call('G1 E12.000 F180')
        self.board._run.assert_any_call('G1 E-5 F1200')
        self.board._move.assert_any_call(y=250., feed=6000,
                                         machine=False)
        self.board._move.assert_any_call(z=8., feed=600, machine=False)
        moves = [call.kwargs for call in self.board._move.call_args_list]
        x250 = next(i for i, move in enumerate(moves)
                    if move.get('x') == 250.)
        low_y = next(i for i, move in enumerate(moves)
                     if move.get('y') == 250.)
        final_x = next(i for i, move in enumerate(moves)
                       if move.get('x') == 276.5)
        self.assertLess(x250, low_y)
        self.assertLess(low_y, final_x)
        final_z = next(i for i, move in enumerate(moves)
                     if move.get('z') == 8.)
        self.assertLess(final_x, final_z)
        scripts = [call.args[0] for call in self.board._run.call_args_list]
        self.assertLess(scripts.index('G1 E12.000 F180'),
                        scripts.index('G1 E-5 F1200'))
        self.assertEqual(scripts.count('M400'), 1)
        self.board._run.reset_mock()
        heater.get_status.return_value = {'can_extrude': False}
        with self.assertRaisesRegex(RuntimeError, 'not hot enough'):
            self.board.cmd_purge(GCmd())
        self.board._run.assert_not_called()

    def test_purge_flow_envelope_stops_at_x279(self):
        board = self.board
        board.purge_x, board.purge_y = 276.5, 260.
        board.flow_sweep_x = 4.5
        board.purge_max_x = 279.
        board._validate_purge_envelope(RuntimeError)
        board.purge_x = 277.
        with self.assertRaisesRegex(RuntimeError, 'exceeds purge_max_x'):
            board._validate_purge_envelope(RuntimeError)

    def test_flow_strokes_stay_over_bucket_and_apply_pa(self):
        self.board.flow_test_z = 8.
        self.board.flow_sweep_x = 4.5
        self.board.flow_test_accel = 625.
        self.board.flow_slow_speed = 4.5
        self.board.flow_fast_speed = 22.875
        self.board.prep_approach_x = self.board.prep_approach_y = 250.
        self.board.purge_x, self.board.purge_y = 276.5, 260.
        self.board._preflight = lambda command: ([], [], 0)
        self.board.cmd_verify_nozzle_z = mock.Mock()
        self.board._with_motion = lambda command, operation: operation()
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()
        self.board._pause_ms = mock.Mock()
        heater, pa = mock.Mock(), mock.Mock()
        heater.get_status.return_value = {
            'can_extrude': True, 'pressure_advance': .02}
        pa.pa_get_value.return_value = 9
        self.board.printer.lookup_object.side_effect = (
            lambda name, *args: {'extruder': heater, 'pa_adjust': pa}[name])
        command = GCmd()
        self.board.cmd_flow_strokes(command)
        self.assertEqual(pa.pa_get_value.call_count, 3)
        self.board._pause_ms.assert_has_calls([mock.call(50)] * 3)
        self.board._move.assert_any_call(x=276.5, y=260., feed=3000,
                                         machine=False)
        self.board._move.assert_any_call(x=274.25, feed=3000,
                                         machine=False)
        self.board._run.assert_any_call('SET_VELOCITY_LIMIT ACCEL=625')
        self.board._run.assert_any_call('G1 X278.750 E1.13573 F270')
        self.board._run.assert_any_call('G1 X274.250 E2.27146 F1372')
        self.assertFalse(any('X' in call.args[0]
                             and not ('X274.250' in call.args[0]
                                      or 'X278.750' in call.args[0])
                             for call in self.board._run.call_args_list
                             if call.args[0].startswith('G1 ')))
        self.board._run.assert_any_call(
            'SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.01000')
        self.assertIn('pressure advance 0.01000', command.message)
        self.assertTrue(any('T0 flow valid: PA 0.0100 (pass 1)' in msg
                            for msg in command.messages))

    def test_flow_strokes_restore_pa_on_failed_measurement(self):
        self.board.flow_test_z = 8.
        self.board.flow_sweep_x = 4.5
        self.board.flow_test_accel = 625.
        self.board.flow_slow_speed = 4.5
        self.board.flow_fast_speed = 22.875
        self.board.prep_approach_x = self.board.prep_approach_y = 250.
        self.board.purge_x, self.board.purge_y = 270., 250.
        self.board._preflight = lambda command: ([], [], 0)
        self.board.cmd_verify_nozzle_z = mock.Mock()
        self.board._with_motion = lambda command, operation: operation()
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()
        self.board._pause_ms = mock.Mock()
        heater, pa = mock.Mock(), mock.Mock()
        heater.get_status.return_value = {
            'can_extrude': True, 'pressure_advance': .02}
        pa.pa_get_value.return_value = 0
        self.board.printer.lookup_object.side_effect = (
            lambda name, *args: {'extruder': heater, 'pa_adjust': pa}[name])
        command = GCmd()
        self.board.cmd_flow_strokes(command)
        self.assertEqual(pa.pa_get_value.call_count, 35)
        self.board._run.assert_any_call(
            'SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.02000')
        self.assertIn('pressure advance unchanged', command.message)

    def test_cool_for_dock_uses_stock_minus_100_rule(self):
        self.board._preflight = lambda command: ([], [], 1)
        self.board.cmd_verify_nozzle_z = mock.Mock()
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()
        self.board.prep_approach_x = self.board.prep_approach_y = 250.
        self.board.cooldown_x, self.board.cooldown_y = 266.5, 13.8
        self.board.cooldown_wipe_z = 2.
        self.board.cooldown_lift_z = 10.
        self.board.cooldown_fan_speed = .6
        heater = mock.Mock()
        heater.get_status.side_effect = [
            {'temperature': 220.}, {'temperature': 121.}]
        self.board.printer.lookup_object.side_effect = (
            lambda name: {'extruder1': heater}[name])
        reactor = self.board.printer.get_reactor.return_value
        reactor.monotonic.side_effect = [0., 1., 2.]
        self.board.cmd_cool_for_dock(GCmd(HOTEND=220.))
        self.board._run.assert_any_call(
            'SET_HEATER_TEMPERATURE HEATER=extruder1 TARGET=120.0')
        self.board._run.assert_any_call(
            'SET_FAN_SPEED FAN=fanM106 SPEED=0.600')
        self.board._run.assert_any_call('SET_FAN_SPEED FAN=fanM106 SPEED=0')
        self.board.cmd_verify_nozzle_z.assert_called_once()
        self.board._raise_z.assert_called_once()
        self.assertEqual(self.board._move.call_args_list, [
            mock.call(x=250., y=250., feed=6000, machine=False),
            mock.call(y=13.8, feed=6000, machine=False),
            mock.call(x=266.5, feed=3000, machine=False),
            mock.call(z=2., feed=600, machine=False),
            mock.call(z=10., feed=600, machine=False)])

    def test_cool_for_dock_lifts_from_wiper_on_timeout(self):
        self.board._preflight = lambda command: ([], [], 1)
        self.board.cmd_verify_nozzle_z = mock.Mock()
        self.board._raise_z = mock.Mock()
        self.board._move = mock.Mock()
        self.board._run = mock.Mock()
        self.board.prep_approach_x = self.board.prep_approach_y = 250.
        self.board.cooldown_x, self.board.cooldown_y = 266.5, 13.8
        self.board.cooldown_wipe_z = 2.
        self.board.cooldown_lift_z = 10.
        self.board.cooldown_fan_speed = .6
        heater = mock.Mock()
        heater.get_status.return_value = {'temperature': 220.}
        self.board.printer.lookup_object.side_effect = (
            lambda name: {'extruder1': heater}[name])
        reactor = self.board.printer.get_reactor.return_value
        reactor.monotonic.side_effect = [0., 181.]
        with self.assertRaisesRegex(RuntimeError, 'did not cool'):
            self.board.cmd_cool_for_dock(GCmd(HOTEND=220.))
        self.board._move.assert_any_call(z=10., feed=600, machine=False)
        self.board._run.assert_any_call('SET_FAN_SPEED FAN=fanM106 SPEED=0')
        self.board._run.assert_any_call(
            'RESTORE_GCODE_STATE NAME=C5_COOLDOWN_TRAVEL MOVE=0')

    def test_print_homing_recovers_head_then_homes_only_z(self):
        calls = []
        self.board._preflight = mock.Mock(side_effect=[
            ([True, True, False, True], [], 2), ([True]*4, [], None)])
        self.board.cmd_recover_attached = lambda cmd: calls.append('recover')
        self.board._run = calls.append
        self.board.cmd_home_for_print(GCmd())
        self.assertEqual(calls, ['recover', 'G28 Z'])

    def test_failed_recovery_never_homes_z(self):
        self.board._preflight = lambda command, **kwargs: ([], [], 2)
        self.board._run = mock.Mock()
        self.board.cmd_recover_attached = mock.Mock(
            side_effect=RuntimeError('dock failed'))
        with self.assertRaisesRegex(RuntimeError, 'dock failed'):
            self.board.cmd_home_for_print(GCmd())
        self.board._run.assert_not_called()

    def test_recovery_uses_xy_homing_and_no_fixture_probe(self):
        self.board.safe_z = 10.
        self.board.printer.lookup_object.return_value = mock.Mock(z_hop=10.)
        self.board._preflight = lambda command, **kwargs: ([], [], 2)
        self.board._with_motion = lambda command, op: op()
        calls = []
        self.board._run = calls.append
        self.board._dock = lambda cmd, tool, **kw: calls.append((tool, kw))
        self.board.cmd_recover_attached(GCmd())
        self.assertEqual(calls, ['G28 X Y', (2, {'raise_z': False})])

    def test_print_homing_requires_all_heads_parked(self):
        self.board._run = mock.Mock()
        self.board._preflight = lambda command, **kwargs: (
            [True, False, True, True], [], None)
        with self.assertRaisesRegex(RuntimeError, 'all heads'):
            self.board.cmd_home_for_print(GCmd())
        self.board._run.assert_not_called()
        self.board._preflight = lambda command, **kwargs: ([True]*4, [], None)
        self.board.cmd_home_for_print(GCmd())
        self.board._run.assert_called_once_with('G28')
        self.assertFalse(self.board._g28_passthrough)

    def test_all_axis_g28_uses_tool_recovery_and_partial_homing_delegates(self):
        calls = []
        self.board.cmd_home_for_print = lambda gcmd: calls.append('recover')
        self.board.prev_G28 = lambda gcmd: calls.append('normal')
        self.board.attached_tool_from_pins = lambda: 1
        self.board.cmd_G28(GCmd())
        self.board.cmd_G28(GCmd(X='0', Y='0', Z='0'))
        self.board.cmd_G28(GCmd(X='0', Y='0'))
        self.assertEqual(calls, ['recover', 'recover', 'normal'])
        with self.assertRaisesRegex(RuntimeError, 'Dock the attached tool'):
            self.board.cmd_G28(GCmd(Z='0'))
        self.board.attached_tool_from_pins = lambda: None
        self.board.cmd_G28(GCmd(Z='0'))
        self.assertEqual(calls[-1], 'normal')
        self.board._g28_passthrough = True
        self.board.cmd_G28(GCmd())
        self.assertEqual(calls[-1], 'normal')

    def test_g28_wrapper_keeps_previous_safe_z_home_handler(self):
        previous = mock.Mock()
        self.board.gcode = mock.Mock()
        self.board.gcode.register_command.side_effect = [previous, None, None]
        self.board.printer.lookup_object.return_value.extruder_stepper = None
        self.board._handle_ready()
        self.assertIs(self.board.prev_G28, previous)
        self.assertEqual(self.board.gcode.register_command.call_args_list, [
            mock.call('G28', None),
            mock.call('G28', self.board.cmd_G28),
            mock.call('SET_PRESSURE_ADVANCE', None),
        ])

    def test_no_head_all_axis_home_delegates_once_without_recursion(self):
        calls = []
        self.board._preflight = lambda command, **kwargs: ([True]*4, [], None)
        self.board.prev_G28 = lambda command: calls.append('normal G28')
        self.board._run = lambda script: self.board.cmd_G28(GCmd())
        self.board.cmd_G28(GCmd())
        self.assertEqual(calls, ['normal G28'])
        self.assertFalse(self.board._g28_passthrough)

    def test_missing_dock_confirmation_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'Missing dock confirmation for T1'):
            self.board._validate_sensor_state(
                GCmd(), [True, False, True, True], None)

    def test_zero_toolchange_wait_skips_reactor_but_keeps_verification(self):
        self.board.release_latch_wait_ms = 0
        self.board._pause_ms(0)
        self.board.printer.get_reactor.assert_not_called()
        self.board._run = mock.Mock()
        self.board._sensor_state = mock.Mock(return_value=(
            [True] * 4, [False] * 4, None))
        self.board._verify(GCmd(), None, parked=0)
        self.board._run.assert_called_once_with('M400')
        self.board._sensor_state.assert_called_once()
        self.board.printer.get_reactor.assert_not_called()

    def test_failed_verification_turns_off_heaters(self):
        commands = []
        self.board._run = commands.append
        self.board._pause_ms = mock.Mock()
        self.board.sensor_settle_ms = 0
        self.board._sensor_state = mock.Mock(return_value=(
            [True, False, True, True], [False] * 4, None))

        with self.assertRaisesRegex(RuntimeError, 'Missing dock confirmation'):
            self.board._verify(GCmd(), None, parked=1)

        self.assertIn('TURN_OFF_HEATERS', commands)

    def test_attached_calibration_rolls_back_if_dock_fails(self):
        original = [list(values) for values in self.board.measurements]
        self.board._preflight = mock.Mock(return_value=([True] * 4, [], 0))
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._run = mock.Mock()
        self.board._apply_offsets = mock.Mock()
        self.board._calibrate_tool_xy = mock.Mock(
            side_effect=lambda gcmd, tool: self.board.measurements[tool].__setitem__(0, 99.))
        self.board._dock = mock.Mock(side_effect=RuntimeError('dock failed'))

        with self.assertRaisesRegex(RuntimeError, 'dock failed'):
            self.board.cmd_calibrate_attached(
                GCmd(BUILDPLATE_REMOVED=1))

        self.assertEqual(self.board.measurements, original)

    def test_move_rejects_non_finite_coordinates(self):
        self.board.printer.command_error = RuntimeError
        with self.assertRaisesRegex(RuntimeError, 'must be finite'):
            self.board._move(x=float('nan'))

    def test_machine_move_bypasses_gcode_and_resyncs_position(self):
        toolhead, gcode_move = mock.Mock(), mock.Mock()
        self.board.printer.lookup_object.side_effect = {
            'toolhead': toolhead, 'gcode_move': gcode_move}.__getitem__
        self.board._run = mock.Mock()
        self.board._move(x=298., y=50., feed=1200)
        toolhead.manual_move.assert_called_once_with([298., 50., None], 20.)
        gcode_move.reset_last_position.assert_called_once_with()
        self.board._run.assert_not_called()
        toolhead.manual_move.side_effect = RuntimeError('move rejected')
        with self.assertRaisesRegex(RuntimeError, 'move rejected'):
            self.board._move(x=299.)
        self.assertEqual(gcode_move.reset_last_position.call_count, 2)

    def test_purge_coordinates_can_keep_print_transform(self):
        self.board._run = mock.Mock()
        self.board._move(z=1., feed=600, machine=False)
        self.board._run.assert_called_once_with('G1 Z1.0000 F600')

    def test_all_offsets_failure_rolls_back_entire_batch(self):
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(return_value=0.)
        original = [list(v) for v in self.board.measurements]
        station = self.board.station_x
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board.configfile = mock.Mock()
        commands = []
        def run(command):
            commands.append(command)
            if command.startswith('C5_LEVELBOARD_REFERENCE'):
                self.board.station_x += 1.
            if command.startswith('C5_TOOL_OFFSET_CALIBRATE'):
                self.assertIn('SAVE=0', command)
                self.board.measurements[0][0] += .1
                if 'T=2 ' in command:
                    raise RuntimeError('T2 failed')
        self.board._run = run
        self.board.cmd_save_offsets_json = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'T2 failed'):
            self.board.cmd_calibrate_all_offsets(GCmd(
                BUILDPLATE_REMOVED=1, SAVE=1))
        self.assertEqual(self.board.measurements, original)
        self.assertEqual(self.board.station_x, station)
        self.board.configfile.set.assert_not_called()
        self.board.cmd_save_offsets_json.assert_not_called()
        self.assertFalse(any('TOOL=extruder3' in cmd for cmd in commands))

    def test_all_offsets_json_failure_also_rolls_back(self):
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(return_value=0.)
        original = [list(v) for v in self.board.measurements]
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board.configfile = mock.Mock()
        def run(command):
            if command.startswith('C5_TOOL_OFFSET_CALIBRATE'):
                self.board.measurements[0][0] += .1
        self.board._run = run
        self.board.cmd_save_offsets_json = mock.Mock(
            side_effect=RuntimeError('disk full'))
        with self.assertRaisesRegex(RuntimeError, 'disk full'):
            self.board.cmd_calibrate_all_offsets(GCmd(
                BUILDPLATE_REMOVED=1, SAVE=1))
        self.assertEqual(self.board.measurements, original)
        self.board.configfile.set.assert_not_called()

    def test_all_offsets_saves_once_after_final_dock(self):
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._check_plate_removed = mock.Mock(return_value=0.)
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board._run = mock.Mock()
        self.board.configfile = mock.Mock()
        def save(command):
            self.board._run.assert_called_with('AFC_UNSELECT_TOOL')
            self.assertEqual(sum('C5_TOOL_OFFSET_CALIBRATE' in c.args[0]
                                 for c in self.board._run.call_args_list), 4)
        self.board.cmd_save_offsets_json = mock.Mock(side_effect=save)
        self.board.cmd_calibrate_all_offsets(GCmd(
            BUILDPLATE_REMOVED=1, SAVE=1))
        self.board.cmd_save_offsets_json.assert_called_once()
        self.board.configfile.set.assert_not_called()

    def test_stock_pin_checks_plate_before_levelboard_or_tool_pickup(self):
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._probe_stock_pin = mock.Mock(side_effect=(0., -1.))
        self.board._run = mock.Mock()
        self.board.cmd_calibrate_all_offsets(GCmd(BUILDPLATE_REMOVED=1,
                                                   SAVE=0))
        self.assertEqual(self.board._probe_stock_pin.call_count, 2)
        self.assertEqual(self.board._probe_stock_pin.call_args_list[0].args[1:],
                         (43., 226.))
        self.assertEqual(self.board._probe_stock_pin.call_args_list[1].args[1:],
                         (28.5, 214.5))
        self.assertEqual(self.board._run.call_args_list[0].args[0],
                         'C5_LEVELBOARD_REFERENCE_CALIBRATE SAVE=0 '
                         'BUILDPLATE_REMOVED=1')
        self.assertIsNone(self.board._stock_pin_approach_z)

    def test_stock_pin_rejects_plate_present_before_tool_motion(self):
        self.board._preflight = mock.Mock(return_value=([], [], None))
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._probe_stock_pin = mock.Mock(side_effect=(0., -.2))
        self.board._run = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'Build plate may'):
            self.board.cmd_calibrate_all_offsets(GCmd(BUILDPLATE_REMOVED=1,
                                                       SAVE=0))
        self.assertFalse(any('AFC_SELECT_TOOL' in c.args[0]
                             for c in self.board._run.call_args_list))

    def test_all_offsets_docks_attached_head_before_stock_pin_check(self):
        self.board._preflight = mock.Mock(side_effect=[
            ([], [], 2), ([], [], None)])
        self.board._raise_z = mock.Mock()
        self.board._with_motion = lambda gcmd, operation: operation()
        self.board._run = mock.Mock()
        self.board._check_plate_removed = mock.Mock(return_value=0.)
        self.board.cmd_calibrate_all_offsets(GCmd(BUILDPLATE_REMOVED=1,
                                                   SAVE=0))
        self.board._run.assert_any_call('C5_TOOL_DOCK')
        self.board._raise_z.assert_called_once_with()
        self.board._check_plate_removed.assert_called_once()
        self.assertLess(self.board._run.call_args_list.index(
            mock.call('C5_TOOL_DOCK')),
            self.board._run.call_args_list.index(
                mock.call('C5_LEVELBOARD_REFERENCE_CALIBRATE SAVE=0 '
                          'BUILDPLATE_REMOVED=1')))

    def test_stock_pin_approach_never_exceeds_configured_probe_floor(self):
        self.board.safe_z = 10.
        self.board.z_probe_target = -8.
        self.board._move = mock.Mock()
        self.board._raise_z = mock.Mock()
        self.board._estop = mock.Mock(return_value=1.)
        self.board._stock_pin_approach_z = -5.
        self.board._probe_fixture_z(GCmd(), 28., 215.)
        self.board._estop.assert_called_once_with('Z', -5., mock.ANY)
        self.board._estop.reset_mock()
        self.board._stock_pin_approach_z = -12.
        self.board._probe_fixture_z(GCmd(), 28., 215.)
        self.board._estop.assert_called_once_with('Z', -8., mock.ANY)

    def test_stock_pin_uses_three_repeatable_probe_samples(self):
        session = mock.Mock()
        session.pull_probed_results.return_value = [
            mock.Mock(bed_z=z) for z in (1.00, 1.02, 1.01)]
        probe = mock.Mock()
        probe.start_probe_session.return_value = session
        probe.get_position.return_value = [43., 226., 5., 0.]
        probe.get_probe_params.return_value = {
            'sample_retract_dist': 2., 'lift_speed': 5.}
        self.board.printer.lookup_object.return_value = probe
        self.board._move = mock.Mock()
        self.board._raise_z = mock.Mock()
        self.assertAlmostEqual(self.board._probe_stock_pin(GCmd(), 43., 226.),
                               1.01)
        self.assertEqual(session.run_probe.call_count, 3)
        session.end_probe_session.assert_called_once()

    def test_stock_pin_rejects_inconsistent_samples(self):
        session = mock.Mock()
        session.pull_probed_results.return_value = [
            mock.Mock(bed_z=z) for z in (1.00, 1.12, 1.01)]
        probe = mock.Mock()
        probe.start_probe_session.return_value = session
        probe.get_position.return_value = [43., 226., 5., 0.]
        probe.get_probe_params.return_value = {
            'sample_retract_dist': 2., 'lift_speed': 5.}
        self.board.printer.lookup_object.return_value = probe
        self.board._move = mock.Mock()
        self.board._raise_z = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, 'not repeatable'):
            self.board._probe_stock_pin(GCmd(), 43., 226.)
        session.end_probe_session.assert_called_once()

    def test_load_check_requires_attached_and_matching_active_hotend(self):
        self.board._preflight = mock.Mock(return_value=([], [], 1))
        active = self.board.printer.lookup_object.return_value.get_extruder()
        active.get_name.return_value = 'extruder1'
        self.board.cmd_check_load_tool(GCmd(T=1))
        with self.assertRaisesRegex(RuntimeError, 'requested tool'):
            self.board.cmd_check_load_tool(GCmd(T=0))
        active.get_name.return_value = 'extruder'
        with self.assertRaisesRegex(RuntimeError, 'Active hotend'):
            self.board.cmd_check_load_tool(GCmd(T=1))

    def test_status_reports_conflict_instead_of_erroring(self):
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        # T1 shows both sensors active at once - exactly the case the old
        # implementation refused to report anything for.
        sensor = {'dock1': True, 'grab1': True}
        self.board._button = lambda name, gcmd: sensor.get(name, False)
        gcmd = GCmd()

        self.board.cmd_status(gcmd)

        self.assertEqual(gcmd.messages,
                         ['attached=conflict(T1) dock=0100 grab=0100'])

    def test_status_reports_attached_tool_normally(self):
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        sensor = {'dock0': True, 'dock2': True, 'dock3': True, 'grab1': True}
        self.board._button = lambda name, gcmd: sensor.get(name, False)
        gcmd = GCmd()

        self.board.cmd_status(gcmd)

        self.assertEqual(gcmd.messages,
                         ['attached=T1 dock=1011 grab=0100'])
        self.assertEqual(self.board.active, 1)


if __name__ == '__main__':
    unittest.main()
