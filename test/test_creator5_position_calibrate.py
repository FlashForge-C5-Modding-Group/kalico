import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = (Path(__file__).resolve().parents[1] / 'klippy' / 'extras'
               / 'creator5_toolchanger.py')
SPEC = importlib.util.spec_from_file_location('creator5_position', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **params):
        self.params = params
        self.messages = []

    def get_int(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.messages.append(message)


class PositionCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.board = MODULE.Creator5Toolchanger.__new__(
            MODULE.Creator5Toolchanger)
        self.board.docks = [[298.219, 56.472], [298.605, 107.100],
                            [298.200, 156.920], [298.238, 207.140]]

    def test_without_tool_asks_for_selection_without_motion(self):
        gcmd = GCmd()
        self.board.cmd_extruder_position_calibrate(gcmd)
        self.assertIn('T=0..3', gcmd.messages[0])

    def test_measurement_uses_stock_endstop_reference(self):
        x_home = mock.Mock()
        y_home = mock.Mock()
        x_home.measure_axis.return_value = -2.3
        y_home.measure_axis.return_value = 24.6
        self.board.printer = mock.Mock()
        self.board.printer.get_reactor.return_value.monotonic.return_value = 0.
        self.board.printer.lookup_object.side_effect = (
            lambda name, default=None: {
                'hd_home X': x_home, 'hd_home Y': y_home}.get(name, default))
        self.board.max_mount_correction = 2.
        self.board.sensor_settle_ms = 0
        self.board.pickup_latch_wait_ms = 0
        self.board.pullback_slow_distance = 20.
        self.board.pullback_speed = 80.
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        sensor = {'dock0': True, 'grab0': True}
        self.board._button = lambda name, gcmd: sensor.get(name, False)
        commands = []
        self.board._run = commands.append

        def move(**kwargs):
            commands.append(kwargs)
            if kwargs.get('x') == 275.:
                sensor['dock0'] = False

        self.board._move = move
        # Use a nearby stored coordinate, as required by the correction limit.
        self.board.docks[0] = [297.5, 55.5]
        result = self.board._measure_holder_position(GCmd(), 0)
        self.assertAlmostEqual(result[0], 297.3)
        self.assertAlmostEqual(result[1], 55.4)
        x_home.measure_axis.assert_called_once_with(-5.)
        y_home.measure_axis.assert_called_once_with(0.)
        self.assertIn('SET_KINEMATIC_POSITION X=295.000 Y=80.000 '
                      'SET_HOMED=xy', commands)

    def test_contact_wait_times_out_without_motion(self):
        clock = [0.]
        reactor = mock.Mock()
        reactor.monotonic.side_effect = lambda: clock[0]
        reactor.pause.side_effect = lambda deadline: clock.__setitem__(
            0, deadline)
        self.board.printer = mock.Mock()
        self.board.printer.get_reactor.return_value = reactor
        self.board.position_calibration_timeout = .12
        self.board.dock_buttons = ['dock%d' % i for i in range(4)]
        self.board.grab_buttons = ['grab%d' % i for i in range(4)]
        self.board._button = lambda name, gcmd: False
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            self.board._wait_for_holder_contact(GCmd(), 0)
        self.assertGreaterEqual(clock[0], .12)

    def test_holder_save_preserves_nozzle_offsets(self):
        with tempfile.TemporaryDirectory() as temp:
            path = os.path.join(temp, 'extruder.json')
            with open(path, 'w') as target:
                target.write('{"t0_offset_x": 16.25, "t0_offset_y": 213.9, '
                             '"x_check_pos": 298.2, "y_check_pos": 56.4, '
                             '"x_check_pos1": 298.6}\n/* stock comment */\n')
            self.board.extruder_json_path = path
            self.board._save_holder_position(GCmd(), 0, 297.3, 55.4)
            data, suffix = self.board._read_stock_json(path)
            self.assertEqual(data['t0_offset_x'], 16.25)
            self.assertEqual(data['t0_offset_y'], 213.9)
            self.assertEqual(data['x_check_pos1'], 298.6)
            self.assertEqual(data['x_check_pos'], 297.3)
            self.assertEqual(data['y_check_pos'], 55.4)
            self.assertEqual(suffix, '/* stock comment */')
            self.assertEqual(len(list(Path(temp).glob('extruder.json.bak-*'))), 1)

    def test_contact_timeout_never_saves(self):
        with tempfile.TemporaryDirectory() as temp:
            self.board.extruder_json_path = os.path.join(temp, 'extruder.json')
            with open(self.board.extruder_json_path, 'w') as target:
                json.dump({'x_check_pos': 298.219}, target)
            self.board.printer = mock.Mock()
            reactor = mock.Mock()
            reactor.monotonic.return_value = 0.
            self.board.printer.get_reactor.return_value = reactor
            toolhead = mock.Mock(get_position=lambda: [250., 50., 20.])
            self.board.printer.lookup_object.side_effect = (
                lambda name, default=None: {
                    'toolhead': toolhead,
                }.get(name, default))
            self.board.safe_z = 10.
            self.board.position_calibration_accel = 1000.
            self.board.position_calibration_release_wait_ms = 0
            self.board._preflight = lambda gcmd, **kwargs: (
                [True] * 4, [False] * 4, None)
            self.board._with_motion = lambda gcmd, operation: operation()
            self.board._wait_for_holder_contact = mock.Mock(
                side_effect=RuntimeError('contact timeout'))
            self.board._save_holder_position = mock.Mock()
            self.board._run = mock.Mock()
            kin = toolhead.get_kinematics()
            with self.assertRaisesRegex(RuntimeError, 'contact timeout'):
                self.board.cmd_extruder_position_calibrate(GCmd(T=0))
            self.board._save_holder_position.assert_not_called()
            kin.clear_homing_state.assert_called_with('xy')
            self.board._run.assert_any_call(
                'SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1')
            self.board._run.assert_any_call(
                'SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=1')
            self.board._run.assert_any_call(
                'RESTORE_GCODE_STATE NAME=C5_HOLDER_CALIBRATE MOVE=0')

    def test_saves_only_after_re_dock(self):
        with tempfile.TemporaryDirectory() as temp:
            self.board.extruder_json_path = os.path.join(temp, 'extruder.json')
            with open(self.board.extruder_json_path, 'w') as target:
                json.dump({'x_check_pos': 298.219}, target)
            self.board.printer = mock.Mock()
            reactor = mock.Mock()
            reactor.monotonic.return_value = 0.
            self.board.printer.get_reactor.return_value = reactor
            toolhead = mock.Mock(get_position=lambda: [250., 50., 20.])
            self.board.printer.lookup_object.side_effect = (
                lambda name, default=None: {
                    'toolhead': toolhead,
                }.get(name, default))
            self.board.safe_z = 10.
            self.board.position_calibration_accel = 1000.
            self.board.position_calibration_release_wait_ms = 0
            self.board.dock_buttons = ['dock%d' % i for i in range(4)]
            self.board.grab_buttons = ['grab%d' % i for i in range(4)]
            self.board.door_buttons = []
            self.board._button = lambda name, gcmd: name in ('dock0', 'grab0')
            self.board._preflight = lambda gcmd, **kwargs: (
                [True] * 4, [False] * 4, None)
            self.board._with_motion = lambda gcmd, operation: operation()
            self.board._wait_for_holder_contact = mock.Mock()
            self.board._measure_holder_position = lambda gcmd, tool: (
                297.3, 55.4)
            calls = []
            def dock(gcmd, tool):
                self.assertEqual(self.board.docks[tool], [297.3, 55.4])
                calls.append('dock')
            self.board._dock = dock
            self.board._save_holder_position = (
                lambda gcmd, tool, x, y: calls.append('save'))
            self.board._run = mock.Mock(side_effect=lambda command:
                calls.append('home') if command == 'G28 X Y' else None)

            self.board.cmd_extruder_position_calibrate(GCmd(T=0))

            self.assertEqual(calls, ['home', 'dock', 'save'])
            self.assertEqual(self.board.docks[0], [297.3, 55.4])

    def test_partial_motor_disable_is_recovered(self):
        with tempfile.TemporaryDirectory() as temp:
            self.board.extruder_json_path = os.path.join(temp, 'extruder.json')
            with open(self.board.extruder_json_path, 'w') as target:
                json.dump({'x_check_pos': 298.219}, target)
            self.board.printer = mock.Mock()
            self.board.printer.get_reactor.return_value.monotonic.return_value = 0.
            toolhead = mock.Mock(get_position=lambda: [250., 50., 20.])
            self.board.printer.lookup_object.side_effect = (
                lambda name, default=None: {
                    'toolhead': toolhead,
                }.get(name, default))
            self.board.safe_z = 10.
            self.board.position_calibration_accel = 1000.
            self.board._preflight = lambda gcmd, **kwargs: (
                [True] * 4, [False] * 4, None)
            self.board._with_motion = lambda gcmd, operation: operation()
            commands = []

            def run(command):
                commands.append(command)
                if command == 'SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=0':
                    raise RuntimeError('Y disable failed')

            self.board._run = run
            with self.assertRaisesRegex(RuntimeError, 'Y disable failed'):
                self.board.cmd_extruder_position_calibrate(GCmd(T=0))
            self.assertIn('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1',
                          commands)
            self.assertIn('SET_STEPPER_ENABLE STEPPER=stepper_y ENABLE=1',
                          commands)
            toolhead.get_kinematics().clear_homing_state.assert_called_with(
                'xy')


if __name__ == '__main__':
    unittest.main()
