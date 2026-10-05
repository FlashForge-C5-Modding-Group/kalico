import importlib.util
from pathlib import Path
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'gcode_move', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'gcode_move.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **params):
        self.params = params

    def get_float(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def get_int(self, name, default=None, **kwargs):
        return self.params.get(name, default)

    def get(self, name, default=None):
        return self.params.get(name, default)

    def error(self, message):
        return RuntimeError(message)


class Creator5OffsetGuardTests(unittest.TestCase):
    def setUp(self):
        self.move = MODULE.GCodeMove.__new__(MODULE.GCodeMove)
        self.move.printer = mock.Mock()
        self.guard = mock.Mock()
        self.guard.validate_nozzle_z_offset.side_effect = RuntimeError(
            'unsafe nozzle offset')
        self.move.printer.lookup_object.return_value = self.guard
        self.move.homing_position = [0., 0., 2.86, 0.]
        self.move.base_position = [0.] * 4
        self.move.last_position = [0.] * 4

    def test_rejected_z_offset_does_not_partially_change_xy(self):
        with self.assertRaisesRegex(RuntimeError, 'unsafe nozzle offset'):
            self.move.cmd_SET_GCODE_OFFSET(GCmd(X=5., Z=0.))
        self.assertEqual(self.move.homing_position, [0., 0., 2.86, 0.])
        self.assertEqual(self.move.base_position, [0.] * 4)
        self.guard.validate_nozzle_z_offset.assert_called_once_with(
            mock.ANY, 0.)

    def test_restore_state_checks_nozzle_z_before_restoring(self):
        self.move.saved_states = {
            'old': {'homing_position': [0., 0., 0., 0.]}}
        with self.assertRaisesRegex(RuntimeError, 'unsafe nozzle offset'):
            self.move.cmd_RESTORE_GCODE_STATE(GCmd(NAME='old'))
        self.assertEqual(self.move.homing_position[2], 2.86)


if __name__ == '__main__':
    unittest.main()
