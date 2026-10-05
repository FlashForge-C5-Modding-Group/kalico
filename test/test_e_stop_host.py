import pathlib
import sys
import types
import unittest
from unittest import mock


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'klippy'))
with mock.patch.dict(sys.modules, {
        'klippy': types.ModuleType('klippy'),
        'klippy.pins': types.ModuleType('klippy.pins')}):
    from extras import e_stop


class EStopHostTest(unittest.TestCase):
    def test_preserves_no_movement_sentinel_mode(self):
        probe = e_stop.EStopEndstopWrapper.__new__(
            e_stop.EStopEndstopWrapper)
        probe.printer = mock.Mock()
        homing = mock.Mock()
        homing.probing_move.return_value = [9999., 0., 0.]
        probe.printer.lookup_object.return_value = homing
        pos = [10., 20., 30.]

        result = probe.e_stop_move(pos, 5.)

        self.assertEqual(result, [9999., 0., 0.])
        homing.probing_move.assert_called_once_with(
            probe, pos, 5., rase=False, safe_mode=False)


if __name__ == '__main__':
    unittest.main()
