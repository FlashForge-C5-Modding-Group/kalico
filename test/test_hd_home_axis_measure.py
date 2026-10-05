import pathlib
import sys
import types
import unittest
from unittest import mock


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'klippy'))
with mock.patch.dict(sys.modules, {
        'klippy': types.ModuleType('klippy'),
        'klippy.pins': types.ModuleType('klippy.pins')}):
    from extras import hd_home


class HdHomeAxisMeasureTest(unittest.TestCase):
    def test_returns_measured_axis_and_restores_target(self):
        probe = hd_home.HdlmtHome.__new__(hd_home.HdlmtHome)
        probe.stepper_name = 'X'
        probe.position_offset = 10.
        probe.speed = 2.
        probe._probe = mock.Mock(return_value=[-2.3, 80., 10.])

        self.assertEqual(probe.measure_axis(-5.), -2.3)
        self.assertEqual(probe.position_offset, 10.)
        probe._probe.assert_called_once_with(2.)

    def test_restores_target_on_probe_failure(self):
        probe = hd_home.HdlmtHome.__new__(hd_home.HdlmtHome)
        probe.stepper_name = 'Y'
        probe.position_offset = -10.
        probe.speed = 2.
        probe._probe = mock.Mock(side_effect=RuntimeError('endstop failed'))

        with self.assertRaisesRegex(RuntimeError, 'endstop failed'):
            probe.measure_axis(0.)
        self.assertEqual(probe.position_offset, -10.)


if __name__ == '__main__':
    unittest.main()
