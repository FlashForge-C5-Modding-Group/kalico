# Host-side MCLib VFA command tests.
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "klippy/extras/mclib.py"
with mock.patch.dict(sys.modules, {
        "klippy": types.ModuleType("klippy"),
        "klippy.stepper": types.ModuleType("klippy.stepper")}):
    spec = importlib.util.spec_from_file_location("c5_mclib_host", MODULE_PATH)
    mclib = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mclib)


class MCLibHostTest(unittest.TestCase):
    def make_mclib(self, command):
        motor = mclib.MCLIB.__new__(mclib.MCLIB)
        motor.name = "stepper_x"
        motor.oid = 7
        motor.calibrated_damping = {
            4: (40, 41, 42),
            1: (10, 11, 12),
            2: (20, 21, 22),
        }
        motor.set_resonance_damp_cmd = command
        motor.mcu = mock.Mock()
        return motor

    def test_restore_sends_each_saved_harmonic(self):
        command = mock.Mock()
        motor = self.make_mclib(command)
        gcmd = mock.Mock()

        motor.cmd_MCLIB_APPLY_CALIBRATION(gcmd)

        self.assertEqual(command.send.call_args_list, [
            mock.call([7, 1, 10, 11, 12]),
            mock.call([7, 2, 20, 21, 22]),
            mock.call([7, 4, 40, 41, 42]),
        ])
        gcmd.respond_info.assert_called_once()

    def test_set_before_command_lookup_uses_requested_harmonic(self):
        motor = self.make_mclib(None)
        gcmd = mock.Mock()
        gcmd.get_int.return_value = 2
        gcmd.get_float.side_effect = [0.02, 3.1, 3.2]

        motor.cmd_MCLIB_SET_RESONANCE_DAMP(gcmd)

        motor.mcu.add_config_cmd.assert_called_once_with(
            "mclib_set_resonance_damp oid=7 tdx=2 amp=20 phase1=3100 phase2=3200")


if __name__ == "__main__":
    unittest.main()
