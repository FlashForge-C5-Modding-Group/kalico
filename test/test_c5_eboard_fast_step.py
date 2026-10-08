"""Keep the Creator 5 eboard extruder on stock rising-edge stepping."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


# Load only the stepper module, without importing Klippy's optional runtime
# dependencies (such as cffi) into this small configuration test.
package = types.ModuleType("c5_fast_step_test_package")
package.__path__ = [str(Path(__file__).resolve().parents[1] / "klippy")]
sys.modules[package.__name__] = package
chelper = types.ModuleType(package.__name__ + ".chelper")
sys.modules[chelper.__name__] = chelper
spec = importlib.util.spec_from_file_location(
    package.__name__ + ".stepper", Path(package.__path__[0]) / "stepper.py"
)
stepper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stepper)


class FakeCommand:
    def get_command_tag(self):
        return 1


class FakeMCU:
    def __init__(self, fast):
        self.constants = {"STEPPER_STEP_BOTH_EDGE": 1,
                          "MCU": "n32g455ccl7"}
        if fast:
            self.constants.update(
                C5_EBOARD_FAST_EXTRUDER_STEP=1, STEPPER_OPTIMIZED_EDGE=18
            )
        self.commands = []

    def get_constants(self):
        return self.constants

    def get_printer(self):
        return self

    config_error = ValueError

    def seconds_to_clock(self, seconds):
        return round(seconds * 144000000)

    def add_config_cmd(self, command, on_restart=False):
        self.commands.append(command)

    def lookup_command(self, command):
        return FakeCommand()

    def lookup_query_command(self, *args, **kwargs):
        return FakeCommand()

    def get_max_stepper_error(self):
        return 0.000025


def build_stepper(mcu, pin="PB14", pulse=None):
    s = stepper.MCU_stepper.__new__(stepper.MCU_stepper)
    s._mcu = mcu
    s._step_pin = pin
    s._dir_pin = "PB15"
    s._step_pulse_duration = pulse
    s._invert_step = 0
    s._req_step_both_edge = s._step_both_edge = False
    s._oid = len(mcu.commands) + 1
    s._stepqueue = object()
    s._build_config()
    return s


class FastStepTest(unittest.TestCase):
    def setUp(self):
        class FakeLib:
            def stepcompress_fill(self, *args):
                pass

        self.ffi_patch = patch.object(
            stepper.chelper, "get_ffi", return_value=(None, FakeLib()), create=True
        )
        self.ffi_patch.start()
        self.addCleanup(self.ffi_patch.stop)

    def test_fast_capable_firmware_still_uses_rising_edge(self):
        mcu = FakeMCU(fast=True)
        steppers = [build_stepper(mcu) for _ in range(4)]
        configs = [c for c in mcu.commands if c.startswith("config_stepper ")]
        self.assertNotIn("c5_eboard_fast_extruder_step", mcu.commands)
        self.assertEqual(len(configs), 4)
        self.assertTrue(all("invert_step=0 step_pulse_ticks=72" in c
                            for c in configs))
        self.assertTrue(all(not s._step_both_edge for s in steppers))

    def test_old_firmware_keeps_rising_edge_and_short_pulse(self):
        mcu = FakeMCU(fast=False)
        s = build_stepper(mcu)
        self.assertNotIn("c5_eboard_fast_extruder_step", mcu.commands)
        self.assertTrue(
            any("invert_step=0 step_pulse_ticks=72" in c for c in mcu.commands)
        )
        self.assertFalse(s._step_both_edge)

    def test_excessively_long_explicit_pulse_is_rejected(self):
        mcu = FakeMCU(fast=True)
        with self.assertRaisesRegex(ValueError, "step_pulse_duration"):
            build_stepper(mcu, pulse=0.000002)
        self.assertNotIn("c5_eboard_fast_extruder_step", mcu.commands)

    def test_too_short_explicit_pulse_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "step_pulse_duration"):
            build_stepper(FakeMCU(fast=False), pulse=0.0000001)

    def test_other_mcu_keeps_generic_two_microsecond_pulse(self):
        mcu = FakeMCU(fast=False)
        mcu.constants["MCU"] = "stm32f103"
        s = build_stepper(mcu)
        self.assertFalse(s._step_both_edge)
        self.assertTrue(any("step_pulse_ticks=288" in c for c in mcu.commands))


if __name__ == "__main__":
    unittest.main()
