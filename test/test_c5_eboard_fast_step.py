"""Keep the Creator 5 eboard TMC and MCU step-edge modes matched."""

import pytest

from klippy import stepper


class FakeCommand:
    def get_command_tag(self):
        return 1


class FakeMCU:
    def __init__(self, fast):
        self.constants = {"STEPPER_STEP_BOTH_EDGE": 1}
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


@pytest.fixture(autouse=True)
def fake_stepcompress(monkeypatch):
    class FakeLib:
        def stepcompress_fill(self, *args):
            pass

    monkeypatch.setattr(stepper.chelper, "get_ffi", lambda: (None, FakeLib()))


def test_fast_firmware_enables_tmc_once_for_four_extruders():
    mcu = FakeMCU(fast=True)
    steppers = [build_stepper(mcu) for _ in range(4)]
    configs = [c for c in mcu.commands if c.startswith("config_stepper ")]
    assert mcu.commands.count("c5_eboard_fast_extruder_step") == 1
    assert len(configs) == 4
    assert all("invert_step=-1 step_pulse_ticks=14" in c for c in configs)
    assert all(s._step_both_edge for s in steppers)


def test_old_firmware_keeps_rising_edge_and_two_microsecond_pulse():
    mcu = FakeMCU(fast=False)
    s = build_stepper(mcu)
    assert "c5_eboard_fast_extruder_step" not in mcu.commands
    assert any("invert_step=0 step_pulse_ticks=288" in c for c in mcu.commands)
    assert not s._step_both_edge


def test_fast_firmware_rejects_incompatible_explicit_pulse():
    with pytest.raises(ValueError, match="at most 500ns"):
        build_stepper(FakeMCU(fast=True), pulse=0.000002)
