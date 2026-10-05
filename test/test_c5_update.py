#!/usr/bin/env python3
# This file may be distributed under the terms of the GNU GPLv3 license.

import bz2
import gzip
import lzma
import tarfile
from unittest import mock
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "scripts" / "build_c5_update.py"
SPEC = importlib.util.spec_from_file_location("c5_update", TOOL_PATH)
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)

LEVEL_PROFILE = TOOL.BOARD_PROFILES["levelBoard"]
APP_START = LEVEL_PROFILE["app_start"]
APP_END = LEVEL_PROFILE["app_end"]
def c5_pin_enumerations():
    eboard = {"PA%d" % pin: pin for pin in range(16)}
    eboard.update({"PB%d" % pin: 16 + pin for pin in range(16)})
    eboard.update({"PC%d" % pin: 32 + pin for pin in range(13, 16)})
    eboard["PG0"] = 96
    eboard["ADC_TEMPERATURE"] = 0xfe

    heaterboard = {"PA%d" % pin: pin for pin in range(16)}
    heaterboard.update({"PB%d" % pin: 16 + pin for pin in range(16)})
    heaterboard.update({"PC%d" % pin: 32 + pin for pin in range(16)})
    heaterboard["PD2"] = 50
    heaterboard["ADC_TEMPERATURE"] = 0xfe

    levelboard = {"PA%d" % pin: pin for pin in range(8)}
    levelboard.update({"PA9": 9, "PA10": 10, "PB1": 17, "PD0": 48})
    return {
        "eBoard": eboard,
        "heaterBoard": heaterboard,
        "levelBoard": levelboard,
    }


C5_REQUIRED_PIN_ENUMERATIONS = c5_pin_enumerations()
MAINBOARD_REQUIRED_CONSTANTS = {
    "MCU": "gd32h737vgt6", "ADC_MAX": 4095,
    "CLOCK_FREQ": 600000000, "RECEIVE_WINDOW": 192,
    "RESERVE_PINS_serial": "PA10,PA9", "SERIAL_BAUD": 230400,
    "STATS_SUMSQ_BASE": 256,
}

MAINBOARD_REQUIRED_COMMANDS = {
    "identify offset=%u count=%c", "allocate_oids count=%c", "get_config",
    "finalize_config crc=%u", "get_clock", "get_uptime",
    "emergency_stop", "clear_shutdown", "reset",
    "debug_nop", "debug_ping data=%*s",
    "debug_read order=%c addr=%u", "debug_write order=%c addr=%u val=%u",
    ("config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c "
     "step_pulse_ticks=%u"),
    "queue_step oid=%c interval=%u count=%hu add=%hi",
    "set_next_step_dir oid=%c dir=%c",
    "reset_step_clock oid=%c clock=%u", "stepper_get_position oid=%c",
    "stepper_stop_on_trigger oid=%c trsync_oid=%c",
    "config_endstop oid=%c pin=%c pull_up=%c",
    ("endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c "
     "rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c"),
    "endstop_query_state oid=%c", "config_trsync oid=%c",
    ("trsync_start oid=%c report_clock=%u report_ticks=%u "
     "expire_reason=%c"),
    "trsync_set_timeout oid=%c clock=%u",
    "trsync_trigger oid=%c reason=%c",
    ("config_digital_out oid=%c pin=%u value=%c default_value=%c "
     "max_duration=%u"),
    "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u",
    "queue_digital_out oid=%c clock=%u on_ticks=%u",
    "update_digital_out oid=%c value=%c",
    "set_digital_out pin=%u value=%c", "config_analog_in oid=%c pin=%u",
    ("query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c "
     "rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c"),
    "config_buttons oid=%c button_count=%c",
    "buttons_add oid=%c pos=%c pin=%u pull_up=%c",
    ("buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c "
     "invert=%c"),
    "buttons_ack oid=%c count=%c",
    "config_mclib oid=%c stepper=%u rs=%u ls=%u km=%u",
    "mclib_config_microstep oid=%c interpolate=%c mstep=%u",
    "mclib_config_stalldetect oid=%c stallthrs=%u",
    "mclib_set_current oid=%c run_current=%u hold_current=%u",
    ("mclib_set_resonance_damp oid=%c tdx=%c amp=%u phase1=%u "
     "phase2=%u"),
    "get_mcu_version",
}

MAINBOARD_REQUIRED_RESPONSES = {
    "identify_response offset=%u data=%.*s",
    "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
    "clock clock=%u", "uptime high=%u clock=%u",
    "stats count=%u sum=%u sumsq=%u", "starting",
    "is_shutdown static_string_id=%hu",
    "shutdown clock=%u static_string_id=%hu", "pong data=%*s",
    "debug_result val=%u", "stepper_position oid=%c pos=%i",
    "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
    "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
    "analog_in_state oid=%c next_clock=%u value=%hu",
    "buttons_state oid=%c ack_count=%c state=%*s",
    "mcu_version year=%u date=%u version=%u",
}

MAINBOARD_OPTIONAL_GENERIC_RESPONSES = {
    "sensor_bulk_data oid=%c sequence=%hu data=%*s",
    ("sensor_bulk_status oid=%c clock=%u query_ticks=%u "
     "next_sequence=%hu buffered=%u possible_overflows=%hu"),
}
MAINBOARD_STEPPER_ENUM = {
    "stepper_x": 0, "stepper_y": 1, "stepper_z": 2, "extruder": 3,
}


def mainboard_pin_enumeration():
    pins = {"PA%d" % pin: pin for pin in range(11)}
    pins.update({"PA%d" % pin: pin for pin in range(13, 16)})
    for port, base in (("B", 16), ("C", 32), ("D", 48), ("E", 64)):
        pins.update({"P%s%d" % (port, pin): base + pin
                     for pin in range(16)})
    pins.update({"PH2": 114, "PH3": 115})
    pins.update({"PJ%d" % pin: 144 + pin for pin in range(16)})
    return pins


MAINBOARD_REQUIRED_ENUMERATIONS = {
    "stepper": MAINBOARD_STEPPER_ENUM,
    "pin": mainboard_pin_enumeration(),
}
def require_mainboard_profile(case):
    if "mainBoardGD" not in TOOL.BOARD_PROFILES:
        case.fail("mainBoardGD builder profile is not implemented")
    return TOOL.BOARD_PROFILES["mainBoardGD"]


def mainboard_dictionary_data(kconfig=None, commands=None, responses=None,
                              enumerations=None, constants=None, output=None):
    return dictionary_data(
        kconfig=kconfig, board="mainBoardGD", commands=commands,
        responses=responses, enumerations=enumerations, constants=constants,
        output=output)

def dictionary_data(kconfig=None, version="test-version",
                    build_versions="test-tools", board="levelBoard",
                    commands=None, responses=None, enumerations=None,
                    constants=None, output=None):
    profile = TOOL.BOARD_PROFILES[board]
    command_formats = (profile["required_commands"] if commands is None
                       else commands)
    commands = {name: index for index, name in enumerate(
        sorted(set(command_formats) - {"identify offset=%u count=%c"}), 2)}
    if "identify offset=%u count=%c" in command_formats:
        commands["identify offset=%u count=%c"] = 1
    first_response = max(commands.values()) + 1
    response_formats = (profile["required_responses"] if responses is None
                        else responses)
    responses = {name: index for index, name in enumerate(
        sorted(set(response_formats) -
               {"identify_response offset=%u data=%.*s"}), first_response)}
    if "identify_response offset=%u data=%.*s" in response_formats:
        responses["identify_response offset=%u data=%.*s"] = 0
    value = {
        "commands": commands, "responses": responses,
        "output": {} if output is None else output,
        "config": dict(profile["required_constants"] if constants is None
                       else constants),
        "enumerations": dict(profile.get("required_enumerations", {}) if
                             enumerations is None else enumerations),
        "kconfig": profile["seed_config"] if kconfig is None else kconfig,
        "version": version, "build_versions": build_versions,
    }
    return json.dumps(value, separators=(",", ":"), sort_keys=True).encode()


def record(kind, address=0, payload=b""):
    body = bytes((len(payload), address >> 8, address & 0xff, kind)) + payload
    return b":" + (body + bytes((-sum(body) & 0xff,))
                   ).hex().upper().encode() + b"\n"


def ihex(records=(), sp=None, reset=None, eof=True, board="levelBoard"):
    if board == "mainBoardGD" and board not in TOOL.BOARD_PROFILES:
        raise AssertionError("mainBoardGD builder profile is not implemented")
    profile = TOOL.BOARD_PROFILES[board]
    app_start = profile["app_start"]
    if sp is None:
        sp = profile["ram_end"]
    if reset is None:
        reset = app_start + 9
    data = record(4, payload=struct.pack(">H", app_start >> 16))
    vectors = struct.pack("<II", sp, reset)
    data += record(0, app_start & 0xffff, vectors + b"\x00\xbf\x00\xbf")
    for entry in records:
        data += entry
    if eof:
        data += record(1)
    return data
def tar_bytes(entries):
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w",
                      format=tarfile.GNU_FORMAT) as archive:
        for entry in entries:
            info = tarfile.TarInfo(entry[0])
            data = entry[1] if len(entry) > 1 else b""
            kind = entry[2] if len(entry) > 2 else tarfile.REGTYPE
            linkname = entry[3] if len(entry) > 3 else ""
            info.type = kind
            info.linkname = linkname
            info.mode = (entry[4] if len(entry) > 4 else
                         (0o755 if kind == tarfile.DIRTYPE else 0o664))
            info.uid = 12
            info.gid = 34
            info.uname = "fixture-user"
            info.gname = "fixture-group"
            info.mtime = 123456789
            info.size = (len(data) if kind in
                         (tarfile.REGTYPE, tarfile.AREGTYPE) else 0)
            archive.addfile(info, io.BytesIO(data) if info.size else None)
    return output.getvalue()


def corrupt_tar_name(value, replacement):
    damaged = bytearray(value)
    damaged[:100] = b"\0" * 100
    damaged[:len(replacement)] = replacement
    damaged[148:156] = b"        "
    checksum = sum(damaged[:512])
    damaged[148:156] = ("%06o\0 " % checksum).encode("ascii")
    return bytes(damaged)


class IntelHexTests(unittest.TestCase):
    def parse(self, value):
        return TOOL.parse_ihex("levelBoard", value)

    def assert_rejected(self, value, *words):
        with self.assertRaises(TOOL.ToolError) as raised:
            self.parse(value)
        text = str(raised.exception).lower()
        for word in words:
            self.assertIn(word.lower(), text)

    def test_accepts_sparse_image_and_reports_vectors_holes_and_hash(self):
        data = ihex([record(0, 0x4020, b"abc")])
        image = self.parse(data)
        self.assertEqual(image["stack_pointer"], 0x20004000)
        self.assertEqual(image["reset_handler"], APP_START + 9)
        self.assertEqual(image["mapped_byte_count"], 15)
        self.assertEqual(image["intervals"], [
                         [APP_START, APP_START + 12],
                         [APP_START + 0x20, APP_START + 0x23]])
        self.assertEqual(image["holes"][0], [APP_START + 12, APP_START + 0x20])
        self.assertEqual(image["holes"][-1], [APP_START + 0x23, APP_END])
        self.assertEqual(image["normalization"], "application-48k-ff-fill-v1")
        normalized = bytearray(b"\xff" * (APP_END - APP_START))
        normalized[:12] = (struct.pack("<II", 0x20004000, APP_START + 9) +
                           b"\x00\xbf\x00\xbf")
        normalized[0x20:0x23] = b"abc"
        self.assertEqual(image["normalized_sha256"],
                         hashlib.sha256(normalized).hexdigest())

    def test_rejects_bad_checksum_with_line_context(self):
        value = bytearray(ihex())
        value[value.index(b"\n") -
    1] = ord("0") if value[value.index(b"\n") -
     1] != ord("0") else ord("1")
        self.assert_rejected(bytes(value), "checksum", "line 1")

    def test_rejects_record_count_mismatch(self):
        line = record(0, 0x4000, b"1234")
        damaged = b":05" + line[3:]
        self.assert_rejected(damaged + record(1), "length", "line 1")

    def test_rejects_bad_extended_address_records_and_segment_address(self):
        self.assert_rejected(
    record(
        4,
        1,
        b"\x08\x00") +
        record(1),
        "address",
         "line 1")
        self.assert_rejected(
    record(
        4,
        0,
        b"\x08") +
        record(1),
        "length",
         "line 1")
        self.assert_rejected(
    record(
        2,
        0,
        b"\x08\x00") +
        record(1),
        "unsupported",
         "02")

    def test_overlap_requires_identical_bytes(self):
        same = ihex([record(0, 0x4008, b"\x00\xbf")])
        self.parse(same)
        different = ihex([record(0, 0x4008, b"\x01\xbf")])
        self.assert_rejected(different, "overlap", "08004008")

    def test_rejects_bytes_below_and_at_upper_bound(self):
        below = record(4,
    payload=b"\x08\x00") + record(0,
    0x3fff,
    b"x") + record(0,
    0x4000,
    struct.pack("<II",
    0x20004000,
     APP_START + 9)) + record(1)
        self.assert_rejected(below, "range", "08003fff")
        upper = ihex([record(4, payload=b"\x08\x01"), record(0, 0, b"x")])
        self.assert_rejected(upper, "range", "08010000")

    def test_detects_window_overflow_before_range(self):
        value = record(4, payload=b"\xff\xff") + \
                       record(0, 0xffff, b"ab") + record(1)
        self.assert_rejected(value, "window", "line 2")

    def test_stack_pointer_bounds_alignment_and_upper_endpoint(self):
        self.parse(ihex(sp=0x20004000))
        for sp, expected in ((0x1ffffff8, "sram"),
                             (0x20004008, "sram"), (0x20000004, "align")):
            with self.subTest(sp=hex(sp)):
                self.assert_rejected(ihex(sp=sp), expected)

    def test_heaterboard_rom_and_sram_boundaries(self):
        accepted = ihex([
            record(4, payload=b"\x08\x07"),
            record(0, 0xffff, b"x"),
        ], board="heaterBoard")
        parsed = TOOL.parse_ihex("heaterBoard", accepted)
        self.assertEqual(parsed["intervals"][-1],
                         [0x0807ffff, 0x08080000])
        for value, address in (
                (ihex([record(4, payload=b"\x08\x08"),
                       record(0, 0, b"x")], board="heaterBoard"),
                 "08080000"),
                (ihex([record(4, payload=b"\x08\x00"),
                       record(0, 0xffff, b"x")], board="heaterBoard"),
                 "0800ffff"),
                (ihex(sp=0x20020008, board="heaterBoard"), "sram")):
            with self.subTest(address=address), self.assertRaises(
                    TOOL.ToolError) as raised:
                TOOL.parse_ihex("heaterBoard", value)
            self.assertIn(address, str(raised.exception).lower())
    def test_mainboard_uses_no_offset_vectors_and_full_flash_range(self):
        require_mainboard_profile(self)
        parsed = TOOL.parse_ihex(
            "mainBoardGD", ihex(board="mainBoardGD"))
        self.assertEqual(parsed["stack_pointer"], 0x24080000)
        self.assertEqual(parsed["reset_handler"], 0x08000009)
        self.assertEqual(parsed["intervals"][0],
                         [0x08000000, 0x0800000c])
        self.assertEqual(parsed["holes"][-1][1], 0x08100000)
        self.assertEqual(parsed["normalization"],
                         "application-1024k-ff-fill-v1")
        upper = ihex([
            record(4, payload=b"\x08\x0f"),
            record(0, 0xffff, b"x"),
        ], board="mainBoardGD")
        upper_report = TOOL.parse_ihex("mainBoardGD", upper)
        self.assertEqual(upper_report["intervals"][-1],
                         [0x080fffff, 0x08100000])
        for value, address in (
                (ihex([record(4, payload=b"\x07\xff"),
                       record(0, 0xffff, b"x")], board="mainBoardGD"),
                 "07ffffff"),
                (ihex([record(4, payload=b"\x08\x10"),
                       record(0, 0, b"x")], board="mainBoardGD"),
                 "08100000")):
            with self.subTest(address=address), self.assertRaises(
                    TOOL.ToolError) as raised:
                TOOL.parse_ihex("mainBoardGD", value)
            self.assertIn(address, str(raised.exception).lower())

    def test_mainboard_axi_stack_pointer_bounds_and_reset_target(self):
        require_mainboard_profile(self)
        for sp in (0x24000008, 0x24080000):
            with self.subTest(accepted=hex(sp)):
                parsed = TOOL.parse_ihex(
                    "mainBoardGD", ihex(sp=sp, board="mainBoardGD"))
                self.assertEqual(parsed["stack_pointer"], sp)
        for sp, word in ((0x23fffff8, "(?i:sram)"),
                         (0x24080008, "(?i:sram)"),
                         (0x24000004, "(?i:align)")):
            with self.subTest(rejected=hex(sp)), self.assertRaisesRegex(
                    TOOL.ToolError, word):
                TOOL.parse_ihex(
                    "mainBoardGD", ihex(sp=sp, board="mainBoardGD"))
        for reset, word in ((0x08000008, "(?i:thumb)"),
                            (0x08100001, "(?i:reset)")):
            with self.subTest(reset=hex(reset)), self.assertRaisesRegex(
                    TOOL.ToolError, word):
                TOOL.parse_ihex(
                    "mainBoardGD",
                    ihex(reset=reset, board="mainBoardGD"))
    def test_reset_requires_thumb_and_application_target(self):
        self.assert_rejected(ihex(reset=APP_START + 8), "thumb")
        self.assert_rejected(ihex(reset=APP_END + 1), "reset", "range")

    def test_eof_is_exactly_once_and_final(self):
        self.assert_rejected(ihex(eof=False), "eof")
        self.assert_rejected(ihex() + record(1), "after eof")
        self.assert_rejected(ihex() + record(0, 0x4008, b"x"), "after eof")

    def test_start_linear_address_is_metadata(self):
        value = ihex([record(5, payload=struct.pack(">I", APP_START))])
        parsed = self.parse(value)
        self.assertEqual(parsed["start_linear_address"], APP_START)
        self.assertEqual(parsed["mapped_byte_count"], 12)
        self.assert_rejected(
            ihex([record(5, 1, struct.pack(">I", APP_START))]), "address")
        self.assert_rejected(ihex([record(5, payload=struct.pack(">I", 0))]),
                             "start", "application")

    def test_rejects_non_ascii_blank_internal_and_trailing_whitespace(self):
        self.assert_rejected(
    ihex().replace(
        b"\n",
        b"\n\n",
        1),
        "blank",
         "line 2")
        self.assert_rejected(ihex().replace(b"\n", b" \n", 1), "whitespace")
        self.assert_rejected(ihex() + b"\xff", "ascii")


class FoundationTests(unittest.TestCase):
    def run_cli(self, *args, env=None):
        return subprocess.run([sys.executable,
    str(TOOL_PATH),
    *args],
    cwd=ROOT,
    text=True,
    capture_output=True,
     env=env)

    def test_each_subcommand_has_help_and_global_defaults(self):
        for command in ("build", "validate", "inspect", "package", "all"):
            with self.subTest(command=command):
                result = self.run_cli(command, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage:", result.stdout.lower())
        parsed = TOOL.create_argument_parser().parse_args(
            ["build", "--board", "levelBoard", "--output-dir", "x"])
        self.assertEqual((parsed.cross_prefix, parsed.openssl,
                         parsed.jobs), ("arm-none-eabi-", "openssl", 1))

    def test_expected_input_error_has_exit_two_without_traceback(self):
        result = self.run_cli(
            "validate", "--board", "levelBoard",
            "--firmware", "missing.hex",
            "--elf", "missing.elf",
            "--dictionary", "missing.dict")
        self.assertEqual(result.returncode, 2)
        self.assertNotIn("traceback", result.stderr.lower())
        self.assertEqual(result.stdout, "")

    def test_package_commands_reject_dirty_repository_before_work(self):
        dirty = {"commit": "a" * 40, "dirty": True}
        commands = (
            (["package", "--template", "template.tgz",
              "--firmware", "levelBoard", "firmware.hex", "firmware.elf",
              "firmware.dict", "--output", "Creator5Pro-test.tgz"],
             "package_update"),
            (["all", "--board", "levelBoard", "--template", "template.tgz",
              "--output-dir", "build", "--output",
              "Creator5Pro-test.tgz"],
             "run_all_stages"),
        )
        for arguments, operation_name in commands:
            with self.subTest(command=arguments[0]), \
                    mock.patch.object(TOOL, "_repository_state",
                                      return_value=dirty), \
                    mock.patch.object(TOOL, operation_name) as operation, \
                    contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertEqual(TOOL.main(arguments), 2)
                self.assertIn("dirty repository", stderr.getvalue())
                operation.assert_not_called()

    def test_output_root_policy_accepts_external_or_out_only(self):
        with tempfile.TemporaryDirectory() as outside:
            self.assertEqual(
    TOOL.validate_output_root(
        Path(outside)),
         Path(outside).resolve())
        self.assertEqual(
    TOOL.validate_output_root(
        ROOT / "out" / "firmware"),
         (ROOT / "out" / "firmware").resolve())
        for path in (ROOT, ROOT / "scripts", ROOT / "src" / "generated"):
            with self.subTest(path=path), self.assertRaises(TOOL.ToolError):
                TOOL.validate_output_root(path)
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_output_root(Path(ROOT.anchor))

    def test_build_rejects_nonempty_output_before_running_tools(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "build"
            output.mkdir()
            sentinel = output / "keep.txt"
            sentinel.write_text("owned")
            with self.assertRaisesRegex(TOOL.ToolError, "not empty") as raised:
                TOOL.build_firmware(
                    "levelBoard", output, 1, "missing-toolchain-", "openssl")
            self.assertEqual(raised.exception.exit_code, 2)
            self.assertEqual(sentinel.read_text(), "owned")

    def test_output_root_rejects_symlink_into_repository(self):
        with tempfile.TemporaryDirectory() as temp:
            link = Path(temp) / "linked"
            try:
                link.symlink_to(ROOT / "scripts", target_is_directory=True)
            except OSError:
                self.skipTest("symlink creation unavailable")
            with self.assertRaises(TOOL.ToolError):
                TOOL.validate_output_root(link)
    def test_output_root_rejects_repository_out_alias_into_source(self):
        with tempfile.TemporaryDirectory() as temp:
            fake_repo = Path(temp)
            source = fake_repo / "src"
            source.mkdir()
            try:
                (fake_repo / "out").symlink_to(
                    source, target_is_directory=True)
            except OSError:
                self.skipTest("symlink creation unavailable")
            original = TOOL.REPO_ROOT
            TOOL.REPO_ROOT = fake_repo
            try:
                with self.assertRaisesRegex(
                        TOOL.ToolError,
                        "outside.*repository|out directory"):
                    TOOL.validate_output_root(fake_repo / "out" / "generated")
            finally:
                TOOL.REPO_ROOT = original

    def test_build_path_filter_rejects_command_substitution_and_make_syntax(
        self):
        for value in ("safe\x60touch-pwned\x60", "safe$(touch-pwned)",
                      "safe;touch-pwned", "safe#comment", "safe=value",
                      r"C:\repo\out", "safe:colon", r"safe\suffix"):
            with self.subTest(value=value):
                self.assertTrue(TOOL._unsupported_build_path(value))
        for value in ("arm-none-eabi-", "/tmp/build", "safe+path"):
            with self.subTest(safe=value):
                self.assertFalse(TOOL._unsupported_build_path(value))

    def test_filesystem_failure_is_exit_two_without_traceback_or_path(self):
        with tempfile.TemporaryDirectory() as temp:
            parent = Path(temp) / "regular-file"
            parent.write_text("not a directory")
            output = parent / "build"
            result = self.run_cli(
                "build", "--board", "levelBoard",
                "--output-dir", str(output))
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertNotIn("traceback", result.stderr.lower())
            self.assertNotIn(str(Path(temp)), result.stderr)
            self.assertEqual(result.stdout, "")

    def test_cyclic_symlinks_are_sanitized_input_and_output_errors(self):
        with tempfile.TemporaryDirectory() as temp:
            loop = Path(temp) / "loop"
            try:
                loop.symlink_to(loop, target_is_directory=True)
            except OSError:
                self.skipTest("symlink creation unavailable")
            for arguments in (
                    ("build", "--board", "levelBoard",
                     "--output-dir", str(loop / "build")),
                    ("validate", "--board", "levelBoard",
                     "--firmware", str(loop), "--elf", "missing.elf",
                     "--dictionary", "missing.dict")):
                with self.subTest(command=arguments[0]):
                    result = self.run_cli(*arguments)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertNotIn("traceback", result.stderr.lower())
                    self.assertNotIn(str(Path(temp)), result.stderr)
                    self.assertEqual(result.stdout, "")


class ConfigTests(unittest.TestCase):

    def test_sanitizer_redacts_paths_and_secret(self):
        secret = "test-generated-secret"
        sanitized = TOOL.sanitize_diagnostic(
    f"failure {secret} at C:\\private\\file", secret)
        self.assertNotIn(secret, sanitized)
        self.assertNotIn("C:\\private", sanitized)
        self.assertIn("[redacted]", sanitized)
    def test_objdump_parser_rejects_unrecognized_section_rows(self):
        output = """Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .bad name     00000004  20004000  20004000  00000034  2**2
                  ALLOC
"""
        with self.assertRaisesRegex(TOOL.ToolError, "ambiguous.*section"):
            TOOL._parse_sections(output, 1024)

    def test_dictionary_uses_resolved_kconfig_not_literal_lines(self):
        contradictory = (
            TOOL.BOARD_PROFILES["levelBoard"]["seed_config"] +
            "CONFIG_MACH_STM32F103=y\n")
        with self.assertRaisesRegex(TOOL.ToolError, "Kconfig|hardware"):
            TOOL._load_dictionary(
                "levelBoard", dictionary_data(kconfig=contradictory))

    def test_dictionary_membership_uses_message_parser_classification(self):
        value = json.loads(dictionary_data())
        required_response = "trigger_threshold threshold=%i"
        value["responses"][required_response] = \
            value["commands"]["get_mcu_version"]
        with self.assertRaisesRegex(TOOL.ToolError,
                                    "response|membership|conflicting "
                                    "message ID"):
            TOOL._load_dictionary("levelBoard", json.dumps(value).encode())
    def test_dictionary_rejects_conflicting_ids_and_nonoutput_names(self):
        value = json.loads(dictionary_data())
        responses = sorted(
            TOOL.BOARD_PROFILES["levelBoard"]["required_responses"] -
            {"identify_response offset=%u data=%.*s"})
        value["responses"][responses[1]] = value["responses"][responses[0]]
        with self.assertRaisesRegex(
                TOOL.ToolError, "conflicting.*ID|ID.*conflict"):
            TOOL._load_dictionary("levelBoard", json.dumps(value).encode())

        value = json.loads(dictionary_data())
        next_id = max(value["responses"].values()) + 1
        value["responses"]["duplicate value=%u"] = next_id
        value["responses"]["duplicate value=%c"] = next_id + 1
        with self.assertRaisesRegex(
                TOOL.ToolError, "conflicting.*name|name.*conflict"):
            TOOL._load_dictionary("levelBoard", json.dumps(value).encode())

    def test_dictionary_metadata_paths_are_explicitly_redacted(self):
        report = TOOL._load_dictionary(
            "levelBoard", dictionary_data(
                version="version[/private/build/tree]",
                build_versions=r"v@C:\private\toolchain"))
        self.assertEqual(report["version"], "[redacted: unsafe metadata]")
        self.assertEqual(
    report["build_versions"],
     "[redacted: unsafe metadata]")

    def test_malformed_dictionary_does_not_log_parser_traceback(self):
        value = json.loads(dictionary_data())
        value["commands"]["malformed value=%q"] = max(
            value["commands"].values()) + 100
        captured = io.StringIO()
        with contextlib.redirect_stderr(captured):
            with self.assertRaisesRegex(
                    TOOL.ToolError, "invalid protocol dictionary"):
                TOOL._load_dictionary("levelBoard", json.dumps(value).encode())
        self.assertEqual(captured.getvalue(), "")


class SharedN32ProfileTests(unittest.TestCase):
    def test_virtual_pg0_is_exposed_only_by_eboard(self):
        for board, expected in (("eBoard", 96),
                                ("heaterBoard", None),
                                ("levelBoard", None)):
            with self.subTest(board=board):
                pins = (TOOL.BOARD_PROFILES[board]
                        ["required_enumerations"]["pin"])
                self.assertEqual(pins.get("PG0"), expected)

    def test_profiles_reserve_the_physical_usart1_pins(self):
        for board in ("eBoard", "heaterBoard", "levelBoard"):
            with self.subTest(board=board):
                self.assertEqual(
                    TOOL.BOARD_PROFILES[board]["required_constants"]
                    ["RESERVE_PINS_serial"], "PA10,PA9")

    def test_profile_serial_reservation_blocks_host_pin_claims(self):
        pins_path = ROOT / "klippy" / "pins.py"
        spec = importlib.util.spec_from_file_location("c5_test_pins", pins_path)
        pins_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pins_module)
        for board in ("eBoard", "heaterBoard", "levelBoard"):
            resolver = pins_module.PinResolver()
            reservation = (TOOL.BOARD_PROFILES[board]["required_constants"]
                           ["RESERVE_PINS_serial"])
            for pin in reservation.split(","):
                resolver.reserve_pin(pin, "serial")
            for pin in ("PA9", "PA10"):
                with self.subTest(board=board, pin=pin), \
                        self.assertRaisesRegex(
                            pins_module.error, "reserved for serial"):
                    resolver.update_command("set_digital_out pin=" + pin)
            self.assertEqual(
                resolver.update_command("set_digital_out pin=PA0"),
                "set_digital_out pin=PA0")

    def test_profiles_require_exact_package_pin_surfaces(self):
        for board, pins in C5_REQUIRED_PIN_ENUMERATIONS.items():
            with self.subTest(board=board):
                self.assertEqual(
                    TOOL.BOARD_PROFILES[board]["required_enumerations"],
                    {"pin": pins})

    def test_dictionary_rejects_missing_or_unbonded_package_pins(self):
        extra_pin = {
            "eBoard": ("PC0", 32),
            "heaterBoard": ("PD3", 51),
            "levelBoard": ("PA8", 8),
        }
        for board, expected in C5_REQUIRED_PIN_ENUMERATIONS.items():
            missing = dict(expected)
            missing.pop(next(iter(expected)))
            extra = dict(expected)
            name, value = extra_pin[board]
            extra[name] = value
            for kind, pins in (("missing", missing), ("extra", extra)):
                with self.subTest(board=board, kind=kind), \
                        self.assertRaisesRegex(
                            TOOL.ToolError,
                            "dictionary enumeration pin does not match"):
                    TOOL._load_dictionary(
                        board, dictionary_data(
                            board=board, enumerations={"pin": pins}))


class MainboardProfileTests(unittest.TestCase):
    def setUp(self):
        require_mainboard_profile(self)
    def test_profile_has_exact_target_dictionary_and_updater_contract(self):
        profile = require_mainboard_profile(self)
        self.assertEqual(profile["firmware_name"], "mainBoardGD.hex")
        self.assertEqual((profile["app_start"], profile["app_end"]),
                         (0x08000000, 0x08100000))
        self.assertEqual((profile["ram_start"], profile["ram_end"]),
                         (0x24000000, 0x24080000))
        self.assertEqual(profile["normalization"],
                         "application-1024k-ff-fill-v1")
        self.assertEqual(profile["updater_name"], "ISPCommand")
        self.assertEqual(profile["required_constants"],
                         MAINBOARD_REQUIRED_CONSTANTS)
        self.assertEqual(profile["required_commands"],
                         MAINBOARD_REQUIRED_COMMANDS)
        self.assertEqual(profile["required_responses"],
                         MAINBOARD_REQUIRED_RESPONSES)
        self.assertEqual(profile["required_enumerations"],
                         MAINBOARD_REQUIRED_ENUMERATIONS)
        required_config = {
            "MACH_STM32": "y", "MACH_GD32H7": "y",
            "MACH_GD32H737VGT6": "y", "C5_MAINBOARDGD": "y",
            "MCU": "gd32h737vgt6", "STM32_CLOCK_REF_25M": "y",
            "GD32_SERIAL_USART0": "y", "WANT_ADC": "y",
            "WANT_BUTTONS": "y", "INLINE_STEPPER_HACK": "y",
            "HAVE_STEPPER_OPTIMIZED_BOTH_EDGE": "n",
            "WANT_STEPPER_OPTIMIZED_BOTH_EDGE": "n",
            "ARMCM_FLASH_SIZE_IS_TOTAL": "y",
            "ARMCM_EXPLICIT_RESET_ENTRY": "y",
            "FLASH_APPLICATION_ADDRESS": "0x08000000",
            "FLASH_BOOT_ADDRESS": "0x08000000", "FLASH_SIZE": "0x100000",
            "RAM_START": "0x24000000", "RAM_SIZE": "0x80000",
            "STACK_SIZE": "4096", "CLOCK_FREQ": "600000000",
            "CLOCK_REF_FREQ": "25000000", "SERIAL_BAUD": "230400",
        }
        self.assertEqual(profile["required_config"], required_config)
        self.assertTrue({
            "C5_EBOARD", "C5_HEATERBOARD", "C5_LEVELBOARD", "MACH_STM32H7",
        }.issubset(profile["forbidden_config"]))
        fixture = ROOT / "test" / "configs" / "c5-mainboardgd.config"
        self.assertEqual(
            TOOL._load_resolved_config("mainBoardGD", fixture),
            TOOL._load_resolved_config_text(
                "mainBoardGD", profile["seed_config"]))
        for board in ("eBoard", "heaterBoard", "levelBoard"):
            self.assertEqual(TOOL.BOARD_PROFILES[board]["updater_name"],
                             "IAPCommand")

    def test_dictionary_accepts_exact_constants_formats_and_enumerations(self):
        report = TOOL._load_dictionary(
            "mainBoardGD", dictionary_data(board="mainBoardGD"))
        self.assertEqual(report["constants"], MAINBOARD_REQUIRED_CONSTANTS)
        self.assertEqual(report["resolved_config"]["MCU"],
                         "gd32h737vgt6")
        self.assertEqual(
            report["resolved_config"]["HAVE_STEPPER_OPTIMIZED_BOTH_EDGE"],
            "n")
        self.assertEqual(
            report["resolved_config"]["WANT_STEPPER_OPTIMIZED_BOTH_EDGE"],
            "n")

    def test_dictionary_accepts_upstream_generic_sensor_bulk_responses(self):
        report = TOOL._load_dictionary(
            "mainBoardGD", mainboard_dictionary_data(
                responses=(MAINBOARD_REQUIRED_RESPONSES |
                           MAINBOARD_OPTIONAL_GENERIC_RESPONSES)))
        self.assertEqual(report["constants"], MAINBOARD_REQUIRED_CONSTANTS)

    def test_dictionary_rejects_each_omitted_required_interface(self):
        for msgformat in sorted(MAINBOARD_REQUIRED_COMMANDS):
            with self.subTest(kind="command", msgformat=msgformat), \
                    self.assertRaisesRegex(TOOL.ToolError,
                                           "missing required command"):
                TOOL._load_dictionary(
                    "mainBoardGD", mainboard_dictionary_data(
                        commands=MAINBOARD_REQUIRED_COMMANDS - {msgformat}))
        for msgformat in sorted(MAINBOARD_REQUIRED_RESPONSES):
            with self.subTest(kind="response", msgformat=msgformat), \
                    self.assertRaisesRegex(TOOL.ToolError,
                                           "missing required response"):
                TOOL._load_dictionary(
                    "mainBoardGD", mainboard_dictionary_data(
                        responses=MAINBOARD_REQUIRED_RESPONSES - {msgformat}))

    def test_dictionary_requires_exact_pin_and_four_slot_motor_names(self):
        TOOL._load_dictionary("mainBoardGD", mainboard_dictionary_data())
        variants = []
        for enum_name, member in (("stepper", "extruder"),
                                  ("pin", "PJ15")):
            missing = {name: dict(values) for name, values in
                       MAINBOARD_REQUIRED_ENUMERATIONS.items()}
            missing[enum_name].pop(member)
            variants.append(("missing-" + member, missing))
        renamed = {name: dict(values) for name, values in
                   MAINBOARD_REQUIRED_ENUMERATIONS.items()}
        renamed["stepper"]["stepper_e"] = renamed["stepper"].pop("extruder")
        variants.append(("renamed-extruder", renamed))
        extra = {name: dict(values) for name, values in
                 MAINBOARD_REQUIRED_ENUMERATIONS.items()}
        extra["pin"]["PA11"] = 11
        variants.append(("unapproved-pa11", extra))
        for name, enumerations in variants:
            with self.subTest(name=name), self.assertRaisesRegex(
                    TOOL.ToolError,
                    "dictionary enumeration (pin|stepper) does not match"):
                TOOL._load_dictionary(
                    "mainBoardGD", mainboard_dictionary_data(
                        enumerations=enumerations))

    def test_dictionary_rejects_wrong_constants_and_other_board_contracts(self):
        for name in sorted(MAINBOARD_REQUIRED_CONSTANTS):
            constants = dict(MAINBOARD_REQUIRED_CONSTANTS)
            value = constants[name]
            constants[name] = (
                value + 1 if isinstance(value, int) else value + "x")
            with self.subTest(constant=name), self.assertRaisesRegex(
                    TOOL.ToolError, "dictionary constant"):
                TOOL._load_dictionary(
                    "mainBoardGD", mainboard_dictionary_data(
                        constants=constants))
        for board in ("eBoard", "heaterBoard", "levelBoard"):
            with self.subTest(mainboard_as=board), self.assertRaises(
                    TOOL.ToolError):
                TOOL._load_dictionary(board, mainboard_dictionary_data())
            with self.subTest(board_as_mainboard=board), self.assertRaises(
                    TOOL.ToolError):
                TOOL._load_dictionary(
                    "mainBoardGD", dictionary_data(board=board))

    def test_dictionary_rejects_omitted_stock_interfaces(self):
        modern_adc = (
            "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c "
            "rest_ticks=%u bytes_per_report=%c min_value=%hu max_value=%hu "
            "range_check_count=%c")
        forbidden = (
            ("commands", "get_basic_param num=%u"),
            ("commands", "set_trigger_threshold threshold=%i"),
            ("commands", modern_adc),
            ("commands", ("config_lis2dw oid=%c bus_oid=%c "
                          "bus_oid_type=%c lis_chip_type=%c")),
            ("commands", "query_lis2dw oid=%c rest_ticks=%u"),
            ("commands", "query_lis2dw_status oid=%c"),
            ("responses", "param_value value=%u reserve=%u"),
            ("responses", "trigger_threshold threshold=%i"),
            ("responses", "peel_data value=%i"),
            ("commands", "pa_action action=%u pc=%u"),
            ("responses", "pa_value value=%u"),
            ("output", "GDMainboard close=%hu Close_num=%hu Temp_waketime=%hu"),
        )
        for table, msgformat in forbidden:
            value = json.loads(mainboard_dictionary_data())
            value[table][msgformat] = max(
                set(value["commands"].values()) |
                set(value["responses"].values()) |
                set(value["output"].values()) | {0}) + 1
            with self.subTest(table=table, msgformat=msgformat), \
                    self.assertRaises(TOOL.ToolError):
                TOOL._load_dictionary(
                    "mainBoardGD", json.dumps(value).encode())
        constants = dict(MAINBOARD_REQUIRED_CONSTANTS,
                         STEPPER_OPTIMIZED_EDGE=1)
        with self.assertRaises(TOOL.ToolError):
            TOOL._load_dictionary(
                "mainBoardGD",
                mainboard_dictionary_data(constants=constants))


SYNTHETIC_TOOL_NAMES = tuple("arm-none-eabi-" + name for name in
                             ("as", "ld", "objcopy", "objdump", "nm", "gcc"))


@unittest.skipUnless(all(shutil.which(name) for name in SYNTHETIC_TOOL_NAMES),
                     "ARM binutils are required for synthetic ELF validation")
class SyntheticElfValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace = tempfile.TemporaryDirectory()
        cls.directory = Path(cls.workspace.name)
        cls.dictionary = cls.directory / "firmware.dict"
        cls.dictionary.write_bytes(dictionary_data())
        (cls.directory / "dictionary.z").write_bytes(
            zlib.compress(cls.dictionary.read_bytes(), 9))
        (cls.directory / "fixture.S").write_text(r"""
.syntax unified
.cpu cortex-m3
.thumb
.section .vectors,"a",%progbits
.word 0x20004000
.word reset_handler + 1
.section .text,"ax",%progbits
.thumb_func
.global reset_handler
.type reset_handler,%function
reset_handler:
  nop
  nop
  bx lr
.size reset_handler, .-reset_handler
.section .rodata,"a",%progbits
.balign 4
.global command_identify_data
.type command_identify_data,%object
command_identify_data:
.incbin "dictionary.z"
.size command_identify_data, .-command_identify_data
.section .data,"aw",%progbits
.global data_word
data_word:
.word 0x12345678
.section .bss,"aw",%nobits
.space 8
""", encoding="ascii")
        (cls.directory / "fixture.ld").write_text("""
ENTRY(reset_handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 0xC000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4000
}
SECTIONS {
  .vectors 0x08004000 : { KEEP(*(.vectors)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
  .data : { *(.data*) } > RAM AT> FLASH
  .bss (NOLOAD) : { *(.bss*) } > RAM
}
""", encoding="ascii")
        cls._checked(["arm-none-eabi-as", "-mcpu=cortex-m3", "-mthumb",
                      "-o", "fixture.o", "fixture.S"])
        cls._checked(["arm-none-eabi-ld", "-T", "fixture.ld", "-o",
                      "firmware.elf", "fixture.o"])
        cls._checked(["arm-none-eabi-objcopy", "-O", "ihex", "firmware.elf",
                      "firmware.hex"])
        cls.elf = cls.directory / "firmware.elf"
        cls.firmware = cls.directory / "firmware.hex"

    @classmethod
    def tearDownClass(cls):
        cls.workspace.cleanup()

    @classmethod
    def _checked(cls, command):
        result = subprocess.run(command, cwd=cls.directory, text=True,
                                capture_output=True)
        if result.returncode:
            raise RuntimeError(
    "synthetic ARM fixture failed: " +
     result.stderr)

    def run_cli(self, firmware="firmware.hex", elf="firmware.elf",
                dictionary="firmware.dict"):
        return subprocess.run(
            [sys.executable, str(TOOL_PATH), "validate",
             "--board", "levelBoard", "--firmware", firmware,
             "--elf", elf, "--dictionary", dictionary],
            cwd=self.directory, text=True, capture_output=True)

    def test_validate_cli_accepts_relative_inputs_and_initialized_data(self):
        result = self.run_cli()
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["resolved_config"]["MACH_N32G430F8S7"], "y")
        self.assertEqual(report["resolved_config"]["RAM_SIZE"], "0x4000")
        self.assertNotIn(str(self.directory), result.stdout)

    def test_validate_cli_rejects_hex_and_dictionary_correspondence_failures(
        self):
        lines = self.firmware.read_bytes().splitlines(keepends=True)
        for index, line in enumerate(lines):
            raw = bytearray.fromhex(line[1:].strip().decode())
            if raw[3] == 0 and raw[0] and ((raw[1] << 8) | raw[2]) >= 0x4008:
                raw[4] ^= 1
                raw[-1] = (-sum(raw[:-1])) & 0xff
                lines[index] = b":" + raw.hex().upper().encode() + b"\n"
                break
        (self.directory / "mutated.hex").write_bytes(b"".join(lines))
        mismatch = self.run_cli(firmware="mutated.hex")
        self.assertEqual(mismatch.returncode, 2, mismatch.stderr)
        self.assertIn("mismatch", mismatch.stderr.lower())
        (self.directory / "unrelated.dict").write_bytes(
            dictionary_data(version="unrelated"))
        unrelated = self.run_cli(dictionary="unrelated.dict")
        self.assertEqual(unrelated.returncode, 2, unrelated.stderr)
        self.assertIn("embedded dictionary", unrelated.stderr.lower())

    def test_validate_cli_bounds_initialized_data_vma_and_flash_lma(self):
        for option, value, output, words in (
                ("--change-section-vma", ".data=0x20003fff",
                 "crossing-vma.elf",
                 ("ram", "bounds")),
                ("--change-section-vma", ".data=0x10000000", "outside-vma.elf",
                 ("allocated", "outside", "sram")),
                ("--change-section-vma", ".bss=0x08005000", "bad-bss.elf",
                 ("bss", "sram")),
                ("--change-section-lma", ".data=0x0800ffff", "bad-lma.elf",
                 ("load", "flash", "bounds"))):
            with self.subTest(option=option):
                self._checked(["arm-none-eabi-objcopy", option, value,
                               "firmware.elf", output])
                result = self.run_cli(elf=output)
                self.assertEqual(result.returncode, 2, result.stderr)
                for word in words:
                    self.assertIn(word, result.stderr.lower())
                self.assertNotIn("traceback", result.stderr.lower())
    def test_package_cli_rejects_unsupported_template_without_publishing(self):
        openssl = shutil.which("openssl")
        if not openssl:
            self.skipTest("OpenSSL is required for unsupported templates")
        secret = hashlib.sha256(os.urandom(32)).hexdigest()
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "Creator5Pro-unsupported.tgz"
            plain = tar_bytes([("./plain", b"not canonical")])
            with mock.patch.dict(os.environ,
                                 {"C5_UPDATE_PASSPHRASE": secret},
                                 clear=False):
                template = Path(temp) / "template.tgz"
                template.write_bytes(
                    TOOL.openssl_crypt(plain, decrypt=False, openssl=openssl))
                stderr = io.StringIO()
                with mock.patch.object(
                        TOOL, "_repository_state",
                        return_value={"commit": "a" * 40, "dirty": False}), \
                        contextlib.redirect_stderr(stderr), \
                        contextlib.redirect_stdout(io.StringIO()):
                    returncode = TOOL.main([
                        "--openssl", openssl, "package",
                        "--template", str(template),
                        "--firmware", "levelBoard",
                        str(self.directory / "firmware.hex"),
                        str(self.directory / "firmware.elf"),
                        str(self.directory / "firmware.dict"),
                        "--output", str(output),
                    ])
            diagnostic = stderr.getvalue()
            self.assertEqual(returncode, 2, diagnostic)
            self.assertIn("unsupported canonical template", diagnostic)
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())
            self.assertNotIn(secret, diagnostic)

class N32G45xRegisterModelTests(unittest.TestCase):
    MODEL_SOURCE = ROOT / "test" / "n32g45x_register_model.c"
    VIRTUAL_ENDSTOP_SOURCE = ROOT / "test" / "c5_virtual_endstop.c"

    def compile_model(self, workspace, reference_frequency, source=None,
                      name="n32-model", include_dirs=()):
        source = self.MODEL_SOURCE if source is None else source
        executable = workspace / (name + (".exe" if os.name == "nt" else ""))
        include_dirs = (ROOT / "test", ROOT / "test" /
                        "c5_levelboard_stubs") + tuple(include_dirs)
        compiler = next((shutil.which(candidate)
                         for candidate in ("cc", "gcc", "clang")
                         if shutil.which(candidate)), None)
        if compiler:
            command = [compiler, "-std=c11", "-O2"]
            for include_dir in include_dirs:
                command.extend(("-I", str(include_dir)))
            command.extend((
                "-DCONFIG_CLOCK_REF_FREQ=%d" % reference_frequency,
                str(source), "-o", str(executable)))
            completed = subprocess.run(
                command, capture_output=True, text=True)
            return completed, executable

        if os.name != "nt":
            self.fail("A host C compiler is required for the N32 models")
        vswhere = (Path(os.environ["ProgramFiles(x86)"]) /
                   "Microsoft Visual Studio" / "Installer" / "vswhere.exe")
        located = subprocess.run(
            [str(vswhere), "-latest", "-products", "*", "-requires",
             "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
             "-property", "installationPath"],
            check=True, capture_output=True, text=True)
        installation = Path(located.stdout.strip())
        vcvars = installation / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
        script = workspace / ("compile-" + name + ".cmd")
        include_flags = " ".join('/I"%s"' % path for path in include_dirs)
        object_path = workspace / (name + ".obj")
        script.write_text(
            '@call "%s" >nul\n'
            '@cl /nologo /O2 /std:c11 %s '
            '/DCONFIG_CLOCK_REF_FREQ=%d "%s" /Fo:"%s" /Fe:"%s"\n'
            % (vcvars, include_flags, reference_frequency, source,
               object_path, executable))
        completed = subprocess.run(
            [os.environ["COMSPEC"], "/d", "/c", str(script)],
            capture_output=True, text=True)
        return completed, executable

    def test_eboard_virtual_endstop_is_not_physical_gpio(self):
        with tempfile.TemporaryDirectory() as temp:
            compiled, executable = self.compile_model(
                Path(temp), 12000000, self.VIRTUAL_ENDSTOP_SOURCE,
                "virtual-endstop",
                (ROOT / "test" / "c5_virtual_endstop_stubs",
                 ROOT / "src"))
            self.assertEqual(
                compiled.returncode, 0, compiled.stdout + compiled.stderr)
            executed = subprocess.run(
                [str(executable)], capture_output=True, text=True)
            self.assertEqual(
                executed.returncode, 0, executed.stdout + executed.stderr)
            self.assertEqual(
                executed.stdout.strip(), "virtual-endstop-model-ok")

    def test_production_gpio_and_clock_startup_register_behavior(self):
        for reference_frequency in (12000000, 8000000, 16000000, 24000000):
            with self.subTest(reference_frequency=reference_frequency):
                with tempfile.TemporaryDirectory() as temp:
                    compiled, executable = self.compile_model(
                        Path(temp), reference_frequency)
                    self.assertEqual(
                        compiled.returncode, 0,
                        compiled.stdout + compiled.stderr)
                    executed = subprocess.run(
                        [str(executable)], capture_output=True, text=True)
                    self.assertEqual(
                        executed.returncode, 0,
                        executed.stdout + executed.stderr)
                    self.assertEqual(
                        executed.stdout.strip(),
                        "ref=%d clock=144000000 model-ok"
                        % reference_frequency)

    def test_unsupported_n32_pll_plans_are_rejected_at_compile_time(self):
        for reference_frequency in (1000000, 20000000, 25000000):
            with self.subTest(reference_frequency=reference_frequency):
                with tempfile.TemporaryDirectory() as temp:
                    compiled, unused = self.compile_model(
                        Path(temp), reference_frequency)
                    self.assertNotEqual(compiled.returncode, 0)


@unittest.skipUnless(os.environ.get("RUN_C5_BUILD_TESTS") == "1",
                     "set RUN_C5_BUILD_TESTS=1 with the ARM toolchain "
                     "available")
class RealFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace = tempfile.TemporaryDirectory()
        workspace = Path(cls.workspace.name)
        cls.output = workspace / "levelBoard"
        cls.eboard_output = workspace / "eBoard"
        cls.heaterboard_output = workspace / "heaterBoard"
        cls.repo_config = ROOT / ".config"
        cls.config_before = (cls.repo_config.read_bytes()
                             if cls.repo_config.exists() else None)
        cls.build_report = TOOL.build_firmware(
            "levelBoard", cls.output, 1, "arm-none-eabi-", "openssl")
        cls.eboard_build_report = TOOL.build_firmware(
            "eBoard", cls.eboard_output, 1, "arm-none-eabi-", "openssl")
        cls.heaterboard_build_report = TOOL.build_firmware(
            "heaterBoard", cls.heaterboard_output, 1,
            "arm-none-eabi-", "openssl")
    @classmethod
    def tearDownClass(cls):
        cls.workspace.cleanup()

    def test_actual_c5_build_is_fully_valid_and_does_not_touch_repo_config(
        self):
        paths = self.build_report["products"]
        for key in ("elf", "bin", "firmware", "dictionary", "config"):
            self.assertTrue((self.output / paths[key]).is_file(), key)
        current = (self.repo_config.read_bytes()
                   if self.repo_config.exists() else None)
        self.assertEqual(current, self.config_before)
        firmware = self.build_report["firmware"]
        self.assertEqual(
    firmware["normalization"],
     "application-48k-ff-fill-v1")
        self.assertEqual(
    firmware["bounds"]["application"], [
        APP_START, APP_END])
        self.assertEqual(firmware["constants"]["RECEIVE_WINDOW"], 192)
        self.assertEqual(firmware["constants"]["SERIAL_BAUD"], 230400)
        self.assertEqual(firmware["resolved_config"]["MCU"],
                         "n32g430f8s7")
        self.assertNotIn(str(self.output), json.dumps(self.build_report))

    def test_mutated_dictionary_is_rejected_as_not_embedded(self):
        paths = self.build_report["products"]
        source = self.output / paths["dictionary"]
        mutated = self.output / "unrelated.dict"
        value = json.loads(source.read_text())
        value["version"] = "unrelated"
        mutated.write_text(
    json.dumps(
        value,
        separators=(
            ",",
            ":"),
             sort_keys=True))
        with self.assertRaisesRegex(TOOL.ToolError, "embedded dictionary"):
            TOOL.validate_firmware(
                "levelBoard", self.output / paths["firmware"],
                self.output / paths["elf"], mutated,
                "arm-none-eabi-", "openssl")

    def test_data_flash_load_crossing_application_limit_is_rejected(self):
        paths = self.build_report["products"]
        changed = self.output / "crossing-data.elf"
        objcopy = TOOL._tool_path("arm-none-eabi-objcopy")
        subprocess.run([objcopy,
    "--change-section-lma",
    ".data=0x0800fff0",
    str(self.output / paths["elf"]),
    str(changed)],
     check=True)
        with self.assertRaisesRegex(TOOL.ToolError, "section .data.*crosses"):
            TOOL.validate_firmware(
                "levelBoard", self.output / paths["firmware"], changed,
                self.output / paths["dictionary"],
                "arm-none-eabi-", "openssl")

    def test_mutated_hex_load_byte_is_rejected_as_elf_mismatch(self):
        paths = self.build_report["products"]
        source = (self.output / paths["firmware"]).read_bytes()
        lines = source.splitlines(keepends=True)
        for index, line in enumerate(lines):
            raw = bytearray.fromhex(line[1:].strip().decode())
            if raw[3] == 0 and raw[0] and ((raw[1] << 8) | raw[2]) >= 0x4008:
                raw[4] ^= 1
                raw[-1] = (-sum(raw[:-1])) & 0xff
                lines[index] = b":" + raw.hex().upper().encode() + b"\n"
                break
        mutated = self.output / "mutated.hex"
        mutated.write_bytes(b"".join(lines))
        with self.assertRaisesRegex(
                TOOL.ToolError, "ELF.*mismatch|mismatch.*ELF"):
            TOOL.validate_firmware(
                "levelBoard", mutated, self.output / paths["elf"],
                self.output / paths["dictionary"],
                "arm-none-eabi-", "openssl")

    def test_actual_eboard_build_and_cross_board_rejections(self):
        paths = self.eboard_build_report["products"]
        self.assertEqual(paths["firmware"], "eBoard.hex")
        for key in ("elf", "bin", "firmware", "dictionary", "config"):
            self.assertTrue((self.eboard_output / paths[key]).is_file(), key)
        report = self.eboard_build_report["firmware"]
        self.assertEqual(report["bounds"]["application"],
                         [0x08010000, 0x08040000])
        self.assertEqual(report["constants"]["SERIAL_BAUD"], 460800)
        level_paths = self.build_report["products"]
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "levelBoard", self.eboard_output / paths["firmware"],
                self.eboard_output / paths["elf"],
                self.eboard_output / paths["dictionary"])
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "eBoard", self.output / level_paths["firmware"],
                self.output / level_paths["elf"],
                self.output / level_paths["dictionary"])

    def test_actual_heaterboard_build_and_cross_board_rejections(self):
        paths = self.heaterboard_build_report["products"]
        self.assertEqual(paths["firmware"], "heaterBoard.hex")
        for key in ("elf", "bin", "firmware", "dictionary", "config"):
            self.assertTrue(
                (self.heaterboard_output / paths[key]).is_file(), key)
        report = self.heaterboard_build_report["firmware"]
        self.assertEqual(report["bounds"]["application"],
                         [0x08010000, 0x08080000])
        self.assertEqual(report["bounds"]["sram"],
                         [0x20000000, 0x20020000])
        self.assertEqual(report["stack_pointer"], 0x20020000)
        self.assertEqual(report["constants"]["MCU"], "n32g455rel7")
        self.assertEqual(report["constants"]["SERIAL_BAUD"], 230400)
        self.assertEqual(report["normalization"],
                         "application-448k-ff-fill-v1")
        dictionary = json.loads(
            (self.heaterboard_output / paths["dictionary"]).read_text())
        adc_query = [command for command in dictionary["commands"]
                     if command.startswith("query_analog_in ")]
        self.assertEqual(adc_query, [
            "query_analog_in oid=%c clock=%u sample_ticks=%u "
            "sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu "
            "range_check_count=%c"])
        self.assertNotIn("bytes_per_report", adc_query[0])

        eboard = self.eboard_build_report["products"]
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "heaterBoard", self.eboard_output / eboard["firmware"],
                self.eboard_output / eboard["elf"],
                self.eboard_output / eboard["dictionary"])
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "eBoard", self.heaterboard_output / paths["firmware"],
                self.heaterboard_output / paths["elf"],
                self.heaterboard_output / paths["dictionary"])
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "heaterBoard", self.heaterboard_output / paths["firmware"],
                self.eboard_output / eboard["elf"],
                self.eboard_output / eboard["dictionary"])


    def test_package_rejects_cross_board_and_cross_role_products(self):
        level = self.build_report["products"]
        eboard = self.eboard_build_report["products"]
        heaterboard = self.heaterboard_build_report["products"]
        triples = [
            ("eBoard", self.output, level),
            ("levelBoard", self.eboard_output, eboard),
            ("heaterBoard", self.eboard_output, eboard),
            ("eBoard", self.heaterboard_output, heaterboard),
        ]
        mixed_roles = [
            ("eBoard", {
                "firmware": self.eboard_output / eboard["firmware"],
                "elf": self.output / level["elf"],
                "dictionary": self.eboard_output / eboard["dictionary"],
            }),
            ("eBoard", {
                "firmware": self.eboard_output / eboard["firmware"],
                "elf": self.eboard_output / eboard["elf"],
                "dictionary": self.output / level["dictionary"],
            }),
            ("heaterBoard", {
                "firmware": (self.heaterboard_output /
                             heaterboard["firmware"]),
                "elf": self.eboard_output / eboard["elf"],
                "dictionary": self.eboard_output / eboard["dictionary"],
            }),
        ]
        cases = []
        for board, directory, products in triples:
            cases.append((board, {role: directory / products[role]
                                  for role in ("firmware", "elf",
                                               "dictionary")}))
        cases.extend(mixed_roles)
        root = self.output.parent
        for index, (board, products) in enumerate(cases):
            output = root / ("Creator5Pro-cross-%d.tgz" % index)
            with self.subTest(board=board, index=index), self.assertRaises(
                    TOOL.ToolError):
                TOOL.package_update(root / "template.tgz",
                                    {board: products}, output)
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())
@unittest.skipUnless(os.environ.get("RUN_C5_BUILD_TESTS") == "1",
                     "set RUN_C5_BUILD_TESTS=1 with the ARM toolchain "
                     "available")
class MainboardRealFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if "mainBoardGD" not in TOOL.BOARD_PROFILES:
            raise AssertionError(
                "mainBoardGD builder profile is not implemented")
        cls.workspace = tempfile.TemporaryDirectory()
        cls.output = Path(cls.workspace.name) / "mainBoardGD"
        cls.repo_config = ROOT / ".config"
        cls.config_before = (cls.repo_config.read_bytes()
                             if cls.repo_config.exists() else None)
        cls.build_report = TOOL.build_firmware(
            "mainBoardGD", cls.output, 1, "arm-none-eabi-", "openssl")

    @classmethod
    def tearDownClass(cls):
        cls.workspace.cleanup()

    def test_actual_mainboard_build_and_cross_board_rejections(self):
        paths = self.build_report["products"]
        self.assertEqual(paths["firmware"], "mainBoardGD.hex")
        for key in ("elf", "bin", "firmware", "dictionary", "config"):
            self.assertTrue((self.output / paths[key]).is_file(), key)
        current = (self.repo_config.read_bytes()
                   if self.repo_config.exists() else None)
        self.assertEqual(current, self.config_before)
        report = self.build_report["firmware"]
        self.assertEqual(report["bounds"]["application"],
                         [0x08000000, 0x08100000])
        self.assertEqual(report["bounds"]["sram"],
                         [0x24000000, 0x24080000])
        self.assertEqual(report["stack_pointer"], 0x24080000)
        self.assertEqual(report["constants"], MAINBOARD_REQUIRED_CONSTANTS)
        self.assertEqual(report["normalization"],
                         "application-1024k-ff-fill-v1")
        dictionary_path = self.output / paths["dictionary"]
        dictionary = json.loads(dictionary_path.read_text())
        self.assertTrue(MAINBOARD_OPTIONAL_GENERIC_RESPONSES.issubset(
            dictionary["responses"]))
        self.assertNotIn("STEPPER_OPTIMIZED_EDGE", dictionary["config"])
        self.assertNotIn("get_basic_param num=%u", dictionary["commands"])
        self.assertNotIn("param_value value=%u reserve=%u",
                         dictionary["responses"])
        self.assertEqual(
            TOOL._load_dictionary(
                "mainBoardGD", dictionary_path.read_bytes())["constants"],
            MAINBOARD_REQUIRED_CONSTANTS)
        with self.assertRaises(TOOL.ToolError):
            TOOL.validate_firmware(
                "levelBoard", self.output / paths["firmware"],
                self.output / paths["elf"], dictionary_path)
        wrong_dictionary = self.output / "levelBoard.dict"
        wrong_dictionary.write_bytes(dictionary_data())
        with self.assertRaisesRegex(TOOL.ToolError, "embedded dictionary"):
            TOOL.validate_firmware(
                "mainBoardGD", self.output / paths["firmware"],
                self.output / paths["elf"], wrong_dictionary)


class ArchiveInspectionTests(unittest.TestCase):
    def inspect(self, value, **kwargs):
        return TOOL.inspect_archive(value, **kwargs)

    def assert_rejected(self, value, *words, **kwargs):
        with self.assertRaises(TOOL.ToolError) as raised:
            self.inspect(value, **kwargs)
        text = str(raised.exception).lower()
        for word in words:
            self.assertIn(word.lower(), text)

    def test_reports_sanitized_metadata_and_recurses_by_magic(self):
        inner = tar_bytes([("./payload", b"nested")])
        outer = tar_bytes([
            ("./plain", b"opaque"),
            ("./gzip.bin", gzip.compress(inner)),
            ("./xz.bin", lzma.compress(inner)),
            ("./bzip.bin", bz2.compress(inner)),
        ])
        report = self.inspect(outer)
        self.assertEqual(report["format"], "tar")
        self.assertEqual(report["sha256"], hashlib.sha256(outer).hexdigest())
        plain = report["members"][0]
        self.assertEqual(plain, {
            "path": "./plain", "canonical_path": "plain",
            "type": "file", "size": 6, "mode": 0o664,
            "uid": 12, "gid": 34, "uname": "fixture-user",
            "gname": "fixture-group", "mtime": 123456789,
            "link_target": None,
            "sha256": hashlib.sha256(b"opaque").hexdigest(),
            "leading_dot_slash": True, "unsafe": False,
        })
        for member, wrapper in zip(report["members"][1:],
                                   ("gzip", "xz", "bzip2")):
            self.assertEqual(member["nested"]["format"], wrapper)
            self.assertEqual(member["nested"]["payload"]["format"], "tar")
            self.assertEqual(member["nested"]["payload"]["members"][0]["path"],
                             "./payload")
        self.assertNotIn("data", json.dumps(report))

    def test_accepts_normal_dot_roots_and_safe_relative_links(self):
        value = tar_bytes([
            ("./", b"", tarfile.DIRTYPE),
            ("./dir/", b"", tarfile.DIRTYPE),
            ("./dir/data", b"value"),
            ("./dir/alias", b"", tarfile.SYMTYPE, "data"),
            ("./hard", b"", tarfile.LNKTYPE, "./dir/data"),
        ])
        report = self.inspect(value)
        types = {member["canonical_path"]: member["type"]
                 for member in report["members"]}
        self.assertEqual(types, {".": "directory", "dir": "directory",
                                 "dir/data": "file", "dir/alias": "symlink",
                                 "hard": "hardlink"})
        links = [entry for entry in report["members"] if entry["link_target"]]
        self.assertEqual([entry["unsafe"] for entry in links], [False, False])

    def test_accepts_finite_repeated_symlink_traversal(self):
        value = tar_bytes([
            ("a", b"", tarfile.SYMTYPE, "."),
            ("b", b"", tarfile.SYMTYPE, "a/a"),
        ])
        report = self.inspect(value)
        self.assertEqual([member["canonical_path"]
                          for member in report["members"]], ["a", "b"])

    def test_rejects_unsafe_member_paths_before_extraction(self):
        cases = (
            ("/absolute", "absolute"),
            ("../escape", "traversal"),
            ("safe/../../escape", "traversal"),
            ("C:/drive", "drive"),
            ("./C:/drive", "drive"),
            ("back\\slash", "backslash"),
        )
        for name, expected in cases:
            with self.subTest(name=name):
                self.assert_rejected(tar_bytes([(name, b"x")]), expected, name)
        control = corrupt_tar_name(tar_bytes([("good", b"x")]), b"bad\nname")
        self.assert_rejected(control, "control", "bad")
        embedded_nul = corrupt_tar_name(
            tar_bytes([("good", b"x")]), b"bad\0hidden")
        self.assert_rejected(embedded_nul, "nul", "name")

    def test_rejects_escaping_links_cycles_and_symlink_ancestor_writes(self):
        cases = (
            ([("link", b"", tarfile.SYMTYPE, "/outside")],
             ("symlink", "absolute")),
            ([("hard", b"", tarfile.LNKTYPE, "/outside")],
             ("hardlink", "absolute")),
            ([("link", b"", tarfile.SYMTYPE, "C:/outside")],
             ("symlink", "drive")),
            ([("dir/link", b"", tarfile.SYMTYPE, "../../outside")],
             ("symlink", "escape")),
            ([("d", b"", tarfile.DIRTYPE),
              ("d/root", b"", tarfile.SYMTYPE, ".."),
              ("d/escape", b"", tarfile.SYMTYPE, "root/..")],
             ("symlink", "escape")),
            ([("link", b"", tarfile.SYMTYPE, "./C:/outside")],
             ("symlink", "drive")),
            ([("hard", b"", tarfile.LNKTYPE, "../outside")],
             ("hardlink", "escape")),
            ([("hard", b"", tarfile.LNKTYPE, "missing")],
             ("hardlink", "dangling")),
            ([("a", b"", tarfile.SYMTYPE, "b"),
              ("b", b"", tarfile.SYMTYPE, "a")],
             ("symlink", "cycle")),
            ([("a", b"", tarfile.SYMTYPE, "b/c"),
              ("b", b"", tarfile.SYMTYPE, "a")],
             ("symlink", "cycle")),
            ([("a", b"", tarfile.LNKTYPE, "b"),
              ("b", b"", tarfile.LNKTYPE, "a")],
             ("hardlink", "cycle")),
            ([("target", b"", tarfile.DIRTYPE),
              ("link", b"", tarfile.SYMTYPE, "target"),
              ("link/file", b"x")],
             ("symlink", "ancestor")),
        )
        for entries, words in cases:
            with self.subTest(words=words):
                self.assert_rejected(tar_bytes(entries), *words)

    def test_rejects_special_members_duplicates_and_file_parents(self):
        specials = ((tarfile.CHRTYPE, "device"),
                    (tarfile.BLKTYPE, "device"),
                    (tarfile.FIFOTYPE, "fifo"),
                    (tarfile.GNUTYPE_SPARSE, "sparse"),
                    (b"Z", "unrecognized"),
                    (b"s", "unrecognized"))
        for kind, expected in specials:
            with self.subTest(kind=kind):
                self.assert_rejected(tar_bytes([("special", b"", kind)]),
                                     expected, "special")
        self.assert_rejected(
            tar_bytes([("./same", b"one"), ("same", b"two")]),
            "duplicate", "same")
        self.assert_rejected(
            tar_bytes([("parent", b"file"), ("parent/child", b"x")]),
            "parent", "file")

    def test_validates_tar_checksum_truncation_and_trailing_bytes(self):
        value = tar_bytes([("file", b"content")])
        damaged = bytearray(value)
        damaged[0] ^= 1
        self.assert_rejected(bytes(damaged), "checksum", "header")
        self.assert_rejected(value[:1024], "truncated", "tar")
        self.assert_rejected(value + b"not-padding", "trailing", "tar")

    def test_claimed_nested_archives_and_unknown_top_level_must_parse(self):
        self.assert_rejected(tar_bytes([("broken.tar.xz", b"not archive")]),
                             "broken.tar.xz", "archive")
        self.assert_rejected(b"PK\x03\x04not-a-supported-package", "unknown",
                             "format")
        inner = tar_bytes([("leaf", b"ok")])
        truncated = (("gzip", gzip.compress(inner)[:-4]),
                     ("xz", lzma.compress(inner)[:-4]),
                     ("bzip2", bz2.compress(inner)[:-4]))
        for kind, value in truncated:
            with self.subTest(kind=kind):
                self.assert_rejected(value, "truncated", kind)
        trailing = (("gzip", gzip.compress(inner) + b"garbage"),
                    ("xz", lzma.compress(inner) + b"garbage"),
                    ("bzip2", bz2.compress(inner) + b"garbage"))
        for kind, value in trailing:
            with self.subTest(kind=kind):
                self.assert_rejected(value, "trailing", kind)

    def test_accepts_empty_tar_and_legal_xz_stream_padding(self):
        empty = b"\0" * 1024
        report = self.inspect(empty)
        self.assertEqual(report["format"], "tar")
        self.assertEqual(report["members"], [])

        inner = tar_bytes([("leaf", b"ok")])
        split = len(inner) // 2
        padded = (lzma.compress(inner[:split]) + b"\0" * 4 +
                  lzma.compress(inner[split:]) + b"\0" * 8)
        report = self.inspect(padded)
        self.assertEqual(report["format"], "xz")
        self.assertEqual(report["payload"]["format"], "tar")
        self.assertEqual(report["payload"]["members"][0]["path"], "leaf")
        self.assert_rejected(lzma.compress(inner) + b"\0" * 2,
                             "xz", "padding")

    def test_xz_decoder_enforces_memory_limit_before_expansion(self):
        inner = tar_bytes([("leaf", b"ok")])
        high_memory = lzma.compress(inner, format=lzma.FORMAT_XZ, preset=6)
        with mock.patch.object(TOOL, "MAX_LAYER_BYTES", 1024 * 1024):
            self.assert_rejected(high_memory, "xz", "memory", "limit")

    def test_enforces_input_layer_total_member_and_depth_limits(self):
        value = tar_bytes([("file", b"x" * 2048)])
        with mock.patch.object(TOOL, "MAX_LAYER_BYTES", 100):
            self.assert_rejected(value, "input", "limit")
        compressed = gzip.compress(value)
        with mock.patch.object(TOOL, "MAX_LAYER_BYTES", 4096):
            self.assert_rejected(compressed, "decompressed", "limit")
        with mock.patch.object(TOOL, "MAX_TOTAL_EXPANDED", len(value) + 1024):
            self.assert_rejected(value, "cumulative", "limit")
        encrypted = b"Salted__" + b"12345678" + b"x" * 16
        with mock.patch.object(
                TOOL, "MAX_TOTAL_EXPANDED", len(encrypted) + 14):
            with mock.patch.object(TOOL, "openssl_crypt") as crypt:
                self.assert_rejected(encrypted, "cumulative", "limit",
                                     decrypt=True)
                crypt.assert_not_called()
        members = tar_bytes([("one", b"1"), ("two", b"2"), ("three", b"3")])
        with mock.patch.object(TOOL, "MAX_ARCHIVE_MEMBERS", 2):
            self.assert_rejected(members, "member", "limit")
        nested = tar_bytes([("leaf", b"ok")])
        for index in range(4):
            nested = tar_bytes([("layer%d.tar" % index, nested)])
        with mock.patch.object(TOOL, "MAX_ARCHIVE_DEPTH", 2):
            self.assert_rejected(nested, "depth", "limit")

    def test_extract_temp_validates_then_removes_private_workspace(self):
        value = tar_bytes([("./dir/", b"", tarfile.DIRTYPE),
                           ("./dir/file", b"content"),
                           ("./hard", b"", tarfile.LNKTYPE, "./dir/file")])
        report = self.inspect(value, extract_temp=True)
        self.assertTrue(report["extraction_validated"])
        self.assertNotIn("extraction_path", report)
        with tempfile.TemporaryDirectory() as link_probe:
            probe = Path(link_probe) / "link"
            try:
                probe.symlink_to("target")
            except OSError:
                pass
            else:
                linked = tar_bytes([
                    ("./target", b"content"),
                    ("./alias", b"", tarfile.SYMTYPE, "target")])
                self.inspect(linked, extract_temp=True)
        with tempfile.TemporaryDirectory() as temp:
            sentinel = Path(temp) / "sentinel"
            sentinel.write_text("unchanged")
            hostile = tar_bytes([("../sentinel", b"changed")])
            self.assert_rejected(hostile, "traversal", extract_temp=True)
            self.assertEqual(sentinel.read_text(), "unchanged")

        restrictive = tar_bytes([
            ("locked", b"", tarfile.DIRTYPE, "", 0o000),
            ("locked/file", b"content"),
        ])
        report = self.inspect(restrictive, extract_temp=True)
        self.assertTrue(report["extraction_validated"])

    def test_inspect_cli_outputs_json_and_never_exposes_temp_path(self):
        with tempfile.TemporaryDirectory() as temp:
            package = Path(temp) / "fixture.tar"
            package.write_bytes(tar_bytes([("./payload", b"ok")]))
            result = subprocess.run(
                [sys.executable, str(TOOL_PATH), "inspect", "--input",
                 str(package), "--extract-temp"], cwd=ROOT, text=True,
                capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report["format"], "tar")
            self.assertTrue(report["extraction_validated"])
            self.assertNotIn(str(Path(temp).resolve()), result.stdout)

    @unittest.skipUnless(shutil.which("openssl"), "OpenSSL is required")
    def test_real_openssl_roundtrip_requires_secret_and_redacts_failures(self):
        secret = hashlib.sha256(os.urandom(32)).hexdigest()
        wrong = hashlib.sha256(os.urandom(32)).hexdigest()
        plain = tar_bytes([("./payload", b"secret-free-content")])
        with mock.patch.dict(os.environ, {"C5_UPDATE_PASSPHRASE": secret},
                             clear=False):
            encrypted = TOOL.openssl_crypt(plain, decrypt=False,
                                           openssl=shutil.which("openssl"))
            self.assertTrue(encrypted.startswith(b"Salted__"))
            decrypted = TOOL.openssl_crypt(
                encrypted, decrypt=True, openssl=shutil.which("openssl"))
            self.assertEqual(decrypted, plain)
            report = self.inspect(encrypted, decrypt=True,
                                  openssl=shutil.which("openssl"))
            self.assert_rejected(encrypted, "--decrypt")
            for malformed in (b"Salted__", b"Salted__12345678bad"):
                with self.subTest(length=len(malformed)):
                    with self.assertRaisesRegex(TOOL.ToolError,
                                                "header|ciphertext|length"):
                        TOOL.openssl_crypt(
                            malformed, decrypt=True,
                            openssl=shutil.which("openssl"))
            self.assertEqual(report["format"], "encrypted")
            self.assertEqual(report["payload"]["format"], "tar")
        with mock.patch.dict(os.environ, {"C5_UPDATE_PASSPHRASE": wrong},
                             clear=False):
            with self.assertRaises(TOOL.ToolError) as raised:
                self.inspect(encrypted, decrypt=True,
                             openssl=shutil.which("openssl"))
            self.assertIn(raised.exception.exit_code, (2, 3))
            self.assertNotIn(secret, str(raised.exception))
            self.assertNotIn(wrong, str(raised.exception))
            with tempfile.TemporaryDirectory() as temp:
                package = Path(temp) / "encrypted.tgz"
                package.write_bytes(encrypted)
                environment = os.environ.copy()
                environment["C5_UPDATE_PASSPHRASE"] = wrong
                result = subprocess.run(
                    [sys.executable, str(TOOL_PATH), "--openssl",
                     shutil.which("openssl"), "inspect", "--input",
                     str(package), "--decrypt"], cwd=ROOT, text=True,
                    capture_output=True, env=environment)
                self.assertIn(result.returncode, (2, 3))
                self.assertEqual(result.stdout, "")
                self.assertNotIn(secret, result.stderr)
                self.assertNotIn(wrong, result.stderr)
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assert_rejected(encrypted, "passphrase", decrypt=True,
                                 openssl=shutil.which("openssl"))

    @unittest.skipUnless(os.environ.get("C5_CANONICAL_TEMPLATE"),
                         "set C5_CANONICAL_TEMPLATE for canonical inspection")
    def test_canonical_package_and_embedded_control_are_inspectable(self):
        package = Path(os.environ["C5_CANONICAL_TEMPLATE"])
        if not package.is_file():
            self.skipTest("canonical template is unavailable")
        openssl = shutil.which("openssl")
        if not openssl or not os.environ.get("C5_UPDATE_PASSPHRASE"):
            self.skipTest("canonical OpenSSL/passphrase inputs are unavailable")
        encrypted = package.read_bytes()
        report = self.inspect(encrypted, decrypt=True, openssl=openssl)
        self.assertEqual(report["format"], "encrypted")
        self.assertEqual(report["payload"]["format"], "tar")
        plain = TOOL.openssl_crypt(encrypted, decrypt=True, openssl=openssl)
        with tarfile.open(fileobj=io.BytesIO(plain), mode="r:") as archive:
            candidates = [member for member in archive.getmembers()
                          if Path(member.name).name.startswith("control-")]
            self.assertEqual(len(candidates), 1)
            control = archive.extractfile(candidates[0]).read()
        control_report = self.inspect(control)
        self.assertEqual(control_report["format"], "tar")
        self.assertEqual(len(control_report["members"]), 10)


@unittest.skipUnless(shutil.which("sh") and shutil.which("md5sum"),
                     "sh and md5sum are required for package construction")
class PackageTransformationTests(unittest.TestCase):
    STOCK_INSTALLER = (b"#!/bin/sh\n"
                       b"keep_before()\n{\n  :\n}\n"
                       b"update_other()\n{\n"
                       b"  rm -rf /usr/data/logs/NIM\n"
                       b"  cp ./start.img /usr/share/start.img\n"
                       b"}\n"
                       b"keep_after()\n{\n  :\n}\n"
                       b"update_other\n"
                       b"echo done\n")

    @staticmethod
    def _checksum_list(files, order):
        return b"".join(
            hashlib.md5(files[name]).hexdigest().encode("ascii") +
            b"  " + name.encode("ascii") + b"\n"
            for name in order)

    def _template(self):
        control_files = {
            "./eBoard.hex": b"unrelated-e-board",
            "./heaterBoard.hex": b"unrelated-heater-board",
            "./IAPCommand": b"iap-command",
            "./ISPCommand": b"isp-command",
            "./levelBoard.hex": b"old-level-board",
            "./mainBoardGD.hex": b"unrelated-main-board",
            "./mcu.img": b"display-image",
            "./run.sh": (b"#!/bin/sh\necho stock-control\n"
                          b"cd /usr/prog/PROGRAM/control/\necho 2\n"
                          b"sync\nsleep 3\n\nexit 0\n"),
            "./Update": b"",
        }
        checksum_order = [
            "./eBoard.hex", "./heaterBoard.hex", "./IAPCommand",
            "./ISPCommand", "./levelBoard.hex", "./mainBoardGD.hex",
            "./mcu.img", "./run.sh", "./Update"]
        checksums = self._checksum_list(control_files, checksum_order)
        control = tar_bytes([
            ("./eBoard.hex", control_files["./eBoard.hex"],
             tarfile.REGTYPE, "", 0o777),
            ("./heaterBoard.hex", control_files["./heaterBoard.hex"],
             tarfile.REGTYPE, "", 0o777),
            ("./IAPCommand", control_files["./IAPCommand"],
             tarfile.REGTYPE, "", 0o775),
            ("./ISPCommand", control_files["./ISPCommand"],
             tarfile.REGTYPE, "", 0o775),
            ("./levelBoard.hex", control_files["./levelBoard.hex"],
             tarfile.REGTYPE, "", 0o777),
            ("./mainBoardGD.hex", control_files["./mainBoardGD.hex"],
             tarfile.REGTYPE, "", 0o777),
            ("./mcu.img", control_files["./mcu.img"],
             tarfile.REGTYPE, "", 0o775),
            ("./md5sum.list", checksums, tarfile.REGTYPE, "", 0o664),
            ("./run.sh", control_files["./run.sh"],
             tarfile.REGTYPE, "", 0o777),
            ("./Update", b"", tarfile.REGTYPE, "", 0o775),
        ])
        outer = tar_bytes([
            ("./control-fixture.tar.xz", control),
            ("./end.img", b"end-image", tarfile.REGTYPE, "", 0o775),
            ("./kernel-fixture.tar.xz", tar_bytes([("kernel", b"x")])),
            ("./library-fixture.tar.xz", tar_bytes([("library", b"x")])),
            ("./play", b"play-helper", tarfile.REGTYPE, "", 0o775),
            ("./runFirmwareExe.sh", self.STOCK_INSTALLER,
             tarfile.REGTYPE, "", 0o775),
            ("./software-fixture.tar.xz", tar_bytes([("software", b"x")])),
            ("./start.img", b"start-image", tarfile.REGTYPE, "", 0o775),
        ])
        model = TOOL._inspect_archive_model(outer)
        return outer, TOOL._locate_template_members(model)

    def test_installer_removes_only_function_and_sole_call(self):
        transformed = TOOL._suppress_update_other(self.STOCK_INSTALLER)
        expected = (b"#!/bin/sh\n"
                    b"keep_before()\n{\n  :\n}\n"
                    b"keep_after()\n{\n  :\n}\n"
                    b"echo done\n")
        self.assertEqual(transformed, expected)
        TOOL._check_shell_syntax(transformed, shutil.which("sh"))
        for bad in (self.STOCK_INSTALLER + b"update_other\n",
                    self.STOCK_INSTALLER.replace(
                        b"update_other()\n", b"renamed()\n")):
            with self.subTest(bad=hashlib.sha256(bad).hexdigest()):
                with self.assertRaisesRegex(
                        TOOL.ToolError,
                        "exactly one.*update_other|update_other.*exactly one"):
                    TOOL._suppress_update_other(bad)

    def test_current_template_reducer_keeps_selected_failure_only(self):
        control = (
            b"# free 28M\nrm /usr/prog/qt-4.8.6 -rf\n"
            b"# free 22M\nrm /usr/prog/opencv-4.10 -rf\n"
            b"# free 3M\nrm /usr/prog/wifi/8821cu.ko*\nsync\n"
            b"# check update result\n"
            b"if [ -f eboard.log ];then\n"
            b" if grep -q fail eboard.log; then\n"
            b"  cat $WORK_DIR/eBoard_fail.img > /dev/fb0\n"
            b" fi\nfi\n\n"
            b"if [ -f heater.log ];then\n"
            b" if grep -q fail heater.log; then\n"
            b"  cat $WORK_DIR/heaterBoard_fail.img > /dev/fb0\n"
            b" fi\nfi\n\n"
            b"if [ -f level.log ];then\n"
            b" if grep -q fail level.log; then\n"
            b"  cat $WORK_DIR/levelBoard_fail.img > /dev/fb0\n"
            b" fi\nfi\n\n"
            b"if [ -f gd.log ];then\n"
            b" if ! grep -q finished gd.log; then\n"
            b"  cat $WORK_DIR/mcu_fail.img > /dev/fb0\n"
            b" fi\nfi\n\n"
            b"# remove small version\necho keep\n")
        transformed = TOOL._suppress_space_reclaim(control, False)
        transformed = TOOL._filter_result_checks(
            transformed, ["mainBoardGD"])
        self.assertNotIn(b"qt-4.8.6", transformed)
        self.assertNotIn(b"eBoard_fail.img", transformed)
        self.assertIn(b"if ! grep -q finished gd.log", transformed)
        self.assertIn(b"mcu_fail.img", transformed)
        TOOL._check_shell_syntax(transformed, shutil.which("sh"))

        outer = (
            b"rm /usr/prog/PROGRAM/control/*.tar.xz*\n"
            b"rm /usr/prog/PROGRAM/library/*.tar.xz*\n"
            b"rm /usr/prog/PROGRAM/kernel/*.tar.xz*\n"
            b"rm /usr/prog/PROGRAM/software/*.tar.xz*\n"
            b"rm /usr/prog/qt-4.8.6 -rf\n"
            b"rm /usr/prog/opencv-4.10 -rf\n"
            b"rm /usr/prog/wifi/8821cu.ko*\nsync\necho keep\n")
        self.assertEqual(TOOL._suppress_space_reclaim(outer, True),
                         b"echo keep\n")

    def test_update_marker_removed_before_version_prune_cd(self):
        script = (b"#!/bin/sh\n"
                  b"if [ -f gd.log ];then\n"
                  b" if ! grep -q finished gd.log; then\n"
                  b"  touch $WORK_DIR/Update\n"
                  b"  sleep 10000\n"
                  b" fi\nfi\n"
                  b"cd /usr/prog/PROGRAM/control/\n"
                  b"DIR_COUNT=2\n"
                  b"sync\nsleep 3\n\n"
                  b"exit 0\n")
        transformed = TOOL._remove_update_marker(script)
        expected = script.replace(
            b"cd /usr/prog/PROGRAM/control/\n",
            b"rm -f $WORK_DIR/Update\nsync\n\n"
            b"cd /usr/prog/PROGRAM/control/\n")
        self.assertEqual(transformed, expected)
        self.assertIn(b"touch $WORK_DIR/Update", transformed)
        TOOL._check_shell_syntax(transformed, shutil.which("sh"))
        for bad in (script + b"cd /usr/prog/PROGRAM/control/\n",
                    script.replace(b"cd /usr/prog/PROGRAM/control/\n",
                                   b"cd /tmp\n")):
            with self.assertRaisesRegex(
                    TOOL.ToolError, "version-prune change"):
                TOOL._remove_update_marker(bad)

    def test_reduced_plaintext_is_reproducible_and_exactly_allowlisted(self):
        unused, profile = self._template()
        replacement = ihex()
        first, evidence = TOOL._build_reduced_plaintext(
            profile, {"levelBoard": replacement}, shutil.which("sh"),
            shutil.which("md5sum"))
        second, second_evidence = TOOL._build_reduced_plaintext(
            profile, {"levelBoard": replacement}, shutil.which("sh"),
            shutil.which("md5sum"))
        self.assertEqual(first, second)
        self.assertEqual(evidence["plaintext_sha256"],
                         second_evidence["plaintext_sha256"])
        model = TOOL._inspect_archive_model(first)
        summary = TOOL._assert_reduced_profile(
            model, evidence, ["levelBoard"], shutil.which("md5sum"))
        self.assertEqual([item["label"] for item in summary["outer"]], [
            "control package", "end image", "play helper",
            "outer installer", "start image"])
        self.assertEqual([item["label"] for item in summary["control"]], [
            "IAPCommand", "levelBoard firmware", "control display image",
            "checksum list", "control script", "Update marker"])
        self.assertNotIn(b"eBoard.hex", first)
        self.assertNotIn(b"kernel-fixture", first)
        with tarfile.open(fileobj=io.BytesIO(first), mode="r:") as outer:
            control_info = outer.getmembers()[0]
            control = outer.extractfile(control_info).read()
            with tarfile.open(fileobj=io.BytesIO(control), mode="r:") as inner:
                values = {member.name: inner.extractfile(member).read()
                          for member in inner.getmembers()}
        self.assertEqual(values["./levelBoard.hex"], replacement)
        retained_order = ["./IAPCommand", "./levelBoard.hex", "./mcu.img",
                          "./run.sh", "./Update"]
        self.assertEqual(values["./md5sum.list"],
                         self._checksum_list(values, retained_order))

    def test_reducer_supports_heaterboard_and_canonical_combined_order(self):
        unused, profile = self._template()
        eboard = ihex(board="eBoard")
        heaterboard = ihex(board="heaterBoard")
        levelboard = ihex()
        shell = shutil.which("sh")
        md5sum = shutil.which("md5sum")
        eboard_plain, eboard_evidence = TOOL._build_reduced_plaintext(
            profile, {"eBoard": eboard}, shell, md5sum)
        TOOL._assert_reduced_profile(
            TOOL._inspect_archive_model(eboard_plain), eboard_evidence,
            ["eBoard"], md5sum)
        self.assertIn(b"eBoard.hex", eboard_plain)
        self.assertNotIn(b"heaterBoard.hex", eboard_plain)
        self.assertNotIn(b"levelBoard.hex", eboard_plain)


        heater_plain, heater_evidence = TOOL._build_reduced_plaintext(
            profile, {"heaterBoard": heaterboard}, shell, md5sum)
        heater_summary = TOOL._assert_reduced_profile(
            TOOL._inspect_archive_model(heater_plain), heater_evidence,
            ["heaterBoard"], md5sum)
        self.assertEqual(
            {item["label"] for item in heater_summary["control"]}, {
                "IAPCommand", "heaterBoard firmware", "control display image",
                "checksum list", "control script", "Update marker"})
        with tarfile.open(fileobj=io.BytesIO(heater_plain), mode="r:") as outer:
            control = outer.extractfile(outer.getmembers()[0]).read()
            with tarfile.open(fileobj=io.BytesIO(control), mode="r:") as inner:
                members = inner.getmembers()
                values = {member.name: inner.extractfile(member).read()
                          for member in members}
                retained_order = [member.name for member in members
                                  if member.name != "./md5sum.list"]
        self.assertEqual(set(values), {
            "./IAPCommand", "./heaterBoard.hex", "./mcu.img",
            "./md5sum.list", "./run.sh", "./Update"})
        self.assertEqual(values["./heaterBoard.hex"], heaterboard)
        self.assertEqual(values["./md5sum.list"],
                         self._checksum_list(values, retained_order))
        for excluded in (b"eBoard.hex", b"levelBoard.hex", b"mainBoardGD.hex",
                         b"VDS", b"ISPCommand"):
            self.assertNotIn(excluded, heater_plain)

        forward, forward_evidence = TOOL._build_reduced_plaintext(
            profile, {"eBoard": eboard, "heaterBoard": heaterboard,
                      "levelBoard": levelboard}, shell, md5sum)
        reverse, reverse_evidence = TOOL._build_reduced_plaintext(
            profile, {"levelBoard": levelboard, "heaterBoard": heaterboard,
                      "eBoard": eboard}, shell, md5sum)
        self.assertEqual(forward, reverse)
        self.assertEqual(forward_evidence["control_order"],
                         reverse_evidence["control_order"])
        summary = TOOL._assert_reduced_profile(
            TOOL._inspect_archive_model(forward), forward_evidence,
            ["levelBoard", "heaterBoard", "eBoard"], md5sum)
        firmware_labels = [item["label"] for item in summary["control"]
                           if item["label"].endswith(" firmware")]
        self.assertEqual(firmware_labels, [
            "eBoard firmware", "heaterBoard firmware", "levelBoard firmware"])
        with tarfile.open(fileobj=io.BytesIO(forward), mode="r:") as outer:
            control = outer.extractfile(outer.getmembers()[0]).read()
            with tarfile.open(fileobj=io.BytesIO(control), mode="r:") as inner:
                values = {member.name: inner.extractfile(member).read()
                          for member in inner.getmembers()}
        self.assertEqual(values["./eBoard.hex"], eboard)
        self.assertEqual(values["./heaterBoard.hex"], heaterboard)
        self.assertEqual(values["./levelBoard.hex"], levelboard)
    def test_updater_union_is_exact_for_isp_iap_and_mixed_packages(self):
        require_mainboard_profile(self)
        unused, profile = self._template()
        payloads = {
            "eBoard": ihex(board="eBoard"),
            "heaterBoard": ihex(board="heaterBoard"),
            "levelBoard": ihex(),
            "mainBoardGD": ihex(board="mainBoardGD"),
        }
        cases = (
            ({"mainBoardGD": payloads["mainBoardGD"]},
             ["./ISPCommand", "./mainBoardGD.hex", "./mcu.img",
              "./run.sh", "./Update"]),
            ({"eBoard": payloads["eBoard"]},
             ["./eBoard.hex", "./IAPCommand", "./mcu.img",
              "./run.sh", "./Update"]),
            ({board: payloads[board] for board in
             ("eBoard", "mainBoardGD")},
             ["./eBoard.hex", "./IAPCommand", "./ISPCommand",
              "./mainBoardGD.hex", "./mcu.img", "./run.sh", "./Update"]),
            (payloads,
             ["./eBoard.hex", "./heaterBoard.hex", "./IAPCommand",
              "./ISPCommand", "./levelBoard.hex", "./mainBoardGD.hex",
              "./mcu.img", "./run.sh", "./Update"]),
        )
        shell = shutil.which("sh")
        md5sum = shutil.which("md5sum")
        for selected, retained_order in cases:
            with self.subTest(boards=tuple(selected)):
                plaintext, evidence = TOOL._build_reduced_plaintext(
                    profile, selected, shell, md5sum)
                summary = TOOL._assert_reduced_profile(
                    TOOL._inspect_archive_model(plaintext), evidence,
                    selected, md5sum)
                with tarfile.open(fileobj=io.BytesIO(plaintext),
                                  mode="r:") as outer:
                    control = outer.extractfile(outer.getmembers()[0]).read()
                with tarfile.open(fileobj=io.BytesIO(control),
                                  mode="r:") as inner:
                    members = inner.getmembers()
                    values = {member.name: inner.extractfile(member).read()
                              for member in members}
                self.assertEqual(
                    [member.name for member in members
                     if member.name != "./md5sum.list"], retained_order)
                self.assertEqual(set(values),
                                 set(retained_order) | {"./md5sum.list"})
                self.assertEqual(values["./md5sum.list"],
                                 self._checksum_list(values, retained_order))
                updater_labels = {item["label"] for item in summary["control"]
                                  if item["label"] in
                                  {"IAPCommand", "ISPCommand"}}
                expected_updaters = {
                    Path(name).name for name in retained_order
                    if name in ("./IAPCommand", "./ISPCommand")}
                self.assertEqual(updater_labels, expected_updaters)
                firmware_labels = [
                    item["label"] for item in summary["control"]
                    if item["label"].endswith(" firmware")]
                self.assertEqual(
                    firmware_labels,
                    [board + " firmware" for board in
                     TOOL._canonical_boards(selected)])
                for board, firmware in selected.items():
                    self.assertEqual(
                        values["./" + TOOL.BOARD_PROFILES[board]
                               ["firmware_name"]], firmware)

    def test_mainboard_validation_failure_never_publishes_package_or_manifest(
            self):
        require_mainboard_profile(self)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            template = root / "template.tgz"
            template.write_bytes(b"not reached")
            products = {
                "firmware": root / "mainBoardGD.hex",
                "elf": root / "klipper.elf",
                "dictionary": root / "klipper.dict",
            }
            for path in products.values():
                path.write_bytes(b"fixture")
            output = root / "Creator5Pro-mainboardgd.tgz"
            with mock.patch.object(
                    TOOL, "_validate_firmware",
                    side_effect=TOOL.ToolError(
                        "injected mainBoardGD validation failure")) as validate:
                with self.assertRaisesRegex(
                        TOOL.ToolError, "mainBoardGD validation"):
                    TOOL.package_update(
                        template, {"mainBoardGD": products}, output)
            validate.assert_called_once()
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())

    def test_each_approved_template_profile_requires_consistent_hashes(self):
        approved = (
            ("d3c60574199ffd5797f6a6e1f839316dbc3d5dd42e53ca2135ff4b5a30302616",
             "2b05283f39cd68019e2d67b3068dff1e7a9a23507780635bab4bdfead2e48d9c",
             "615dc69a86e0f01a6e32688d4bd8615098e236d51cd7c5afdcd99d3113d3f8a8",
             "a042533ff5be0392455fe06a8f5270b8e"
             "27da04661e8eef830146ad540ba47e6"),
            ("5aeb22a7c0f7f16c286ed74433582ee7fc1557e050dbf5243a3fb48a93960cd6",
             "8f587d850b3876f65a482cf2d1d708348a5dbf2cda45c6211c9b0ed451910f3f",
             "c05dd781da74d1bf78bd8209730e27aad7de4e49abcf0e0265617395df6fbaa6",
             "6fd03bd00a9ef297188d491b4352deef0"
             "721a24b8f1f21373db7363f969e3f0d"),
        )

        def located(control, installer, script):
            return {
                "control_outer": {"sha256": control},
                "outer": {"./runFirmwareExe.sh": {"sha256": installer}},
                "control": {
                    "./run.sh": {"sha256": script},
                    "./IAPCommand": {
                        "sha256": TOOL.CANONICAL_IAP_SHA256},
                },
            }

        for plaintext, control, installer, script in approved:
            with self.subTest(plaintext=plaintext), mock.patch.object(
                    TOOL, "_locate_template_members",
                    return_value=located(control, installer, script)):
                profile = TOOL._canonical_template_profile(
                    {"sha256": plaintext})
                self.assertEqual(profile["control_outer"]["sha256"],
                                 control)

        plaintext, _, _, _ = approved[1]
        _, control, installer, script = approved[0]
        with mock.patch.object(
                TOOL, "_locate_template_members",
                return_value=located(control, installer, script)):
            with self.assertRaisesRegex(TOOL.ToolError,
                                        "canonical control archive"):
                TOOL._canonical_template_profile({"sha256": plaintext})
    def test_hash_gate_rejects_noncanonical_template(self):
        outer, unused = self._template()
        with self.assertRaisesRegex(TOOL.ToolError,
                                    "unsupported canonical template"):
            TOOL._canonical_template_profile(
                TOOL._inspect_archive_model(outer))

    def test_atomic_publication_never_overwrites_either_output(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "Creator5Pro-fixture.tgz"
            manifest = Path(str(output) + ".manifest.json")
            TOOL._publish_outputs(output, b"ciphertext", b"{\"ok\":true}\n")
            self.assertEqual(output.read_bytes(), b"ciphertext")
            self.assertEqual(manifest.read_bytes(), b"{\"ok\":true}\n")
            with self.assertRaisesRegex(TOOL.ToolError, "already exists"):
                TOOL._publish_outputs(output, b"replacement", b"{}\n")
            self.assertEqual(output.read_bytes(), b"ciphertext")
            self.assertEqual(manifest.read_bytes(), b"{\"ok\":true}\n")

        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "Creator5Pro-fixture.tgz"
            manifest = Path(str(output) + ".manifest.json")
            manifest.write_bytes(b"owned")
            with self.assertRaisesRegex(TOOL.ToolError, "already exists"):
                TOOL._publish_outputs(output, b"ciphertext", b"{}\n")
            self.assertFalse(output.exists())
            self.assertEqual(manifest.read_bytes(), b"owned")

    def test_output_contract_rejects_names_collisions_and_repository(
            self):
        with tempfile.TemporaryDirectory() as temp:
            template = Path(temp) / "template.tgz"
            template.write_bytes(b"template")
            valid = Path(temp) / "Creator5Pro-levelboard.tgz"
            self.assertEqual(TOOL._prepare_package_output(
                valid, [template]), valid.resolve())
            for invalid in (Path(temp) / "wrong.tgz",
                            Path(temp) / "Creator5Pro-.tgz", template):
                with self.subTest(path=invalid), self.assertRaises(
                        TOOL.ToolError):
                    TOOL._prepare_package_output(invalid, [template])
            with self.assertRaises(TOOL.ToolError):
                TOOL._prepare_package_output(
                    ROOT / "scripts" / "Creator5Pro-bad.tgz", [template])

    def test_checksum_and_shell_failures_are_rejected_before_packaging(self):
        files = {"./payload": b"content"}
        checksum = self._checksum_list(files, ["./payload"])
        TOOL._verify_md5(files, checksum, shutil.which("md5sum"))
        with self.assertRaisesRegex(TOOL.ToolError, "checksum mismatch"):
            TOOL._verify_md5({"./payload": b"changed"}, checksum,
                             shutil.which("md5sum"))
        with self.assertRaisesRegex(TOOL.ToolError, "sh -n"):
            TOOL._check_shell_syntax(b"#!/bin/sh\nif true; then\n",
                                     shutil.which("sh"))

    def test_manifest_is_schema_two_sanitized_and_marks_hardware(self):
        report = {
            "resolved_config": dict(
                TOOL.BOARD_PROFILES["levelBoard"]["required_config"]),
            "dictionary": {"kconfig": "/private/template/location"},
            "tools": {"python": "test-python",
                      "openssl": "test-openssl"},
        }
        repository = {"commit": "a" * 40, "dirty": False}
        manifest = TOOL._create_manifest(
            {"levelBoard": report},
            TOOL.CANONICAL_TEMPLATE_PROFILES[0]["plaintext"],
            b"plain", b"cipher", {"outer": [], "control": []},
            shutil.which("sh"), shutil.which("md5sum"), repository)
        serialized = json.dumps(manifest)
        self.assertEqual(manifest["schema_version"], 2)
        self.assertFalse(manifest["validation"]["hardware_verified"])
        self.assertEqual(
            manifest["firmwares"]["levelBoard"]["dictionary"]["kconfig"],
            "[redacted: validated separately]")
        self.assertNotIn("resolved_config", manifest)
        self.assertNotIn("firmware", manifest)
        self.assertNotIn("/private/template/location", serialized)
        self.assertNotIn("control-fixture.tar.xz", serialized)
        self.assertEqual(manifest["repository"], repository)
        self.assertRegex(manifest["repository"]["commit"], r"^[0-9a-f]{40}$")

        current_manifest = TOOL._create_manifest(
            {"levelBoard": report},
            TOOL.CANONICAL_TEMPLATE_PROFILES[1]["plaintext"],
            b"plain", b"cipher", {"outer": [], "control": []},
            shutil.which("sh"), shutil.which("md5sum"), repository)
        changes = {(item["effect"], item["status"])
                   for item in current_manifest["installer_changes"]}
        self.assertIn(("host disk-reclaim deletions", "suppressed"),
                      changes)
        self.assertIn(("unselected board failure-result checks",
                       "suppressed"), changes)
        self.assertIn(("selected board failure-result check and image",
                       "retained"), changes)
    def test_package_cli_validation_failure_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            template = root / "template.tgz"
            template.write_bytes(b"not-reached")
            output = root / "Creator5Pro-validation-failure.tgz"
            result = subprocess.run([
                sys.executable, str(TOOL_PATH), "package",
                "--template", str(template),
                "--firmware", "levelBoard", str(root / "missing.hex"),
                str(root / "missing.elf"), str(root / "missing.dict"),
                "--output", str(output)], cwd=ROOT, text=True,
                capture_output=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())
            self.assertNotIn(str(root), result.stderr)


class SelectionContractTests(unittest.TestCase):
    def test_package_rejects_dirty_firmware_before_reading_template(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            output = root / "Creator5Pro-test.tgz"
            products = {
                "firmware": root / "firmware.hex",
                "elf": root / "firmware.elf",
                "dictionary": root / "firmware.dict",
            }
            report = {"dictionary": {
                "version": "v0.13.0-1-gabcdef12-dirty-20260919_test"}}
            with mock.patch.object(
                    TOOL, "_validate_firmware",
                    return_value=(report, b"firmware")):
                with self.assertRaisesRegex(
                        TOOL.ToolError, "dirty firmware.*levelBoard"):
                    TOOL.package_update(
                        root / "template.tgz", {"levelBoard": products},
                        output)
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())


    def test_package_cli_accepts_explicit_firmware_groups(self):
        args = TOOL.create_argument_parser().parse_args([
            "package", "--template", "template.tgz",
            "--firmware", "levelBoard", "l.hex", "l.elf", "l.dict",
            "--firmware", "eBoard", "e.hex", "e.elf", "e.dict",
            "--output", "Creator5Pro-test.tgz",
        ])
        self.assertEqual([group[0] for group in args.firmware],
                         ["levelBoard", "eBoard"])

    def test_mainboard_is_selectable_after_existing_profiles(self):
        require_mainboard_profile(self)
        self.assertEqual(tuple(TOOL.BOARD_PROFILES),
                         ("eBoard", "heaterBoard", "levelBoard",
                          "mainBoardGD"))
        args = TOOL.create_argument_parser().parse_args([
            "package", "--template", "template.tgz",
            "--firmware", "mainBoardGD", "m.hex", "m.elf", "m.dict",
            "--output", "Creator5Pro-mainboardgd.tgz",
        ])
        self.assertEqual(args.firmware[0][0], "mainBoardGD")
        valid = {"firmware": "f.hex", "elf": "f.elf",
                 "dictionary": "f.dict"}
        normalized = TOOL._normalize_firmware_inputs({
            "mainBoardGD": valid, "levelBoard": valid,
            "eBoard": valid, "heaterBoard": valid,
        })
        self.assertEqual(list(normalized),
                         ["eBoard", "heaterBoard", "levelBoard",
                          "mainBoardGD"])

    def test_public_apis_reject_invalid_selection_before_tools(self):
        with mock.patch.object(TOOL, "_tool_path",
                               side_effect=AssertionError("tool ran")):
            with self.assertRaises(TOOL.ToolError):
                TOOL.build_firmware("stock", Path("unused"))
            with self.assertRaises(TOOL.ToolError):
                TOOL.package_update(Path("template"), {}, Path("output"))
            with self.assertRaises(TOOL.ToolError):
                TOOL.run_all_stages(["eBoard", "eBoard"],
                                    Path("template"), Path("build"),
                                    Path("output"))

    def test_ihex_profile_is_explicit(self):
        level = TOOL.parse_ihex("levelBoard", ihex())
        self.assertEqual(level["normalization"],
                         "application-48k-ff-fill-v1")
    def test_eboard_hex_and_dictionary_profiles_are_distinct(self):
        parsed = TOOL.parse_ihex("eBoard", ihex(board="eBoard"))
        self.assertEqual(parsed["normalization"],
                         "application-192k-ff-fill-v1")
        self.assertEqual(parsed["stack_pointer"], 0x20020000)
        eboard_dictionary = dictionary_data(board="eBoard")
        report = TOOL._load_dictionary("eBoard", eboard_dictionary)
        self.assertEqual(report["constants"]["SERIAL_BAUD"], 460800)
        with self.assertRaises(TOOL.ToolError):
            TOOL._load_dictionary("levelBoard", eboard_dictionary)
        with self.assertRaises(TOOL.ToolError):
            TOOL._load_dictionary("eBoard", dictionary_data())

    def test_public_package_mapping_validation_precedes_external_work(self):
        valid = {"firmware": "f.hex", "elf": "f.elf",
                 "dictionary": "f.dict"}
        invalid = [
            {}, {"stock": valid}, {"eBoard": "not-a-mapping"},
            {"eBoard": {"firmware": "f.hex", "elf": "f.elf"}},
            {"eBoard": dict(valid, extra="x")},
        ]
        with mock.patch.object(TOOL, "_prepare_package_output",
                               side_effect=AssertionError("published")):
            for value in invalid:
                with (self.subTest(value=value),
                      self.assertRaises(TOOL.ToolError)):
                    TOOL.package_update("template", value, "output")
        normalized = TOOL._normalize_firmware_inputs({
            "levelBoard": valid, "eBoard": valid})
        self.assertEqual(list(normalized), ["eBoard", "levelBoard"])

    def test_cli_rejects_invalid_selection_without_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            output = root / "Creator5Pro-invalid.tgz"
            base = [sys.executable, str(TOOL_PATH), "package",
                    "--template", str(root / "template.tgz")]
            cases = [
                base + ["--output", str(output)],
                base + ["--firmware", "stock", "a", "b", "c",
                        "--output", str(output)],
                base + ["--firmware", "eBoard", "a", "b", "c",
                        "--firmware", "eBoard", "d", "e", "f",
                        "--output", str(output)],
            ]
            for command in cases:
                result = subprocess.run(command, cwd=ROOT, text=True,
                                        capture_output=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(result.stdout, "")
                self.assertFalse(output.exists())
                self.assertFalse(Path(str(output) + ".manifest.json").exists())

    def test_all_preflights_every_build_directory_before_first_build(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            occupied = root / "all" / "build" / "levelBoard"
            occupied.mkdir(parents=True)
            sentinel = occupied / "owned"
            sentinel.write_text("keep")
            starts = []

            def build_stub(board, output_dir, *unused):
                starts.append(board)
                Path(output_dir).mkdir(parents=True, exist_ok=True)
                (Path(output_dir) / "started").write_text(board)

            output = root / "Creator5Pro-preflight.tgz"
            with mock.patch.object(TOOL, "build_firmware", build_stub):
                with self.assertRaisesRegex(TOOL.ToolError, "not empty"):
                    TOOL.run_all_stages(
                        ["levelBoard", "eBoard"], root / "template.tgz",
                        root / "all", output)
            self.assertEqual(starts, [])
            self.assertEqual(sentinel.read_text(), "keep")
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())

    def test_all_later_build_failure_keeps_builds_but_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            started = []

            def build_stub(board, output_dir, *unused):
                directory = Path(output_dir)
                directory.mkdir(parents=True, exist_ok=True)
                (directory / "started").write_text(board)
                started.append(board)
                if board == "levelBoard":
                    raise TOOL.ToolError("injected later build failure", 3)
                return {"stage": "build"}

            output = root / "Creator5Pro-later-failure.tgz"
            with mock.patch.object(TOOL, "build_firmware", build_stub):
                with self.assertRaisesRegex(TOOL.ToolError, "later build"):
                    TOOL.run_all_stages(
                        ["levelBoard", "eBoard"], root / "template.tgz",
                        root / "all", output)
            self.assertEqual(started, ["eBoard", "levelBoard"])
            for board in started:
                self.assertEqual(
                    (root / "all" / "build" / board / "started").read_text(),
                    board)
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output) + ".manifest.json").exists())
if __name__ == "__main__":
    unittest.main(verbosity=2)
