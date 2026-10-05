#!/usr/bin/env python3
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Build, validate, and package Creator 5 firmware updates."""

import argparse
import bz2
import datetime
import hashlib
import io
import json
import logging
import lzma
from collections.abc import Mapping
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import tarfile
import zlib

REPO_ROOT = Path(__file__).resolve().parents[1]
MAX_LAYER_BYTES = 256 * 1024 * 1024
MAX_TOTAL_EXPANDED = 512 * 1024 * 1024
MAX_ARCHIVE_MEMBERS = 10000
MAX_ARCHIVE_DEPTH = 8
_ARCHIVE_SUFFIXES = (".tar", ".tar.gz", ".tgz", ".tar.xz", ".txz",
                     ".tar.bz2", ".tbz", ".tbz2", ".gz", ".xz", ".bz2")

CANONICAL_TEMPLATE_PROFILES = (
    {
        "plaintext":
            "d3c60574199ffd5797f6a6e1f839316dbc3d5dd42e53ca2135ff4b5a30302616",
        "control":
            "2b05283f39cd68019e2d67b3068dff1e7a9a23507780635bab4bdfead2e48d9c",
        "installer":
            "615dc69a86e0f01a6e32688d4bd8615098e236d51cd7c5afdcd99d3113d3f8a8",
        "control_script":
            "a042533ff5be0392455fe06a8f5270b8e27da04661e8eef830146ad540ba47e6",
    },
    {
        "plaintext":
            "5aeb22a7c0f7f16c286ed74433582ee7fc1557e050dbf5243a3fb48a93960cd6",
        "control":
            "8f587d850b3876f65a482cf2d1d708348a5dbf2cda45c6211c9b0ed451910f3f",
        "installer":
            "c05dd781da74d1bf78bd8209730e27aad7de4e49abcf0e0265617395df6fbaa6",
        "control_script":
            "6fd03bd00a9ef297188d491b4352deef0721a24b8f1f21373db7363f969e3f0d",
        "space_reclaim": True,
    },
    {
        "plaintext":
            "89e7b26e92d27034a07bb15a9323a2879bc29d5a7d8603984151d28243f8f1f6",
        "control":
            "0c62cd01e90ba66860bb2a492a4d94ff7fc8e853087ef67db8196e70192207fb",
        "installer":
            "615dc69a86e0f01a6e32688d4bd8615098e236d51cd7c5afdcd99d3113d3f8a8",
        "control_script":
            "80e0d5de367aad641b61e105a7242483a484d830d052839bcc48e86d87e9cbb6",
        "space_reclaim": True,
    },
)
CANONICAL_IAP_SHA256 = (
    "c258bf965a92dad33b15bff616ef3ac72e958618b9cbb059f0a9cd4602c51f68")
PACKAGE_NAME_RE = re.compile(
    r"^Creator5Pro-[A-Za-z0-9][A-Za-z0-9._-]*\.tgz$")
COMPONENT_NAME_RE = re.compile(
    r"^(?:\./)?(control|kernel|library|software)-.+\.tar\.xz$")
CONTROL_MEMBER_PROFILES = (
    ("./eBoard.hex", "./heaterBoard.hex", "./IAPCommand",
     "./ISPCommand", "./levelBoard.hex", "./mainBoardGD.hex",
     "./mcu.img", "./md5sum.list", "./run.sh", "./Update"),
    ("./eBoard_fail.img", "./eBoard.hex", "./heaterBoard_fail.img",
     "./heaterBoard.hex", "./IAPCommand", "./ISPCommand",
     "./levelBoard_fail.img", "./levelBoard.hex", "./mainBoardGD.hex",
     "./mcu_fail.img", "./mcu.img", "./md5sum.list", "./run.sh",
     "./Update", "./VDS_V1.0.1_0.hex"),
    ("./eBoard_fail.img", "./eBoard.hex", "./heaterBoard_fail.img",
     "./heaterBoard.hex", "./IAPCommand", "./ISPCommand",
     "./ISPCommand-bak", "./levelBoard_fail.img", "./levelBoard.hex",
     "./mainBoardGD.hex", "./mcu_fail.img", "./mcu.img",
     "./md5sum.list", "./run.sh", "./Update", "./VDS_V1.0.1_0.hex"),
)

class ToolError(Exception):
    def __init__(self, message, exit_code=2):
        super().__init__(message)
        self.exit_code = exit_code
def _profile(board):
    try:
        return BOARD_PROFILES[board]
    except (KeyError, TypeError):
        raise ToolError("unknown or unimplemented board: %s" % board)


def _canonical_boards(boards):
    if isinstance(boards, (str, bytes)):
        raise ToolError("board selection must be a sequence")
    try:
        selected = list(boards)
    except TypeError:
        raise ToolError("board selection must be a sequence")
    if not selected:
        raise ToolError("at least one board must be selected")
    seen = set()
    for board in selected:
        _profile(board)
        if board in seen:
            raise ToolError("duplicate board selection: %s" % board)
        seen.add(board)
    return tuple(board for board in BOARD_PROFILES if board in seen)


def sanitize_diagnostic(message, secret=None):
    text = str(message)
    if secret:
        text = text.replace(secret, "[redacted]")
    text = re.sub(
    r"(?i)(?:[a-z]:[\\/]|/)(?:[^\s:'\"]+[\\/])+[^\s:'\"]*",
    "[redacted]",
     text)
    return text


def _inside(path, parent):
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def validate_output_root(path):
    try:
        path = Path(path).expanduser().resolve(strict=False)
        repo = REPO_ROOT.resolve()
    except (OSError, RuntimeError):
        raise ToolError("unable to resolve output path")
    literal_out = repo / "out"
    if (path == Path(path.anchor) or path == repo or
            (_inside(path, repo) and not _inside(path, literal_out))):
        raise ToolError(
            "output must be outside the repository or beneath its "
            "out directory")
    return path


def _hex_context(line_number, message):
    raise ToolError("Intel HEX line %d: %s" % (line_number, message))


def parse_ihex(board, data):
    """Parse and strictly validate a selected-board Intel HEX byte string."""
    profile = _profile(board)
    if isinstance(data, str):
        try:
            data = data.encode("ascii")
        except UnicodeEncodeError:
            raise ToolError("Intel HEX input is not ASCII")
    try:
        text = bytes(data).decode("ascii")
    except UnicodeDecodeError:
        raise ToolError("Intel HEX input is not ASCII")
    if not text:
        raise ToolError("Intel HEX input is empty")
    # splitlines keeps line-ending policy explicit and exposes internal blanks.
    lines = text.splitlines(keepends=True)
    if not lines or not lines[-1].endswith(("\n", "\r")):
        # A final record without an ending newline is legal.
        pass
    memory = {}
    linear_base = 0
    eof_seen = False
    start_linear = None
    for number, raw in enumerate(lines, 1):
        line = raw.rstrip("\r\n")
        endings = raw[len(line):]
        if endings not in ("", "\n", "\r\n"):
            _hex_context(number, "unsupported line ending")
        if not line:
            _hex_context(number, "blank internal record")
        if line != line.strip():
            _hex_context(number, "whitespace outside line ending")
        if eof_seen:
            _hex_context(number, "record after EOF")
        if not line.startswith(":"):
            _hex_context(number, "record does not start with ':'")
        encoded = line[1:]
        if len(encoded) % 2:
            _hex_context(number, "odd hexadecimal length")
        if not re.fullmatch(r"[0-9A-Fa-f]+", encoded or "-"):
            _hex_context(number, "non-hexadecimal record")
        record_bytes = bytes.fromhex(encoded)
        if len(record_bytes) < 5:
            _hex_context(number, "record is too short")
        count = record_bytes[0]
        if len(record_bytes) != count + 5:
            _hex_context(number, "record length does not match byte count")
        if sum(record_bytes) & 0xff:
            _hex_context(number, "checksum mismatch")
        address = (record_bytes[1] << 8) | record_bytes[2]
        kind = record_bytes[3]
        payload = record_bytes[4:-1]
        if kind == 0:
            end_offset = address + count
            if end_offset > 0x10000:
                _hex_context(number, "data crosses a 64 KiB address window")
            absolute = linear_base + address
            end_absolute = absolute + count
            if end_absolute > 0x100000000:
                _hex_context(number, "32-bit address arithmetic overflow")
            if count and (absolute < profile["app_start"] or
                          end_absolute > profile["app_end"]):
                _hex_context(
    number, "data address range 0x%08x..0x%08x is outside application range" %
     (absolute, end_absolute - 1))
            for offset, value in enumerate(payload):
                current = absolute + offset
                previous = memory.get(current)
                if previous is not None and previous != value:
                    _hex_context(
    number,
    "conflicting overlap at 0x%08x" %
     current)
                memory[current] = value
        elif kind == 1:
            if count or address:
                _hex_context(number, "EOF must have zero length and address")
            eof_seen = True
        elif kind == 4:
            if count != 2:
                _hex_context(
    number, "extended linear address record length must be 2")
            if address:
                _hex_context(
    number, "extended linear address record address must be zero")
            linear_base = int.from_bytes(payload, "big") << 16
        elif kind == 5:
            if count != 4:
                _hex_context(
    number, "start linear address record length must be 4")
            if address:
                _hex_context(
    number, "start linear address record address must be zero")
            if start_linear is not None:
                _hex_context(number, "duplicate start linear address record")
            start_linear = int.from_bytes(payload, "big")
            start_target = start_linear & ~1
            if not profile["app_start"] <= start_target < profile["app_end"]:
                _hex_context(
    number, "start linear address is outside application range")
        elif kind in (2, 3):
            _hex_context(
    number,
    "unsupported Intel HEX record type %02x" %
     kind)
        else:
            _hex_context(number, "unknown Intel HEX record type %02x" % kind)
    if not eof_seen:
        raise ToolError("Intel HEX is missing EOF record")
    if not memory:
        raise ToolError("Intel HEX contains no application data")
    missing_vectors = [address for address in range(
        profile["app_start"], profile["app_start"] + 8)
        if address not in memory]
    if missing_vectors:
        raise ToolError(
            "Intel HEX is missing vector bytes at application base")
    stack_pointer, reset_handler = struct.unpack(
        "<II", bytes(memory[address] for address in range(
            profile["app_start"], profile["app_start"] + 8)))
    if not profile["ram_start"] <= stack_pointer <= profile["ram_end"]:
        raise ToolError("stack pointer 0x%08x is outside SRAM" % stack_pointer)
    if stack_pointer & 7:
        raise ToolError(
    "stack pointer 0x%08x is not eight-byte aligned" %
     stack_pointer)
    if not reset_handler & 1:
        raise ToolError(
    "reset handler 0x%08x does not set the Thumb bit" %
     reset_handler)
    reset_target = reset_handler & ~1
    if not profile["app_start"] <= reset_target < profile["app_end"]:
        raise ToolError(
    "reset target 0x%08x is outside application range" %
     reset_target)
    if reset_target not in memory:
        raise ToolError(
    "reset target 0x%08x is not mapped executable data" %
     reset_target)
    addresses = sorted(memory)
    intervals = []
    start = previous = addresses[0]
    for address in addresses[1:]:
        if address != previous + 1:
            intervals.append([start, previous + 1])
            start = address
        previous = address
    intervals.append([start, previous + 1])
    holes = [[left[1], right[0]] for left, right in zip(
        intervals, intervals[1:]) if left[1] != right[0]]
    if intervals[-1][1] < profile["app_end"]:
        holes.append([intervals[-1][1], profile["app_end"]])
    normalized = bytearray(
        b"\xff" * (profile["app_end"] - profile["app_start"]))
    for address, value in memory.items():
        normalized[address - profile["app_start"]] = value
    return {
        "memory": memory,
        "mapped_byte_count": len(memory),
        "intervals": intervals,
        "holes": holes,
        "stack_pointer": stack_pointer,
        "reset_handler": reset_handler,
        "reset_target": reset_target,
        "start_linear_address": start_linear,
        "normalization": profile["normalization"],
        "normalized_sha256": hashlib.sha256(normalized).hexdigest(),
    }


def _mainboard_pin_enumeration():
    pins = {"PA%d" % pin: pin for pin in range(11)}
    pins.update({"PA%d" % pin: pin for pin in range(13, 16)})
    for port, base in (("B", 16), ("C", 32), ("D", 48), ("E", 64)):
        pins.update({"P%s%d" % (port, pin): base + pin
                     for pin in range(16)})
    pins.update({"PH2": 114, "PH3": 115})
    pins.update({"PJ%d" % pin: 144 + pin for pin in range(16)})
    return pins


def _package_pin_enumeration(port_masks, adc_temperature=False):
    pins = {}
    for port, mask in port_masks.items():
        base = (ord(port) - ord("A")) * 16
        pins.update({"P%s%d" % (port, pin): base + pin
                     for pin in range(16) if mask & (1 << pin)})
    if adc_temperature:
        pins["ADC_TEMPERATURE"] = 0xfe
    return pins


# Optional upstream drivers that no stock C5 firmware advertises; the C5
# Kconfig defaults leave them out (src/stm32/Kconfig).
TRIMMED_DRIVER_COMMANDS = {
    "config_st7920", "config_hd44780", "config_hx71x",
    "config_trigger_analog", "config_bmi160", "config_mpu9250",
    "config_icm20948", "config_ads1220", "config_ads131m0x",
    "config_ldc1612",
}


BOARD_PROFILES = {
    "eBoard": {
        "firmware_name": "eBoard.hex",
        "app_start": 0x08010000, "app_end": 0x08040000,
        "ram_start": 0x20000000, "ram_end": 0x20020000,
        "normalization": "application-192k-ff-fill-v1",
        "updater_name": "IAPCommand",
        "seed_config": (
            "CONFIG_MACH_STM32=y\n" "CONFIG_MACH_N32G455=y\n"
            "CONFIG_C5_EBOARD=y\n" "CONFIG_STM32_CLOCK_REF_12M=y\n"
            "CONFIG_STM32_SERIAL_USART1=y\n" "CONFIG_WANT_ADC=y\n"
            "CONFIG_WANT_HARD_PWM=y\n" "CONFIG_WANT_SPI=y\n"
            "CONFIG_WANT_LIS2DW=y\n"),
        "required_config": {
            "MACH_STM32": "y", "MACH_N32G455": "y", "C5_EBOARD": "y",
            "MCU": "n32g455ccl7", "STM32_SERIAL_USART1": "y",
            "WANT_ADC": "y", "WANT_HARD_PWM": "y", "WANT_SPI": "y",
            "WANT_LIS2DW": "y", "FLASH_APPLICATION_ADDRESS": "0x08010000",
            "FLASH_BOOT_ADDRESS": "0x08000000", "FLASH_SIZE": "0x40000",
            "RAM_START": "0x20000000", "RAM_SIZE": "0x20000",
            "CLOCK_FREQ": "144000000", "CLOCK_REF_FREQ": "12000000",
            "SERIAL_BAUD": "460800",
        },
        "forbidden_config": {
            "C5_HEATERBOARD", "C5_LEVELBOARD", "MACH_N32G430F8S7",
        },
        "required_constants": {
            "MCU": "n32g455ccl7", "ADC_MAX": 4095, "CLOCK_FREQ": 144000000,
            "PWM_MAX": 256, "RECEIVE_WINDOW": 192,
            "RESERVE_PINS_serial": "PA10,PA9", "SERIAL_BAUD": 460800,
            "STATS_SUMSQ_BASE": 256,
        },
        "required_enumerations": {
            "pin": dict(_package_pin_enumeration(
                {"A": 0xffff, "B": 0xffff, "C": 0xe000}, True),
                        PG0=96),
        },
        "required_commands": {
            "identify offset=%u count=%c", "allocate_oids count=%c",
            "get_config",
            "finalize_config crc=%u", "get_clock", "get_uptime",
            "emergency_stop", "clear_shutdown", "reset",
            ("config_stepper oid=%c step_pin=%c dir_pin=%c "
             "invert_step=%c step_pulse_ticks=%u"),
            "queue_step oid=%c interval=%u count=%hu add=%hi",
            "set_next_step_dir oid=%c dir=%c",
            "reset_step_clock oid=%c clock=%u",
            "stepper_get_position oid=%c",
            "stepper_stop_on_trigger oid=%c trsync_oid=%c",
            "config_endstop oid=%c pin=%c pull_up=%c",
            ("endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c "
             "rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c"),
            "endstop_query_state oid=%c", "endstop_recover_state oid=%c",
            "config_trsync oid=%c",
            ("trsync_start oid=%c report_clock=%u report_ticks=%u "
             "expire_reason=%c"),
            "trsync_set_timeout oid=%c clock=%u",
            "trsync_trigger oid=%c reason=%c",
            ("config_digital_out oid=%c pin=%u value=%c default_value=%c "
             "max_duration=%u"),
            "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u",
            "queue_digital_out oid=%c clock=%u on_ticks=%u",
            "update_digital_out oid=%c value=%c",
            ("config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu "
             "default_value=%hu max_duration=%u"),
            "queue_pwm_out oid=%c clock=%u value=%hu",
            "config_analog_in oid=%c pin=%u",
            ("query_analog_in oid=%c clock=%u sample_ticks=%u "
             "sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu "
             "range_check_count=%c"),
            "config_spi oid=%c pin=%u cs_active_high=%c",
            "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u",
            "spi_transfer oid=%c data=%*s", "spi_send oid=%c data=%*s",
            "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s",
            "config_lis2dw oid=%c bus_oid=%c bus_oid_type=%c lis_chip_type=%c",
            "query_lis2dw oid=%c rest_ticks=%u", "query_lis2dw_status oid=%c",
            "get_mcu_version", "set_trigger_threshold threshold=%i",
            "get_basic_param num=%u", "remove_peel action=%u",
            "pa_action action=%u pc=%u", "get_emcu_pa_value",
        },
        "required_responses": {
            "identify_response offset=%u data=%.*s",
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
            "clock clock=%u", "uptime high=%u clock=%u",
            "stats count=%u sum=%u sumsq=%u", "starting",
            "is_shutdown static_string_id=%hu",
            "shutdown clock=%u static_string_id=%hu",
            "stepper_position oid=%c pos=%i",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            "analog_in_state oid=%c next_clock=%u value=%hu",
            "spi_transfer_response oid=%c response=%*s",
            "sensor_bulk_data oid=%c sequence=%hu data=%*s",
            ("sensor_bulk_status oid=%c clock=%u query_ticks=%u "
             "next_sequence=%hu buffered=%u possible_overflows=%hu"),
            "mcu_version year=%u date=%u version=%u",
            "trigger_threshold threshold=%i", "param_value value=%u reserve=%u",
            "peel_data value=%i", "pa_value value=%u",
        },
        "forbidden_commands": {"config_reset"} | TRIMMED_DRIVER_COMMANDS,
        "forbidden_responses": {"endstop_recover_state", "pa_action"},
        "forbidden_format_names": {"Levelboard"},
    },
    "heaterBoard": {
        "firmware_name": "heaterBoard.hex",
        "app_start": 0x08010000, "app_end": 0x08080000,
        "ram_start": 0x20000000, "ram_end": 0x20020000,
        "normalization": "application-448k-ff-fill-v1",
        "updater_name": "IAPCommand",
        "seed_config": (
            "CONFIG_MACH_STM32=y\n" "CONFIG_MACH_N32G455=y\n"
            "CONFIG_C5_HEATERBOARD=y\n"
            "CONFIG_STM32_CLOCK_REF_12M=y\n"
            "CONFIG_STM32_SERIAL_USART1=y\n" "CONFIG_WANT_ADC=y\n"
            "CONFIG_WANT_BUTTONS=y\n"),
        "required_config": {
            "MACH_STM32": "y", "MACH_N32G455": "y",
            "C5_HEATERBOARD": "y", "MCU": "n32g455rel7",
            "STM32_SERIAL_USART1": "y", "WANT_ADC": "y",
            "WANT_BUTTONS": "y", "WANT_HARD_PWM": "y",
            "ARMCM_FLASH_SIZE_IS_TOTAL": "y",
            "ARMCM_EXPLICIT_RESET_ENTRY": "y",
            "FLASH_APPLICATION_ADDRESS": "0x08010000",
            "FLASH_BOOT_ADDRESS": "0x08000000", "FLASH_SIZE": "0x80000",
            "RAM_START": "0x20000000", "RAM_SIZE": "0x20000",
            "CLOCK_FREQ": "144000000", "CLOCK_REF_FREQ": "12000000",
            "SERIAL_BAUD": "230400",
        },
        "forbidden_config": {
            "C5_EBOARD", "C5_LEVELBOARD", "MACH_N32G430F8S7",
        },
        "required_constants": {
            "MCU": "n32g455rel7", "ADC_MAX": 4095, "PWM_MAX": 256,
            "CLOCK_FREQ": 144000000, "RECEIVE_WINDOW": 192,
            "RESERVE_PINS_serial": "PA10,PA9", "SERIAL_BAUD": 230400,
            "STATS_SUMSQ_BASE": 256,
        },
        "required_enumerations": {
            "pin": _package_pin_enumeration(
                {"A": 0xffff, "B": 0xffff, "C": 0xffff, "D": 0x0004},
                True),
        },
        "required_commands": {
            "identify offset=%u count=%c", "allocate_oids count=%c",
            "get_config", "finalize_config crc=%u", "get_clock",
            "get_uptime", "emergency_stop", "clear_shutdown", "reset",
            ("config_digital_out oid=%c pin=%u value=%c default_value=%c "
             "max_duration=%u"),
            "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u",
            "queue_digital_out oid=%c clock=%u on_ticks=%u",
            "update_digital_out oid=%c value=%c",
            "set_digital_out pin=%u value=%c",
            "config_analog_in oid=%c pin=%u",
            ("query_analog_in oid=%c clock=%u sample_ticks=%u "
             "sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu "
             "range_check_count=%c"),
            "config_buttons oid=%c button_count=%c",
            "buttons_add oid=%c pos=%c pin=%u pull_up=%c",
            ("buttons_query oid=%c clock=%u rest_ticks=%u "
             "retransmit_count=%c invert=%c"),
            "buttons_ack oid=%c count=%c", "get_mcu_version",
        },
        "required_responses": {
            "identify_response offset=%u data=%.*s",
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
            "clock clock=%u", "uptime high=%u clock=%u",
            "stats count=%u sum=%u sumsq=%u", "starting",
            "is_shutdown static_string_id=%hu",
            "shutdown clock=%u static_string_id=%hu",
            "analog_in_state oid=%c next_clock=%u value=%hu",
            "buttons_state oid=%c ack_count=%c state=%*s",
            "mcu_version year=%u date=%u version=%u",
        },
        "forbidden_commands": {
            "config_reset", "set_trigger_threshold", "get_basic_param",
            "pa_action", "get_emcu_pa_value", "remove_peel",
        } | TRIMMED_DRIVER_COMMANDS,
        "forbidden_responses": {
            "trigger_threshold", "param_value", "pa_value", "peel_data",
            "pa_action",
        },
        "forbidden_format_names": {"Eboard", "Eheaterboard", "Levelboard"},
    },
    "levelBoard": {
        "firmware_name": "levelBoard.hex",
        "app_start": 0x08004000, "app_end": 0x08010000,
        "ram_start": 0x20000000, "ram_end": 0x20004000,
        "normalization": "application-48k-ff-fill-v1",
        "updater_name": "IAPCommand",
        "seed_config": (
            "CONFIG_MACH_STM32=y\n" "CONFIG_MACH_N32G430F8S7=y\n"
            "CONFIG_STM32_CLOCK_REF_8M=y\n" "CONFIG_SERIAL=y\n"
            "CONFIG_STM32_SERIAL_USART1=y\n" "CONFIG_C5_LEVELBOARD=y\n"),
        "required_config": {
            "MACH_STM32": "y", "MACH_N32G430F8S7": "y",
            "C5_LEVELBOARD": "y", "MCU": "n32g430f8s7",
            "STM32_SERIAL_USART1": "y",
            "FLASH_APPLICATION_ADDRESS": "0x08004000",
            "FLASH_BOOT_ADDRESS": "0x08000000", "FLASH_SIZE": "0x10000",
            "RAM_START": "0x20000000", "RAM_SIZE": "0x4000",
            "CLOCK_FREQ": "128000000", "CLOCK_REF_FREQ": "8000000",
            "SERIAL_BAUD": "230400",
        },
        "forbidden_config": {
            "C5_HEATERBOARD", "MACH_STM32F1", "MACH_N32G45x",
        },
        "required_constants": {
            "MCU": "n32g430f8s7", "ADC_MAX": 4095,
            "CLOCK_FREQ": 128000000,
            "RECEIVE_WINDOW": 192, "RESERVE_PINS_serial": "PA10,PA9",
            "SERIAL_BAUD": 230400, "STATS_SUMSQ_BASE": 256,
        },
        "required_enumerations": {
            "pin": _package_pin_enumeration(
                {"A": 0x06ff, "B": 0x0002, "D": 0x0001}),
        },
        "required_commands": {
            "identify offset=%u count=%c", "set_trigger_threshold threshold=%i",
            "get_mcu_version", "get_basic_param num=%u",
            "remove_peel action=%u", "clear_shutdown", "emergency_stop",
            "get_uptime", "get_clock", "finalize_config crc=%u",
            "get_config", "allocate_oids count=%c",
            "stepper_stop_on_trigger oid=%c trsync_oid=%c",
            "endstop_recover_state oid=%c", "endstop_query_state oid=%c",
            ("endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c "
             "rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c"),
            "config_endstop oid=%c pin=%c pull_up=%c",
            "trsync_trigger oid=%c reason=%c",
            "trsync_set_timeout oid=%c clock=%u",
            ("trsync_start oid=%c report_clock=%u report_ticks=%u "
             "expire_reason=%c"),
            "config_trsync oid=%c", "reset",
        },
        "required_responses": {
            "identify_response offset=%u data=%.*s",
            "trigger_threshold threshold=%i",
            "mcu_version year=%u date=%u version=%u",
            "param_value value=%u reserve=%u", "peel_data value=%i",
            "uptime high=%u clock=%u", "clock clock=%u",
            "config is_config=%c crc=%u is_shutdown=%c move_count=%hu",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            "starting", "is_shutdown static_string_id=%hu",
            "shutdown clock=%u static_string_id=%hu",
        },
        "forbidden_commands": {"config_reset", "config_analog_in",
                               "query_analog_in"} | TRIMMED_DRIVER_COMMANDS,
        "forbidden_responses": {"endstop_recover_state"},
        "forbidden_format_names": {"Levelboard"},
    },
    "mainBoardGD": {
        "firmware_name": "mainBoardGD.hex",
        "app_start": 0x08000000, "app_end": 0x08100000,
        "ram_start": 0x24000000, "ram_end": 0x24080000,
        "normalization": "application-1024k-ff-fill-v1",
        "updater_name": "ISPCommand",
        "seed_config": (
            "CONFIG_MACH_STM32=y\n"
            "CONFIG_MACH_GD32H737VGT6=y\n"
            "CONFIG_C5_MAINBOARDGD=y\n"
            "CONFIG_STM32_CLOCK_REF_25M=y\n"
            "CONFIG_GD32_SERIAL_USART0=y\n"
            "CONFIG_WANT_ADC=y\n"
            "CONFIG_WANT_BUTTONS=y\n"),
        "required_config": {
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
        },
        "forbidden_config": {
            "C5_EBOARD", "C5_HEATERBOARD", "C5_LEVELBOARD",
            "MACH_STM32H7", "MACH_N32G455", "MACH_N32G430F8S7",
        },
        "required_constants": {
            "MCU": "gd32h737vgt6", "ADC_MAX": 4095,
            "CLOCK_FREQ": 600000000, "RECEIVE_WINDOW": 192,
            "RESERVE_PINS_serial": "PA10,PA9", "SERIAL_BAUD": 230400,
            "STATS_SUMSQ_BASE": 256,
        },
        "required_enumerations": {
            "stepper": {
                "stepper_x": 0, "stepper_y": 1,
                "stepper_z": 2, "extruder": 3,
            },
            "pin": _mainboard_pin_enumeration(),
        },
        "required_commands": {
            "identify offset=%u count=%c", "allocate_oids count=%c",
            "get_config", "finalize_config crc=%u", "get_clock",
            "get_uptime", "emergency_stop", "clear_shutdown", "reset",
            "debug_nop", "debug_ping data=%*s",
            "debug_read order=%c addr=%u",
            "debug_write order=%c addr=%u val=%u",
            ("config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c "
             "step_pulse_ticks=%u"),
            "queue_step oid=%c interval=%u count=%hu add=%hi",
            "set_next_step_dir oid=%c dir=%c",
            "reset_step_clock oid=%c clock=%u",
            "stepper_get_position oid=%c",
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
            "set_digital_out pin=%u value=%c",
            "config_analog_in oid=%c pin=%u",
            ("query_analog_in oid=%c clock=%u sample_ticks=%u "
             "sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu "
             "range_check_count=%c"),
            "config_buttons oid=%c button_count=%c",
            "buttons_add oid=%c pos=%c pin=%u pull_up=%c",
            ("buttons_query oid=%c clock=%u rest_ticks=%u "
             "retransmit_count=%c invert=%c"),
            "buttons_ack oid=%c count=%c",
            "config_mclib oid=%c stepper=%u rs=%u ls=%u km=%u",
            "mclib_config_microstep oid=%c interpolate=%c mstep=%u",
            "mclib_config_stalldetect oid=%c stallthrs=%u",
            "mclib_set_current oid=%c run_current=%u hold_current=%u",
            ("mclib_set_resonance_damp oid=%c tdx=%c amp=%u phase1=%u "
             "phase2=%u"),
            "get_mcu_version",
        },
        "required_responses": {
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
        },
        "forbidden_commands": {
            "config_reset", "endstop_recover_state", "get_basic_param",
            "set_trigger_threshold", "config_pwm_out", "queue_pwm_out",
            "config_spi", "spi_set_bus", "spi_transfer", "spi_send",
            "config_spi_shutdown", "config_lis2dw", "query_lis2dw",
            "query_lis2dw_status", "mclib_set_pid_params",
            "mclib_identify_motor", "remove_peel", "pa_action",
            "get_emcu_pa_value", "config_tmcuart", "config_neopixel",
            "config_counter",
        } | TRIMMED_DRIVER_COMMANDS,
        "forbidden_responses": {
            "endstop_recover_state", "param_value", "peel_data",
            "trigger_threshold", "spi_transfer_response", "pa_value",
        },
        "forbidden_format_names": {
            "Eboard", "Eheaterboard", "GDMainboard", "Levelboard",
        },
        "forbidden_formats": {
            ("query_analog_in oid=%c clock=%u sample_ticks=%u "
             "sample_count=%c rest_ticks=%u bytes_per_report=%c "
             "min_value=%hu max_value=%hu range_check_count=%c"),
        },
        "forbidden_constants": {"STEPPER_OPTIMIZED_EDGE"},
    },
}


def _unsupported_build_path(value):
    return re.fullmatch(r"[A-Za-z0-9_./+\-]+", str(value)) is None


def _tool_path(name):
    try:
        found = shutil.which(name)
        path = str(Path(found).resolve()) if found else None
    except (OSError, RuntimeError):
        path = None
    if not path:
        raise ToolError("required external tool is unavailable: %s" % name, 3)
    if _unsupported_build_path(path):
        raise ToolError("unsupported build path for %s" % name)
    return path


def _run(command, logical_name, cwd=REPO_ROOT, env=None):
    try:
        result = subprocess.run(
    command,
    cwd=cwd,
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
     check=False)
    except OSError as exc:
        raise ToolError("unable to run %s: %s" % (logical_name, exc), 3)
    if result.returncode:
        diagnostic = result.stderr.decode("utf-8", "replace").strip()
        if not diagnostic:
            diagnostic = result.stdout.decode("utf-8", "replace").strip()
        diagnostic = sanitize_diagnostic(diagnostic)
        if len(diagnostic) > 1000:
            diagnostic = diagnostic[-1000:]
        raise ToolError("%s failed%s" % (logical_name,
                        ": " + diagnostic if diagnostic else ""), 3)
    return result.stdout


def _version(program):
    try:
        result = subprocess.run([program,
    "--version"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    check=False,
     timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return "unavailable"
    if result.returncode:
        return "unavailable"
    line = result.stdout.decode("utf-8", "replace").splitlines()
    if not line:
        return "unavailable"
    value = line[0].strip()
    base = Path(program).name
    value = re.sub(r"^.*[\\/]" + re.escape(base) + r"\s*", "", value)
    return value[:240]


def _load_resolved_config(board, config_path):
    profile = _profile(board)
    module_path = REPO_ROOT / "lib" / "kconfiglib"
    sys.path.insert(0, str(module_path))
    old_srctree = os.environ.get("srctree")
    os.environ["srctree"] = str(REPO_ROOT)
    try:
        import kconfiglib
        kconf = kconfiglib.Kconfig(str(REPO_ROOT / "src" / "Kconfig"),
                                   warn=False)
        kconf.load_config(str(config_path))
        values = {}
        for name in profile["required_config"]:
            symbol = kconf.syms.get(name)
            if symbol is None:
                raise ToolError(
    "required Kconfig symbol is missing: %s" %
     name)
            values[name] = symbol.str_value
        for forbidden in profile["forbidden_config"]:
            symbol = kconf.syms.get(forbidden)
            if symbol is not None and symbol.str_value == "y":
                raise ToolError(
    "incompatible hardware Kconfig selected: %s" %
     forbidden)
    except ToolError:
        raise
    except (OSError, UnicodeError, ValueError):
        raise ToolError("unable to resolve supplied Kconfig")
    finally:
        if old_srctree is None:
            os.environ.pop("srctree", None)
        else:
            os.environ["srctree"] = old_srctree
        try:
            sys.path.remove(str(module_path))
        except ValueError:
            pass
    for name, expected in profile["required_config"].items():
        actual = values[name]
        if name in (
    "FLASH_APPLICATION_ADDRESS",
    "FLASH_BOOT_ADDRESS",
    "FLASH_SIZE",
    "RAM_START",
     "RAM_SIZE"):
            try:
                matches = int(actual, 0) == int(expected, 0)
            except ValueError:
                matches = False
        else:
            matches = actual == expected
        if not matches:
            raise ToolError("resolved Kconfig %s is %s, expected %s" %
                            (name, actual, expected))
    return values


def _load_resolved_config_text(board, config_text):
    if not isinstance(config_text, str) or not config_text:
        raise ToolError("dictionary Kconfig is missing or invalid")
    try:
        with tempfile.TemporaryDirectory() as temp:
            config_path = Path(temp) / ".config"
            config_path.write_text(config_text, encoding="utf-8", newline="\n")
            return _load_resolved_config(board, config_path)
    except OSError:
        raise ToolError("unable to validate dictionary Kconfig")


def _parse_sections(text, elf_size):
    sections = []
    lines = text.splitlines()
    pattern = re.compile(
        r"^\s*\d+\s+(\S+)\s+([0-9a-fA-F]{8,16})\s+"
        r"([0-9a-fA-F]{8,16})\s+([0-9a-fA-F]{8,16})\s+"
        r"([0-9a-fA-F]{8,16})\s+2\*\*\d+\s*$")
    index = 0
    while index < len(lines):
        match = pattern.match(lines[index])
        if not match:
            if re.match(r"^\s*\d+\s+", lines[index]):
                raise ToolError("ambiguous objdump section output")
            index += 1
            continue
        if index + 1 >= len(lines):
            raise ToolError("ambiguous objdump section output")
        flags = {item.strip() for item in lines[index + 1].strip().split(",")}
        if not flags or any(not item for item in flags):
            raise ToolError(
    "ambiguous objdump flags for section %s" %
     match.group(1))
        section = {
            "name": match.group(1), "size": int(match.group(2), 16),
            "vma": int(match.group(3), 16), "lma": int(match.group(4), 16),
            "offset": int(match.group(5), 16), "flags": flags,
        }
        if section["size"] and "CONTENTS" in flags:
            end = section["offset"] + section["size"]
            if end < section["offset"] or end > elf_size:
                raise ToolError(
    "ELF section %s exceeds file bounds" %
     section["name"])
        sections.append(section)
        index += 2
    if not sections:
        raise ToolError("objdump reported no ELF sections")
    return sections


def _redact_dictionary_metadata(value):
    value = str(value)
    if (len(value) > 240 or any(ord(char) < 0x20 for char in value) or
            re.search(r"(?i)(?:[a-z]:[\\/]|/|\\\\)", value)):
        return "[redacted: unsafe metadata]"
    return value


def _load_dictionary(board, data):
    profile = _profile(board)
    try:
        raw = json.loads(data.decode("utf-8"))
    except (AttributeError, UnicodeDecodeError, ValueError) as exc:
        raise ToolError("invalid protocol dictionary JSON: %s" % exc)
    sys.path.insert(0, str(REPO_ROOT / "klippy"))
    previous_logging_disable = logging.root.manager.disable
    try:
        import msgproto
        parser = msgproto.MessageParser()
        logging.disable(logging.CRITICAL)
        try:
            parser.process_identify(data, decompress=False)
        except Exception as exc:
            raise ToolError("invalid protocol dictionary: %s" % exc)
        finally:
            logging.disable(previous_logging_disable)
    finally:
        try:
            sys.path.remove(str(REPO_ROOT / "klippy"))
        except ValueError:
            pass
    constants = parser.get_constants()
    for name, expected in profile["required_constants"].items():
        if constants.get(name) != expected:
            raise ToolError(
    "dictionary constant %s does not match required value" %
     name)
    for name in profile.get("forbidden_constants", ()):
        if name in constants:
            raise ToolError("forbidden dictionary constant is present: %s" %
                            name)
    enumerations = parser.get_enumerations()
    for enum_name, expected in profile.get(
            "required_enumerations", {}).items():
        if enumerations.get(enum_name) != expected:
            raise ToolError(
                "dictionary enumeration %s does not match required values" %
                enum_name)
    resolved_config = _load_resolved_config_text(board, parser.get_kconfig())
    raw_tables = {"command": raw.get("commands", {}),
                  "response": raw.get("responses", {}),
                  "output": raw.get("output", {})}
    id_bindings = {}
    name_bindings = {}
    format_bindings = {}
    for msgtype, table in raw_tables.items():
        if not isinstance(table, dict):
            raise ToolError("dictionary message table is invalid")
        for msgformat, msgid in table.items():
            if not isinstance(msgformat, str) or not isinstance(msgid, int):
                raise ToolError("dictionary message assignment is invalid")
            binding = (msgtype, msgformat)
            previous = id_bindings.setdefault(msgid, binding)
            if previous != binding:
                raise ToolError(
                    "dictionary has conflicting message ID assignments")
            previous_format = format_bindings.setdefault(msgformat,
                                                         (msgtype, msgid))
            if previous_format != (msgtype, msgid):
                raise ToolError(
                    "dictionary has conflicting message format assignments")
            if msgtype != "output":
                name = msgformat.split(" ", 1)[0]
                name_binding = (msgid, msgtype, msgformat)
                previous_name = name_bindings.setdefault(name, name_binding)
                if previous_name != name_binding:
                    raise ToolError(
                        "dictionary has conflicting non-output message names")
    classified = {}
    for msgid, msgtype, msgformat in parser.get_messages():
        if msgformat in format_bindings:
            classified[msgformat] = (msgid, msgtype)
    message_types = {"command": set(), "response": set(), "output": set()}
    for msgformat, (expected_type, expected_id) in format_bindings.items():
        if classified.get(msgformat) != (expected_id, expected_type):
            raise ToolError(
                "dictionary message membership conflicts with MessageParser")
        message_types[expected_type].add(msgformat)
    missing_commands = profile["required_commands"] - message_types["command"]
    missing_responses = (
        profile["required_responses"] - message_types["response"])
    if missing_commands:
        raise ToolError(
    "dictionary is missing required command membership: %s" %
     sorted(missing_commands)[0])
    if missing_responses:
        raise ToolError(
    "dictionary is missing required response membership: %s" %
     sorted(missing_responses)[0])
    command_names = {item.split(" ", 1)[0]
                                for item in message_types["command"]}
    response_names = {item.split(" ", 1)[0]
                                 for item in message_types["response"]}
    for forbidden in profile["forbidden_commands"]:
        if forbidden in command_names:
            raise ToolError(
                "forbidden dictionary command is present: %s" % forbidden)
    for forbidden in profile["forbidden_responses"]:
        if forbidden in response_names:
            raise ToolError(
                "forbidden dictionary response is present: %s" % forbidden)
    all_formats = set().union(*message_types.values())
    forbidden_names = profile["forbidden_format_names"]
    if any(item.split(" ", 1)[0] in forbidden_names for item in all_formats):
        raise ToolError("forbidden dictionary format name is present")
    forbidden_formats = profile.get("forbidden_formats", set())
    if forbidden_formats & all_formats:
        raise ToolError("forbidden dictionary format is present")
    if raw.get("commands", {}).get("identify offset=%u count=%c") != 1:
        raise ToolError("bootstrap identify message ID is not 1")
    if raw.get("responses", {}).get(
        "identify_response offset=%u data=%.*s") != 0:
        raise ToolError("bootstrap identify_response message ID is not 0")
    version, build_versions = parser.get_version_info()
    return {
    "constants": {
        name: constants[name]
        for name in sorted(profile["required_constants"])},
        "version": _redact_dictionary_metadata(version),
        "build_versions": _redact_dictionary_metadata(build_versions),
        "kconfig": "[redacted: validated separately]",
        "resolved_config": resolved_config,
         }


def _read_input_file(value):
    name = Path(value).name
    try:
        path = Path(value).expanduser().resolve(strict=False)
        if not path.is_file():
            raise ToolError("required input is missing: %s" % name)
        data = path.read_bytes()
    except ToolError:
        raise
    except (OSError, RuntimeError):
        raise ToolError("unable to read required input: %s" % name)
    if not data:
        raise ToolError("firmware input is empty: %s" % name)
    return path, data
def _validate_firmware(board, firmware, elf, dictionary,
                       cross_prefix="arm-none-eabi-", openssl="openssl"):
    profile = _profile(board)
    firmware, exact_hex = _read_input_file(firmware)
    elf, elf_data = _read_input_file(elf)
    dictionary, dictionary_data = _read_input_file(dictionary)
    image = parse_ihex(board, exact_hex)
    objdump = _tool_path(cross_prefix + "objdump")
    nm = _tool_path(cross_prefix + "nm")
    env = os.environ.copy()
    env["LC_ALL"] = "C"
    file_info = _run([objdump, "-f", str(elf)], "objdump",
                     env=env).decode("ascii", "replace")
    if "file format elf32-littlearm" not in file_info or not re.search(
        r"architecture:\s*arm", file_info):
        raise ToolError(
            "ELF is not an expected ELF32 little-endian ARM executable")
    entries = re.findall(r"start address 0x([0-9a-fA-F]+)", file_info)
    if len(entries) != 1:
        raise ToolError("objdump did not report exactly one ELF entry point")
    elf_entry = int(entries[0], 16)
    if (image["start_linear_address"] is not None and
            (image["start_linear_address"] & ~1) != (elf_entry & ~1)):
        raise ToolError(
            "Intel HEX start linear address does not match ELF entry point")
    section_text = _run([objdump, "-h", str(elf)], "objdump",
                        env=env).decode("ascii", "replace")
    sections = _parse_sections(section_text, len(elf_data))
    file_backed = []
    code_ranges = []
    allowed = set()
    for section in sections:
        size = section["size"]
        flags = section["flags"]
        if not size or "ALLOC" not in flags:
            continue
        vma_end = section["vma"] + size
        in_flash = (profile["app_start"] <= section["vma"] and
                    vma_end <= profile["app_end"])
        in_ram = (profile["ram_start"] <= section["vma"] and
                  vma_end <= profile["ram_end"])
        has_load_image = "CONTENTS" in flags and "LOAD" in flags
        if has_load_image:
            if vma_end > 0x100000000 or (not in_flash and not in_ram):
                if (profile["ram_start"] <= section["vma"] <=
                        profile["ram_end"] or
                        section["vma"] < profile["ram_start"] < vma_end):
                    raise ToolError(
    "RAM section %s crosses SRAM bounds" %
     section["name"])
                raise ToolError(
    "allocated section %s is outside flash and SRAM" %
     section["name"])
            end = section["lma"] + size
            if (end > 0x100000000 or
                    section["lma"] < profile["app_start"] or
                    end > profile["app_end"]):
                raise ToolError(
    "ELF load section %s crosses application flash bounds" %
     section["name"])
            body = elf_data[section["offset"]:section["offset"] + size]
            for offset, value in enumerate(body):
                address = section["lma"] + offset
                if image["memory"].get(address) != value:
                    raise ToolError("ELF/HEX mismatch in %s at 0x%08x" %
                                    (section["name"], address))
                allowed.add(address)
            file_backed.append(section)
            if "CODE" in flags:
                code_ranges.append((section["vma"], vma_end))
        elif vma_end > 0x100000000 or not in_ram:
            raise ToolError(
    "RAM ALLOC/NOLOAD section %s is outside SRAM bounds" %
     section["name"])
    extra = sorted(set(image["memory"]) - allowed)
    if extra:
        raise ToolError(
    "HEX has byte outside ELF load sections at 0x%08x" %
     extra[0])
    if not any(start <= image["reset_target"] <
               end for start, end in code_ranges):
        raise ToolError("reset target is not within an ELF CODE section")
    nm_text = _run([nm, "-S", "--defined-only", str(elf)],
                   "nm", env=env).decode("ascii", "replace")
    matches = re.findall(
    r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\S\s+command_identify_data$",
    nm_text,
     re.MULTILINE)
    if len(matches) != 1:
        raise ToolError(
            "ELF must define exactly one sized command_identify_data symbol")
    symbol_address, symbol_size = (int(item, 16) for item in matches[0])
    owner = [section for section in file_backed
             if section["vma"] <= symbol_address and
             symbol_address + symbol_size <= section["vma"] + section["size"]]
    if len(owner) != 1 or not symbol_size:
        raise ToolError(
            "command_identify_data is not contained in one ELF load section")
    section = owner[0]
    position = section["offset"] + symbol_address - section["vma"]
    compressed = elf_data[position:position + symbol_size]
    try:
        embedded = zlib.decompress(compressed)
    except zlib.error as exc:
        raise ToolError("embedded dictionary is not valid zlib data: %s" % exc)
    if embedded != dictionary_data:
        raise ToolError(
            "supplied dictionary does not match embedded dictionary")
    dictionary_report = _load_dictionary(board, dictionary_data)
    report = {key: value for key, value in image.items() if key != "memory"}
    report.update({
        "bounds": {
            "application": [profile["app_start"], profile["app_end"]],
            "sram": [profile["ram_start"], profile["ram_end"]]},
        "hex_sha256": hashlib.sha256(exact_hex).hexdigest(),
        "elf_sha256": hashlib.sha256(elf_data).hexdigest(),
        "dictionary_sha256": hashlib.sha256(dictionary_data).hexdigest(),
        "constants": dictionary_report["constants"],
        "dictionary": {key: dictionary_report[key]
                       for key in ("version", "build_versions", "kconfig")},
        "resolved_config": dictionary_report["resolved_config"],
        "tools": {
            "python": sys.version.splitlines()[0],
            "compiler": _version(_tool_path(cross_prefix + "gcc")),
            "objcopy": _version(_tool_path(cross_prefix + "objcopy")),
            "openssl": _version(openssl),
        },
    })
    return report, exact_hex


def validate_firmware(board, firmware, elf, dictionary,
                      cross_prefix="arm-none-eabi-", openssl="openssl"):
    report, unused_exact_hex = _validate_firmware(
        board, firmware, elf, dictionary, cross_prefix, openssl)
    return report


def _preflight_build_output(output_dir):
    output = validate_output_root(output_dir)
    try:
        if output.exists():
            if not output.is_dir():
                raise ToolError("build output path is not a directory")
            if any(output.iterdir()):
                raise ToolError("build output directory is not empty")
    except ToolError:
        raise
    except (OSError, RuntimeError):
        raise ToolError("unable to prepare build output directory")
    return output


def build_firmware(board, output_dir, jobs=1, cross_prefix="arm-none-eabi-",
                   openssl="openssl"):
    profile = _profile(board)
    if not isinstance(jobs, int) or jobs < 1:
        raise ToolError("jobs must be a positive integer")
    output = _preflight_build_output(output_dir)
    try:
        output.mkdir(parents=True, exist_ok=True)
    except OSError:
        raise ToolError("unable to prepare build output directory")
    try:
        python_path = Path(sys.executable).resolve()
        repository_path = REPO_ROOT.resolve()
    except (OSError, RuntimeError):
        raise ToolError("unable to resolve required build paths")
    for item in (repository_path, output, python_path, cross_prefix):
        if _unsupported_build_path(item):
            raise ToolError(
                "unsupported build path contains whitespace or Make "
                "metacharacters")
    make = _tool_path("make")
    gcc = _tool_path(cross_prefix + "gcc")
    objcopy = _tool_path(cross_prefix + "objcopy")
    for suffix in ("as", "ld", "nm", "objdump"):
        _tool_path(cross_prefix + suffix)
    libc_name = _run([gcc, "-print-file-name=libc_nano.a"],
                     "ARM GCC").decode().strip()
    if not libc_name or libc_name == "libc_nano.a" or not Path(
        libc_name).is_file():
        raise ToolError("ARM GCC newlib nano library is unavailable", 3)
    config = output / ".config"
    products_dir = output
    try:
        config.write_text(
            profile["seed_config"], encoding="ascii", newline="\n")
    except OSError:
        raise ToolError("unable to write build configuration")
    make_out = str(products_dir) + os.sep
    env = os.environ.copy()
    for name in ("MAKEFLAGS", "MFLAGS", "MAKEOVERRIDES", "KCONFIG_CONFIG"):
        env.pop(name, None)
    env["LC_ALL"] = "C"
    common = [make, "OUT=" + make_out, "KCONFIG_CONFIG=" + str(config),
              "PYTHON=" + str(python_path), "CROSS_PREFIX=" + cross_prefix]
    _run(common + ["olddefconfig"], "make olddefconfig", env=env)
    resolved = _load_resolved_config(board, config)
    _run(common + ["-j%d" % jobs, "all"], "firmware build", env=env)
    elf = output / "klipper.elf"
    binary = output / "klipper.bin"
    dictionary = output / "klipper.dict"
    firmware = output / profile["firmware_name"]
    for path in (elf, binary, dictionary):
        try:
            valid = path.is_file() and bool(path.stat().st_size)
        except OSError:
            raise ToolError(
    "unable to inspect build product: %s" %
     path.name, 3)
        if not valid:
            raise ToolError("build did not produce nonempty %s" % path.name, 3)
    _run([objcopy, "-O", "ihex", str(elf), str(firmware)], "objcopy", env=env)
    try:
        valid_firmware = firmware.is_file() and bool(firmware.stat().st_size)
    except OSError:
        raise ToolError("unable to inspect %s" % profile["firmware_name"], 3)
    if not valid_firmware:
        raise ToolError(
            "objcopy did not produce %s" % profile["firmware_name"], 3)
    validation = validate_firmware(
        board, firmware, elf, dictionary, cross_prefix, openssl)
    validation["resolved_config"] = resolved
    return {
        "stage": "build",
        "products": {"elf": elf.name, "bin": binary.name,
                     "firmware": firmware.name, "dictionary": dictionary.name,
                     "config": config.name},
        "firmware": validation,
    }


class _ArchiveBudget:
    def __init__(self):
        self.expanded_bytes = 0
        self.members = 0

    def charge(self, size, context):
        if size < 0 or self.expanded_bytes + size > MAX_TOTAL_EXPANDED:
            raise ToolError("cumulative archive expansion limit exceeded while "
                            "reading %s" % context)
        self.expanded_bytes += size

    def member(self, label):
        self.members += 1
        if self.members > MAX_ARCHIVE_MEMBERS:
            raise ToolError("archive member limit exceeded in %s" % label)


def _tar_number(raw, field, label):
    value = raw.rstrip(b"\0 ").lstrip(b" ")
    if not value:
        return 0
    if any(byte < ord("0") or byte > ord("7") for byte in value):
        raise ToolError("%s has an invalid tar %s field" % (label, field))
    return int(value, 8)


def _tar_checksum_valid(header, label=None):
    try:
        stored = _tar_number(header[148:156], "checksum", label or "tar header")
    except ToolError:
        return False
    unsigned = sum(header[:148]) + 8 * ord(" ") + sum(header[156:])
    signed = sum(byte if byte < 128 else byte - 256 for byte in header[:148])
    signed += 8 * ord(" ")
    signed += sum(byte if byte < 128 else byte - 256 for byte in header[156:])
    return stored in (unsigned, signed)


def _tar_magic(data):
    return (len(data) >= 512 and
            data[257:263] in (b"ustar\0", b"ustar "))


def _decode_tar_field(raw, field, label):
    nul = raw.find(b"\0")
    if nul >= 0:
        if any(raw[nul + 1:]):
            raise ToolError("%s has an embedded NUL in tar %s" % (label, field))
        raw = raw[:nul]
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        raise ToolError("%s has a non-UTF-8 tar %s" % (label, field))


def _has_drive_prefixed_component(parts):
    return any(re.match(r"^[A-Za-z]:", part)
               for part in parts if part not in ("", ".", ".."))


def _canonical_member_path(name, label):
    if not name:
        raise ToolError("%s has an empty archive member path" % label)
    if any(ord(char) < 32 or ord(char) == 127 for char in name):
        raise ToolError("%s member path contains a control character: %r" %
                        (label, name))
    if "\\" in name:
        raise ToolError("%s member path contains a backslash: %s" %
                        (label, name))
    if name.startswith("/"):
        raise ToolError("%s member path is absolute: %s" % (label, name))
    raw_parts = name.split("/")
    if _has_drive_prefixed_component(raw_parts):
        raise ToolError("%s member path has a Windows drive prefix: %s" %
                        (label, name))
    parts = []
    for part in raw_parts:
        if part in ("", "."):
            continue
        if part == "..":
            raise ToolError("%s member path contains traversal: %s" %
                            (label, name))
        parts.append(part)
    return "/".join(parts) if parts else "."


def _link_target_parts(member_path, target, is_hardlink, label):
    kind = "hardlink" if is_hardlink else "symlink"
    if not target:
        raise ToolError("%s has an empty %s target for %s" %
                        (label, kind, member_path))
    if any(ord(char) < 32 or ord(char) == 127 for char in target):
        raise ToolError("%s %s target contains a control character: %s" %
                        (label, kind, member_path))
    if "\\" in target:
        raise ToolError("%s %s target contains a backslash: %s" %
                        (label, kind, member_path))
    if target.startswith("/"):
        raise ToolError("%s %s target is absolute: %s" %
                        (label, kind, member_path))
    parts = target.split("/")
    if _has_drive_prefixed_component(parts):
        raise ToolError("%s %s target has a drive prefix: %s" %
                        (label, kind, member_path))
    return parts


def _resolve_archive_path(member_path, target_parts, is_hardlink,
                          symlinks, label):
    kind = "hardlink" if is_hardlink else "symlink"
    resolved = [] if is_hardlink else member_path.split("/")[:-1]
    active = set() if is_hardlink else {member_path}
    frames = [[target_parts, 0, None]]
    traversals = 0
    while frames:
        components, index, owner = frames[-1]
        if index == len(components):
            frames.pop()
            if owner is not None:
                active.remove(owner)
            continue
        part = components[index]
        frames[-1][1] += 1
        if part in ("", "."):
            continue
        if part == "..":
            if not resolved:
                raise ToolError("%s %s target escapes archive root: %s" %
                                (label, kind, member_path))
            resolved.pop()
            continue
        candidate = "/".join(resolved + [part])
        linked_parts = symlinks.get(candidate)
        if linked_parts is None:
            resolved.append(part)
            continue
        if candidate in active:
            raise ToolError("%s has a symlink cycle involving %s" %
                            (label, member_path))
        traversals += 1
        if traversals > MAX_ARCHIVE_MEMBERS:
            raise ToolError("%s symlink traversal limit exceeded while "
                            "resolving %s" % (label, member_path))
        active.add(candidate)
        frames.append([linked_parts, 0, candidate])
    return "/".join(resolved) if resolved else "."


def _validate_tar_relationships(members, label):
    by_path = {}
    for member in members:
        path = member["canonical_path"]
        if path in by_path:
            raise ToolError("%s has a conflicting duplicate path: %s" %
                            (label, path))
        by_path[path] = member
        if path == "." and member["type"] != "directory":
            raise ToolError("%s archive root member is not a directory" %
                            label)

    for member in members:
        path = member["canonical_path"]
        if path == ".":
            continue
        components = path.split("/")
        for count in range(1, len(components)):
            ancestor = "/".join(components[:count])
            parent = by_path.get(ancestor)
            if parent is None:
                continue
            if parent["type"] == "symlink":
                raise ToolError("%s member %s is under symlink ancestor %s" %
                                (label, path, ancestor))
            if parent["type"] != "directory":
                raise ToolError(
                    "%s member %s has non-directory file parent %s" %
                    (label, path, ancestor))

    symlinks = {}
    hardlink_parts = {}
    for member in members:
        kind = member["type"]
        if kind not in ("symlink", "hardlink"):
            continue
        is_hardlink = kind == "hardlink"
        parts = _link_target_parts(
            member["canonical_path"], member["link_target"],
            is_hardlink, label)
        if is_hardlink:
            hardlink_parts[member["canonical_path"]] = parts
        else:
            symlinks[member["canonical_path"]] = parts

    for origin, parts in symlinks.items():
        by_path[origin]["_resolved_target"] = _resolve_archive_path(
            origin, parts, False, symlinks, label)

    hardlinks = {}
    for origin, parts in hardlink_parts.items():
        resolved = _resolve_archive_path(
            origin, parts, True, symlinks, label)
        by_path[origin]["_resolved_target"] = resolved
        hardlinks[origin] = resolved

    for origin in hardlinks:
        seen = set()
        current = origin
        while current in hardlinks:
            if current in seen:
                raise ToolError("%s has a hardlink cycle involving %s" %
                                (label, origin))
            seen.add(current)
            current = hardlinks[current]
        target = by_path.get(current)
        if target is None:
            raise ToolError("%s has a dangling hardlink %s -> %s" %
                            (label, origin, current))
        if target["type"] != "file":
            raise ToolError(
                "%s hardlink %s does not resolve to a regular file" %
                (label, origin))
        by_path[origin]["_hardlink_source"] = current


def _parse_tar(data, label, budget, depth, decrypt, openssl):
    members = []
    offset = 0
    header_number = 0
    while True:
        if offset + 512 > len(data):
            raise ToolError(
                "%s tar is truncated before its end marker" % label)
        header = data[offset:offset + 512]
        if header == b"\0" * 512:
            if offset + 1024 > len(data):
                raise ToolError("%s tar is truncated in its end marker" % label)
            if data[offset + 512:offset + 1024] != b"\0" * 512:
                raise ToolError("%s tar has only one zero end block" % label)
            if any(data[offset + 1024:]):
                raise ToolError(
                    "%s tar has trailing nonpadding garbage" % label)
            break
        header_number += 1
        header_label = "%s tar header %d" % (label, header_number)
        if header[257:263] not in (b"ustar\0", b"ustar "):
            raise ToolError("%s has unsupported or missing tar magic" %
                            header_label)
        if not _tar_checksum_valid(header, header_label):
            raise ToolError("%s checksum mismatch" % header_label)
        name = _decode_tar_field(header[:100], "name", header_label)
        prefix = _decode_tar_field(header[345:500], "prefix", header_label)
        if prefix:
            name = prefix + "/" + name
        canonical = _canonical_member_path(name, label)
        size = _tar_number(header[124:136], "size", header_label)
        mode = _tar_number(header[100:108], "mode", header_label)
        uid = _tar_number(header[108:116], "uid", header_label)
        gid = _tar_number(header[116:124], "gid", header_label)
        mtime = _tar_number(header[136:148], "mtime", header_label)
        linkname = _decode_tar_field(header[157:257], "link name", header_label)
        uname = _decode_tar_field(header[265:297], "user name", header_label)
        gname = _decode_tar_field(header[297:329], "group name", header_label)
        kind_byte = header[156:157]
        if kind_byte in (b"", b"\0", b"0"):
            kind = "file"
        elif kind_byte == b"5":
            kind = "directory"
        elif kind_byte == b"2":
            kind = "symlink"
        elif kind_byte == b"1":
            kind = "hardlink"
        elif kind_byte == b"S":
            raise ToolError("%s contains sparse member %s" % (label, name))
        elif kind_byte in (b"3", b"4"):
            raise ToolError("%s contains device member %s" % (label, name))
        elif kind_byte == b"6":
            raise ToolError("%s contains FIFO member %s" % (label, name))
        else:
            raise ToolError("%s contains unrecognized special member %s "
                            "(type %r)" % (label, name, kind_byte))
        if kind != "file" and size:
            raise ToolError("%s non-file member %s has a nonzero size" %
                            (label, name))
        content_start = offset + 512
        padded_size = (size + 511) & ~511
        content_end = content_start + size
        next_offset = content_start + padded_size
        if (content_end < content_start or next_offset < content_start or
                next_offset > len(data)):
            raise ToolError("%s tar member %s is truncated" % (label, name))
        budget.member(label)
        budget.charge(size, "member %s" % name)
        content = data[content_start:content_end]
        link_target = linkname if kind in ("symlink", "hardlink") else None
        digest = None
        if kind == "file":
            digest = hashlib.sha256(content).hexdigest()
        elif link_target is not None:
            digest = hashlib.sha256(link_target.encode("utf-8")).hexdigest()
        member = {
            "path": name,
            "canonical_path": canonical,
            "type": kind,
            "size": size,
            "mode": mode,
            "uid": uid,
            "gid": gid,
            "uname": uname,
            "gname": gname,
            "mtime": mtime,
            "link_target": link_target,
            "sha256": digest,
            "leading_dot_slash": name.startswith("./"),
            "unsafe": False,
            "_data": content if kind == "file" else None,
        }
        members.append(member)
        offset = next_offset

    _validate_tar_relationships(members, label)
    for member in members:
        if member["type"] != "file":
            continue
        content = member["_data"]
        nested_format = _detect_archive_format(content)
        advertised = member["path"].lower().endswith(_ARCHIVE_SUFFIXES)
        if nested_format is not None or advertised:
            if nested_format is None:
                raise ToolError("archive member %s advertises an archive but "
                                "has an unknown or malformed format" %
                                member["path"])
            try:
                member["nested"] = _inspect_layer(
                    content, member["path"], budget, depth + 1,
                    decrypt, openssl)
            except ToolError as exc:
                raise ToolError("archive member %s: %s" %
                                (member["path"], exc), exc.exit_code)
    return {
        "format": "tar",
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "members": members,
    }


def _detect_archive_format(data):
    if data.startswith(b"Salted__"):
        return "encrypted"
    if data.startswith(b"\x1f\x8b"):
        return "gzip"
    if data.startswith(b"\xfd7zXZ\x00"):
        return "xz"
    if data.startswith(b"BZh"):
        return "bzip2"
    if _tar_magic(data) or (len(data) >= 1024 and not any(data)):
        return "tar"
    return None


def _append_decompressed(output, chunk, label, budget):
    if len(output) + len(chunk) > MAX_LAYER_BYTES:
        raise ToolError("%s decompressed layer limit exceeded" % label)
    budget.charge(len(chunk), "%s decompressed layer" % label)
    output.extend(chunk)


def _decompress_layer(data, kind, label, budget):
    output = bytearray()
    magic = {"gzip": b"\x1f\x8b", "xz": b"\xfd7zXZ\x00",
             "bzip2": b"BZh"}[kind]
    remaining = data
    stream_number = 0
    try:
        while remaining:
            if kind == "xz" and stream_number:
                padding = 0
                while (padding < len(remaining) and
                       remaining[padding] == 0):
                    padding += 1
                if padding % 4:
                    raise ToolError("%s xz stream padding length is not a "
                                    "multiple of four" % label)
                remaining = remaining[padding:]
                if not remaining:
                    break
            if not remaining.startswith(magic):
                raise ToolError("%s %s stream has trailing nonpadding "
                                "garbage" % (label, kind))
            position = 0
            if kind == "gzip":
                decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
                pending = b""
                while not decompressor.eof:
                    if not pending:
                        if position >= len(remaining):
                            raise ToolError("%s %s stream is truncated" %
                                            (label, kind))
                        pending = remaining[position:position + 65536]
                        position += len(pending)
                    chunk = decompressor.decompress(pending, 65536)
                    pending = decompressor.unconsumed_tail
                    _append_decompressed(output, chunk, label, budget)
                remaining = decompressor.unused_data + remaining[position:]
            else:
                if kind == "xz":
                    decompressor = lzma.LZMADecompressor(
                        format=lzma.FORMAT_XZ, memlimit=MAX_LAYER_BYTES)
                else:
                    decompressor = bz2.BZ2Decompressor()
                while not decompressor.eof:
                    if decompressor.needs_input:
                        if position >= len(remaining):
                            raise ToolError("%s %s stream is truncated" %
                                            (label, kind))
                        compressed = remaining[position:position + 65536]
                        position += len(compressed)
                    else:
                        compressed = b""
                    chunk = decompressor.decompress(compressed,
                                                    max_length=65536)
                    _append_decompressed(output, chunk, label, budget)
                remaining = decompressor.unused_data + remaining[position:]
            stream_number += 1
    except ToolError:
        raise
    except (EOFError, OSError, ValueError, zlib.error,
            lzma.LZMAError) as exc:
        raise ToolError("%s %s stream is truncated or malformed: %s" %
                        (label, kind, sanitize_diagnostic(exc)))
    return bytes(output)

def _openssl_algorithm_unavailable(stderr):
    lowered = stderr.lower()
    indicators = ("unsupported", "unknown cipher", "error setting cipher",
                  "initialization error", "digital envelope routines")
    return any(indicator in lowered for indicator in indicators)


def _openssl_once(data, decrypt, openssl, secret, providers):
    command = [openssl, "enc", "-des-ede3-cbc"]
    if decrypt:
        command.append("-d")
    command.extend(["-salt", "-md", "md5", "-pass",
                    "env:C5_UPDATE_PASSPHRASE"])
    command.extend(providers)
    environment = os.environ.copy()
    environment["C5_UPDATE_PASSPHRASE"] = secret
    try:
        result = subprocess.run(command, input=data, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, env=environment,
                                check=False)
    except (FileNotFoundError, PermissionError):
        raise ToolError("OpenSSL executable is unavailable", 3)
    return result


def _openssl_provider_args(openssl, secret):
    probe = b"Creator 5 update crypto preflight"
    result = _openssl_once(probe, False, openssl, secret, ())
    providers = ()
    if result.returncode:
        diagnostic = result.stderr.decode("utf-8", "replace")
        if not _openssl_algorithm_unavailable(diagnostic):
            raise ToolError("OpenSSL crypto preflight failed: %s" %
                            sanitize_diagnostic(diagnostic, secret), 3)
        providers = ("-provider", "default", "-provider", "legacy")
        result = _openssl_once(probe, False, openssl, secret, providers)
        if result.returncode:
            diagnostic = result.stderr.decode("utf-8", "replace")
            raise ToolError("OpenSSL DES-EDE3-CBC is unavailable: %s" %
                            sanitize_diagnostic(diagnostic, secret), 3)
    encrypted = result.stdout
    if (not encrypted.startswith(b"Salted__") or len(encrypted) < 24 or
            (len(encrypted) - 16) % 8):
        raise ToolError("OpenSSL crypto preflight returned invalid ciphertext",
                        3)
    checked = _openssl_once(encrypted, True, openssl, secret, providers)
    if checked.returncode or checked.stdout != probe:
        diagnostic = checked.stderr.decode("utf-8", "replace")
        raise ToolError("OpenSSL crypto preflight roundtrip failed: %s" %
                        sanitize_diagnostic(diagnostic, secret), 3)
    return providers


def openssl_crypt(data, decrypt, openssl="openssl", secret=None):
    if secret is None:
        secret = os.environ.get("C5_UPDATE_PASSPHRASE")
    if not secret:
        raise ToolError("C5_UPDATE_PASSPHRASE must be nonempty for crypto")
    if not isinstance(data, bytes):
        data = bytes(data)
    if decrypt and (not data.startswith(b"Salted__") or len(data) < 24 or
                    (len(data) - 16) % 8):
        raise ToolError("encrypted input has an invalid Salted__ header, salt, "
                        "or ciphertext length")
    providers = _openssl_provider_args(openssl, secret)
    result = _openssl_once(data, decrypt, openssl, secret, providers)
    if result.returncode:
        operation = "decryption" if decrypt else "encryption"
        diagnostic = result.stderr.decode("utf-8", "replace")
        raise ToolError("OpenSSL %s failed: %s" %
                        (operation, sanitize_diagnostic(diagnostic, secret)), 3)
    output = result.stdout
    if not decrypt and (not output.startswith(b"Salted__") or
                        len(output) < 24 or (len(output) - 16) % 8):
        raise ToolError("OpenSSL encryption returned invalid ciphertext", 3)
    return output


def _inspect_layer(data, label, budget, depth, decrypt, openssl):
    if depth > MAX_ARCHIVE_DEPTH:
        raise ToolError("archive nesting depth limit exceeded at %s" % label)
    if len(data) > MAX_LAYER_BYTES:
        raise ToolError("%s input layer limit exceeded" % label)
    kind = _detect_archive_format(data)
    if kind is None:
        raise ToolError("unknown top-level archive format for %s" % label)
    if kind == "tar":
        return _parse_tar(data, label, budget, depth, decrypt, openssl)
    if kind == "encrypted":
        if not decrypt:
            raise ToolError("%s is encrypted; use --decrypt to inspect it" %
                            label)
        if len(data) >= 24 and not (len(data) - 16) % 8:
            maximum_plaintext = len(data) - 17
            remaining_budget = MAX_TOTAL_EXPANDED - budget.expanded_bytes
            if maximum_plaintext > remaining_budget:
                raise ToolError(
                    "cumulative archive expansion limit exceeded before "
                    "decrypting %s" % label)
        plain = openssl_crypt(data, True, openssl)
        if len(plain) > MAX_LAYER_BYTES:
            raise ToolError("%s decrypted layer limit exceeded" % label)
        budget.charge(len(plain), "%s decrypted layer" % label)
        payload = _inspect_layer(plain, "%s decrypted payload" % label,
                                 budget, depth + 1, decrypt, openssl)
        return {
            "format": "encrypted",
            "cipher": "des-ede3-cbc",
            "digest": "md5",
            "size": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
            "payload": payload,
            "_payload": plain,
        }
    plain = _decompress_layer(data, kind, label, budget)
    payload = _inspect_layer(plain, "%s %s payload" % (label, kind),
                             budget, depth + 1, decrypt, openssl)
    return {
        "format": kind,
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "expanded_size": len(plain),
        "payload": payload,
        "_payload": plain,
    }


def _sanitized_archive_tree(value):
    if isinstance(value, dict):
        return {key: _sanitized_archive_tree(item)
                for key, item in value.items() if not key.startswith("_")}
    if isinstance(value, list):
        return [_sanitized_archive_tree(item) for item in value]
    return value


def _tar_payload_model(model):
    current = model
    while current["format"] != "tar":
        current = current["payload"]
    return current


def _temporary_extract(model):
    tar_model = _tar_payload_model(model)
    members = tar_model["members"]
    root_members = sorted(
        (member for member in members if member["type"] == "directory"),
        key=lambda member: member["canonical_path"].count("/"))
    files = [member for member in members if member["type"] == "file"]
    links = [member for member in members
             if member["type"] in ("symlink", "hardlink")]
    with tempfile.TemporaryDirectory(prefix="c5-update-inspect-") as temp:
        root = Path(temp).resolve()

        def destination(member):
            canonical = member["canonical_path"]
            path = root if canonical == "." else root.joinpath(
                *canonical.split("/"))
            resolved = path.resolve(strict=False)
            if resolved != root and not _inside(resolved, root):
                raise ToolError(
                    "temporary extraction containment check failed")
            return path

        try:
            for member in root_members:
                destination(member).mkdir(parents=True, exist_ok=True)
            for member in files:
                path = destination(member)
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("xb") as output:
                    output.write(member["_data"])
                path.chmod(member["mode"] & 0o777)
            for member in links:
                path = destination(member)
                path.parent.mkdir(parents=True, exist_ok=True)
                if member["type"] == "hardlink":
                    source = root.joinpath(
                        *member["_hardlink_source"].split("/"))
                    os.link(source, path)
                else:
                    canonical_target = member["_resolved_target"]
                    target = (root if canonical_target == "." else
                              root.joinpath(*canonical_target.split("/")))
                    resolved_target = target.resolve(strict=False)
                    if (resolved_target != root and
                            not _inside(resolved_target, root)):
                        raise ToolError(
                            "temporary extraction link containment check "
                            "failed")
                    os.symlink(member["link_target"], path)
            for member in reversed(root_members):
                path = destination(member)
                if path != root:
                    path.chmod(member["mode"] & 0o777)
        except ToolError:
            raise
        except OSError as exc:
            raise ToolError("temporary extraction validation failed: %s" %
                            sanitize_diagnostic(exc))


def inspect_archive(data, decrypt=False, extract_temp=False,
                    openssl="openssl"):
    if not isinstance(data, bytes):
        try:
            input_size = len(data)
        except (TypeError, ValueError):
            raise ToolError("archive input must be bytes")
        if input_size > MAX_LAYER_BYTES:
            raise ToolError("archive input layer limit exceeded")
        try:
            data = bytes(data)
        except (TypeError, ValueError):
            raise ToolError("archive input must be bytes")
    if len(data) > MAX_LAYER_BYTES:
        raise ToolError("archive input layer limit exceeded")
    budget = _ArchiveBudget()
    budget.charge(len(data), "archive input")
    model = _inspect_layer(data, "input archive", budget, 0,
                           decrypt, openssl)
    if extract_temp:
        _temporary_extract(model)
    report = _sanitized_archive_tree(model)
    if extract_temp:
        report["extraction_validated"] = True
    report["resource_usage"] = {
        "expanded_bytes": budget.expanded_bytes,
        "members": budget.members,
        "maximum_depth": MAX_ARCHIVE_DEPTH,
    }
    return report


def _read_archive_input(path):
    chunks = []
    total = 0
    with Path(path).open("rb") as source:
        while True:
            chunk = source.read(65536)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_LAYER_BYTES:
                raise ToolError("archive input layer limit exceeded")
            chunks.append(chunk)
    return b"".join(chunks)


def _inspect_archive_model(data, decrypt=False, openssl="openssl"):
    if not isinstance(data, bytes):
        try:
            data = bytes(data)
        except (TypeError, ValueError):
            raise ToolError("archive input must be bytes")
    if len(data) > MAX_LAYER_BYTES:
        raise ToolError("archive input layer limit exceeded")
    budget = _ArchiveBudget()
    budget.charge(len(data), "archive input")
    return _inspect_layer(data, "input archive", budget, 0,
                          decrypt, openssl)


def _regular_member_map(model, label):
    if model.get("format") != "tar":
        raise ToolError("%s is not a plain tar archive" % label)
    members = model.get("members", [])
    if any(member["type"] != "file" for member in members):
        raise ToolError("%s canonical profile permits regular files only" %
                        label)
    return {member["path"]: member for member in members}


def _parse_md5_list(data, expected_paths):
    if not data.endswith(b"\n") or b"\r" in data:
        raise ToolError("control md5sum.list must use LF-terminated lines")
    entries = []
    seen = set()
    for number, line in enumerate(data.splitlines(), 1):
        match = re.fullmatch(rb"([0-9a-f]{32})  (\./[^\x00-\x20]+)", line)
        if not match:
            raise ToolError("control md5sum.list line %d is malformed" %
                            number)
        path = match.group(2).decode("ascii")
        if path in seen:
            raise ToolError("control md5sum.list has duplicate path %s" % path)
        seen.add(path)
        entries.append((path, match.group(1).decode("ascii")))
    if seen != set(expected_paths):
        raise ToolError("control md5sum.list coverage does not match files")
    return entries


def _program_path(program, logical_name):
    try:
        found = shutil.which(str(program))
    except (OSError, RuntimeError):
        found = None
    if not found:
        raise ToolError("required external tool is unavailable: %s" %
                        logical_name, 3)
    return found


def _verify_md5(control_files, checksum_data, md5sum="md5sum"):
    entries = _parse_md5_list(checksum_data, control_files)
    for path, expected in entries:
        actual = hashlib.md5(control_files[path]).hexdigest()
        if actual != expected:
            raise ToolError("control checksum mismatch for %s" % path)
    program = _program_path(md5sum, "md5sum")
    try:
        with tempfile.TemporaryDirectory(prefix="c5-update-md5-") as temp:
            root = Path(temp)
            for path, data in control_files.items():
                destination = root / path[2:]
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(data)
            (root / "md5sum.list").write_bytes(checksum_data)
            environment = os.environ.copy()
            environment["LC_ALL"] = "C"
            result = subprocess.run(
                [program, "-c", "md5sum.list"], cwd=root,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                env=environment, check=False)
    except OSError:
        raise ToolError("unable to run md5sum verification", 3)
    if result.returncode:
        raise ToolError("real md5sum verification failed")
    return entries


def _locate_template_members(model, md5sum="md5sum"):
    outer = _regular_member_map(model, "outer template")
    members = model["members"]
    if len(members) != 8 or any(not member["leading_dot_slash"]
                                for member in members):
        raise ToolError("unsupported canonical template outer profile")
    components = {}
    for member in members:
        match = COMPONENT_NAME_RE.fullmatch(member["path"])
        if match:
            kind = match.group(1)
            if kind in components:
                raise ToolError("unsupported template has duplicate %s "
                                "component" % kind)
            components[kind] = member
    if set(components) != {"control", "kernel", "library", "software"}:
        raise ToolError("unsupported canonical template component profile")
    fixed_paths = {"./end.img", "./play", "./runFirmwareExe.sh",
                   "./start.img"}
    if set(outer) != fixed_paths | {
            member["path"] for member in components.values()}:
        raise ToolError("unsupported canonical template outer allowlist")
    for kind, member in components.items():
        nested = member.get("nested")
        if not nested or nested.get("format") != "tar":
            raise ToolError("canonical %s component is not a plain tar" % kind)
    control_outer = components["control"]
    control_model = control_outer["nested"]
    control = _regular_member_map(control_model, "control template")
    if (tuple(member["path"] for member in control_model["members"]) not in
            CONTROL_MEMBER_PROFILES or
            any(not member["leading_dot_slash"]
                for member in control_model["members"])):
        raise ToolError("unsupported canonical control member profile")
    input_files = {path: member["_data"] for path, member in control.items()
                   if path != "./md5sum.list"}
    checksum_entries = _verify_md5(
        input_files, control["./md5sum.list"]["_data"], md5sum)
    return {
        "model": model,
        "outer": outer,
        "outer_members": members,
        "components": components,
        "control_outer": control_outer,
        "control_model": control_model,
        "control": control,
        "control_members": control_model["members"],
        "checksum_entries": checksum_entries,
    }
def _canonical_template_profile(model, md5sum="md5sum"):
    expected = next(
        (item for item in CANONICAL_TEMPLATE_PROFILES
         if item["plaintext"] == model.get("sha256")), None)
    if expected is None:
        raise ToolError("unsupported canonical template plaintext hash")
    profile = _locate_template_members(model, md5sum)
    if profile["control_outer"]["sha256"] != expected["control"]:
        raise ToolError("unsupported canonical control archive hash")
    gates = (
        (profile["outer"]["./runFirmwareExe.sh"],
         expected["installer"], "outer installer"),
        (profile["control"]["./run.sh"],
         expected["control_script"], "control script"),
        (profile["control"]["./IAPCommand"], CANONICAL_IAP_SHA256,
         "IAPCommand"),
    )
    for member, expected_hash, label in gates:
        if member["sha256"] != expected_hash:
            raise ToolError("unsupported canonical %s hash" % label)
    return profile


def _suppress_update_other(data):
    lines = data.splitlines(keepends=True)
    declaration = re.compile(
        rb"^[ \t]*(?:(?:function[ \t]+update_other(?:[ \t]*\(\))?)|"
        rb"(?:update_other[ \t]*\(\)))[ \t]*(?:\{[ \t]*)?"
        rb"(?:\r?\n)?$")
    definitions = [index for index, line in enumerate(lines)
                   if declaration.fullmatch(line)]
    if len(definitions) != 1:
        raise ToolError("outer installer must contain exactly one "
                        "update_other function definition")
    start = definitions[0]
    if b"{" in lines[start]:
        body_start = start
    elif (start + 1 < len(lines) and
          re.fullmatch(rb"[ \t]*\{[ \t]*(?:\r?\n)?", lines[start + 1])):
        body_start = start + 1
    else:
        raise ToolError("update_other function has no opening brace")
    depth = 1
    end = None
    for index in range(body_start + 1, len(lines)):
        if re.fullmatch(rb"[ \t]*\{[ \t]*(?:\r?\n)?", lines[index]):
            depth += 1
        elif re.fullmatch(rb"[ \t]*\}[ \t]*(?:\r?\n)?", lines[index]):
            depth -= 1
            if not depth:
                end = index
                break
    if end is None:
        raise ToolError("update_other function has no matching closing brace")
    invocation = re.compile(rb"^[ \t]*update_other[ \t]*(?:\r?\n)?$")
    calls = [index for index, line in enumerate(lines)
             if not start <= index <= end and invocation.fullmatch(line)]
    if len(calls) != 1:
        raise ToolError("outer installer must contain exactly one standalone "
                        "update_other invocation")
    removed = set(range(start, end + 1)) | {calls[0]}
    return b"".join(line for index, line in enumerate(lines)
                    if index not in removed)


def _remove_optional_shell_block(data, start, end, required, label):
    if start not in data:
        return data
    if data.count(start) != 1:
        raise ToolError("%s start marker is ambiguous" % label)
    begin = data.index(start)
    finish = data.find(end, begin)
    if finish < 0:
        raise ToolError("%s end marker is missing" % label)
    finish += len(end)
    block = data[begin:finish]
    if any(block.count(token) != 1 for token in required):
        raise ToolError("%s has an unsupported shape" % label)
    return data[:begin] + data[finish:]


def _suppress_space_reclaim(data, outer):
    if outer:
        start = b"rm /usr/prog/PROGRAM/control/*.tar.xz*\n"
        required = (
            b"rm /usr/prog/PROGRAM/library/*.tar.xz*",
            b"rm /usr/prog/PROGRAM/kernel/*.tar.xz*",
            b"rm /usr/prog/PROGRAM/software/*.tar.xz*",
            b"rm /usr/prog/qt-4.8.6 -rf",
            b"rm /usr/prog/opencv-4.10 -rf",
            b"rm /usr/prog/wifi/8821cu.ko*",
        )
    else:
        start = b"# free 28M\n"
        required = (
            b"rm /usr/prog/qt-4.8.6 -rf",
            b"rm /usr/prog/opencv-4.10 -rf",
            b"rm /usr/prog/wifi/8821cu.ko*",
        )
    return _remove_optional_shell_block(
        data, start, b"sync\n", required, "space-reclaim block")


FAILURE_IMAGE_NAMES = {
    "eBoard": "eBoard_fail.img",
    "heaterBoard": "heaterBoard_fail.img",
    "levelBoard": "levelBoard_fail.img",
    "mainBoardGD": "mcu_fail.img",
}


def _filter_result_checks(data, selected_boards):
    marker = b"# check update result\n"
    end_marker = b"# remove small version\n"
    if marker not in data:
        return data
    if data.count(marker) != 1 or data.count(end_marker) != 1:
        raise ToolError("control result-check section is ambiguous")
    begin = data.index(marker) + len(marker)
    finish = data.index(end_marker, begin)
    lines = data[begin:finish].splitlines(keepends=True)
    blocks = []
    index = 0
    while index < len(lines):
        while index < len(lines) and not lines[index].strip():
            index += 1
        if index == len(lines):
            break
        block_start = index
        depth = 0
        while index < len(lines):
            stripped = lines[index].lstrip()
            if stripped.startswith(b"if "):
                depth += 1
            elif stripped.rstrip() == b"fi":
                depth -= 1
            index += 1
            if depth == 0:
                break
        if depth or block_start == index:
            raise ToolError("control result-check block is malformed")
        blocks.append(b"".join(lines[block_start:index]))
    by_board = {}
    for block in blocks:
        matches = [board for board, name in FAILURE_IMAGE_NAMES.items()
                   if name.encode("ascii") in block]
        if len(matches) != 1 or matches[0] in by_board:
            raise ToolError("control result-check profile is unsupported")
        by_board[matches[0]] = block
    if set(by_board) != set(FAILURE_IMAGE_NAMES):
        raise ToolError("control result-check profile is incomplete")
    kept = b"".join(by_board[board] + b"\n"
                       for board in _canonical_boards(selected_boards))
    return data[:begin] + kept + data[finish:]


def _remove_update_marker(data):
    # Stock resolves WORK_DIR as "." and the version pruning below changes
    # the working directory, so the marker must be removed before that cd
    # for $WORK_DIR/Update to name the installed marker file.
    anchor = b"cd /usr/prog/PROGRAM/control/\n"
    if data.count(anchor) != 1:
        raise ToolError("control script has no unique version-prune change "
                        "of directory for update marker removal")
    index = data.index(anchor)
    removal = b"rm -f $WORK_DIR/Update\nsync\n\n"
    return data[:index] + removal + data[index:]


def _check_shell_syntax(data, shell="sh", label="outer installer"):
    program = _program_path(shell, "sh")
    try:
        result = subprocess.run([program, "-n"], input=data,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, check=False)
    except OSError:
        raise ToolError("unable to run sh syntax validation", 3)
    if result.returncode:
        diagnostic = sanitize_diagnostic(
            result.stderr.decode("utf-8", "replace").strip())
        raise ToolError("generated %s failed sh -n%s" %
                        (label, (": " + diagnostic) if diagnostic else ""))


def _member_metadata(member):
    return {key: member[key] for key in
            ("path", "type", "mode", "uid", "gid", "uname", "gname",
             "mtime", "link_target")}


def _write_gnu_tar(entries):
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w",
                      format=tarfile.GNU_FORMAT) as tar:
        for member, data in entries:
            info = tarfile.TarInfo(member["path"])
            info.type = tarfile.REGTYPE
            info.mode = member["mode"]
            info.uid = member["uid"]
            info.gid = member["gid"]
            info.uname = member["uname"]
            info.gname = member["gname"]
            info.mtime = member["mtime"]
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return output.getvalue()


def _build_reduced_plaintext(profile, firmware_data, shell="sh",
                              md5sum="md5sum"):
    selected_boards = _canonical_boards(firmware_data)
    if set(firmware_data) != set(selected_boards):
        raise ToolError("invalid firmware data selection")
    for board in selected_boards:
        if not isinstance(firmware_data[board], bytes):
            raise ToolError("firmware data must be bytes for %s" % board)
    control = profile["control"]
    firmware_paths = {"./" + BOARD_PROFILES[board]["firmware_name"]
                      for board in selected_boards}
    updater_paths = {"./" + BOARD_PROFILES[board]["updater_name"]
                     for board in selected_boards}
    failure_paths = {"./" + FAILURE_IMAGE_NAMES[board]
                     for board in selected_boards
                     if "./" + FAILURE_IMAGE_NAMES[board] in control}
    retained = (updater_paths | failure_paths |
                {"./mcu.img", "./md5sum.list", "./run.sh", "./Update"} |
                firmware_paths)
    checksum_paths = [path for path, unused in profile["checksum_entries"]
                      if path in retained and path != "./md5sum.list"]
    if set(checksum_paths) != retained - {"./md5sum.list"}:
        raise ToolError("retained control checksum order is incomplete")
    control_script = _suppress_space_reclaim(
        control["./run.sh"]["_data"], False)
    control_script = _filter_result_checks(control_script, selected_boards)
    control_script = _remove_update_marker(control_script)
    _check_shell_syntax(control_script, shell, "control script")
    control_data = {
        "./mcu.img": control["./mcu.img"]["_data"],
        "./run.sh": control_script,
        "./Update": control["./Update"]["_data"],
    }
    for path in updater_paths | failure_paths:
        control_data[path] = control[path]["_data"]
    for board in selected_boards:
        control_data["./" + BOARD_PROFILES[board]["firmware_name"]] = (
            firmware_data[board])
    checksum_data = b"".join(
        hashlib.md5(control_data[path]).hexdigest().encode("ascii") +
        b"  " + path.encode("ascii") + b"\n" for path in checksum_paths)
    _verify_md5(control_data, checksum_data, md5sum)
    control_data["./md5sum.list"] = checksum_data
    control_entries = [(member, control_data[member["path"]])
                       for member in profile["control_members"]
                       if member["path"] in retained]
    control_tar = _write_gnu_tar(control_entries)
    if control_tar[257:265] != b"ustar  \0":
        raise ToolError("generated control archive is not GNU tar")
    transformed_installer = _suppress_update_other(
        profile["outer"]["./runFirmwareExe.sh"]["_data"])
    transformed_installer = _suppress_space_reclaim(
        transformed_installer, True)
    _check_shell_syntax(transformed_installer, shell)
    control_path = profile["control_outer"]["path"]
    outer_data = {
        control_path: control_tar,
        "./end.img": profile["outer"]["./end.img"]["_data"],
        "./play": profile["outer"]["./play"]["_data"],
        "./runFirmwareExe.sh": transformed_installer,
        "./start.img": profile["outer"]["./start.img"]["_data"],
    }
    outer_entries = [(member, outer_data[member["path"]])
                     for member in profile["outer_members"]
                     if member["path"] in outer_data]
    plaintext = _write_gnu_tar(outer_entries)
    if plaintext[257:265] != b"ustar  \0":
        raise ToolError("generated outer archive is not GNU tar")
    evidence = {
        "plaintext_sha256": hashlib.sha256(plaintext).hexdigest(),
        "outer_order": [member["path"] for member, unused in outer_entries],
        "control_order": [member["path"] for member, unused in control_entries],
        "outer_metadata": {member["path"]: _member_metadata(member)
                           for member, unused in outer_entries},
        "control_metadata": {member["path"]: _member_metadata(member)
                             for member, unused in control_entries},
        "outer_data": outer_data, "control_data": control_data,
        "control_path": control_path,
    }
    return plaintext, evidence


def _assert_member(member, expected_metadata, expected_data, label):
    if _member_metadata(member) != expected_metadata:
        raise ToolError("generated %s metadata differs from template" % label)
    if member["_data"] != expected_data:
        raise ToolError(
            "generated %s bytes differ from expected output" % label)


def _manifest_member(label, member):
    return {
        "label": label,
        "type": member["type"],
        "mode": member["mode"],
        "size": member["size"],
        "sha256": member["sha256"],
    }


def _assert_reduced_profile(model, evidence, selected_boards, md5sum="md5sum"):
    selected_boards = _canonical_boards(selected_boards)
    if (model.get("format") != "tar" or
            model.get("sha256") != evidence["plaintext_sha256"]):
        raise ToolError("generated plaintext archive hash mismatch")
    members = model["members"]
    if [member["path"] for member in members] != evidence["outer_order"]:
        raise ToolError("generated outer archive order or allowlist mismatch")
    if any(member["type"] != "file" or not member["leading_dot_slash"]
           for member in members):
        raise ToolError("generated outer archive has invalid member type or "
                        "root convention")
    outer = {member["path"]: member for member in members}
    expected_outer = {evidence["control_path"], "./end.img", "./play",
                      "./runFirmwareExe.sh", "./start.img"}
    if (set(outer) != expected_outer or
            set(evidence["outer_data"]) != expected_outer):
        raise ToolError("generated outer archive allowlist mismatch")
    for path in evidence["outer_order"]:
        _assert_member(outer[path], evidence["outer_metadata"][path],
                       evidence["outer_data"][path], path)
    control_member = outer[evidence["control_path"]]
    control_model = control_member.get("nested")
    if not control_model or control_model.get("format") != "tar":
        raise ToolError("generated control package is not a plain GNU tar")
    control_members = control_model["members"]
    if ([member["path"] for member in control_members] !=
            evidence["control_order"]):
        raise ToolError(
            "generated control archive order or allowlist mismatch")
    if any(member["type"] != "file" or not member["leading_dot_slash"]
           for member in control_members):
        raise ToolError("generated control archive has invalid member type or "
                        "root convention")
    control = {member["path"]: member for member in control_members}
    expected_hex_paths = ["./" + BOARD_PROFILES[board]["firmware_name"]
                          for board in selected_boards]
    expected_updater_paths = {
        "./" + BOARD_PROFILES[board]["updater_name"]
        for board in selected_boards}
    expected_failure_paths = {
        "./" + FAILURE_IMAGE_NAMES[board] for board in selected_boards
        if "./" + FAILURE_IMAGE_NAMES[board] in evidence["control_data"]}
    expected_control = (expected_updater_paths | expected_failure_paths |
                        {"./mcu.img", "./md5sum.list", "./run.sh", "./Update"} |
                        set(expected_hex_paths))
    if (set(control) != expected_control or
            set(evidence["control_data"]) != expected_control):
        raise ToolError("generated control archive allowlist mismatch")
    for path in evidence["control_order"]:
        _assert_member(control[path], evidence["control_metadata"][path],
                       evidence["control_data"][path], path)
    if [path for path in control if path.lower().endswith(".hex")] != (
            expected_hex_paths):
        raise ToolError(
            "generated package contains unrelated firmware payload")
    if control["./Update"]["size"] != 0:
        raise ToolError("generated Update marker is not empty")
    checksum_files = {path: member["_data"] for path, member in control.items()
                      if path != "./md5sum.list"}
    _verify_md5(checksum_files, control["./md5sum.list"]["_data"], md5sum)
    outer_labels = {
        evidence["control_path"]: "control package",
        "./end.img": "end image",
        "./play": "play helper",
        "./runFirmwareExe.sh": "outer installer",
        "./start.img": "start image",
    }
    control_labels = {
        "./IAPCommand": "IAPCommand",
        "./ISPCommand": "ISPCommand",
        "./mcu.img": "control display image",
        "./md5sum.list": "checksum list",
        "./run.sh": "control script",
        "./Update": "Update marker",
    }
    for board in selected_boards:
        control_labels["./" + BOARD_PROFILES[board]["firmware_name"]] = (
            board + " firmware")
        failure_path = "./" + FAILURE_IMAGE_NAMES[board]
        if failure_path in control:
            control_labels[failure_path] = board + " failure image"
    return {
        "outer": [_manifest_member(outer_labels[path], outer[path])
                  for path in evidence["outer_order"]],
        "control": [_manifest_member(control_labels[path], control[path])
                    for path in evidence["control_order"]],
    }


def _path_is_tracked(path):
    try:
        relative = path.relative_to(REPO_ROOT.resolve())
    except (ValueError, OSError, RuntimeError):
        return False
    try:
        result = subprocess.run(
            ["git", "ls-files", "--error-unmatch", "--",
             relative.as_posix()], cwd=REPO_ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            check=False)
    except OSError:
        raise ToolError("required external tool is unavailable: git", 3)
    return result.returncode == 0


def _prepare_package_output(output, input_paths):
    try:
        output = Path(output).expanduser().resolve(strict=False)
        inputs = {Path(path).expanduser().resolve(strict=False)
                  for path in input_paths}
    except (OSError, RuntimeError):
        raise ToolError("unable to resolve package output path")
    if not PACKAGE_NAME_RE.fullmatch(output.name):
        raise ToolError("output basename must match Creator5Pro-*.tgz")
    validate_output_root(output.parent)
    manifest = Path(str(output) + ".manifest.json")
    if output in inputs or manifest in inputs:
        raise ToolError("package output overlaps an input path")
    if _path_is_tracked(output) or _path_is_tracked(manifest):
        raise ToolError("package destination may not be a tracked file")
    if output.exists() or manifest.exists():
        raise ToolError("package or manifest output already exists")
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
    except OSError:
        raise ToolError("unable to create package output directory")
    return output


def _write_temporary_sibling(output, data, suffix):
    descriptor = None
    path = None
    try:
        descriptor, raw_path = tempfile.mkstemp(
            prefix="." + output.name + ".", suffix=suffix,
            dir=output.parent)
        path = Path(raw_path)
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        return path
    except OSError:
        if descriptor is not None:
            os.close(descriptor)
        if path is not None:
            try:
                path.unlink()
            except OSError:
                pass
        raise ToolError("unable to prepare package publication")


def _publish_outputs(output, ciphertext, manifest_data):
    output = Path(output)
    manifest = Path(str(output) + ".manifest.json")
    if output.exists() or manifest.exists():
        raise ToolError("package or manifest output already exists")
    package_temp = None
    manifest_temp = None
    package_linked = False
    manifest_linked = False
    try:
        package_temp = _write_temporary_sibling(
            output, ciphertext, ".package.tmp")
        manifest_temp = _write_temporary_sibling(
            output, manifest_data, ".manifest.tmp")
        os.link(package_temp, output)
        package_linked = True
        os.link(manifest_temp, manifest)
        manifest_linked = True
    except FileExistsError:
        raise ToolError("package or manifest output already exists")
    except ToolError:
        raise
    except OSError:
        raise ToolError("atomic package publication failed")
    finally:
        if not manifest_linked and package_linked and package_temp is not None:
            try:
                if output.exists() and output.samefile(package_temp):
                    output.unlink()
            except OSError:
                pass
        for path in (package_temp, manifest_temp):
            if path is not None:
                try:
                    path.unlink()
                except OSError:
                    pass


def _repository_state():
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False)
        status = subprocess.run(
            ["git", "status", "--porcelain", "--untracked-files=normal"],
            cwd=REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False)
    except OSError:
        raise ToolError("required external tool is unavailable: git", 3)
    if revision.returncode or status.returncode:
        raise ToolError("unable to determine repository revision", 3)
    commit = revision.stdout.decode("ascii", "replace").strip()
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ToolError("repository revision is not a full commit hash", 3)
    return {"commit": commit, "dirty": bool(status.stdout.strip())}


def _require_clean_repository():
    repository = _repository_state()
    if repository["dirty"]:
        raise ToolError(
            "refusing to create update package from a dirty repository")
    return repository


def _create_manifest(firmware_reports, template_plaintext_hash, plaintext,
                     ciphertext, members, shell, md5sum, repository):
    firmwares = {}
    tools = {}
    for board in _canonical_boards(firmware_reports):
        firmware = json.loads(json.dumps(firmware_reports[board]))
        firmware.setdefault("dictionary", {})["kconfig"] = (
            "[redacted: validated separately]")
        if not tools:
            tools = dict(firmware.get("tools", {}))
        firmware.pop("tools", None)
        firmwares[board] = firmware
    tools["sh"] = _version(shell)
    tools["md5sum"] = _version(md5sum)
    timestamp = datetime.datetime.now(datetime.timezone.utc).replace(
        microsecond=0).isoformat().replace("+00:00", "Z")
    installer_changes = [
        {"effect": "NIM log deletion", "status": "suppressed"},
        {"effect": "persistent startup image replacement",
         "status": "suppressed"},
        {"effect": "pending-update marker removal on success",
         "status": "added"},
    ]
    reclaim_templates = {profile["plaintext"]
                         for profile in CANONICAL_TEMPLATE_PROFILES
                         if profile.get("space_reclaim")}
    if template_plaintext_hash in reclaim_templates:
        installer_changes.extend([
            {"effect": "host disk-reclaim deletions",
             "status": "suppressed"},
            {"effect": "unselected board failure-result checks",
             "status": "suppressed"},
            {"effect": "selected board failure-result check and image",
             "status": "retained"},
        ])
    return {
        "schema_version": 2,
        "repository": repository,
        "build_timestamp_utc": timestamp,
        "tools": tools,
        "firmwares": firmwares,
        "template_plaintext_sha256": template_plaintext_hash,
        "output": {
            "ciphertext_sha256": hashlib.sha256(ciphertext).hexdigest(),
            "plaintext_sha256": hashlib.sha256(plaintext).hexdigest(),
            "members": members,
        },
        "installer_changes": installer_changes,
        "validation": {
            "passed": [
                "firmware representations",
                "canonical template hash gates",
                "input control checksums",
                "package script transformations and sh -n",
                "reduced archive profile",
                "output control checksums",
                "fresh ciphertext decryption and reinspection",
            ],
            "hardware_verified": False,
        },
    }


def _normalize_firmware_inputs(firmware_inputs):
    if not isinstance(firmware_inputs, Mapping):
        raise ToolError("firmware inputs must be a mapping")
    boards = _canonical_boards(firmware_inputs.keys())
    normalized = {}
    required_roles = {"firmware", "elf", "dictionary"}
    for board in boards:
        products = firmware_inputs[board]
        if not isinstance(products, Mapping):
            raise ToolError("firmware entry for %s must be a mapping" % board)
        roles = set(products)
        if roles != required_roles:
            missing = sorted(required_roles - roles)
            extra = sorted(roles - required_roles)
            detail = (("missing " + ", ".join(missing)) if missing else
                      ("extra " + ", ".join(extra)))
            raise ToolError("firmware roles for %s are invalid: %s" %
                            (board, detail))
        for role in required_roles:
            if not isinstance(products[role], (str, os.PathLike)):
                raise ToolError(
                    "firmware path for %s/%s is invalid" % (board, role))
        normalized[board] = {role: products[role] for role in
                             ("firmware", "elf", "dictionary")}
    return normalized


def _require_clean_firmware_reports(firmware_reports):
    for board in _canonical_boards(firmware_reports):
        version = firmware_reports[board].get(
            "dictionary", {}).get("version", "")
        if "dirty" in str(version).lower():
            raise ToolError("refusing to package dirty firmware for %s" % board)


def package_update(template, firmware_inputs, output,
                   cross_prefix="arm-none-eabi-", openssl="openssl"):
    firmware_inputs = _normalize_firmware_inputs(firmware_inputs)
    input_paths = [template]
    for products in firmware_inputs.values():
        input_paths.extend(products.values())
    output = _prepare_package_output(output, input_paths)
    firmware_reports = {}
    firmware_data = {}
    for board, products in firmware_inputs.items():
        report, exact_hex = _validate_firmware(
            board, products["firmware"], products["elf"],
            products["dictionary"], cross_prefix, openssl)
        firmware_reports[board] = report
        firmware_data[board] = exact_hex
    _require_clean_firmware_reports(firmware_reports)
    try:
        template_data = _read_archive_input(template)
    except OSError:
        raise ToolError("unable to read canonical template")
    if not template_data.startswith(b"Salted__"):
        raise ToolError("canonical package template must be encrypted")
    template_model = _inspect_archive_model(
        template_data, decrypt=True, openssl=openssl)
    if (template_model.get("format") != "encrypted" or
            template_model.get("payload", {}).get("format") != "tar"):
        raise ToolError("canonical template did not decrypt to a plain tar")
    template_plaintext = template_model["_payload"]
    shell_path = _program_path("sh", "sh")
    md5sum_path = _program_path("md5sum", "md5sum")
    profile = _canonical_template_profile(
        template_model["payload"], md5sum_path)
    selected_boards = tuple(firmware_inputs)
    plaintext, evidence = _build_reduced_plaintext(
        profile, firmware_data, shell_path, md5sum_path)
    constructed_model = _inspect_archive_model(plaintext)
    member_summary = _assert_reduced_profile(
        constructed_model, evidence, selected_boards, md5sum_path)
    ciphertext = openssl_crypt(plaintext, False, openssl)
    with tempfile.TemporaryDirectory(prefix="c5-update-verify-") as temp:
        candidate = Path(temp) / "candidate.tgz"
        candidate.write_bytes(ciphertext)
        verified_model = _inspect_archive_model(
            _read_archive_input(candidate), decrypt=True, openssl=openssl)
        if (verified_model.get("format") != "encrypted" or
                verified_model.get("_payload") != plaintext):
            raise ToolError("fresh package decryption differs from plaintext")
        verified_summary = _assert_reduced_profile(
            verified_model["payload"], evidence, selected_boards, md5sum_path)
    if verified_summary != member_summary:
        raise ToolError("fresh package reinspection differs from construction")
    repository = _require_clean_repository()
    manifest = _create_manifest(
        firmware_reports, hashlib.sha256(template_plaintext).hexdigest(),
        plaintext, ciphertext, verified_summary, shell_path, md5sum_path,
        repository)
    manifest_data = (json.dumps(manifest, sort_keys=True, indent=2) +
                     "\n").encode("utf-8")
    _publish_outputs(output, ciphertext, manifest_data)
    return {
        "stage": "package", "package": output.name,
        "manifest": output.name + ".manifest.json",
        "ciphertext_sha256": manifest["output"]["ciphertext_sha256"],
        "plaintext_sha256": manifest["output"]["plaintext_sha256"],
        "firmwares": {board: {
            "hex_sha256": firmware_reports[board]["hex_sha256"],
            "normalized_sha256": firmware_reports[board]["normalized_sha256"],
        } for board in firmware_inputs},
        "validation": manifest["validation"],
    }


def run_all_stages(boards, template, output_dir, output, jobs=1,
                   cross_prefix="arm-none-eabi-", openssl="openssl"):
    boards = _canonical_boards(boards)
    root = validate_output_root(output_dir)
    build_root = root / "build"
    build_dirs = {board: build_root / board for board in boards}
    firmware_inputs = {board: {
        "firmware": build_dirs[board] / BOARD_PROFILES[board]["firmware_name"],
        "elf": build_dirs[board] / "klipper.elf",
        "dictionary": build_dirs[board] / "klipper.dict",
    } for board in boards}
    output_path = _prepare_package_output(
        output, [template] + [path for products in firmware_inputs.values()
                             for path in products.values()])
    if _inside(output_path, build_root.resolve(strict=False)):
        raise ToolError("package output must be outside the build subtree")
    for board in boards:
        _preflight_build_output(build_dirs[board])
    build_reports = {}
    for board in boards:
        build_reports[board] = build_firmware(
            board, build_dirs[board], jobs, cross_prefix, openssl)
    package_report = package_update(
        template, firmware_inputs, output, cross_prefix, openssl)
    return {"stage": "all", "builds": build_reports,
            "package": package_report}


def _add_global(parser):
    parser.add_argument(
    "--cross-prefix",
    default="arm-none-eabi-",
     help="ARM toolchain prefix")
    parser.add_argument(
    "--openssl",
    default="openssl",
     help="OpenSSL executable")
def _cli_firmware_inputs(groups):
    boards = []
    for group in groups:
        board = group[0]
        _profile(board)
        if board in boards:
            raise ToolError("duplicate board selection: %s" % board)
        boards.append(board)
    ordered = _canonical_boards(boards)
    by_board = {group[0]: group[1:] for group in groups}
    return {board: {
        "firmware": Path(by_board[board][0]),
        "elf": Path(by_board[board][1]),
        "dictionary": Path(by_board[board][2]),
    } for board in ordered}


def create_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    _add_global(parser)
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="build and validate C5 firmware")
    build.add_argument("--board", required=True, choices=tuple(BOARD_PROFILES))
    build.add_argument("--output-dir", required=True, type=Path)
    build.add_argument("--jobs", type=int, default=1)
    validate = sub.add_parser("validate",
                              help="validate firmware representations")
    validate.add_argument(
        "--board", required=True, choices=tuple(BOARD_PROFILES))
    validate.add_argument("--firmware", required=True, type=Path)
    validate.add_argument("--elf", required=True, type=Path)
    validate.add_argument("--dictionary", required=True, type=Path)
    inspect = sub.add_parser("inspect", help="inspect an update package")
    inspect.add_argument("--input", required=True, type=Path)
    inspect.add_argument("--decrypt", action="store_true")
    inspect.add_argument("--extract-temp", action="store_true")
    package = sub.add_parser("package",
                             help="create an encrypted update package")
    package.add_argument("--template", required=True, type=Path)
    package.add_argument("--firmware", required=True, action="append", nargs=4,
                         metavar=("BOARD", "HEX", "ELF", "DICT"))
    package.add_argument("--output", required=True, type=Path)
    all_cmd = sub.add_parser("all", help="build, validate, and package")
    all_cmd.add_argument("--board", required=True, action="append",
                         choices=tuple(BOARD_PROFILES))
    all_cmd.add_argument("--template", required=True, type=Path)
    all_cmd.add_argument("--output-dir", required=True, type=Path)
    all_cmd.add_argument("--output", required=True, type=Path)
    all_cmd.add_argument("--jobs", type=int, default=1)
    return parser


def _json_print(report):
    json.dump(report, sys.stdout, sort_keys=True, indent=2)
    sys.stdout.write("\n")


def main(argv=None):
    args = create_argument_parser().parse_args(argv)
    try:
        if args.command == "build":
            _json_print(build_firmware(
                args.board, args.output_dir, args.jobs, args.cross_prefix,
                args.openssl))
        elif args.command == "validate":
            _json_print(validate_firmware(
                args.board, args.firmware, args.elf, args.dictionary,
                args.cross_prefix, args.openssl))
        elif args.command == "inspect":
            archive_data = _read_archive_input(args.input)
            _json_print(inspect_archive(
                archive_data, decrypt=args.decrypt,
                extract_temp=args.extract_temp, openssl=args.openssl))
        elif args.command == "package":
            _require_clean_repository()
            firmware_inputs = _cli_firmware_inputs(args.firmware)
            _json_print(package_update(
                args.template, firmware_inputs, args.output,
                args.cross_prefix, args.openssl))
        elif args.command == "all":
            _require_clean_repository()
            _json_print(run_all_stages(
                args.board, args.template, args.output_dir, args.output,
                args.jobs, args.cross_prefix, args.openssl))
        else:
            raise ToolError("unknown stage")
        return 0
    except ToolError as exc:
        secret = os.environ.get("C5_UPDATE_PASSPHRASE")
        print("error: " + sanitize_diagnostic(exc, secret), file=sys.stderr)
        return exc.exit_code
    except OSError:
        print("error: filesystem operation failed", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
