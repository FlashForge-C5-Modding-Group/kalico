"""Checks for the Creator 5 eBoard's dedicated TMC2209 UART transport."""

import importlib.util
from pathlib import Path
import unittest


SPEC = importlib.util.spec_from_file_location(
    "tmc_uart", Path(__file__).resolve().parents[1]
    / "klippy" / "extras" / "tmc_uart.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeCommand:
    def __init__(self, response=None):
        self.calls = []
        self.response = response

    def send(self, args, minclock=0):
        self.calls.append((args, minclock))
        return {"read": self.response, "#receive_time": 1.25}


class FakeMCU:
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000)


class C5TMCUartTests(unittest.TestCase):
    def setUp(self):
        self.uart = MODULE.MCU_C5_TMC_uart.__new__(MODULE.MCU_C5_TMC_uart)
        self.uart.mcu = FakeMCU()

    def make_reply(self, reg, val):
        reply = bytearray([
            0x05, 0xFF, reg, (val >> 24) & 0xFF, (val >> 16) & 0xFF,
            (val >> 8) & 0xFF, val & 0xFF,
        ])
        reply.append(MODULE.MCU_TMC_uart_bitbang._calc_crc8(self.uart, reply))
        return reply

    def test_read_packet_and_register_value(self):
        self.uart.send_cmd = FakeCommand(self.make_reply(0x02, 0x12345678))
        result = self.uart.reg_read(None, 0, 0x02)
        self.assertEqual(result["data"], 0x12345678)
        self.assertEqual(
            self.uart.send_cmd.calls[0][0],
            [0, bytearray([0x05, 0, 0x02, 0x8F]), 8],
        )

    def test_read_accepts_dma_echo_and_reply(self):
        reply = self.uart._encode(0, 0x02) + self.make_reply(0x02, 0x12345678)
        self.uart.send_cmd = FakeCommand(reply)
        self.assertEqual(self.uart.reg_read(None, 0, 0x02)["data"], 0x12345678)

    def test_read_accepts_stray_byte_before_dma_echo(self):
        reply = (bytearray([0xC5]) + self.uart._encode(0, 0x02)
                 + self.make_reply(0x02, 0x12345678))
        self.uart.send_cmd = FakeCommand(reply)
        self.assertEqual(self.uart.reg_read(None, 0, 0x02)["data"], 0x12345678)

    def test_read_rejects_corrupt_crc(self):
        reply = self.make_reply(0x02, 0x12345678)
        reply[-1] ^= 1
        self.uart.send_cmd = FakeCommand(reply)
        self.assertIsNone(self.uart.reg_read(None, 0, 0x02)["data"])

    def test_write_uses_mcu_clock(self):
        self.uart.send_cmd = FakeCommand()
        self.uart.reg_write(None, 0, 0x6C, 0x140082C3, 1.5)
        args, minclock = self.uart.send_cmd.calls[0]
        self.assertEqual(args[1][:3], bytearray([0x05, 0, 0xEC]))
        self.assertEqual(args[2], 0)
        self.assertEqual(minclock, 1500)


if __name__ == "__main__":
    unittest.main()
