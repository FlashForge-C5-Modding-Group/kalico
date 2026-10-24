import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

MODULE_PATH = Path(__file__).resolve().parents[1] / 'klippy' / 'extras' / 'creator5_dryer.py'
serial_stub = types.ModuleType('serial')
serial_stub.Serial = mock.Mock()
serial_stub.SerialException = OSError
with mock.patch.dict(sys.modules, {'serial': serial_stub}):
    SPEC = importlib.util.spec_from_file_location('creator5_dryer', MODULE_PATH)
    MODULE = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(MODULE)

# A reply in the dryer firmware's M14 format: a flag, then 11 values per chamber
STATUS = ('M14 ok. 0 A:3 2 23 65 312 0 0 0 0 0 0 '
          'B:0 0 41 26 0 0 0 1 0 0 0')


class GCmd:
    def __init__(self, **params):
        self.params = params
        self.messages = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def _number(self, kind, name, default, minval=None, maxval=None):
        if name not in self.params:
            if default is None and 'required' in self.params.get('_required', ()):
                raise self.error('missing %s' % name)
            return default
        value = kind(self.params[name])
        if (minval is not None and value < minval) or (maxval is not None and value > maxval):
            raise self.error('%s out of range' % name)
        return value

    def get_int(self, name, default=None, minval=None, maxval=None):
        return self._number(int, name, default, minval, maxval)

    def get_float(self, name, default=None, minval=None, maxval=None):
        return self._number(float, name, default, minval, maxval)

    def error(self, message):
        return RuntimeError(message)

    def respond_info(self, message):
        self.messages.append(message)


class Creator5DryerTests(unittest.TestCase):
    def make_dryer(self):
        dryer = MODULE.Creator5Dryer.__new__(MODULE.Creator5Dryer)
        dryer.port = '/dev/ttyS3'
        dryer.connected = True
        dryer.in_bootloader = False
        dryer.version = ''
        dryer.commands = MODULE.queue.Queue()
        dryer.chambers = {c: dryer._empty_chamber() for c in ('A', 'B')}
        dryer.sensors = {}
        return dryer

    def sent(self, dryer):
        return dryer.commands.get_nowait()[0]

    def test_parses_both_chambers(self):
        status = MODULE.parse_status(STATUS)
        self.assertEqual(status['A'][:5], [3, 2, 23, 65, 312])
        self.assertEqual(status['B'][:5], [0, 0, 41, 26, 0])

    def test_rejects_other_replies(self):
        self.assertIsNone(MODULE.parse_status('M10 ok.'))
        self.assertIsNone(MODULE.parse_status('Boot'))
        self.assertIsNone(MODULE.parse_status('M14 ok. 0 A:1 2 3'))

    def test_status_fields(self):
        dryer = self.make_dryer()
        dryer.reactor = mock.Mock()
        dryer.reactor.register_async_callback = lambda cb: cb(0.)
        dryer._publish(connected=True, status=MODULE.parse_status(STATUS))
        self.assertEqual(dryer.chambers['A'], {
            'mode': 3, 'state': 2, 'drying': True, 'humidity': 23,
            'temperature': 65, 'remaining_minutes': 312})
        self.assertFalse(dryer.chambers['B']['drying'])

    def test_start_sends_m10_in_minutes(self):
        dryer = self.make_dryer()
        dryer.cmd_start(GCmd(CHAMBER='b', TEMP=60, HOURS=4))
        self.assertEqual(self.sent(dryer), 'M10 C1 L60 S240')
        dryer.cmd_start(GCmd(TEMP=45, MINUTES=90))
        self.assertEqual(self.sent(dryer), 'M10 C2 L45 S90')

    def test_start_checks_the_dryers_limits(self):
        dryer = self.make_dryer()
        for params in ({'TEMP': 90, 'HOURS': 4}, {'TEMP': 40, 'HOURS': 4},
                       {'TEMP': 60, 'MINUTES': 30}, {'TEMP': 60}):
            with self.assertRaises(RuntimeError):
                dryer.cmd_start(GCmd(**params))
        self.assertTrue(dryer.commands.empty())

    def test_stop_pause_dehumidify(self):
        dryer = self.make_dryer()
        dryer.cmd_stop(GCmd(CHAMBER='A'))
        dryer.cmd_pause(GCmd(CHAMBER='AB'))
        dryer.cmd_dehumidify(GCmd(CHAMBER='B', ENABLE=0))
        self.assertEqual([self.sent(dryer) for _ in range(3)],
                         ['M11 C0', 'M12 C2', 'M13 C1 L0'])

    def test_refuses_commands_when_not_connected(self):
        dryer = self.make_dryer()
        dryer.connected = False
        with self.assertRaises(RuntimeError):
            dryer.cmd_stop(GCmd())
        with self.assertRaises(RuntimeError):
            dryer.cmd_start(GCmd(CHAMBER='C', TEMP=60, HOURS=4))

    def test_request_skips_unrelated_lines(self):
        dryer = self.make_dryer()
        ser = mock.Mock()
        ser.readline.side_effect = [b'M14 ok. junk\r\n', b'M16 ok. V1.0.2\r\n']
        self.assertEqual(dryer._request(ser, 'M16', 'M16 ok.'), 'M16 ok. V1.0.2')
        ser.write.assert_called_once_with(b'M16\n')


if __name__ == '__main__':
    unittest.main()
