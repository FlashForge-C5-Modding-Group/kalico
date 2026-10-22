import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

EXTRAS_DIR = Path(__file__).resolve().parents[1] / 'klippy' / 'extras'


def load(name):
    spec = importlib.util.spec_from_file_location(name, EXTRAS_DIR / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with mock.patch.dict(sys.modules, {'klippy': types.SimpleNamespace(stepper=None)}):
    mclib = load('mclib')
c5_motion_guard = load('c5_motion_guard')


class FakeConfig:
    def __init__(self, printer, values=None):
        self.printer, self.values = printer, values or {}

    def get_printer(self):
        return self.printer

    def getboolean(self, name, default):
        return self.values.get(name, default)

    def getchoice(self, name, choices, default):
        return choices[self.values.get(name, default)]

    def getfloat(self, name, default, **kw):
        return self.values.get(name, default)

    def getint(self, name, default, **kw):
        return self.values.get(name, default)

    def getlist(self, name, default):
        return self.values.get(name, default)

    def error(self, msg):
        return ValueError(msg)


class FakeMotor:
    query_status_cmd = object()

    def __init__(self):
        self.pending = []
        self.health = mclib.MCLIB._empty_health()
        self.queries = 0
        self.last_status = None

    def answer(self, **params):
        self.pending.append(dict({'stall': 0, 'stalls': 0, 'slips': 0,
                                  'slip_total': 0, 'samples': 0,
                                  'torque_min': 0, 'torque_max': 0,
                                  'err_min': 0, 'err_max': 0}, **params))

    def query_health(self):
        self.queries += 1
        for params in self.pending:
            mclib.MCLIB._handle_status(self, params)
        self.pending = []
        return True

    def take_health(self):
        h, self.health = self.health, mclib.MCLIB._empty_health()
        return h

    def get_stall_threshold(self):
        return 1.

    def set_slip_mode(self, mode):
        self.slip_mode = mode
        return True


class MotionGuardTest(unittest.TestCase):
    def setUp(self):
        self.state = 'standby'
        self.motors = {a: FakeMotor() for a in c5_motion_guard.AXES}
        self.toolchanger = mock.Mock(busy=False)
        self.gcode = mock.Mock()
        self.reactor = mock.Mock()
        self.reactor.monotonic.return_value = 0.
        self.reactor.register_callback.side_effect = lambda cb: cb(0.)
        stats = mock.Mock()
        stats.get_status.side_effect = lambda e: {'state': self.state}
        objects = {'gcode': self.gcode, 'print_stats': stats,
                   'creator5_toolchanger': self.toolchanger}
        objects.update({'mclib ' + a: m for a, m in self.motors.items()})
        self.printer = mock.Mock()
        self.printer.get_reactor.return_value = self.reactor
        self.printer.lookup_object.side_effect = (
            lambda name, default=None: objects.get(name, default))

    def guard(self, **values):
        g = c5_motion_guard.C5MotionGuard(FakeConfig(self.printer, values))
        g._ready()
        return g

    def start_print(self, g):
        self.state = 'printing'
        g._poll(0.)   # print start: discard old counts
        g._poll(0.)   # settle window

    def test_idle_counts_before_a_print_are_ignored(self):
        g = self.guard()
        self.motors['stepper_x'].answer(slips=5)
        self.start_print(g)
        g._poll(0.)
        self.assertEqual(g.print_totals['stepper_x']['slips'], 0)
        self.assertIsNone(g.tripped)

    def test_skipped_steps_warn(self):
        g = self.guard()
        self.start_print(g)
        self.motors['stepper_y'].answer(slips=-2, samples=10)
        g._poll(0.)
        g._poll(0.)
        self.assertIn('stepper_y skipped steps: 2', g.tripped)
        self.gcode.respond_raw.assert_called_once()
        self.gcode.run_script.assert_not_called()

    def test_pause_action(self):
        g = self.guard(action='pause')
        self.start_print(g)
        self.motors['stepper_x'].answer(slips=1)
        g._poll(0.)
        g._poll(0.)
        self.gcode.run_script.assert_called_once_with('PAUSE')

    def test_toolchange_windows_are_ignored(self):
        g = self.guard()
        self.start_print(g)
        self.toolchanger.busy = True
        self.motors['stepper_x'].answer(slips=3, stalls=2)
        g._poll(0.)
        g._poll(0.)
        self.toolchanger.busy = False
        g._poll(0.)
        g._poll(0.)
        self.assertEqual(g.print_totals['stepper_x']['slips'], 0)
        self.assertEqual(g.print_totals['stepper_x']['ignored'], 5)
        self.assertIsNone(g.tripped)

    def test_stalls_only_count_on_calibrated_axes(self):
        g = self.guard(stall_axes=['z'])
        self.start_print(g)
        self.motors['stepper_x'].answer(stalls=4)
        g._poll(0.)
        g._poll(0.)
        self.assertIsNone(g.tripped)
        self.motors['stepper_z'].answer(stalls=1)
        g._poll(0.)
        g._poll(0.)
        self.assertEqual(g.tripped, 'stepper_z stalled')

    def test_record_suggests_threshold(self):
        g = self.guard()
        gcmd = mock.Mock()
        gcmd.get_int.return_value = 0
        g.cmd_RECORD(gcmd)
        self.motors['stepper_x'].answer(samples=100, torque_min=2000,
                                        torque_max=9000)
        g._poll(0.)
        g._poll(0.)
        g._poll(0.)
        gcmd.get_int.return_value = 1
        g.cmd_RECORD(gcmd)
        text = gcmd.respond_info.call_args[0][0]
        self.assertIn('stepper_x: torque 2.000 to 9.000', text)
        self.assertIn('about 1.000', text)


    def test_slip_watch_is_sent_at_ready_and_on_change(self):
        g = self.guard(slip_watch='holding')
        self.assertEqual({m.slip_mode for m in self.motors.values()}, {1})
        gcmd = mock.Mock()
        gcmd.get_int.side_effect = lambda name, default, **kw: default
        gcmd.get.side_effect = lambda name, default: (
            'always' if name == 'WATCH' else default)
        g.cmd_GUARD(gcmd)
        self.assertEqual({m.slip_mode for m in self.motors.values()}, {2})
        self.assertEqual(g.get_status(0.)['slip_watch'], 'always')


if __name__ == '__main__':
    unittest.main()
