import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

EXTRAS_DIR = Path(__file__).resolve().parents[1] / 'klippy' / 'extras'


class FakeFan:
    def __init__(self, config, default_shutdown_speed=0.):
        self.speeds = []

    def set_speed(self, speed):
        self.speeds.append(speed)

    def get_status(self, eventtime):
        return {}


extras_package = types.ModuleType('klippy.extras')
extras_package.__path__ = [str(EXTRAS_DIR)]
fan_module = types.ModuleType('klippy.extras.fan')
fan_module.Fan = FakeFan
with mock.patch.dict(sys.modules, {
        'klippy': types.ModuleType('klippy'),
        'klippy.extras': extras_package,
        'klippy.extras.fan': fan_module,
        'klippy.extras.filament_switch_sensor': types.ModuleType(
            'klippy.extras.filament_switch_sensor')}):
    toolchanger_spec = importlib.util.spec_from_file_location(
        'klippy.extras.creator5_toolchanger',
        EXTRAS_DIR / 'creator5_toolchanger.py')
    toolchanger = importlib.util.module_from_spec(toolchanger_spec)
    sys.modules['klippy.extras.creator5_toolchanger'] = toolchanger
    toolchanger_spec.loader.exec_module(toolchanger)
    spec = importlib.util.spec_from_file_location(
        'klippy.extras.heater_fan_throttled',
        EXTRAS_DIR / 'heater_fan_throttled.py')
    MODULE = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(MODULE)


class Config:
    def __init__(self, printer, **options):
        self.printer = printer
        self.options = options

    def get_printer(self):
        return self.printer

    def get(self, name, default=None):
        return self.options.get(name, default)

    def getlist(self, name, default=None):
        return self.options.get(name, default)

    def getfloat(self, name, default=None, **kwargs):
        return self.options.get(name, default)

    def getboolean(self, name, default=None):
        return self.options.get(name, default)


class Printer:
    def __init__(self):
        self.objects = {'gcode': mock.Mock()}

    def load_object(self, config, name):
        pass

    def register_event_handler(self, event, handler):
        pass

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class Heater:
    def __init__(self, temp, target, pwm):
        self.temp, self.target, self.last_pwm_value = temp, target, pwm

    def get_temp(self, eventtime):
        return self.temp, self.target


class HeaterFanThrottledTests(unittest.TestCase):
    def setUp(self):
        # The switch class is imported when the fan is created
        patcher = mock.patch.dict(sys.modules, {
            'klippy': types.ModuleType('klippy'),
            'klippy.extras': extras_package,
            'klippy.extras.creator5_toolchanger': toolchanger})
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_fan(self, **options):
        printer = Printer()
        fan = MODULE.PrinterHeaterFanThrottled(
            Config(printer, fan_speed=0.9, **options))
        return printer, fan

    def test_throttles_near_target_without_a_switch(self):
        printer, fan = self.make_fan()
        fan.heaters = [Heater(59., 60., 0.1)]
        fan.callback(0.)
        self.assertEqual(fan.fan.speeds, [0.5])
        self.assertIsNone(printer.lookup_object('output_pin chamber_heat_fan_full'))

    def test_switch_off_keeps_the_throttling(self):
        printer, fan = self.make_fan(full_speed_switch='chamber_heat_fan_full')
        fan.heaters = [Heater(59., 60., 0.1)]
        fan.callback(0.)
        self.assertEqual(fan.fan.speeds, [0.5])
        self.assertEqual(
            printer.lookup_object('output_pin chamber_heat_fan_full')
            .get_status(0.), {'value': 0.})

    def test_switch_on_runs_full_speed_while_heating(self):
        printer, fan = self.make_fan(full_speed_switch='chamber_heat_fan_full')
        fan.full_speed_switch.enabled = True
        fan.heaters = [Heater(59., 60., 0.1)]
        fan.callback(0.)
        fan.heaters = [Heater(30., 60., 1.)]
        fan.callback(1.)
        self.assertEqual(fan.fan.speeds, [1.])

    def test_switch_on_still_stops_when_the_heater_is_off(self):
        printer, fan = self.make_fan(full_speed_switch='chamber_heat_fan_full',
                                     run_while_cooling=False)
        fan.full_speed_switch.enabled = True
        fan.heaters = [Heater(70., 0., 0.)]
        fan.callback(0.)
        self.assertEqual(fan.fan.speeds, [])


if __name__ == '__main__':
    unittest.main()
