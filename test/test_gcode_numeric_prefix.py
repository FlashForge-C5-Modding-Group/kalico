import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


package = types.ModuleType('klippy')
package.__path__ = [str(Path(__file__).resolve().parents[1] / 'klippy')]
with mock.patch.dict(sys.modules, {
        'klippy': package,
        'klippy.mathutil': types.ModuleType('klippy.mathutil')}):
    spec = importlib.util.spec_from_file_location(
        'klippy.gcode', Path(package.__path__[0]) / 'gcode.py')
    gcode = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gcode)


class GCodeNumericPrefixTests(unittest.TestCase):
    def setUp(self):
        dispatch = gcode.GCodeDispatch.__new__(gcode.GCodeDispatch)
        dispatch.printer = mock.Mock()
        dispatch.printer.config_error = ValueError
        dispatch.printer.get_reactor.return_value.monotonic.return_value = 0.
        dispatch.ready_gcode_handlers = {}
        dispatch.base_gcode_handlers = {}
        dispatch.gcode_handlers = dispatch.ready_gcode_handlers
        dispatch.gcode_help = {}
        dispatch.output_callbacks = []
        dispatch.is_printer_ready = True
        dispatch.is_fileinput = False
        dispatch._script_context = 0
        self.dispatch = dispatch
        self.calls = []

    def test_creator5_extended_command_registers_and_dispatches(self):
        self.dispatch.register_command('C5_PREPARE_FILAMENT_LOAD',
                                       lambda cmd: self.calls.append(
                                           (cmd.get_command(), cmd.get('TOOL'),
                                            cmd.get('TEMP'))))
        self.dispatch.run_script_from_command(
            'C5_PREPARE_FILAMENT_LOAD TOOL=2 TEMP=220')
        self.assertEqual(self.calls,
                         [('C5_PREPARE_FILAMENT_LOAD', '2', '220')])

    def test_normal_gcode_and_invalid_numeric_name(self):
        self.dispatch.register_command(
            'G1', lambda cmd: self.calls.append(
                (cmd.get_command(), cmd.get('X'))))
        self.dispatch.run_script_from_command('G1 X10')
        self.assertEqual(self.calls, [('G1', '10')])
        self.dispatch.printer.get_reactor.assert_not_called()
        with self.assertRaisesRegex(ValueError, 'invalid name'):
            self.dispatch.register_command('C5PREPARE', lambda cmd: None)

    def test_long_macro_yields_between_commands(self):
        reactor = self.dispatch.printer.get_reactor.return_value
        clock = iter([0., .006, .012, .012, .018, .024, .024])
        reactor.monotonic.side_effect = lambda: next(clock)
        self.dispatch.register_command(
            'G1', lambda cmd: self.calls.append(cmd.get('X')))
        self.dispatch.run_script_from_command(
            'G1 X1\nG1 X2\nG1 X3\nG1 X4')
        self.assertEqual(self.calls, ['1', '2', '3', '4'])
        self.assertEqual(reactor.pause.call_count, 2)
        reactor.pause.assert_called_with(reactor.NOW)


if __name__ == '__main__':
    unittest.main()
