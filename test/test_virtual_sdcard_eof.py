import importlib.util
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'virtual_sdcard', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'virtual_sdcard.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PRINT_STATS_SPEC = importlib.util.spec_from_file_location(
    'print_stats', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'print_stats.py')
PRINT_STATS_MODULE = importlib.util.module_from_spec(PRINT_STATS_SPEC)
PRINT_STATS_SPEC.loader.exec_module(PRINT_STATS_MODULE)


class VirtualSDEOFTests(unittest.TestCase):
    def make_sd(self, text):
        sd = MODULE.VirtualSD.__new__(MODULE.VirtualSD)
        sd.current_file = io.TextIOWrapper(io.BytesIO(text.encode()), newline='')
        self.addCleanup(sd.current_file.close)
        sd.file_position = sd.next_file_position = 0
        sd.file_size = len(text.encode())
        sd.auto_creator5_start = False
        sd.creator5_start_done = False
        sd.creator5_stop_done = False
        sd.cancelled_work = False
        sd.creator5_preloaded_definitions = set()
        sd.must_pause_work = sd.cmd_from_sd = False
        sd.work_timer = object()
        sd._in_worker_script = False
        sd.reactor = mock.Mock(NOW=0., NEVER=999.)
        sd.reactor.monotonic.return_value = 0.
        sd.gcode = mock.Mock()
        sd.gcode.error = RuntimeError
        sd.gcode.get_mutex.return_value.test.return_value = False
        sd.print_stats = mock.Mock()
        sd.on_error_gcode = mock.Mock()
        sd.on_error_gcode.render.return_value = 'TURN_OFF_HEATERS'
        return sd

    def attach_print_stats(self, sd):
        printer = mock.Mock()
        printer.get_reactor.return_value = sd.reactor
        printer.lookup_object.return_value = sd.gcode
        gcode_move = mock.Mock()
        gcode_move.get_status.return_value = {
            'position': SimpleNamespace(e=0.0), 'extrude_factor': 1.0}
        printer.load_object.return_value = gcode_move
        config = mock.Mock()
        config.get_printer.return_value = printer
        sd.printer = printer
        sd.print_stats = PRINT_STATS_MODULE.PrintStats(config)

    def cancel_from_script(self, sd, cleanup):
        previous = sd.reactor.pause.side_effect
        sd.reactor.pause.side_effect = AssertionError(
            'worker cancellation attempted to wait')
        try:
            sd.do_cancel()
        finally:
            sd.reactor.pause.side_effect = previous
        cleanup.append('cancel returned')

    def assert_cancelled(self, sd, original_file, cleanup):
        self.assertTrue(original_file.closed)
        self.assertFalse(sd.is_active())
        self.assertEqual(sd.print_stats.get_status(
            sd.reactor.monotonic())['state'], 'cancelled')
        self.assertEqual(cleanup, ['cancel returned'])
        events = [c.args[0] for c in sd.printer.send_event.call_args_list]
        self.assertNotIn('print_stats:complete_printing', events)

    def test_automatic_start_cancel_allows_new_print(self):
        sd = self.make_sd('M109 S220\nG1 X10\n')
        sd.auto_creator5_start = True
        self.attach_print_stats(sd)
        original_file = sd.current_file
        cleanup = []
        commands = []
        def dispatch(line):
            commands.append(line)
            if line.startswith('C5_PRINT_START'):
                self.cancel_from_script(sd, cleanup)
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        self.assert_cancelled(sd, original_file, cleanup)
        self.assertNotIn('G1 X10', commands)
        self.assertNotIn('C5_PRINT_STOP', commands)

        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'retry.gcode').write_text('M109 S220\nG1 X20\n')
            sd.sdcard_dirname = directory
            gcmd = mock.Mock()
            gcmd.get.return_value = 'retry.gcode'
            gcmd.error = sd.gcode.error
            commands.clear()
            sd.printer.send_event.reset_mock()
            sd.gcode.run_script.side_effect = commands.append
            sd.cmd_SDCARD_PRINT_FILE(gcmd)
            retry_file = sd.current_file
            self.addCleanup(retry_file.close)
            worker = sd.reactor.register_timer.call_args.args[0]
            worker(0.)
        self.assertTrue(retry_file.closed)
        self.assertFalse(sd.is_active())
        self.assertEqual(sum(c.startswith('C5_PRINT_START')
                             for c in commands), 1)
        self.assertIn('G1 X20', commands)
        self.assertEqual(commands.count('C5_PRINT_STOP'), 1)
        self.assertEqual(sd.print_stats.get_status(
            sd.reactor.monotonic())['state'], 'complete')

    def test_automatic_start_error_can_cancel_cleanup(self):
        for text in ('G1 X10\n', 'M109 S220\nG1 X10\n'):
            with self.subTest(text=text):
                sd = self.make_sd(text)
                sd.auto_creator5_start = True
                self.attach_print_stats(sd)
                original_file = sd.current_file
                cleanup = []
                commands = []
                sd.on_error_gcode.render.return_value = 'CANCEL_PRINT'
                def dispatch(line):
                    commands.append(line)
                    if line.startswith('C5_PRINT_START'):
                        raise sd.gcode.error('startup failed')
                    if line == 'CANCEL_PRINT':
                        self.cancel_from_script(sd, cleanup)
                sd.gcode.run_script.side_effect = dispatch
                sd.work_handler(0.)
                self.assert_cancelled(sd, original_file, cleanup)
                self.assertNotIn('G1 X10', commands)
                self.assertNotIn('C5_PRINT_STOP', commands)

    def test_automatic_end_cancel_does_not_complete(self):
        sd = self.make_sd('M109 S220\nG1 X10\n')
        sd.auto_creator5_start = True
        self.attach_print_stats(sd)
        original_file = sd.current_file
        cleanup = []
        commands = []
        def dispatch(line):
            commands.append(line)
            if line == 'C5_PRINT_STOP':
                self.cancel_from_script(sd, cleanup)
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        self.assert_cancelled(sd, original_file, cleanup)
        self.assertIn('G1 X10', commands)
        self.assertNotIn(mock.call('Done printing file'),
                         sd.gcode.respond_raw.call_args_list)

    def test_automatic_end_error_can_cancel_cleanup(self):
        for error_type in (RuntimeError, ValueError):
            with self.subTest(error_type=error_type):
                sd = self.make_sd('M109 S220\nG1 X10\n')
                sd.auto_creator5_start = True
                self.attach_print_stats(sd)
                original_file = sd.current_file
                cleanup = []
                commands = []
                sd.on_error_gcode.render.return_value = 'CANCEL_PRINT'
                def dispatch(line):
                    commands.append(line)
                    if line == 'C5_PRINT_STOP':
                        raise error_type('stop failed')
                    if line == 'CANCEL_PRINT':
                        self.cancel_from_script(sd, cleanup)
                sd.gcode.run_script.side_effect = dispatch
                sd.work_handler(0.)
                self.assert_cancelled(sd, original_file, cleanup)
                self.assertIn('G1 X10', commands)
                self.assertNotIn(mock.call('Done printing file'),
                                 sd.gcode.respond_raw.call_args_list)

    def test_worker_scripts_wait_for_dispatcher(self):
        for blocked_script in ('start', 'end'):
            with self.subTest(blocked_script=blocked_script):
                sd = self.make_sd('M109 S220\nG1 X10\n')
                sd.auto_creator5_start = True
                self.attach_print_stats(sd)
                original_file = sd.current_file
                locked = [blocked_script == 'start']
                commands = []
                sd.gcode.get_mutex.return_value.test.side_effect = (
                    lambda: locked[0])
                def release_dispatcher(waketime):
                    locked[0] = False
                def dispatch(line):
                    if locked[0]:
                        raise sd.gcode.error('dispatcher is still occupied')
                    commands.append(line)
                    if line == 'G1 X10' and blocked_script == 'end':
                        locked[0] = True
                sd.reactor.pause.side_effect = release_dispatcher
                sd.gcode.run_script.side_effect = dispatch
                sd.work_handler(0.)
                self.assertTrue(original_file.closed)
                self.assertFalse(sd.is_active())
                self.assertIn('G1 X10', commands)
                self.assertEqual(commands.count('C5_PRINT_STOP'), 1)
                self.assertEqual(sd.print_stats.get_status(
                    sd.reactor.monotonic())['state'], 'complete')

    def test_end_macro_with_and_without_newline(self):
        for suffix in ('', '\n', '\r\n'):
            with self.subTest(suffix=suffix):
                text = '; unicode: café\nG1 X10\nC5_PRINT_STOP' + suffix
                sd = self.make_sd(text)
                sd.work_handler(0.)
                self.assertEqual([c.args[0].strip() for c in
                                  sd.gcode.run_script.call_args_list],
                                 ['; unicode: café', 'G1 X10', 'C5_PRINT_STOP'])
                self.assertEqual(sd.file_position, len(text.encode()))
                sd.print_stats.note_complete.assert_called_once()

    def test_pause_on_final_command_does_not_replay_it_on_resume(self):
        sd = self.make_sd('G1 X10\nPAUSE')
        def dispatch(line):
            if line == 'PAUSE':
                sd.do_pause()
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        sd.print_stats.note_pause.assert_called_once()
        self.assertEqual(sd.file_position, sd.file_size)
        sd.do_resume()
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_count, 2)
        sd.print_stats.note_complete.assert_called_once()

    def test_final_command_failure_runs_error_handler_not_completion(self):
        sd = self.make_sd('C5_PRINT_STOP')
        def dispatch(line):
            if line == 'C5_PRINT_STOP':
                raise RuntimeError('dock failed')
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        sd.gcode.run_script.assert_called_with('TURN_OFF_HEATERS')
        sd.print_stats.note_error.assert_called_once_with('dock failed')
        sd.print_stats.note_complete.assert_not_called()

    def test_unterminated_command_across_read_boundary(self):
        text = ';' + 'x' * 9000 + '\nC5_PRINT_STOP'
        sd = self.make_sd(text)
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_count, 2)
        sd.gcode.run_script.assert_called_with('C5_PRINT_STOP')
        self.assertEqual(sd.file_position, sd.file_size)

    def test_empty_file(self):
        sd = self.make_sd('')
        sd.work_handler(0.)
        sd.gcode.run_script.assert_not_called()
        sd.print_stats.note_complete.assert_called_once()

    def test_dense_file_yields_before_next_read(self):
        sd = self.make_sd('G1 X1\n' * 100)
        clock = iter(i * 0.002 for i in range(1000))
        sd.reactor.monotonic.side_effect = lambda: next(clock)
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_count, 100)
        # One yield is from reading the block; the rest must come while
        # dispatching the lines in that same block.
        self.assertGreater(sd.reactor.pause.call_count, 1)

    def test_automatic_start_runs_before_file_once(self):
        sd = self.make_sd('M140 S60\nM104 S220\nT2\nG1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(commands[0],
                         'C5_PRINT_START TOOL=2 HOTEND=220 BED=60 '
                         'MATERIAL0=UNKNOWN MATERIAL1=UNKNOWN '
                         'MATERIAL2=UNKNOWN MATERIAL3=UNKNOWN')
        self.assertEqual(commands[-2:], ['G1 X10', 'C5_PRINT_STOP'])
        self.assertEqual(sd.file_position, sd.file_size)

    def test_automatic_start_maps_all_materials_independent_of_initial_tool(self):
        for tool in range(4):
            with self.subTest(tool=tool):
                sd = self.make_sd(
                    '; filament_type = PLA;PETG;ABS;ASA\n'
                    'M109 S220\nT%d\nT0\nG1 X10\n' % tool)
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                commands = [c.args[0]
                            for c in sd.gcode.run_script.call_args_list]
                self.assertEqual(
                    commands[0],
                    'C5_PRINT_START TOOL=%d HOTEND=220 BED=0 '
                    'MATERIAL0=PLA MATERIAL1=PETG MATERIAL2=ABS MATERIAL3=ASA'
                    % tool)
                self.assertIn('G1 X10', commands)
                sd.print_stats.note_complete.assert_called_once()

    def test_automatic_start_defaults_invalid_materials_independently(self):
        metadata = (
            ('', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = \n',
             'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = PLA;PETG\n',
             'PLA', 'PETG', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = PLA;PETG; ;ABS\n',
             'PLA', 'PETG', 'UNKNOWN', 'ABS'),
            ('; filament_type = PLA;PETG;PLA HOTEND=300\n',
             'PLA', 'PETG', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = PLA;PETG;PLA#\n',
             'PLA', 'PETG', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = PLA;PETG;"PLA\n',
             'PLA', 'PETG', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = PLA;PETG;PLA"\n',
             'PLA', 'PETG', 'UNKNOWN', 'UNKNOWN'),
            ('; filament_type = ;PLA HOTEND=300;abs\n',
             'UNKNOWN', 'UNKNOWN', 'ABS', 'UNKNOWN'),
            ('; filament_type_extra = PLA;PETG;ABS\n',
             'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'),
            ('; default_filament_type = PLA;PETG;ABS\n',
             'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'),
            ('filament_type = PLA;PETG;ABS\n',
             'UNKNOWN', 'UNKNOWN', 'UNKNOWN', 'UNKNOWN'),
        )
        for value, material0, material1, material2, material3 in metadata:
            with self.subTest(metadata=value):
                sd = self.make_sd(value + 'M109 S220\nT2\nG1 X10\n')
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                self.assertEqual(
                    sd.gcode.run_script.call_args_list[0].args[0],
                    'C5_PRINT_START TOOL=2 HOTEND=220 BED=0 '
                    'MATERIAL0=%s MATERIAL1=%s MATERIAL2=%s MATERIAL3=%s'
                    % (material0, material1, material2, material3))
                sd.print_stats.note_complete.assert_called_once()

    def test_automatic_start_ignores_material_slots_beyond_four(self):
        for extra in (';PC', ';PLA HOTEND=999', ';;PETG;ABS'):
            with self.subTest(extra=extra):
                sd = self.make_sd(
                    '; filament_type = PLA;PETG;ABS;ASA%s\n'
                    'M109 S220\nT2\nG1 X10\n' % extra)
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                self.assertEqual(
                    sd.gcode.run_script.call_args_list[0].args[0],
                    'C5_PRINT_START TOOL=2 HOTEND=220 BED=0 '
                    'MATERIAL0=PLA MATERIAL1=PETG MATERIAL2=ABS MATERIAL3=ASA')

    def test_automatic_start_normalizes_quoted_material_slots(self):
        for value in ('" petg-cf "', "' petg-cf '"):
            with self.subTest(value=value):
                sd = self.make_sd(
                    '; filament_type = pla; %s ;pa6+gf;abs\n'
                    'M109 S220\nT1\nG1 X10\n' % value)
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                self.assertEqual(
                    sd.gcode.run_script.call_args_list[0].args[0],
                    'C5_PRINT_START TOOL=1 HOTEND=220 BED=0 '
                    'MATERIAL0=PLA MATERIAL1=PETG-CF '
                    'MATERIAL2=PA6+GF MATERIAL3=ABS')

    def test_automatic_start_uses_authoritative_footer_beyond_header(self):
        text = ('; filament_type = PLA;PLA;PLA;PLA\n'
                'M109 S220\nT2\nG1 X10\n;'
                + 'x' * 70000
                + '\n; CONFIG_BLOCK_START\n'
                '; filament_type = ABS;PETG;PA6+GF;ASA\n'
                '; CONFIG_BLOCK_END\nG1 X20\n')
        sd = self.make_sd(text)
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(
            commands[0],
            'C5_PRINT_START TOOL=2 HOTEND=220 BED=0 '
            'MATERIAL0=ABS MATERIAL1=PETG MATERIAL2=PA6+GF MATERIAL3=ASA')
        self.assertEqual(commands.count('G1 X10'), 1)
        self.assertEqual(commands.count('G1 X20'), 1)
        self.assertEqual(commands[-1], 'C5_PRINT_STOP')
        self.assertEqual(sd.file_position, sd.file_size)
        sd.print_stats.note_complete.assert_called_once()

    def test_footer_material_overrides_prefix_even_when_windows_overlap(self):
        for value, material in (('PETG', 'PETG'), ('', 'UNKNOWN'),
                                ('PLA HOTEND=999', 'UNKNOWN')):
            with self.subTest(value=value):
                sd = self.make_sd(
                    '; filament_type = PLA\nM109 S220\nG1 X10\n'
                    '; CONFIG_BLOCK_START\n; filament_type = %s'
                    % value)
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                self.assertEqual(
                    sd.gcode.run_script.call_args_list[0].args[0],
                    'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 MATERIAL0='
                    + material + ' MATERIAL1=UNKNOWN MATERIAL2=UNKNOWN '
                    'MATERIAL3=UNKNOWN')
                sd.print_stats.note_complete.assert_called_once()

    def test_truncated_header_material_does_not_select_partial_token(self):
        start = 'M109 S220\nG1 X10\n;'
        metadata = '\n; filament_type = PLA'
        text = (start + 'x' * (65536 - len(start) - len(metadata))
                + metadata + '-CF\n;' + 'x' * 70000)
        sd = self.make_sd(text)
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(
            sd.gcode.run_script.call_args_list[0].args[0],
            'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 '
            'MATERIAL0=UNKNOWN MATERIAL1=UNKNOWN '
            'MATERIAL2=UNKNOWN MATERIAL3=UNKNOWN')
        sd.print_stats.note_complete.assert_called_once()

    def test_footer_material_requires_complete_line_at_read_boundary(self):
        metadata = '; filament_type = ABS\n'
        tail = metadata + ';' + 'x' * (65536 - len(metadata.encode()) - 1)
        for line_start, material in (('\n', 'ABS'), ('x', 'PLA')):
            with self.subTest(line_start=line_start):
                text = ('; filament_type = PLA\nM109 S220\nG1 X10\n;'
                        + 'x' * 70000 + line_start + tail)
                sd = self.make_sd(text)
                sd.auto_creator5_start = True
                sd.work_handler(0.)
                self.assertEqual(
                    sd.gcode.run_script.call_args_list[0].args[0],
                    'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 MATERIAL0='
                    + material + ' MATERIAL1=UNKNOWN MATERIAL2=UNKNOWN '
                    'MATERIAL3=UNKNOWN')
                sd.print_stats.note_complete.assert_called_once()

    def test_automatic_start_handles_utf8_split_tail_and_rewinds_stream(self):
        tail = '\n; filament_type = PETG\nG1 X20\n'
        suffix = 'y' * (65535 - len(tail.encode())) + tail
        comment = ';' + 'x' * 70000 + 'é' + suffix.split('\n', 1)[0]
        text = 'M109 S220\nG1 X10\n' + comment + tail
        sd = self.make_sd(text)
        sd.auto_creator5_start = True
        def check_rewind(line):
            if line.startswith('C5_PRINT_START '):
                self.assertEqual(sd.current_file.tell(), 0)
        sd.gcode.run_script.side_effect = check_rewind
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(
            commands[0],
            'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 MATERIAL0=PETG '
            'MATERIAL1=UNKNOWN MATERIAL2=UNKNOWN MATERIAL3=UNKNOWN')
        self.assertEqual(commands.count(comment), 1)
        self.assertEqual(commands.count('G1 X10'), 1)
        self.assertEqual(commands.count('G1 X20'), 1)
        self.assertEqual(sd.file_position, len(text.encode()))
        sd.print_stats.note_complete.assert_called_once()

    def test_explicit_start_ignores_malformed_material_and_keeps_preloading(self):
        define = ('EXCLUDE_OBJECT_DEFINE NAME=cube '
                  'POLYGON=[[10,10],[20,10],[20,20],[10,20]]')
        start = 'C5_PRINT_START TOOL=1 HOTEND=210 BED=50'
        text = (define + '\n' + start + '\nG1 X10\n;'
                + 'x' * 70000
                + '\n; filament_type = "PLA HOTEND=999\n')
        sd = self.make_sd(text)
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(commands[:3], [
            'EXCLUDE_OBJECT_DEFINE RESET=1', define, start])
        self.assertEqual(commands.count(define), 1)
        self.assertEqual(
            [line for line in commands if line.startswith('C5_PRINT_START')],
            [start])
        self.assertFalse(any(line.startswith('_SET_MPC_MATERIAL')
                             for line in commands))
        self.assertIn('G1 X10', commands)
        sd.print_stats.note_complete.assert_called_once()

    def test_automatic_start_passes_first_layer_height(self):
        sd = self.make_sd('; first_layer_height = 0.08\nM104 S220\nG1 X10')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 '
                         'FIRST_LAYER_HEIGHT=0.08 MATERIAL0=UNKNOWN '
                         'MATERIAL1=UNKNOWN MATERIAL2=UNKNOWN MATERIAL3=UNKNOWN')

    def test_automatic_start_prefers_print_wait_over_early_preheat(self):
        sd = self.make_sd('M104 S300\nM109 S220\nG1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 '
                         'MATERIAL0=UNKNOWN MATERIAL1=UNKNOWN '
                         'MATERIAL2=UNKNOWN MATERIAL3=UNKNOWN')

    def test_existing_start_is_not_duplicated(self):
        sd = self.make_sd('C5_PRINT_START TOOL=1 HOTEND=210 BED=50\nG1 X10')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'C5_PRINT_START TOOL=1 HOTEND=210 BED=50')
        self.assertEqual(sd.gcode.run_script.call_args_list[-1].args[0],
                         'C5_PRINT_STOP')
        self.assertEqual(sd.gcode.run_script.call_count, 3)

    def test_existing_end_is_not_run_twice(self):
        sd = self.make_sd('M109 S220\nG1 X10\nC5_PRINT_STOP')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(commands.count('C5_PRINT_STOP'), 1)
        sd.print_stats.note_complete.assert_called_once()

    def test_automatic_end_failure_marks_print_error(self):
        sd = self.make_sd('M109 S220\nG1 X10')
        sd.auto_creator5_start = True
        def dispatch(line):
            if line == 'C5_PRINT_STOP':
                raise RuntimeError('dock failed')
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        sd.print_stats.note_error.assert_called_once_with('dock failed')
        sd.print_stats.note_complete.assert_not_called()
        sd.gcode.run_script.assert_called_with('TURN_OFF_HEATERS')
        sd.gcode.respond_raw.assert_not_called()

    def test_paused_job_runs_end_only_after_resuming_to_eof(self):
        sd = self.make_sd('M109 S220\nG1 X10\nPAUSE')
        sd.auto_creator5_start = True
        def dispatch(line):
            if line == 'PAUSE':
                sd.do_pause()
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertNotIn('C5_PRINT_STOP', commands)
        sd.print_stats.note_pause.assert_called_once()
        sd.do_resume()
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(commands.count('C5_PRINT_STOP'), 1)
        sd.print_stats.note_complete.assert_called_once()

    def test_cancel_does_not_run_normal_end(self):
        sd = self.make_sd('M109 S220\nG1 X10\nG1 X20')
        sd.auto_creator5_start = True
        def dispatch(line):
            if line == 'G1 X10':
                sd.do_cancel()
        sd.gcode.run_script.side_effect = dispatch
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertNotIn('C5_PRINT_STOP', commands)
        sd.print_stats.note_cancel.assert_called_once()
        sd.print_stats.note_complete.assert_not_called()

    def test_slicer_z_offset_cannot_override_calibrated_nozzle(self):
        sd = self.make_sd('M109 S220\nSET_GCODE_OFFSET Z=0 MOVE=1\n'
                          'SET_GCODE_OFFSET Z_ADJUST=-1\nG92 Z0\n'
                          'G92 E0\nG1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertTrue(commands[0].startswith('C5_PRINT_START '))
        self.assertEqual(commands[1:], [
            'M109 S220', 'G92 E0', 'G1 X10', 'C5_PRINT_STOP'])
        self.assertEqual(sd.file_position, sd.file_size)

    def test_slicer_z_offset_fast_path_keeps_exact_matching(self):
        check = MODULE.creator5_slicer_z_offset
        self.assertFalse(check('G1 X12 Y20 E0.4'))
        self.assertFalse(check('  G92 E0 ; reset extruder'))
        self.assertTrue(check('  g92 z0 ; slicer origin'))
        self.assertTrue(check(' set_gcode_offset Z_ADJUST=-0.1'))
        self.assertFalse(check('SET_GCODE_OFFSET X=1'))
        self.assertFalse(check('; SET_GCODE_OFFSET Z=0'))

    def test_object_polygons_are_available_before_adaptive_start_once(self):
        define = ('EXCLUDE_OBJECT_DEFINE NAME=cube '
                  'POLYGON=[[10,10],[20,10],[20,20],[10,20]]')
        sd = self.make_sd('; résumé\n' + define + '\nM109 S220\nG1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        commands = [c.args[0] for c in sd.gcode.run_script.call_args_list]
        self.assertEqual(commands[:2], [
            'EXCLUDE_OBJECT_DEFINE RESET=1', define])
        self.assertTrue(commands[2].startswith('C5_PRINT_START '))
        self.assertEqual(commands.count(define), 1)
        self.assertEqual(commands[-3:], ['M109 S220', 'G1 X10',
                                         'C5_PRINT_STOP'])
        self.assertEqual(sd.file_position, sd.file_size)

    def test_partial_object_definition_is_not_preloaded(self):
        header = (';' + 'x' * 65500 + '\n'
                  'EXCLUDE_OBJECT_DEFINE NAME=x POLYGON=[')[:65536]
        self.assertEqual(MODULE.creator5_object_definitions(header), [])

    def test_missing_temperature_stops_before_motion(self):
        sd = self.make_sd('G1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        sd.print_stats.note_error.assert_called_once()
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'TURN_OFF_HEATERS')


if __name__ == '__main__':
    unittest.main()
