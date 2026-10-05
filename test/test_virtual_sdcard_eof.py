import importlib.util
import io
from pathlib import Path
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'virtual_sdcard', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'virtual_sdcard.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


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
        sd.reactor = mock.Mock(NOW=0., NEVER=999.)
        sd.reactor.monotonic.return_value = 0.
        sd.gcode = mock.Mock()
        sd.gcode.error = RuntimeError
        sd.gcode.get_mutex.return_value.test.return_value = False
        sd.print_stats = mock.Mock()
        sd.on_error_gcode = mock.Mock()
        sd.on_error_gcode.render.return_value = 'TURN_OFF_HEATERS'
        return sd

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
                         'C5_PRINT_START TOOL=2 HOTEND=220 BED=60')
        self.assertEqual(commands[-2:], ['G1 X10', 'C5_PRINT_STOP'])
        self.assertEqual(sd.file_position, sd.file_size)

    def test_automatic_start_passes_first_layer_height(self):
        sd = self.make_sd('; first_layer_height = 0.08\nM104 S220\nG1 X10')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'C5_PRINT_START TOOL=0 HOTEND=220 BED=0 '
                         'FIRST_LAYER_HEIGHT=0.08')

    def test_automatic_start_prefers_print_wait_over_early_preheat(self):
        sd = self.make_sd('M104 S300\nM109 S220\nG1 X10\n')
        sd.auto_creator5_start = True
        sd.work_handler(0.)
        self.assertEqual(sd.gcode.run_script.call_args_list[0].args[0],
                         'C5_PRINT_START TOOL=0 HOTEND=220 BED=0')

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
        self.assertEqual(commands, [
            'C5_PRINT_START TOOL=0 HOTEND=220 BED=0', 'M109 S220',
            'G92 E0', 'G1 X10', 'C5_PRINT_STOP'])
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
        self.assertEqual(commands[:3], [
            'EXCLUDE_OBJECT_DEFINE RESET=1', define,
            'C5_PRINT_START TOOL=0 HOTEND=220 BED=0'])
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
