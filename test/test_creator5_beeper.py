import importlib.util
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = (Path(__file__).resolve().parents[1] / 'klippy' / 'extras'
               / 'creator5_beeper.py')
SPEC = importlib.util.spec_from_file_location('creator5_beeper', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **params):
        self.params = params
        self.messages = []

    def get_float(self, name, default, **kwargs):
        return self.params.get(name, default)

    def get_int(self, name, default, **kwargs):
        return self.params.get(name, default)

    def respond_info(self, message):
        self.messages.append(message)


class Creator5BeeperTest(unittest.TestCase):
    def setUp(self):
        self.beeper = MODULE.Creator5Beeper.__new__(MODULE.Creator5Beeper)
        self.beeper.command = 'cmd_pwm'
        self.beeper.channel = 'pc12'
        self.beeper._configured = False
        self.beeper._config_error = None
        self.beeper.printer = mock.Mock(command_error=RuntimeError)
        self.beeper.executor = mock.Mock()
        self.beeper.executor.submit.side_effect = (
            lambda function, *args, **kwargs: function(*args, **kwargs))
        self.beeper.reactor = mock.Mock()
        self.beeper.reactor.monotonic.return_value = 10.

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_uses_factory_host_pwm_and_turns_off_after_five_seconds(self, run):
        run.return_value = mock.Mock(returncode=0, stderr=b'')
        self.beeper.cmd_C5_BUZZER(GCmd())
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[0], ['cmd_pwm', 'disable', 'pc12'])
        self.assertEqual(commands[1], [
            'cmd_pwm', 'config', 'pc12', 'freq=50000000',
            'max_level=300', 'active_level=1', 'accuracy_priority=freq'])
        self.assertIn(['cmd_pwm', 'set_wc', 'pc12', '10000', '10000'],
                      commands)
        self.assertEqual(commands[-2:], [
            ['cmd_pwm', 'set_wc', 'pc12', '1', '0'],
            ['cmd_pwm', 'disable_channels', 'pc12']])
        self.beeper.reactor.pause.assert_called_once_with(15.)

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_duration_ms_frequency_and_level_are_configurable(self, run):
        run.return_value = mock.Mock(returncode=0, stderr=b'')
        self.beeper.cmd_C5_BUZZER(GCmd(DURATION=250, FREQUENCY=1000,
                                       LEVEL=75))
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn(['cmd_pwm', 'set_wc', 'pc12', '18750', '31250'],
                      commands)
        self.beeper.reactor.pause.assert_called_once_with(10.25)

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_reuses_configuration_for_later_notes(self, run):
        run.return_value = mock.Mock(returncode=0, stderr=b'')
        self.beeper.cmd_C5_BUZZER(GCmd(DURATION=100))
        self.beeper.cmd_C5_BUZZER(GCmd(DURATION=100, FREQUENCY=1000))
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(sum(cmd[1] == 'config' for cmd in commands), 1)
        self.assertEqual(sum(cmd[1] == 'disable' for cmd in commands), 1)
        self.assertEqual(sum(cmd[1] == 'enable_channels' for cmd in commands),
                         2)
        self.assertEqual(sum(cmd[1] == 'disable_channels' for cmd in commands),
                         2)

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_enable_failure_still_disables_output(self, run):
        def execute(args, **kwargs):
            return mock.Mock(returncode=int(args[1] == 'enable_channels'),
                             stderr=b'enable failed')
        run.side_effect = execute
        with self.assertRaisesRegex(RuntimeError, 'enable failed'):
            self.beeper.cmd_C5_BUZZER(GCmd())
        self.assertEqual(run.call_args.args[0],
                         ['cmd_pwm', 'disable_channels', 'pc12'])
        self.beeper.reactor.pause.assert_not_called()
        self.beeper.executor.submit.assert_called()

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_turns_off_if_wait_is_interrupted(self, run):
        run.return_value = mock.Mock(returncode=0, stderr=b'')
        self.beeper.reactor.pause.side_effect = RuntimeError('interrupted')
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            self.beeper.cmd_C5_BUZZER(GCmd())
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[-1],
                         ['cmd_pwm', 'disable_channels', 'pc12'])

    @mock.patch.object(MODULE.subprocess, 'run', side_effect=FileNotFoundError())
    def test_missing_host_utility_reports_error(self, run):
        with self.assertRaisesRegex(RuntimeError,
                                    'Creator 5 buzzer command failed'):
            self.beeper.cmd_C5_BUZZER(GCmd())

    @mock.patch.object(MODULE.subprocess, 'run', side_effect=FileNotFoundError())
    def test_optional_tune_does_not_abort_print_if_utility_missing(self, run):
        gcmd = GCmd(OPTIONAL=1)
        self.beeper.cmd_C5_BUZZER(gcmd)
        self.assertIn('tune skipped', gcmd.messages[0])

    @mock.patch.object(MODULE.subprocess, 'run')
    def test_config_failure_is_not_retried_for_later_notes(self, run):
        # On hardware, retrying disable/config after a failed attempt hangs
        # instead of failing fast, so a broken channel must only be touched
        # once per session (e.g. across a multi-note startup tune).
        run.return_value = mock.Mock(returncode=1, stderr=b'not permitted')
        first = GCmd(OPTIONAL=1)
        second = GCmd(OPTIONAL=1)
        self.beeper.cmd_C5_BUZZER(first)
        self.beeper.cmd_C5_BUZZER(second)
        self.assertEqual(run.call_args_list, [
            mock.call(['cmd_pwm', 'disable', 'pc12'], stdin=mock.ANY,
                      stdout=mock.ANY, stderr=mock.ANY, check=False,
                      timeout=3)])
        self.assertIn('tune skipped', first.messages[0])
        self.assertIn('tune skipped', second.messages[0])


if __name__ == '__main__':
    unittest.main()
