import importlib.util
import logging
from pathlib import Path
import tempfile
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'queuelogger', Path(__file__).resolve().parents[1]
    / 'klippy' / 'queuelogger.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class QueueLoggerRolloverTests(unittest.TestCase):
    def test_banner_does_not_reenter_rotating_emit(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / 'printer.log'
            listener = MODULE.QueueListener(str(log_path), False)
            try:
                listener.set_rollover_info('config', 'saved config')
                listener.setFormatter(logging.Formatter('%(message)s'))
                with mock.patch.object(listener, 'shouldRollover',
                                       return_value=True):
                    listener.doRollover()
                self.assertIn('saved config', log_path.read_text())
                self.assertIn('Log rollover', log_path.read_text())
            finally:
                listener.stop()
                listener.close()

    def test_stale_rollover_clock_does_not_repeat_same_day(self):
        with tempfile.TemporaryDirectory() as directory:
            listener = MODULE.QueueListener(
                str(Path(directory) / 'printer.log'), False)
            try:
                listener.doRollover()
                listener.rolloverAt = 0
                record = logging.makeLogRecord({'msg': 'test'})
                self.assertFalse(listener.shouldRollover(record))
            finally:
                listener.stop()
                listener.close()


if __name__ == '__main__':
    unittest.main()
