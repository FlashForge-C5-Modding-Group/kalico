import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'creator5_filament_sync', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'creator5_filament_sync.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **values):
        self.values = values
        self.messages = []

    def get(self, name, default=None):
        return self.values.get(name, default)

    def get_int(self, name, default=None, **kwargs):
        value = self.values.get(name, default)
        if value is None:
            raise ValueError('missing ' + name)
        return int(value)

    def error(self, message):
        return ValueError(message)

    def respond_info(self, message):
        self.messages.append(message)


class Creator5FilamentSyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.sync = MODULE.Creator5FilamentSync.__new__(
            MODULE.Creator5FilamentSync)
        self.sync.filename = str(Path(self.temp.name) / 'c5_filaments.json')
        self.sync.moonraker_url = 'http://127.0.0.1:7125'
        self.sync.filaments = [MODULE.empty_filament() for _ in range(4)]
        self.sync.printer = mock.Mock(config_error=ValueError)
        self.sync._queue_publish = mock.Mock()

    def test_set_and_clear_persist_four_tool_slots(self):
        self.sync.set_tool_fields(2, material='pctg', color='aabbcc',
                                  nozzle_temp=255, bed_temp=80)
        self.assertEqual(self.sync.filaments[2], {
            'material': 'PCTG', 'color': '#AABBCC',
            'nozzle_temp': 255, 'bed_temp': 80, 'spool_id': None})
        self.assertEqual(self.sync._load(), self.sync.filaments)
        self.assertEqual(len(json.loads(Path(self.sync.filename).read_text())),
                         4)
        self.sync.clear_tool(2)
        self.assertEqual(self.sync._load()[2], MODULE.empty_filament())
        self.assertEqual(self.sync._queue_publish.call_count, 2)

    def test_invalid_color_does_not_change_saved_inventory(self):
        with self.assertRaisesRegex(ValueError, 'Color'):
            self.sync.set_tool_fields(0, material='PLA', color='red')
        self.assertFalse(Path(self.sync.filename).exists())
        self.assertEqual(self.sync.filaments[0], MODULE.empty_filament())

    def test_publishes_orca_lane_data_for_each_tool(self):
        self.sync.filaments[1] = {
            'material': 'PETG', 'color': '#123456',
            'nozzle_temp': 240, 'bed_temp': 75}
        requests = []
        response = mock.MagicMock()
        response.__enter__.return_value.status = 200
        with mock.patch.object(MODULE, 'urlopen', side_effect=lambda req, **kw:
                               requests.append(req) or response):
            self.sync._publish(self.sync.filaments)
        self.assertEqual(len(requests), 4)
        payload = json.loads(requests[1].data)
        self.assertEqual(payload['namespace'], 'lane_data')
        self.assertEqual(payload['key'], 'c5_tool1')
        self.assertEqual(payload['value']['lane'], '1')
        self.assertEqual(payload['value']['material'], 'PETG')
        self.assertEqual(payload['value']['color'], '#123456')


if __name__ == '__main__':
    unittest.main()
