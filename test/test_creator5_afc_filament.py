import importlib.util
import unittest
from pathlib import Path
from unittest import mock


SPEC = importlib.util.spec_from_file_location(
    'AFC_spool', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'AFC_spool.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GCmd:
    def __init__(self, **values):
        self.values = values

    def get(self, name, default=None):
        return self.values.get(name, default)

    def get_int(self, name, default=None, **kwargs):
        return int(self.values.get(name, default))

    def error(self, message):
        return ValueError(message)


class Creator5AFCFilamentTests(unittest.TestCase):
    def setUp(self):
        self.spool = MODULE.AFCSpool.__new__(MODULE.AFCSpool)
        self.spool.creator5_tool_lanes = {'T2': 2}
        self.spool.creator5_filament_sync = mock.Mock()
        self.spool.creator5_filament_sync.filaments = [
            {'material': '', 'color': '', 'bed_temp': 0,
             'nozzle_temp': 0} for _ in range(4)]
        self.spool.afc = mock.Mock(lanes={})

    def test_afc_color_material_and_temperatures_target_standalone_tool(self):
        self.spool.cmd_SET_COLOR(GCmd(LANE='T2', COLOR='3366CC'))
        self.spool.cmd_SET_MATERIAL(GCmd(LANE='T2', MATERIAL='PETG'))
        self.spool.cmd_AFC_SET_SPOOL_TEMP(
            GCmd(LANE='T2', BED_TEMP=80, EXTRUDER_TEMP=245))
        self.assertEqual(
            self.spool.creator5_filament_sync.set_tool_fields.call_args_list,
            [mock.call(2, color='3366CC', spool_id=None),
             mock.call(2, material='PETG', spool_id=None),
             mock.call(2, bed_temp=80, nozzle_temp=245)])

    def test_afc_clear_targets_standalone_tool(self):
        self.spool.cmd_AFC_CLEAR_FILAMENT(GCmd(LANE='T2'))
        self.spool.creator5_filament_sync.clear_tool.assert_called_once_with(2)

    def test_ready_registers_afc_commands_for_standalone_tools(self):
        tool = mock.Mock(creator5_tool_index=2)
        self.spool.afc.tools = {'extruder2': tool}
        self.spool.gcode = mock.Mock()
        self.spool._register_creator5_tool_macros()
        self.assertEqual(self.spool.creator5_tool_lanes, {'T2': 2})
        self.assertIs(tool.creator5_filament_sync,
                      self.spool.creator5_filament_sync)
        self.assertEqual(self.spool.gcode.register_mux_command.call_count, 5)
        names = [call.args[0] for call in
                 self.spool.gcode.register_mux_command.call_args_list]
        self.assertEqual(names, ['SET_COLOR', 'SET_MATERIAL',
                                 'AFC_SET_SPOOL_TEMP', 'SET_SPOOL_ID',
                                 'AFC_CLEAR_FILAMENT'])

    def test_spoolman_assignment_uses_afc_spool_data(self):
        self.spool.afc.spoolman = 'http://spoolman'
        self.spool.afc.moonraker.get_spool.return_value = {
            'filament': {'material': 'ASA', 'color_hex': 'AABBCC',
                         'settings_extruder_temp': 255,
                         'settings_bed_temp': 95}}
        self.spool.cmd_SET_SPOOL_ID(GCmd(LANE='T2', SPOOL_ID='42'))
        self.spool.afc.moonraker.get_spool.assert_called_once_with(42)
        self.spool.creator5_filament_sync.set_tool_fields.assert_called_once_with(
            2, spool_id=42, material='ASA', color='AABBCC',
            nozzle_temp=255, bed_temp=95)


if __name__ == '__main__':
    unittest.main()
