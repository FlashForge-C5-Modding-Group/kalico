"""Exercise Creator 5's saved AFC alias through the real PREP restore path."""

import copy
import importlib.util
import json
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

SPEC = importlib.util.spec_from_file_location(
    'AFC_prep', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'AFC_prep.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def saved_state():
    return {
        'Heads': {
            'extruder': {
                'name': 'extruder', 'unit': 'Heads', 'hub': 'direct',
                'extruder': 'extruder', 'buffer': None, 'buffer_status': None,
                'lane': 0, 'map': 'T2', 'load': True, 'prep': True,
                'tool_loaded': True, 'loaded_to_hub': True,
                'material': 'ASA', 'density': 1.07, 'diameter': 1.75,
                'empty_spool_weight': 230, 'need_purge': True,
                'remember_spool': True, 'spool_id': 42, 'color': 'AABBCC',
                'weight': 610, 'extruder_temp': 255, 'bed_temp': 95,
                'runout_lane': 'extruder2', 'filament_status': 'Loaded',
                'filament_status_led': 'ready', 'status': 'Tooled',
                'dist_hub': 0, 'td1_data': {'td': 2.3},
            },
        },
        'Backup': {
            'extruder2': {
                'spool_id': 84, 'material': 'PETG', 'color': '112233',
                'map': 'T0', 'runout_lane': 'extruder',
                'tool_loaded': False, 'loaded_to_hub': True,
            },
        },
        'system': {
            'current_load': 'extruder', 'num_units': 2, 'num_lanes': 2,
            'num_extruders': 2, 'bypass': {'enabled': False},
            'extruders': {
                'extruder': {'lane_loaded': 'extruder'},
                'extruder2': {'lane_loaded': 'extruder'},
            },
        },
    }


def lane(name, unit, extruder):
    return types.SimpleNamespace(
        name=name, unit=unit, standalone_lane=True, extruder_obj=extruder,
        spool_id=None, material='', color='', weight=0, filament_density=1.24,
        filament_diameter=1.75, empty_spool_weight=0, bed_temp=0,
        extruder_temp=0, need_purge=False, runout_lane=None, map=None,
        loaded_to_hub=False, tool_loaded=False, td1_data={},
        multi_hubs_found=False)


class Creator5AFCStateTests(unittest.TestCase):
    def setUp(self):
        self.prep = MODULE.afcPrep.__new__(MODULE.afcPrep)
        self.prep.printer = types.SimpleNamespace(
            state_message='Printer is ready')
        self.prep.logger = mock.Mock()
        self.prep._rename_macros = mock.Mock()
        self.prep._td1_prep = mock.Mock()
        self.prep.delay = 0.1
        self.prep.assignTcmd = True
        self.prep.enable = False
        self.tool0 = types.SimpleNamespace(
            name='extruder0', creator5_tool_index=0, lane_loaded=None,
            set_status_led=mock.Mock(), on_shuttle=mock.Mock(return_value=False),
            estats=mock.Mock())
        self.tool2 = types.SimpleNamespace(
            name='extruder2', creator5_tool_index=2, lane_loaded=None,
            set_status_led=mock.Mock(), on_shuttle=mock.Mock(return_value=False),
            estats=mock.Mock())
        self.lane0 = lane('extruder0', 'Heads', self.tool0)
        self.lane2 = lane('extruder2', 'Backup', self.tool2)
        self.tool0.tc_lane = self.lane0
        self.tool2.tc_lane = self.lane2
        self.afc = types.SimpleNamespace(
            tools={'extruder': self.tool0, 'extruder2': self.tool2},
            lanes={'extruder0': self.lane0, 'extruder2': self.lane2},
            units={}, handle_moonraker_connect=mock.Mock(),
            led_tool_unloaded='off', snapmaker_printer=False, spoolman=None,
            tool_cmds={}, function=mock.Mock(), spool=mock.Mock(),
            get_bypass_state=mock.Mock(return_value=False),
            bypass=types.SimpleNamespace(name='bypass', filament_present=False),
            current=None, prep_done=False, buffers={},
            verify_macro_positions=mock.Mock(return_value=''),
            save_vars=mock.Mock(), error=mock.Mock())
        self.afc.function.get_current_lane_obj.return_value = None
        self.prep.afc = self.afc

    def restore(self, state):
        self.afc.units = {}
        for current_lane in self.afc.lanes.values():
            if current_lane.unit not in self.afc.units:
                self.afc.units[current_lane.unit] = types.SimpleNamespace(
                    name=current_lane.unit, type='Toolchanger', lanes={},
                    system_Test=mock.Mock(return_value=True),
                    logo='', logo_error='')
            self.afc.units[current_lane.unit].lanes[current_lane.name] = current_lane
        parsed = []
        original_load = json.load

        def load(stream):
            result = original_load(stream)
            parsed.append(result)
            return result

        with tempfile.TemporaryDirectory() as directory:
            self.afc.VarFile = str(Path(directory) / 'AFC')
            state_file = Path(self.afc.VarFile + '.unit')
            state_file.write_text(json.dumps(state), encoding='utf-8')
            with mock.patch.object(MODULE.json, 'load', side_effect=load):
                self.prep.PREP(None)
            # Migration must not write the file before the existing normal save.
            self.assertEqual(json.loads(state_file.read_text(encoding='utf-8')),
                             state)
        self.assertTrue(self.afc.prep_done)
        self.afc.save_vars.assert_called_once_with()
        self.afc.error.AFC_error.assert_not_called()
        return parsed[0]

    def test_legacy_metadata_mapping_and_loaded_state_restore_before_save(self):
        state = saved_state()
        restored_at_save = []
        self.afc.save_vars.side_effect = lambda: restored_at_save.append(
            (self.lane0.spool_id, self.lane0.map, self.tool0.lane_loaded,
             self.lane2.runout_lane, self.afc.prep_done))

        migrated = self.restore(state)

        self.assertNotIn('extruder', migrated['Heads'])
        self.assertEqual(migrated['Heads']['extruder0'], state['Heads']['extruder'])
        self.assertNotIn('extruder', migrated['system']['extruders'])
        self.assertEqual(self.lane0.spool_id, 42)
        self.assertEqual(self.lane0.material, 'ASA')
        self.assertEqual(self.lane0.color, 'AABBCC')
        self.assertEqual(self.lane0.weight, 610)
        self.assertEqual(self.lane0.filament_density, 1.07)
        self.assertEqual(self.lane0.filament_diameter, 1.75)
        self.assertEqual(self.lane0.empty_spool_weight, 230)
        self.assertEqual(self.lane0.bed_temp, 95)
        self.assertEqual(self.lane0.extruder_temp, 255)
        self.assertTrue(self.lane0.need_purge)
        self.assertTrue(self.lane0.loaded_to_hub)
        self.assertTrue(self.lane0.tool_loaded)
        self.assertEqual(self.lane0.td1_data, {'td': 2.3})
        self.assertEqual(self.lane0.runout_lane, 'extruder2')
        self.assertEqual(self.afc.tool_cmds, {'T2': 'extruder0', 'T0': 'extruder2'})
        self.assertEqual(self.tool0.lane_loaded, 'extruder0')
        self.assertEqual(self.tool2.lane_loaded, 'extruder0')
        self.assertEqual(self.lane2.runout_lane, 'extruder0')
        self.assertEqual(restored_at_save,
                         [(42, 'T2', 'extruder0', 'extruder0', True)])

    def test_existing_alias_records_win_without_merging_legacy_metadata(self):
        state = saved_state()
        target = {'spool_id': 99, 'material': 'PLA', 'map': 'T3',
                  'runout_lane': 'extruder2'}
        target_extruder = {'lane_loaded': 'extruder2', 'new_metadata': 'keep'}
        state['Heads']['extruder0'] = target
        state['system']['extruders']['extruder0'] = target_extruder

        migrated = self.restore(state)

        self.assertEqual(migrated['Heads']['extruder0'], target)
        self.assertEqual(migrated['Heads']['extruder'], state['Heads']['extruder'])
        self.assertEqual(migrated['system']['extruders']['extruder0'],
                         target_extruder)
        self.assertEqual(self.lane0.spool_id, 99)
        self.assertEqual(self.lane0.material, 'PLA')
        self.assertEqual(self.lane0.map, 'T3')
        self.assertEqual(self.tool0.lane_loaded, 'extruder2')
        self.assertEqual(self.lane2.runout_lane, 'extruder0')
        self.assertEqual(self.prep.logger.warning.call_count, 2)

    def test_lane_and_extruder_records_migrate_independently(self):
        for existing in ('lane', 'extruder'):
            with self.subTest(existing=existing):
                self.setUp()
                state = saved_state()
                if existing == 'lane':
                    state['Heads']['extruder0'] = {'spool_id': 99}
                else:
                    state['system']['extruders']['extruder0'] = {
                        'lane_loaded': 'extruder2'}

                migrated = self.restore(state)

                self.assertEqual(self.lane0.spool_id,
                                 99 if existing == 'lane' else 42)
                self.assertEqual(self.tool0.lane_loaded,
                                 'extruder0' if existing == 'lane' else 'extruder2')
                self.assertIn('extruder0', migrated['Heads'])
                self.assertIn('extruder0', migrated['system']['extruders'])

    def test_migration_is_idempotent(self):
        migrated = self.restore(saved_state())
        self.setUp()

        repeated = self.restore(copy.deepcopy(migrated))

        self.assertEqual(repeated, migrated)
        self.assertEqual(self.lane0.spool_id, 42)
        self.assertEqual(self.tool0.lane_loaded, 'extruder0')
        self.assertEqual(self.lane2.runout_lane, 'extruder0')

    def test_stale_references_are_fixed_without_legacy_records(self):
        state = saved_state()
        state['Heads']['extruder0'] = state['Heads'].pop('extruder')
        state['system']['extruders']['extruder0'] = (
            state['system']['extruders'].pop('extruder'))

        migrated = self.restore(state)

        self.assertEqual(migrated['Heads']['extruder0'],
                         state['Heads']['extruder0'])
        self.assertEqual(self.tool0.lane_loaded, 'extruder0')
        self.assertEqual(self.tool2.lane_loaded, 'extruder0')
        self.assertEqual(self.lane2.runout_lane, 'extruder0')
        self.prep.logger.warning.assert_not_called()

    def test_fresh_and_unrelated_partial_state_is_unchanged(self):
        for state in ({}, {'system': {}}, {'Heads': {}},
                      {'Other': {'extruder': {'spool_id': 7}}}):
            with self.subTest(state=state):
                self.setUp()
                self.assertEqual(self.restore(state), state)
                self.assertIsNone(self.lane0.spool_id)

    def test_only_creator5_physical_tool_zero_alias_is_migrated(self):
        for mismatch in ('other_printer', 'other_index', 'old_name',
                         'missing_physical_tool', 'missing_alias_lane',
                         'nonstandalone_lane', 'different_lane_name',
                         'different_lane_tool', 'live_legacy_lane'):
            with self.subTest(mismatch=mismatch):
                self.setUp()
                if mismatch == 'other_printer':
                    del self.tool0.creator5_tool_index
                elif mismatch == 'other_index':
                    self.tool0.creator5_tool_index = 1
                elif mismatch == 'old_name':
                    self.tool0.name = 'extruder'
                elif mismatch == 'missing_physical_tool':
                    self.afc.tools['extruder0'] = self.afc.tools.pop('extruder')
                elif mismatch == 'missing_alias_lane':
                    del self.afc.lanes['extruder0']
                elif mismatch == 'nonstandalone_lane':
                    self.lane0.standalone_lane = False
                elif mismatch == 'different_lane_name':
                    self.lane0.name = 'another_lane'
                elif mismatch == 'different_lane_tool':
                    self.lane0.extruder_obj = self.tool2
                else:
                    self.afc.lanes['extruder'] = lane(
                        'extruder', 'Heads', self.tool0)
                state = saved_state()

                self.assertEqual(self.restore(state), state)

    def test_unit_name_comes_from_configured_alias_lane(self):
        self.lane0.unit = 'CustomHeads'
        state = saved_state()
        state['CustomHeads'] = state.pop('Heads')

        migrated = self.restore(state)

        self.assertIn('extruder0', migrated['CustomHeads'])
        self.assertNotIn('extruder', migrated['CustomHeads'])
        self.assertEqual(self.lane0.spool_id, 42)


if __name__ == '__main__':
    unittest.main()
