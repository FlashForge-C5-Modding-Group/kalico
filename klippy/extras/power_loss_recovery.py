# Power-loss / interrupted-print recovery
#
# Models the stock Creator 5 firmware's RecoveryFunc (recovery.json +
# recovery_flag.json, reverse engineered from the printer's own host
# application): periodically snapshot enough state during an SD print to
# resume it later, and never clear the "in progress" flag except on a
# clean finish/cancel -- so if the flag is still set at the next klippy
# startup, the previous session ended abnormally (power loss, crash, MCU
# fault) and a resume is offered.
#
# Unlike stock, this does not auto-resume. RECOVER_PRINT must be invoked
# explicitly, since re-homing, reheating and toolchanging unattended after
# an unknown interruption is not something to do without a human present.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
import logging
import os
import time

class PowerLossRecovery:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.state_path = os.path.expanduser(
            config.get('state_path', '~/printer_data/power_loss_recovery.json'))
        self.save_interval = config.getfloat('save_interval', 10., above=0.)
        self.min_z_clearance = config.getfloat('min_z_clearance', 5., above=0.)
        self.toolchanger_name = config.get('toolchanger', 'creator5_toolchanger')
        self.pending = None
        self._last_state = 'standby'
        self._timer = self.reactor.register_timer(self._poll)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.gcode.register_command(
            'RECOVER_PRINT', self.cmd_RECOVER_PRINT,
            desc='Resume the print interrupted at the last abnormal '
                 'shutdown, using the saved power_loss_recovery state')
        self.gcode.register_command(
            'CLEAR_POWER_LOSS_RECOVERY', self.cmd_CLEAR_POWER_LOSS_RECOVERY,
            desc='Discard any pending power-loss-recovery state without '
                 'resuming')

    def _handle_ready(self):
        self.pending = self._load_state()
        if self.pending is not None and self.pending.get('in_progress'):
            logging.info(
                'power_loss_recovery: found an interrupted print (%s at '
                'byte %d/%d); call RECOVER_PRINT to resume or '
                'CLEAR_POWER_LOSS_RECOVERY to discard',
                self.pending.get('filename'), self.pending.get('file_position', 0),
                self.pending.get('file_size', 0))
        else:
            self.pending = None
        self.reactor.update_timer(self._timer, self.reactor.NOW)

    def get_status(self, eventtime=None):
        available = self.pending is not None and self.pending.get('in_progress')
        status = {'recovery_available': available}
        if available:
            status['filename'] = self.pending.get('filename')
            status['file_position'] = self.pending.get('file_position')
            status['file_size'] = self.pending.get('file_size')
        return status

    # -- persistence -------------------------------------------------
    def _load_state(self):
        try:
            with open(self.state_path, 'r') as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def _write_state(self, state):
        tmp_path = self.state_path + '.tmp'
        try:
            with open(tmp_path, 'w') as f:
                json.dump(state, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.state_path)
        except OSError as exc:
            logging.warning('power_loss_recovery: failed to save state: %s', exc)

    # -- periodic snapshot --------------------------------------------
    def _poll(self, eventtime):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is None:
            return eventtime + self.save_interval
        status = print_stats.get_status(eventtime)
        state = status['state']
        if state == 'printing':
            self._snapshot(eventtime, status)
        elif self._last_state in ('printing', 'paused') and state in (
                'complete', 'cancelled', 'error', 'standby'):
            # A clean end to the previous print (even an error/cancel is a
            # *known* end, unlike silence from a power loss) -- clear the
            # flag so the next startup does not offer a stale resume.
            self._write_state({'in_progress': False})
            self.pending = None
        self._last_state = state
        return eventtime + self.save_interval

    def _snapshot(self, eventtime, print_stats_status):
        virtual_sdcard = self.printer.lookup_object('virtual_sdcard', None)
        gcode_move = self.printer.lookup_object('gcode_move')
        if virtual_sdcard is None:
            return
        sd_status = virtual_sdcard.get_status(eventtime)
        gm_status = gcode_move.get_status(eventtime)
        file_position = sd_status.get('file_position', 0)
        if not file_position:
            # Nothing has actually been streamed yet for this file.
            return
        pos = gm_status['position']
        heater_targets = {}
        heaters = self.printer.lookup_object('heaters', None)
        if heaters is not None:
            for name in heaters.get_status(eventtime)['available_heaters']:
                short_name = name.split()[-1]
                heater = heaters.lookup_heater(short_name)
                heater_targets[short_name] = heater.get_status(eventtime)['target']
        fan_speed = 0.
        fan = self.printer.lookup_object('fan', None)
        if fan is not None:
            fan_speed = fan.get_status(eventtime).get('speed', 0.)
        tool = None
        toolchanger = self.printer.lookup_object(self.toolchanger_name, None)
        if toolchanger is not None:
            tool = toolchanger.get_status(eventtime).get('active_tool')
        state = {
            'in_progress': True,
            'filename': print_stats_status['filename'],
            'file_position': file_position,
            'file_size': sd_status.get('file_size', 0),
            'tool': tool,
            'position': [pos.x, pos.y, pos.z, pos.e],
            'speed_factor': gm_status['speed_factor'],
            'extrude_factor': gm_status['extrude_factor'],
            'absolute_coord': gm_status['absolute_coordinates'],
            'absolute_extrude': gm_status['absolute_extrude'],
            'heater_targets': heater_targets,
            'fan_speed': fan_speed,
            'timestamp': time.time(),
        }
        self._write_state(state)

    # -- resume ---------------------------------------------------------
    cmd_RECOVER_PRINT_help = 'Resume the print interrupted at the last abnormal shutdown'
    def cmd_RECOVER_PRINT(self, gcmd):
        print_stats = self.printer.lookup_object('print_stats')
        if print_stats.get_status(self.reactor.monotonic())['state'] in (
                'printing', 'paused'):
            raise gcmd.error('A print is already active')
        state = self.pending
        if state is None or not state.get('in_progress'):
            raise gcmd.error('No pending power-loss-recovery state')
        virtual_sdcard = self.printer.lookup_object('virtual_sdcard')
        filename = state['filename']
        full_path = os.path.join(virtual_sdcard.sdcard_dirname, filename)
        if not os.path.isfile(full_path):
            raise gcmd.error('Recovery file no longer exists: %s' % filename)
        actual_size = os.path.getsize(full_path)
        if state.get('file_size') and actual_size != state['file_size']:
            raise gcmd.error(
                'Recovery file size changed (%d -> %d bytes); refusing to '
                'resume %s' % (state['file_size'], actual_size, filename))
        file_position = state['file_position']
        if not (0 <= file_position < actual_size):
            raise gcmd.error('Recovery file position out of range')

        gcmd.respond_info('Recovering print %s at byte %d/%d'
                          % (filename, file_position, actual_size))
        self.gcode.run_script_from_command('G28')
        tool = state.get('tool')
        if tool is not None:
            heater = 'extruder' if tool == 0 else 'extruder%d' % tool
            self.gcode.run_script_from_command(
                'AFC_SELECT_TOOL TOOL=%s' % heater)
        heater_targets = state.get('heater_targets', {})
        for name, target in heater_targets.items():
            if target > 0:
                self.gcode.run_script_from_command(
                    'SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.1f'
                    % (name, target))
        for name, target in heater_targets.items():
            if target > 0:
                self.gcode.run_script_from_command(
                    'TEMPERATURE_WAIT SENSOR=%s MINIMUM=%.1f'
                    % (name, target - 2.))
        toolhead = self.printer.lookup_object('toolhead')
        x, y, z, e = state['position']
        cur_z = toolhead.get_position()[2]
        safe_z = max(z + self.min_z_clearance, cur_z)
        self.gcode.run_script_from_command('G90')
        self.gcode.run_script_from_command('G1 Z%.3f F600' % safe_z)
        self.gcode.run_script_from_command('G1 X%.3f Y%.3f F6000' % (x, y))
        self.gcode.run_script_from_command('G1 Z%.3f F600' % z)
        self.gcode.run_script_from_command('G92 E%.5f' % e)
        if not state.get('absolute_coord', True):
            self.gcode.run_script_from_command('G91')
        if state.get('absolute_extrude', True):
            self.gcode.run_script_from_command('M82')
        else:
            self.gcode.run_script_from_command('M83')
        self.gcode.run_script_from_command(
            'M220 S%.1f' % (state.get('speed_factor', 1.) * 100.))
        self.gcode.run_script_from_command(
            'M221 S%.1f' % (state.get('extrude_factor', 1.) * 100.))
        fan_speed = state.get('fan_speed', 0.)
        self.gcode.run_script_from_command('M106 S%d' % round(fan_speed * 255))
        self.gcode.run_script_from_command('M23 %s' % filename)
        self.gcode.run_script_from_command('M26 S%d' % file_position)
        self.gcode.run_script_from_command('M24')
        self._write_state({'in_progress': False})
        self.pending = None

    cmd_CLEAR_POWER_LOSS_RECOVERY_help = 'Discard any pending power-loss-recovery state'
    def cmd_CLEAR_POWER_LOSS_RECOVERY(self, gcmd):
        self._write_state({'in_progress': False})
        self.pending = None
        gcmd.respond_info('Power-loss-recovery state cleared')


def load_config(config):
    return PowerLossRecovery(config)
