# Publish Creator 5 standalone tool filaments for OrcaSlicer's Moonraker sync.
# This reports inventory only; tool selection remains with AFC and the
# physical Creator 5 toolchanger.
import json
import logging
import os
import re
import tempfile
from concurrent.futures import ThreadPoolExecutor
from urllib.request import Request, urlopen


COLOR_RE = re.compile(r'^#[0-9A-Fa-f]{6}$')


def empty_filament():
    return {'material': '', 'color': '', 'nozzle_temp': 0, 'bed_temp': 0,
            'spool_id': None}


class Creator5FilamentSync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.filename = config.get(
            'filename', '/usr/data/config/c5_filaments.json')
        self.moonraker_url = config.get(
            'moonraker_url', 'http://127.0.0.1:7125').rstrip('/')
        self.filaments = self._load()
        self.executor = ThreadPoolExecutor(max_workers=1,
                                           thread_name_prefix='c5-filament')
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect',
                                            self._handle_disconnect)
        # AFC clears lane_data on its own startup. Publish after that event too.
        self.printer.register_event_handler('afc:moonraker_connect',
                                            self._queue_publish)
        for name, handler, desc in (
            ('C5_SYNC_FILAMENTS', self.cmd_sync,
             'Republish Creator 5 tool filaments to Moonraker'),
            ('C5_FILAMENT_STATUS', self.cmd_status,
             'Show the filament assigned to each Creator 5 tool')):
            self.gcode.register_command(name, handler, desc=desc)

    def _load(self):
        result = [empty_filament() for _ in range(4)]
        if not os.path.exists(self.filename):
            return result
        try:
            with open(self.filename, encoding='utf-8') as source:
                saved = json.load(source)
            if not isinstance(saved, list) or len(saved) != 4:
                raise ValueError('expected four tool records')
            for index, entry in enumerate(saved):
                if not isinstance(entry, dict):
                    raise ValueError('invalid T%d record' % index)
                material = entry.get('material', '')
                color = entry.get('color', '')
                if (not isinstance(material, str) or not isinstance(color, str)
                        or (color and not COLOR_RE.fullmatch(color))):
                    raise ValueError('invalid T%d material or color' % index)
                nozzle_temp = int(entry.get('nozzle_temp', 0))
                bed_temp = int(entry.get('bed_temp', 0))
                spool_id = entry.get('spool_id')
                if not (0 <= nozzle_temp <= 350 and 0 <= bed_temp <= 130):
                    raise ValueError('invalid T%d temperature' % index)
                if (spool_id is not None and
                        (not isinstance(spool_id, int) or spool_id <= 0)):
                    raise ValueError('invalid T%d spool ID' % index)
                result[index] = {
                    'material': material,
                    'color': color.upper(),
                    'nozzle_temp': nozzle_temp,
                    'bed_temp': bed_temp,
                    'spool_id': spool_id,
                }
        except (OSError, ValueError, TypeError) as exc:
            raise self.printer.config_error(
                'Cannot load Creator 5 filament inventory: %s' % exc)
        return result

    def _save(self, records):
        directory = os.path.dirname(self.filename) or '.'
        os.makedirs(directory, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix='.c5_filaments-',
                                         suffix='.json', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as target:
                json.dump(records, target, indent=2)
                target.write('\n')
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, self.filename)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _publish(self, records):
        for index, entry in enumerate(records):
            payload = {
                'namespace': 'lane_data',
                'key': 'c5_tool%d' % index,
                'value': dict(entry, lane=str(index)),
            }
            request = Request(
                self.moonraker_url + '/server/database/item',
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(request, timeout=3) as response:
                if response.status != 200:
                    raise IOError('Moonraker returned HTTP %d' % response.status)

    def _queue_publish(self, *args):
        records = [dict(entry) for entry in self.filaments]
        future = self.executor.submit(self._publish, records)
        def report_result(done):
            try:
                done.result()
            except Exception:
                logging.exception('Creator 5 filament sync to Moonraker failed; '
                                  'use C5_SYNC_FILAMENTS to retry')
        future.add_done_callback(report_result)

    def _handle_ready(self):
        self.reactor.register_timer(self._startup_publish,
                                    self.reactor.monotonic() + 5.)

    def _startup_publish(self, eventtime):
        self._queue_publish()
        return self.reactor.NEVER

    def _handle_disconnect(self):
        self.executor.shutdown(wait=False)

    def get_status(self, eventtime):
        return {'tools': [dict(entry) for entry in self.filaments]}

    def _set_record(self, index, record):
        if not 0 <= index <= 3:
            raise ValueError('Tool index must be 0..3')
        updated = [dict(entry) for entry in self.filaments]
        updated[index] = record
        self._save(updated)
        self.filaments = updated
        self._queue_publish()

    def set_tool_fields(self, index, **fields):
        if not 0 <= index <= 3:
            raise ValueError('Tool index must be 0..3')
        record = dict(self.filaments[index])
        for key, value in fields.items():
            if key == 'material':
                if not isinstance(value, str):
                    raise ValueError('Material must be text')
                value = value.strip().upper()
            elif key == 'color':
                value = value.strip().upper()
                if re.fullmatch(r'[0-9A-F]{6}', value):
                    value = '#' + value
                if value and not COLOR_RE.fullmatch(value):
                    raise ValueError('Color must be six hexadecimal digits')
            elif key == 'nozzle_temp':
                value = int(value)
                if not 0 <= value <= 350:
                    raise ValueError('Nozzle temperature must be 0..350')
            elif key == 'bed_temp':
                value = int(value)
                if not 0 <= value <= 130:
                    raise ValueError('Bed temperature must be 0..130')
            elif key == 'spool_id':
                if value is not None:
                    value = int(value)
                    if value <= 0:
                        raise ValueError('Spool ID must be positive')
            else:
                raise ValueError('Unknown filament field: %s' % key)
            record[key] = value
        self._set_record(index, record)

    def clear_tool(self, index):
        self._set_record(index, empty_filament())

    def cmd_sync(self, gcmd):
        self._queue_publish()
        gcmd.respond_info('Creator 5 filament sync queued for Moonraker')

    def cmd_status(self, gcmd):
        for index, entry in enumerate(self.filaments):
            gcmd.respond_info('T%d: %s %s' % (
                index, entry['material'] or 'empty', entry['color']))


def load_config(config):
    return Creator5FilamentSync(config)
