# FlashForge VDS filament dryer (two chambers, A and B) on the Creator 5
# SoC's /dev/ttyS3, using the dryer's own line protocol (M10-M14, M16).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import queue
import threading

import serial

CHAMBERS = {'A': 0, 'B': 1, 'AB': 2}
MIN_TEMP, MAX_TEMP = 45, 85
MIN_MINUTES, MAX_MINUTES = 60, 2880
REPLY_TIMEOUT = 2.
MAX_MISSED_POLLS = 3


def parse_status(line):
    """'M14 ok. <n> A:<11 values> B:<11 values>' -> {'A': [...], 'B': [...]},
    or None when the line isn't a status reply."""
    parts = line.split()
    if len(parts) < 2 or parts[0] != 'M14' or parts[1] != 'ok.':
        return None
    chambers = {}
    current = None
    for part in parts[2:]:
        if part[:2] in ('A:', 'B:'):
            current = chambers.setdefault(part[0], [])
            part = part[2:]
            if not part:
                continue
        if current is None:
            continue
        try:
            current.append(int(part))
        except ValueError:
            return None
    if any(len(values) < 11 for values in chambers.values()):
        return None
    return chambers


class DryerChamberSensor:
    """A chamber as a temperature_sensor object, so the UIs graph it."""
    def __init__(self, printer, pheaters, name):
        self.temperature = self.humidity = 0.
        self.measured_min = self.measured_max = None
        printer.add_object('temperature_sensor ' + name, self)
        pheaters.available_sensors.append('temperature_sensor ' + name)

    def update(self, temperature, humidity):
        self.temperature, self.humidity = float(temperature), float(humidity)
        if self.measured_min is None:
            self.measured_min = self.measured_max = self.temperature
        self.measured_min = min(self.measured_min, self.temperature)
        self.measured_max = max(self.measured_max, self.temperature)

    def get_status(self, eventtime):
        return {'temperature': round(self.temperature, 2),
                'humidity': round(self.humidity, 2),
                'measured_min_temp': round(self.measured_min or 0., 2),
                'measured_max_temp': round(self.measured_max or 0., 2)}


class Creator5Dryer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('serial', '/dev/ttyS3')
        self.baud = config.getint('baud', 115200)
        self.poll_interval = config.getfloat('poll_interval', 2., above=0.5)
        self.sensors = {}
        if config.getboolean('chamber_sensors', True):
            pheaters = self.printer.load_object(config, 'heaters')
            self.sensors = {c: DryerChamberSensor(self.printer, pheaters,
                                                  'dryer_' + c.lower())
                            for c in ('A', 'B')}
        self.connected = self.in_bootloader = False
        self.version = ''
        self.chambers = {c: self._empty_chamber() for c in ('A', 'B')}
        self.commands = queue.Queue()
        self.stopping = False
        self.thread = None
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect',
                                            self._handle_disconnect)
        for name, handler, desc in (
                ('C5_DRYER_START', self.cmd_start,
                 'Start drying: CHAMBER=A|B|AB TEMP=45-85 HOURS=1-48 (or MINUTES=60-2880)'),
                ('C5_DRYER_STOP', self.cmd_stop, 'Stop drying: CHAMBER=A|B|AB'),
                ('C5_DRYER_PAUSE', self.cmd_pause,
                 'Pause drying (heaters off): CHAMBER=A|B|AB; resume with C5_DRYER_START'),
                ('C5_DRYER_DEHUMIDIFY', self.cmd_dehumidify,
                 'Keep a chamber dry after drying: CHAMBER=A|B|AB ENABLE=0|1'),
                ('C5_DRYER_STATUS', self.cmd_status, 'Show the dryer state')):
            self.gcode.register_command(name, handler, desc=desc)

    @staticmethod
    def _empty_chamber():
        return {'drying': False, 'state': 0, 'mode': 0, 'temperature': 0,
                'humidity': 0, 'remaining_minutes': 0}

    def _handle_ready(self):
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.thread = threading.Thread(target=self._run, name='creator5_dryer')
        self.thread.daemon = True
        self.thread.start()

    def _handle_disconnect(self):
        self.stopping = True

    # ---- serial worker thread ----------------------------------------------
    def _run(self):
        ser = None
        missed = 0
        have_version = False
        while not self.stopping:
            if ser is None:
                try:
                    # exclusive: a Klipper restart's new thread waits for this one
                    ser = serial.Serial(self.port, self.baud, timeout=REPLY_TIMEOUT,
                                        exclusive=True)
                except (OSError, serial.SerialException) as e:
                    logging.info("creator5_dryer: cannot open %s: %s", self.port, e)
                    ser = None
                    self._wait(10.)
                    continue
            try:
                command = self.commands.get(timeout=self.poll_interval)
            except queue.Empty:
                command = None
            try:
                if command is not None:
                    line, reply_to = command
                    reply = self._request(ser, line, line.split()[0] + ' ok.')
                    self._respond(reply_to, line, reply)
                    continue
                reply = self._request(ser, 'M14', 'M14 ok.')
            except (OSError, serial.SerialException) as e:
                logging.info("creator5_dryer: %s: %s", self.port, e)
                ser.close()
                ser = None
                self._publish(connected=False)
                continue
            status = parse_status(reply) if reply else None
            if status is not None:
                missed = 0
                self._publish(connected=True, bootloader=False, status=status)
                if not have_version:
                    version = self._request(ser, 'M16', 'M16 ok.')
                    have_version = bool(version and version.startswith('M16 ok.'))
                    if have_version:
                        self._publish(version=version)
            elif reply and 'Boot' in reply:
                self._publish(connected=True, bootloader=True)
            else:
                missed += 1
                if missed >= MAX_MISSED_POLLS:
                    have_version = False
                    self._publish(connected=False)
        if ser is not None:
            ser.close()

    def _wait(self, seconds):
        try:
            command = self.commands.get(timeout=seconds)
        except queue.Empty:
            return
        self._respond(command[1], command[0], None)

    def _request(self, ser, line, expect):
        ser.reset_input_buffer()
        ser.write((line + '\n').encode())
        for _ in range(5):
            reply = ser.readline().decode('ascii', 'replace').strip()
            if not reply:
                return None
            if reply.startswith(expect) or 'Boot' in reply:
                return reply
        return None

    def _publish(self, connected=None, bootloader=None, status=None, version=None):
        def apply(eventtime):
            if version is not None:
                self.version = version[len('M16 ok.'):].strip()
            if connected is not None:
                self.connected = connected
            if bootloader is not None:
                self.in_bootloader = bootloader
            for chamber, values in (status or {}).items():
                info = {'mode': values[0], 'state': values[1],
                        'drying': values[1] == 2, 'humidity': values[2],
                        'temperature': values[3],
                        'remaining_minutes': values[4]}
                self.chambers[chamber] = info
                if chamber in self.sensors:
                    self.sensors[chamber].update(values[3], values[2])
        self.reactor.register_async_callback(apply)

    def _respond(self, reply_to, line, reply):
        def respond(eventtime):
            if reply and reply.startswith(line.split()[0] + ' ok.'):
                self.gcode.respond_info("Dryer: %s done" % (reply_to,))
            elif reply and 'Boot' in reply:
                self.gcode.respond_info(
                    "Dryer: '%s' not done, the dryer is in its bootloader"
                    % (reply_to,))
            else:
                self.gcode.respond_info(
                    "Dryer: '%s' not done, no reply from the dryer on %s"
                    % (reply_to, self.port))
        self.reactor.register_async_callback(respond)

    # ---- G-code ----------------------------------------------------------------
    def _send(self, gcmd, line, what):
        if not self.connected:
            raise gcmd.error("Dryer not connected on %s" % (self.port,))
        self.commands.put((line, what))
        gcmd.respond_info("Dryer: %s sent" % (what,))

    def _chamber(self, gcmd):
        name = gcmd.get('CHAMBER', 'AB').upper()
        if name not in CHAMBERS:
            raise gcmd.error("CHAMBER must be A, B or AB")
        return name, CHAMBERS[name]

    def cmd_start(self, gcmd):
        name, chamber = self._chamber(gcmd)
        temp = gcmd.get_int('TEMP', minval=MIN_TEMP, maxval=MAX_TEMP)
        hours = gcmd.get_float('HOURS', None, minval=1., maxval=48.)
        minutes = gcmd.get_int('MINUTES', None if hours is None else int(round(hours * 60)),
                               minval=MIN_MINUTES, maxval=MAX_MINUTES)
        if minutes is None:
            raise gcmd.error("Give HOURS or MINUTES")
        self._send(gcmd, 'M10 C%d L%d S%d' % (chamber, temp, minutes),
                   "drying chamber %s at %d C for %d min" % (name, temp, minutes))

    def cmd_stop(self, gcmd):
        name, chamber = self._chamber(gcmd)
        self._send(gcmd, 'M11 C%d' % (chamber,), "stop chamber %s" % (name,))

    def cmd_pause(self, gcmd):
        name, chamber = self._chamber(gcmd)
        self._send(gcmd, 'M12 C%d' % (chamber,), "pause chamber %s" % (name,))

    def cmd_dehumidify(self, gcmd):
        name, chamber = self._chamber(gcmd)
        enable = gcmd.get_int('ENABLE', 1, minval=0, maxval=1)
        self._send(gcmd, 'M13 C%d L%d' % (chamber, enable),
                   "dehumidify chamber %s %s" % (name, 'on' if enable else 'off'))

    def cmd_status(self, gcmd):
        if not self.connected:
            gcmd.respond_info("Dryer: not connected on %s" % (self.port,))
            return
        if self.in_bootloader:
            gcmd.respond_info("Dryer: in its bootloader (needs a firmware update)")
            return
        lines = ["Dryer firmware %s" % (self.version or 'unknown',)]
        for name in ('A', 'B'):
            c = self.chambers[name]
            lines.append("Chamber %s: %s, %d C, %d%% humidity%s" % (
                name, 'drying' if c['drying'] else 'idle', c['temperature'],
                c['humidity'], ', %d min left' % c['remaining_minutes']
                if c['drying'] else ''))
        gcmd.respond_info("\n".join(lines))

    def get_status(self, eventtime):
        return {'connected': self.connected, 'in_bootloader': self.in_bootloader,
                'firmware_version': self.version,
                'chambers': {k: dict(v) for k, v in self.chambers.items()}}


def load_config(config):
    return Creator5Dryer(config)
