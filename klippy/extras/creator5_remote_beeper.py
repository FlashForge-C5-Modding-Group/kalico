# Creator 5 host PWM buzzer. Works unmodified in either deployment:
# - Printer SoC (Klipper runs on the printer's own Linux board): the
#   cmd_pwm binary is on PATH there, so this calls it directly, with
#   the exact same programming sequence as creator5_beeper.py.
# - Tunneled (Klipper runs on an SBC, see opencreator-tunnel): cmd_pwm
#   doesn't exist on the SBC, so this forwards one line over the
#   gadget's 5th serial port (ttyGS4, USB interface 1.4) to the tunnel
#   bridge's beep control channel instead (printer/c5_bridge.py or
#   printer-socat/c5_socat.py's BeepServer/run_beep_server, which runs
#   the same sequence locally on the printer). Riding the USB link
#   itself, not the network, means it still works with no Wi-Fi/LAN on
#   the printer.
# Which path is used is auto-detected once at startup by checking
# whether cmd_pwm is on this host's PATH, so the same [section] works
# for both installs; `serial`/`baud` only matter for the tunneled case.
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import select
import shutil
import subprocess
import termios


class Creator5RemoteBeeper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.command = config.get('command', 'cmd_pwm')
        self.channel = config.get('channel', 'pc12')
        self.serial = config.get('serial', '/dev/ttyGS4')
        self.baud = config.getint('baud', 115200)
        self.timeout = config.getfloat('timeout', 4., above=0.)
        self.local_command = shutil.which(self.command)
        self.reactor = self.printer.get_reactor()
        self._pwm_configured = False
        aio = self.printer.load_object(config, 'aio_executor')
        self.executor = aio.allocate_executor('creator5_remote_beeper')
        self.gcode.register_command('C5_BUZZER', self.cmd_C5_BUZZER,
                                    desc='Sound the Creator 5 host PWM buzzer')

    # ---- local path: Klipper running on the printer's own SoC ----

    def _run_local(self, *args, tolerate_already_working=False):
        try:
            result = subprocess.run([self.local_command] + list(args),
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    check=False, timeout=3)
        except (OSError, subprocess.SubprocessError) as exc:
            raise self.printer.command_error(
                'Creator 5 buzzer command failed: %s' % (exc,))
        if result.returncode:
            detail = result.stderr.decode('utf-8', 'replace').strip()
            # The kernel PWM driver rejects config/set_level/set_prescale
            # with this exact error once the channel is already in its
            # "working" state, including across a Klipper restart since
            # that state lives in the kernel, not here, and
            # disable_channels does not release it. Tolerate it on those
            # three calls: it means the one-time setup already happened.
            if (tolerate_already_working
                    and 'operation not permitted' in detail.lower()):
                return
            raise self.printer.command_error(
                'Creator 5 buzzer command failed (%d): %s'
                % (result.returncode, detail))

    def _play_tone_local(self, duration_ms, frequency, level):
        period = max(2, int(round(50000000. / frequency)))
        high = period * level // 200
        # set_level/set_prescale are only ever called here with fixed
        # constants (100, 6), never derived from this tone's own
        # parameters, so they are one-time setup alongside config.
        # Call them at most once per process lifetime, not on every
        # beep: real hardware testing showed the kernel soc_pwm driver
        # can get a cmd_pwm process stuck forever in kernel D state
        # waiting on its internal mutex if config is called repeatedly
        # (even just to hit its expected "already configured" error) --
        # a genuine kernel deadlock only a reboot clears.
        if not self._pwm_configured:
            self._run_local('config', self.channel, 'freq=50000000',
                            'max_level=300', 'active_level=1',
                            'accuracy_priority=freq', tolerate_already_working=True)
            self._run_local('set_level', self.channel, '100',
                            tolerate_already_working=True)
            self._run_local('set_prescale', self.channel, '6',
                            tolerate_already_working=True)
            self._pwm_configured = True
        try:
            self._run_local('set_wc', self.channel, str(period), str(high))
            self._run_local('enable_channels', self.channel)
            self.reactor.pause(self.reactor.monotonic() + duration_ms / 1000.)
        finally:
            try:
                self._run_local('set_wc', self.channel, '1', '0')
            finally:
                self._run_local('disable_channels', self.channel)

    # ---- remote path: Klipper running on an SBC over the tunnel ----

    def _open_serial(self):
        try:
            fd = os.open(self.serial, os.O_RDWR | os.O_NOCTTY)
        except OSError as exc:
            raise self.printer.command_error(
                'Creator 5 buzzer: cannot open tunnel serial %s: %s'
                % (self.serial, exc))
        try:
            speed = getattr(termios, 'B%d' % self.baud)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = (
                termios.tcgetattr(fd))
            iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                       | termios.ISTRIP | termios.INLCR | termios.IGNCR
                       | termios.ICRNL | termios.IXON)
            oflag &= ~termios.OPOST
            lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                       | termios.ISIG | termios.IEXTEN)
            cflag &= ~(termios.CSIZE | termios.PARENB)
            cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, speed, speed, cc])
            # A tty's kernel read buffer outlives any one open()/close():
            # a reply that arrived after a previous call gave up and
            # closed its fd would otherwise sit here and be mistaken
            # for this call's reply. Discard it before writing.
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        return fd

    def _send_remote(self, duration_ms, frequency, level):
        line = "BEEP DURATION=%d FREQUENCY=%d LEVEL=%d\n" % (
            duration_ms, frequency, level)
        fd = self._open_serial()
        try:
            view = memoryview(line.encode('ascii'))
            deadline = self.reactor.monotonic() + self.timeout
            while view:
                ready = select.select([], [fd], [],
                                      max(0., deadline - self.reactor.monotonic()))
                if not ready[1]:
                    raise self.printer.command_error(
                        'Creator 5 buzzer bridge write timed out')
                n = os.write(fd, view)
                view = view[n:]
            data = b""
            deadline = self.reactor.monotonic() + self.timeout
            while b"\n" not in data and len(data) < 256:
                remaining = deadline - self.reactor.monotonic()
                if remaining <= 0:
                    raise self.printer.command_error(
                        'Creator 5 buzzer bridge did not respond')
                ready = select.select([fd], [], [], remaining)
                if not ready[0]:
                    continue
                chunk = os.read(fd, 256)
                if not chunk:
                    break
                data += chunk
        finally:
            os.close(fd)
        reply = data.split(b"\n", 1)[0].decode('utf-8', 'replace').strip()
        if reply != 'OK':
            raise self.printer.command_error(
                'Creator 5 buzzer command failed: %s'
                % (reply or 'no response',))

    def _beep(self, duration_ms, frequency, level):
        if self.local_command:
            self._play_tone_local(duration_ms, frequency, level)
        else:
            self._send_remote(duration_ms, frequency, level)

    def cmd_C5_BUZZER(self, gcmd):
        duration_ms = gcmd.get_int('DURATION', 5000, minval=1,
                                   maxval=30000)
        frequency = gcmd.get_int('FREQUENCY', 2500, minval=20, maxval=20000)
        level = gcmd.get_int('LEVEL', 100, minval=0, maxval=100)
        optional = gcmd.get_int('OPTIONAL', 0, minval=0, maxval=1)
        try:
            self.executor.submit(self._beep, duration_ms, frequency, level)
        except self.printer.command_error as exc:
            if not optional:
                raise
            gcmd.respond_info('Creator 5 buzzer unavailable; tune skipped: %s'
                              % (exc,))


def load_config(config):
    return Creator5RemoteBeeper(config)
