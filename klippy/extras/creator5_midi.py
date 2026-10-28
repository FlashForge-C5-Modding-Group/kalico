# Play MIDI files on the Creator 5's single-voice host PWM buzzer.
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The buzzer plays one tone at a time, so a file is reduced to one line: at
# every moment the highest note sounding (usually the melody). Drums (MIDI
# channel 10) are left out. Playback runs on its own thread: the channel is
# configured once (shared with [creator5_beeper]), enabled, and only its
# waveform changes from note to note.
import os
import struct
import threading
import time

MIN_NOTE = 0.02     # s: notes shorter than this are merged into the next
REPEAT_GAP = 0.03   # s: silence that separates a note from its own repeat


# In creator5_beeper.py

class Creator5Beeper:
    def __init__(self, config):
        # ... existing setup ...
        self._configured = False

    def ensure_configured(self, run):
        """Configure the PWM channel once per session."""
        if not self._configured:
            try:
                run('disable', self.channel)
                run('config', self.channel, 'freq=50000000',
                    'max_level=300', 'active_level=1',
                    'accuracy_priority=freq')
                run('set_level', self.channel, '100')
                run('set_prescale', self.channel, '6')
            except self.printer.command_error as exc:
                self._config_error = exc
                raise
            self._configured = True

class MidiError(Exception):
    pass


def _varlen(data, pos):
    value = 0
    for _ in range(4):
        byte = data[pos]
        pos += 1
        value = (value << 7) | (byte & 0x7f)
        if not byte & 0x80:
            return value, pos
    raise MidiError("bad variable-length number")


def parse_midi(data):
    """Standard MIDI file -> (ticks per quarter, events), where events are
    (tick, order, kind, channel, note, value) for notes and tempo changes."""
    if data[:4] != b"MThd":
        raise MidiError("not a MIDI file")
    length, fmt, ntracks, division = struct.unpack(">IHHH", data[4:14])
    if fmt > 1:
        raise MidiError("MIDI format %d not supported (0 or 1 only)" % fmt)
    if division & 0x8000:
        raise MidiError("SMPTE time division not supported")
    pos = 8 + length
    events = []
    order = 0
    for _ in range(ntracks):
        if data[pos:pos + 4] != b"MTrk":
            raise MidiError("missing track")
        (tlen,) = struct.unpack(">I", data[pos + 4:pos + 8])
        pos += 8
        end = pos + tlen
        tick = 0
        status = None
        while pos < end:
            delta, pos = _varlen(data, pos)
            tick += delta
            byte = data[pos]
            if byte == 0xff:
                kind, pos = data[pos + 1], pos + 2
                mlen, pos = _varlen(data, pos)
                if kind == 0x51 and mlen == 3:
                    tempo = int.from_bytes(data[pos:pos + 3], "big")
                    events.append((tick, order, "tempo", 0, 0, tempo))
                pos += mlen
                if kind == 0x2f:
                    break
                continue
            if byte in (0xf0, 0xf7):
                slen, pos = _varlen(data, pos + 1)
                pos += slen
                continue
            if byte & 0x80:
                status = byte
                pos += 1
            elif status is None:
                raise MidiError("data byte without status")
            kind, channel = status & 0xf0, status & 0x0f
            if kind in (0xc0, 0xd0):
                pos += 1
                continue
            note, velocity = data[pos], data[pos + 1]
            pos += 2
            if kind == 0x90 and velocity:
                events.append((tick, order, "on", channel, note, velocity))
            elif kind in (0x80, 0x90):
                events.append((tick, order, "off", channel, note, 0))
            order += 1
        pos = end
    # Same tick: tempo first, then note-offs, then note-ons
    rank = {"tempo": 0, "off": 1, "on": 2}
    events.sort(key=lambda e: (e[0], rank[e[2]], e[1]))
    return division, events


def melody(data, channel=None, transpose=0):
    """[(start s, frequency Hz or None)] for the highest sounding note,
    ending with (end s, None)."""
    division, events = parse_midi(data)
    tempo = 500000.         # us per quarter note until a tempo event
    last_tick = 0
    now = 0.
    sounding = {}
    segments = []
    for tick, _, kind, chan, note, value in events:
        now += (tick - last_tick) * tempo / division / 1e6
        last_tick = tick
        if kind == "tempo":
            tempo = float(value)
            continue
        if chan == 9 or (channel is not None and chan != channel):
            continue
        key = (chan, note)
        if kind == "on":
            sounding[key] = sounding.get(key, 0) + 1
        elif key in sounding:
            sounding[key] -= 1
            if not sounding[key]:
                del sounding[key]
        top = max((n for _, n in sounding), default=None)
        freq = None if top is None else 440. * 2 ** ((top + transpose - 69) / 12.)
        # Never before the previous segment (a re-strike starts REPEAT_GAP late)
        at = max(now, segments[-1][0]) if segments else now
        # Every note-on of the top note is heard as its own note: one tone can
        # only repeat a pitch with a gap, so it is never merged into the last
        strike = kind == "on" and note == top
        if (strike and segments and segments[-1][1] == freq
                and segments[-1][0] < at):
            segments.append((at, None))
            segments.append((at + REPEAT_GAP, freq))
        elif (strike and len(segments) >= 2 and segments[-1] == (at, None)
                and segments[-2][1] == freq):
            segments.append((at + REPEAT_GAP, freq))
        elif segments and segments[-1][0] == at:
            segments[-1] = (at, freq)
        elif not segments or segments[-1][1] != freq:
            segments.append((at, freq))
    if not segments:
        raise MidiError("no notes to play")
    segments.append((now, None))
    # Drop notes too short for cmd_pwm to switch to in time
    result = [segments[0]]
    for start, freq in segments[1:]:
        if start - result[-1][0] < MIN_NOTE and len(result) > 1:
            result[-1] = (result[-1][0], freq)
        else:
            result.append((start, freq))
    # Leading silence isn't played
    offset = result[0][0]
    return [(start - offset, freq) for start, freq in result]


class Creator5Midi:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        config_dir = os.path.dirname(
            self.printer.get_start_args().get('config_file', ''))
        self.folder = os.path.expanduser(
            config.get('folder', os.path.join(config_dir, 'midi')))
        self.default_level = config.getint('level', 60, minval=1, maxval=100)
        self.beeper = None
        self.thread = None
        self.stop = threading.Event()
        self.playing = None
        self.printer.register_event_handler('klippy:connect', self._connect)
        self.printer.register_event_handler('klippy:shutdown', self._halt)
        self.printer.register_event_handler('klippy:disconnect', self._halt)
        for name, handler, desc in (
                ('C5_MIDI', self.cmd_C5_MIDI,
                 'Play a MIDI file from the midi folder on the buzzer'),
                ('C5_MIDI_LIST', self.cmd_C5_MIDI_LIST,
                 'List the MIDI files in the midi folder'),
                ('C5_MIDI_STOP', self.cmd_C5_MIDI_STOP,
                 'Stop MIDI playback')):
            self.gcode.register_command(name, handler, desc=desc)

    def _connect(self):
        # Only the printer's own buzzer: a tunnel SBC has no cmd_pwm
        self.beeper = self.printer.lookup_object('creator5_beeper', None)

    def _halt(self):
        self.stop.set()

    def _files(self):
        try:
            return sorted(f for f in os.listdir(self.folder)
                          if f.lower().endswith(('.mid', '.midi')))
        except OSError:
            return []

    def cmd_C5_MIDI_LIST(self, gcmd):
        files = self._files()
        gcmd.respond_info("MIDI files in %s:\n%s" % (
            self.folder, "\n".join(files) if files else "(none)"))

    def cmd_C5_MIDI_STOP(self, gcmd):
        if self.thread is None or not self.thread.is_alive():
            gcmd.respond_info("No MIDI playing")
            return
        self.stop.set()
        gcmd.respond_info("Stopping %s" % (self.playing,))

    def cmd_C5_MIDI(self, gcmd):
        if self.beeper is None:
            raise gcmd.error("C5_MIDI needs the printer's own buzzer "
                             "([creator5_beeper]); not available here")
        if self.thread is not None and self.thread.is_alive():
            raise gcmd.error("Already playing %s; C5_MIDI_STOP first"
                             % (self.playing,))
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(
                self.reactor.monotonic())['state'] in (
                'printing', 'paused'):
            raise gcmd.error("Not while printing")
        name = gcmd.get('FILE', None)
        if not name:
            raise gcmd.error("FILE= is required; files: %s"
                             % (", ".join(self._files()) or "(none)"))
        path = os.path.join(self.folder, os.path.basename(name))
        if not os.path.isfile(path):
            raise gcmd.error("No %s in %s" % (os.path.basename(name),
                                               self.folder))
        speed = gcmd.get_float('SPEED', 1., minval=0.1, maxval=10.)
        transpose = gcmd.get_int('TRANSPOSE', 0, minval=-48, maxval=48)
        level = gcmd.get_int('LEVEL', self.default_level, minval=1, maxval=100)
        channel = gcmd.get_int('CHANNEL', 0, minval=0, maxval=16)
        try:
            with open(path, 'rb') as f:
                segments = melody(f.read(), channel - 1 if channel else None,
                                  transpose)
        except (MidiError, IndexError, struct.error) as exc:
            raise gcmd.error("Can't play %s: %s" % (os.path.basename(path),
                                                    exc))
        # The channel is configured here, on the gcode thread, so a failure
        # is reported to the caller; the thread only changes the waveform.
        self.beeper.ensure_configured(self.beeper._run)
        self.stop.clear()
        self.playing = os.path.basename(path)
        self.beeper.busy = True
        self.thread = threading.Thread(
            target=self._play, args=(segments, speed, level),
            name='c5-midi', daemon=True)
        self.thread.start()
        gcmd.respond_info("Playing %s (%.0f s)" % (
            self.playing, segments[-1][0] / speed))

    def _play(self, segments, speed, level, clock=time.monotonic):
        run = self.beeper.execute
        channel = self.beeper.channel
        enabled = False
        start = clock()
        try:
            for i, (at, freq) in enumerate(segments):
                wait = start + at / speed - clock()
                if wait > 0 and self.stop.wait(wait):
                    break
                if self.stop.is_set():
                    break
                # Behind schedule: go straight to the newest due note
                if (i + 1 < len(segments)
                        and start + segments[i + 1][0] / speed <= clock()):
                    continue
                if freq is None:
                    run('set_wc', channel, '1', '0')
                    continue
                high, low = self.beeper.wave_counts(freq, level)
                run('set_wc', channel, str(high), str(low))
                if not enabled:
                    run('enable_channels', channel)
                    enabled = True
        except Exception as exc:
            # Not on the reactor's thread: hand the message to it
            msg = "MIDI playback stopped: %s" % (exc,)
            self.reactor.register_async_callback(
                lambda eventtime: self.gcode.respond_info(msg))
        finally:
            try:
                run('set_wc', channel, '1', '0')
            except Exception:
                pass
            try:
                if enabled:
                    run('disable_channels', channel)
            except Exception:
                pass
            self.beeper.busy = False
            self.playing = None

    def get_status(self, eventtime):
        return {'playing': self.playing, 'folder': self.folder}


def load_config(config):
    return Creator5Midi(config)
