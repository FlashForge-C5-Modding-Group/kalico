# Play MIDI files on the Creator 5's host PWM buzzer.
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Every instrument plays at once: chelper/c5_buzzer.c mixes square waves and
# noise for drums, and writes each sample to the PWM driver from a real-time
# thread. The channel is configured once (shared with [creator5_beeper]).
import os
import struct
import threading

SAMPLE_RATE = 16000
DEFAULT_TRANSPOSE = 12  # semitones: the buzzer is thin below this
EV_NOTE_OFF, EV_NOTE_ON, EV_VOLUME, EV_ALL_OFF = range(4)
DRUM_CHANNEL = 9


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
            elif kind == 0xb0 and note == 7:
                events.append((tick, order, "volume", channel, 0, velocity))
            elif kind == 0xb0 and note in (120, 123):
                events.append((tick, order, "all_off", channel, 0, 0))
            order += 1
        pos = end
    # Same tick: tempo first, then note-offs, then note-ons
    rank = {"tempo": 0, "volume": 0, "off": 1, "all_off": 1, "on": 2}
    events.sort(key=lambda e: (e[0], rank[e[2]], e[1]))
    return division, events


def song_events(data, channel=None, speed=1.):
    """([(ms, kind, channel, note, value)], length ms) with leading silence
    removed; channel limits the song to that one, without its drums."""
    division, events = parse_midi(data)
    tempo = 500000.         # us per quarter note until a tempo event
    last_tick = 0
    now = 0.
    out = []
    first_note = None
    for tick, _, kind, chan, note, value in events:
        now += (tick - last_tick) * tempo / division / 1e3 / speed
        last_tick = tick
        if kind == "tempo":
            tempo = float(value)
            continue
        if channel is not None and chan != channel:
            continue
        if kind == "on":
            if first_note is None:
                first_note = now
            out.append((now, EV_NOTE_ON, chan, note, value))
        elif kind == "off":
            out.append((now, EV_NOTE_OFF, chan, note, 0))
        elif kind == "volume":
            out.append((now, EV_VOLUME, chan, 0, value))
        else:
            out.append((now, EV_ALL_OFF, chan, 0, 0))
    if first_note is None:
        raise MidiError("no notes to play")
    return ([(int(max(at - first_note, 0.)), kind, chan, note, value)
             for at, kind, chan, note, value in out],
            int(now - first_note) + 1)


def note_steps(transpose):
    """Oscillator step per sample for each MIDI note."""
    return [int(440. * 2 ** ((note + transpose - 69) / 12.)
                * 4294967296. / SAMPLE_RATE) & 0xffffffff
            for note in range(128)]


class Creator5Midi:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        config_dir = os.path.dirname(
            self.printer.get_start_args().get('config_file', ''))
        self.folder = os.path.expanduser(
            config.get('folder', os.path.join(config_dir, 'midi')))
        self.default_level = config.getint('level', 100, minval=1, maxval=100)
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
        transpose = gcmd.get_int('TRANSPOSE', DEFAULT_TRANSPOSE,
                                 minval=-48, maxval=48)
        level = gcmd.get_int('LEVEL', self.default_level, minval=1, maxval=100)
        channel = gcmd.get_int('CHANNEL', 0, minval=0, maxval=16)
        try:
            with open(path, 'rb') as f:
                events, length_ms = song_events(
                    f.read(), channel - 1 if channel else None, speed)
        except (MidiError, IndexError, struct.error) as exc:
            raise gcmd.error("Can't play %s: %s" % (os.path.basename(path),
                                                    exc))
        # The channel is configured here, on the gcode thread, so a failure
        # is reported to the caller.
        self.beeper.ensure_configured(self.beeper._run)
        self.stop.clear()
        self.playing = os.path.basename(path)
        self.beeper.busy = True
        self.thread = threading.Thread(
            target=self._play, args=(events, length_ms, transpose, level),
            name='c5-midi', daemon=True)
        self.thread.start()
        gcmd.respond_info("Playing %s (%.0f s)" % (
            self.playing, length_ms / 1000.))

    def _printing(self):
        stats = self.printer.lookup_object('print_stats', None)
        return stats is not None and stats.get_status(
            self.reactor.monotonic())['state'] in ('printing', 'paused')

    def _get_ffi(self):
        from klippy import chelper
        return chelper.get_ffi()

    def _play(self, events, length_ms, transpose, level):
        run = self.beeper.execute
        channel = self.beeper.channel
        ffi_main, ffi_lib = self._get_ffi()
        enabled = False
        try:
            run('set_wc', channel, '1', '0')
            run('enable_channels', channel)
            enabled = True
            song = ffi_main.new('struct c5_buzzer_event[]', [
                (at, kind, chan, note, value)
                for at, kind, chan, note, value in events])
            steps = ffi_main.new('uint32_t[]', note_steps(transpose))
            name = ffi_main.new('char[]', channel.encode())
            if ffi_lib.c5_buzzer_play(name, song, len(events), length_ms,
                                      steps, level / 100.):
                raise RuntimeError("can't open the PWM driver")
            while ffi_lib.c5_buzzer_active() and not self._printing():
                if self.stop.wait(.05):
                    break
        except Exception as exc:
            # Not on the reactor's thread: hand the message to it
            msg = "MIDI playback stopped: %s" % (exc,)
            self.reactor.register_async_callback(
                lambda eventtime: self.gcode.respond_info(msg))
        finally:
            ffi_lib.c5_buzzer_stop()
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
