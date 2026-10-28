import importlib.util
import struct
import threading
import unittest
from pathlib import Path
from unittest import mock

MODULE_PATH = (Path(__file__).resolve().parents[1] / 'klippy' / 'extras'
               / 'creator5_midi.py')
SPEC = importlib.util.spec_from_file_location('creator5_midi', MODULE_PATH)
midi = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(midi)


def varlen(value):
    out = [value & 0x7f]
    value >>= 7
    while value:
        out.insert(0, (value & 0x7f) | 0x80)
        value >>= 7
    return bytes(out)


def track(events):
    """events: (delta ticks, raw bytes after the delta)"""
    body = b"".join(varlen(d) + raw for d, raw in events)
    body += b"\x00\xff\x2f\x00"
    return b"MTrk" + struct.pack(">I", len(body)) + body


def smf(*tracks, division=480, fmt=1):
    return (b"MThd" + struct.pack(">IHHH", 6, fmt, len(tracks), division)
            + b"".join(tracks))


def tempo(us):
    return b"\xff\x51\x03" + us.to_bytes(3, "big")


def freq(note):
    return 440. * 2 ** ((note - 69) / 12.)


class MelodyTest(unittest.TestCase):
    def test_highest_note_wins_and_drums_are_skipped(self):
        notes = track([
            (0, b"\x90\x3c\x64"),        # C4 on
            (0, b"\x40\x64"),            # E4 on (running status)
            (240, b"\x43\x64"),          # G4 on at half a beat
            (0, b"\x99\x24\x64"),        # drum on (channel 10): ignored
            (240, b"\x90\x43\x00"),      # G4 off (velocity 0)
            (240, b"\x80\x40\x00"),      # E4 off
            (0, b"\x80\x3c\x00"),        # C4 off
        ])
        segs = midi.melody(smf(track([(0, tempo(500000))]), notes))
        self.assertEqual([round(t, 3) for t, _ in segs], [0., .25, .5, .75])
        self.assertAlmostEqual(segs[0][1], freq(0x40))
        self.assertAlmostEqual(segs[1][1], freq(0x43))
        self.assertAlmostEqual(segs[2][1], freq(0x40))
        self.assertIsNone(segs[3][1])

    def test_tempo_change_and_transpose(self):
        notes = track([
            (0, b"\x90\x45\x64"), (480, b"\x80\x45\x00"),   # one beat at 120
            (0, tempo(1000000)),                           # then 60 bpm
            (0, b"\x90\x45\x64"), (480, b"\x80\x45\x00"),   # one beat at 60
        ])
        segs = midi.melody(smf(notes, fmt=0), transpose=12)
        # The repeat is separated by a short gap so it is heard twice
        self.assertEqual([round(t, 3) for t, _ in segs], [0., .5, .53, 1.5])
        self.assertAlmostEqual(segs[0][1], 880.)
        self.assertIsNone(segs[1][1])
        self.assertAlmostEqual(segs[2][1], 880.)

    def test_channel_filter_and_short_notes(self):
        notes = track([
            (0, b"\x90\x30\x64"),          # channel 1, low
            (0, b"\x91\x50\x64"),          # channel 2, high
            (480, b"\x80\x30\x00"), (0, b"\x81\x50\x00"),
            (0, b"\x90\x48\x64"), (4, b"\x80\x48\x00"),   # 4 ticks: ~4 ms
            (0, b"\x90\x4a\x64"), (480, b"\x80\x4a\x00"),
        ])
        only_first = midi.melody(smf(notes, fmt=0), channel=0)
        self.assertAlmostEqual(only_first[0][1], freq(0x30))
        everything = midi.melody(smf(notes, fmt=0))
        self.assertAlmostEqual(everything[0][1], freq(0x50))
        # The 4 ms note is merged away; the next one plays from its start
        self.assertNotIn(round(freq(0x48), 3),
                         [round(f, 3) for _, f in everything if f])
        self.assertIn(round(freq(0x4a), 3),
                      [round(f, 3) for _, f in everything if f])

    def test_repeated_notes_are_not_merged(self):
        notes = track([
            (0, b"\x90\x45\x64"),          # A4 on
            (240, b"\x90\x45\x64"),        # A4 struck again while held
            (240, b"\x91\x45\x64"),        # A4 again from another channel
            (240, b"\x80\x45\x00"), (0, b"\x80\x45\x00"),
            (0, b"\x81\x45\x00"),
        ])
        segs = midi.melody(smf(notes, fmt=0))
        sounding = [round(t, 3) for t, f in segs if f]
        self.assertEqual(sounding, [0., .28, .53])      # three separate notes
        gaps = [round(t, 3) for t, f in segs if f is None]
        self.assertEqual(gaps, [.25, .5, .75])

    def test_unison_on_two_channels_is_one_note(self):
        notes = track([
            (0, b"\x90\x45\x64"), (0, b"\x91\x45\x64"),
            (480, b"\x80\x45\x00"), (0, b"\x81\x45\x00"),
        ])
        segs = midi.melody(smf(notes, fmt=0))
        self.assertEqual([round(t, 3) for t, f in segs if f], [0.])

    def test_rejects_bad_files(self):
        with self.assertRaisesRegex(midi.MidiError, "not a MIDI"):
            midi.melody(b"RIFF....")
        with self.assertRaisesRegex(midi.MidiError, "no notes"):
            midi.melody(smf(track([(0, tempo(500000))])))
        with self.assertRaisesRegex(midi.MidiError, "SMPTE"):
            midi.melody(smf(track([]), division=0xE728))


class PlaybackTest(unittest.TestCase):
    def make_player(self):
        player = midi.Creator5Midi.__new__(midi.Creator5Midi)
        player.stop = threading.Event()
        player.reactor = mock.Mock()
        player.gcode = mock.Mock()
        player.playing = "song.mid"
        player.commands = []
        beeper = mock.Mock(channel='pc12', busy=True)
        beeper.execute.side_effect = lambda *a: player.commands.append(a)
        beeper.wave_counts.side_effect = lambda f, level: (int(f), 1)
        player.beeper = beeper
        return player

    def test_enables_once_changes_waveform_and_ends_silent(self):
        player = self.make_player()
        now = [0.]
        player.stop.wait = lambda delay: now.__setitem__(0, now[0] + delay)
        player._play([(0., 440.), (0.5, None), (1., 880.), (1.5, None)],
                     1., 60, clock=lambda: now[0])
        self.assertEqual(player.commands, [
            ('set_wc', 'pc12', '440', '1'),
            ('enable_channels', 'pc12'),
            ('set_wc', 'pc12', '1', '0'),
            ('set_wc', 'pc12', '880', '1'),
            ('set_wc', 'pc12', '1', '0'),
            ('set_wc', 'pc12', '1', '0'),
            ('disable_channels', 'pc12')])
        self.assertFalse(player.beeper.busy)
        self.assertIsNone(player.playing)

    def test_stop_silences_and_disables(self):
        player = self.make_player()
        player.stop.set()
        player._play([(0., 440.), (5., None)], 1., 60, clock=lambda: 0.)
        self.assertEqual(player.commands, [('set_wc', 'pc12', '1', '0')])
        self.assertFalse(player.beeper.busy)


if __name__ == '__main__':
    unittest.main()
