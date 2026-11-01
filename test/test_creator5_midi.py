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


class SongEventsTest(unittest.TestCase):
    def test_keeps_every_note_and_drums_in_order(self):
        notes = track([
            (0, b"\x90\x3c\x64"),        # C4 on
            (0, b"\x40\x64"),            # E4 on (running status)
            (0, b"\x99\x24\x50"),        # drum on (channel 10)
            (240, b"\x90\x3c\x00"),      # C4 off (velocity 0)
            (240, b"\x80\x40\x00"),      # E4 off
        ])
        events, length = midi.song_events(
            smf(track([(0, tempo(500000))]), notes))
        self.assertEqual(events, [
            (0, midi.EV_NOTE_ON, 0, 0x3c, 0x64),
            (0, midi.EV_NOTE_ON, 0, 0x40, 0x64),
            (0, midi.EV_NOTE_ON, 9, 0x24, 0x50),
            (250, midi.EV_NOTE_OFF, 0, 0x3c, 0),
            (500, midi.EV_NOTE_OFF, 0, 0x40, 0)])
        self.assertEqual(length, 501)

    def test_tempo_and_speed(self):
        notes = track([
            (0, b"\x90\x45\x64"), (480, b"\x80\x45\x00"),   # one beat at 120
            (0, tempo(1000000)),                           # then 60 bpm
            (0, b"\x90\x45\x64"), (480, b"\x80\x45\x00"),   # one beat at 60
        ])
        events, _ = midi.song_events(smf(notes, fmt=0))
        self.assertEqual([e[0] for e in events], [0, 500, 500, 1500])
        events, _ = midi.song_events(smf(notes, fmt=0), speed=2.)
        self.assertEqual([e[0] for e in events], [0, 250, 250, 750])

    def test_channel_filter_leaves_out_everything_else(self):
        notes = track([
            (0, b"\x90\x30\x64"), (0, b"\x91\x50\x64"),
            (0, b"\x99\x24\x64"),
            (480, b"\x80\x30\x00"), (0, b"\x81\x50\x00"),
        ])
        events, _ = midi.song_events(smf(notes, fmt=0), channel=0)
        self.assertEqual({e[2] for e in events}, {0})

    def test_volume_and_all_notes_off(self):
        notes = track([
            (0, b"\xb0\x07\x40"),
            (0, b"\x90\x45\x64"),
            (480, b"\xb0\x7b\x00"),
        ])
        events, _ = midi.song_events(smf(notes, fmt=0))
        self.assertEqual([e[1] for e in events],
                         [midi.EV_VOLUME, midi.EV_NOTE_ON, midi.EV_ALL_OFF])
        self.assertEqual(events[0][4], 0x40)

    def test_leading_silence_is_removed(self):
        notes = track([(480, b"\x90\x45\x64"), (480, b"\x80\x45\x00")])
        events, length = midi.song_events(smf(notes, fmt=0))
        self.assertEqual([e[0] for e in events], [0, 500])
        self.assertEqual(length, 501)

    def test_note_steps(self):
        steps = midi.note_steps(12)
        self.assertEqual(steps[69], int(880. * 2 ** 32 / midi.SAMPLE_RATE))
        self.assertTrue(all(0 <= step < 2 ** 32 for step in steps))

    def test_rejects_bad_files(self):
        with self.assertRaisesRegex(midi.MidiError, "not a MIDI"):
            midi.song_events(b"RIFF....")
        with self.assertRaisesRegex(midi.MidiError, "no notes"):
            midi.song_events(smf(track([(0, tempo(500000))])))
        with self.assertRaisesRegex(midi.MidiError, "SMPTE"):
            midi.song_events(smf(track([]), division=0xE728))


class PlaybackTest(unittest.TestCase):
    def make_player(self):
        player = midi.Creator5Midi.__new__(midi.Creator5Midi)
        player.stop = threading.Event()
        player.reactor = mock.Mock()
        player.gcode = mock.Mock()
        player.printer = mock.Mock()
        player.printer.lookup_object.return_value = None
        player.playing = "song.mid"
        player.commands = []
        beeper = mock.Mock(channel='pc12', busy=True)
        beeper.execute.side_effect = lambda *a: player.commands.append(a)
        player.beeper = beeper
        self.ffi = mock.Mock()
        self.lib = mock.Mock()
        self.lib.c5_buzzer_play.return_value = 0
        player._get_ffi = lambda: (self.ffi, self.lib)
        return player

    def play(self, player):
        player._play([(0, midi.EV_NOTE_ON, 0, 69, 100)], 100, 12, 50)

    def test_enables_plays_and_ends_silent(self):
        player = self.make_player()
        self.lib.c5_buzzer_active.side_effect = [1, 0]
        self.play(player)
        self.assertEqual(player.commands, [
            ('set_wc', 'pc12', '1', '0'),
            ('enable_channels', 'pc12'),
            ('set_wc', 'pc12', '1', '0'),
            ('disable_channels', 'pc12')])
        args = self.lib.c5_buzzer_play.call_args.args
        self.assertEqual(args[2:4], (1, 100))
        self.assertEqual(args[5], .5)
        self.lib.c5_buzzer_stop.assert_called_once_with()
        self.assertFalse(player.beeper.busy)
        self.assertIsNone(player.playing)

    def test_stop_ends_playback(self):
        player = self.make_player()
        player.stop.set()
        self.lib.c5_buzzer_active.return_value = 1
        self.play(player)
        self.lib.c5_buzzer_stop.assert_called_once_with()
        self.assertFalse(player.beeper.busy)

    def test_print_starting_ends_playback(self):
        player = self.make_player()
        stats = mock.Mock()
        stats.get_status.return_value = {'state': 'printing'}
        player.printer.lookup_object.return_value = stats
        self.lib.c5_buzzer_active.return_value = 1
        self.play(player)
        self.lib.c5_buzzer_stop.assert_called_once_with()

    def test_driver_failure_is_reported_and_disables(self):
        player = self.make_player()
        self.lib.c5_buzzer_play.return_value = -1
        self.play(player)
        player.reactor.register_async_callback.assert_called_once()
        self.assertEqual(player.commands[-1], ('disable_channels', 'pc12'))
        self.assertFalse(player.beeper.busy)


if __name__ == '__main__':
    unittest.main()
