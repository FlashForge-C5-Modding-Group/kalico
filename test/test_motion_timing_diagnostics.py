# Offline motion timing capture regression tests
#
# Copyright (C) 2026 wondercrash <155874349+wondercrash@users.noreply.github.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Offline timing capture regressions; no printer devices are opened.

The native cases need the Linux helper built by test/conftest.py (gcc, CFFI,
project dependencies).  socketpair peers acknowledge real protocol blocks;
only Python clock/step generation and absent printer hardware are controlled.
"""
import copy
import io
import json
import socket
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from klippy import chelper, mcu, msgproto, reactor, serialhdl
from klippy.extras import motion_queuing, print_stats, virtual_sdcard


class ControlledReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0

    def __init__(self):
        self.now = 10.0
        self.callbacks = []
        self.timer_updates = []

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        return SimpleNamespace(callback=callback, waketime=waketime)

    def update_timer(self, timer, waketime):
        timer.waketime = waketime
        self.timer_updates.append(waketime)

    def register_callback(self, callback, waketime=NOW):
        self.callbacks.append(callback)

    register_async_callback = register_callback

    def drain(self):
        callbacks, self.callbacks = self.callbacks, []
        for callback in callbacks:
            callback(self.now)


class OfflineGCode:
    error = RuntimeError

    def __init__(self):
        self.commands = {}
        self.options = {}
        self.responses = []

    def register_command(self, name, callback, **kwargs):
        self.commands[name] = callback
        self.options[name] = kwargs

    def respond_info(self, message, **kwargs):
        self.responses.append(message)


class Command:
    error = RuntimeError

    def __init__(self, enable=None):
        self.enable = enable
        self.responses = []

    def get_int(self, name, default=None, **kwargs):
        value = self.enable if name == 'ENABLE' else default
        if value is None:
            raise self.error('Missing parameter ' + name)
        if 'minval' in kwargs and value < kwargs['minval']:
            raise self.error('Below minimum')
        if 'maxval' in kwargs and value > kwargs['maxval']:
            raise self.error('Above maximum')
        return value

    def respond_info(self, message, **kwargs):
        self.responses.append(message)


class OfflineMCU:
    error = RuntimeError

    def __init__(self, name, reactor):
        self.name = name
        self.reactor = reactor
        self.enabled = False
        self.format_calls = []
        self.live_dumps = 0
        self.snapshot_reads = 0
        self.history = b'first-history'

    def get_name(self):
        return self.name

    def is_fileoutput(self):
        return False

    def estimated_print_time(self, eventtime):
        return eventtime

    def set_timing_diagnostics(self, enabled):
        self.enabled = bool(enabled)

    def get_timing_diagnostics(self):
        if not self.enabled:
            return []
        return [dict(kind=1, bytes=7, eventtime=self.reactor.now,
                     conv_time=10.0, est_freq=1000000.0,
                     req_clock=4294967300, min_clock=4294967200,
                     conv_clock=4294967000, send_seq=1, receive_seq=1,
                     ready_bytes=0, upcoming_bytes=7, need_ack_bytes=0)]

    def get_timing_debug_snapshot(self):
        self.snapshot_reads += 1
        return dict(connected=True, sent=[dict(data=self.history,
                    sent_time=self.reactor.now,
                    receive_time=self.reactor.now + .001)], received=[])

    def format_timing_debug_snapshot(self, snapshot):
        self.format_calls.append(snapshot)
        result = copy.deepcopy(snapshot)
        for side in ('sent', 'received'):
            for record in result[side]:
                record['data'] = record['data'].hex()
                record['decoded'] = ['offline history']
        return result

    def dump_timing_debug(self, snapshot=None):
        self.live_dumps += 1
        return 'offline live debug ' + self.name


class OfflinePrinter:
    command_error = RuntimeError

    def __init__(self, reactor, boards):
        self.reactor = reactor
        self.boards = boards
        self.handlers = {}
        self.gcode = OfflineGCode()
        self.stats = SimpleNamespace(
            state='standby', get_status=lambda eventtime: {'state': self.stats.state})
        self.gcode_move = SimpleNamespace(get_status=lambda eventtime=None: {
            'position': SimpleNamespace(e=0.0), 'extrude_factor': 1.0})
        self.toolhead = SimpleNamespace(
            get_last_move_time=lambda: self.reactor.now + .250,
            register_lookahead_callback=lambda callback:
                callback(self.reactor.now + .250))
        self.warning_receipts = []
        self.register_event_handler('mcu:timer_too_close',
                                    lambda *args: self.warning_receipts.append(args))

    def get_reactor(self):
        return self.reactor

    def lookup_objects(self, module=None):
        return [(board.get_name(), board) for board in self.boards]

    def lookup_object(self, name, default=None):
        objects = {'gcode': self.gcode, 'print_stats': self.stats,
                   'gcode_move': self.gcode_move, 'toolhead': self.toolhead}
        objects.update({board.get_name(): board for board in self.boards})
        return objects.get(name, default)

    def load_object(self, config, name, default=None):
        return self.lookup_object(name, default)

    def register_event_handler(self, name, callback):
        self.handlers.setdefault(name, []).append(callback)

    def send_event(self, name, *args):
        for callback in self.handlers.get(name, []):
            callback(*args)

    def invoke_shutdown(self, message):
        raise AssertionError('Unexpected shutdown: ' + message)

    def is_shutdown(self):
        return False

    def get_start_args(self):
        return {}


class OfflineShutdownPrinter(OfflinePrinter):
    """Deliver shutdown asynchronously, without a hardware printer lifecycle."""
    def __init__(self, reactor, boards):
        super().__init__(reactor, boards)
        self.shutdown_messages = []

    def invoke_async_shutdown(self, message):
        self.reactor.register_async_callback(
            lambda eventtime: self.invoke_shutdown(message))

    def invoke_shutdown(self, message):
        if self.is_shutdown():
            return
        self.shutdown_messages.append(message)
        self.send_event('klippy:shutdown')

    def is_shutdown(self):
        return bool(self.shutdown_messages)


class MotionTimingTests(unittest.TestCase):
    def setUp(self):
        self.clock = ControlledReactor()
        self.boards = [OfflineMCU('mcu', self.clock),
                       OfflineMCU('eboard', self.clock)]
        self.printer = OfflinePrinter(self.clock, self.boards)
        self.config = SimpleNamespace(get_printer=lambda: self.printer)
        patcher = mock.patch.object(motion_queuing, 'get_danger_options',
                                    return_value=SimpleNamespace(bgflush_extra_time=.250))
        patcher.start()
        self.addCleanup(patcher.stop)
        self.motion = motion_queuing.PrinterMotionQueuing(self.config)
        # No configured steppers: replace the hardware generation boundary only.
        self.motion.steppersyncmgr_gen_steps = lambda *args: 0
        self.log_messages = []
        logger = mock.patch.object(motion_queuing.logging, 'info', self.collect_log)
        logger.start()
        self.addCleanup(logger.stop)

    def collect_log(self, message, *args, **kwargs):
        self.log_messages.append(message % args if args else message)

    def dispatch(self, name, enable=None):
        self.assertIn(name, self.printer.gcode.commands,
                      'Missing motion timing diagnostic G-code command: ' + name)
        command = Command(enable)
        self.printer.gcode.commands[name](command)
        self.clock.drain()
        return command

    def enable(self):
        self.dispatch('SET_MOTION_TIMING_DIAGNOSTICS', 1)

    def dump(self):
        before = len(self.log_messages)
        self.dispatch('DUMP_MOTION_TIMING_DIAGNOSTICS')
        captures = [json.loads(message.split(' ', 1)[1])
                    for message in self.log_messages[before:]
                    if message.startswith('MOTION_TIMING_DIAGNOSTICS ')]
        self.assertEqual(len(captures), 1, 'Dump must emit one JSON capture')
        return captures[0]

    def records(self, kind):
        return [record for record in self.dump()['python_ring']
                if record['kind'] == kind]

    def captures(self):
        return [json.loads(message.split(' ', 1)[1])
                for message in self.log_messages
                if message.startswith('MOTION_TIMING_DIAGNOSTICS ')]

    def start_print(self, filename='timing.gcode'):
        if not isinstance(self.printer.stats, print_stats.PrintStats):
            self.printer.stats = print_stats.PrintStats(self.config)
        stats = self.printer.stats
        stats.set_current_file(filename)
        stats.note_start()
        return stats

    def assert_same_evidence(self, actual, expected):
        self.assertEqual(
            {key: value for key, value in actual.items() if key != 'trigger'},
            {key: value for key, value in expected.items() if key != 'trigger'})

    def test_terminal_boundaries_freeze_before_deferred_dump_and_stay_frozen(self):
        for terminal in ('complete', 'cancelled', 'error', 'shutdown'):
            with self.subTest(terminal=terminal):
                self.clock.now = 10.0
                self.enable()
                stats = self.start_print()
                self.motion._advance_flush_time(10.0)
                self.printer.send_event('mcu:timer_too_close', 'mcu', 10.0)
                self.clock.now = 11.0
                self.motion._advance_flush_time(11.0)
                self.log_messages.clear()
                for board in self.boards:
                    board.format_calls.clear()
                self.clock.now = 12.0
                if terminal == 'shutdown':
                    self.printer.send_event('klippy:shutdown')
                    trigger = 'klippy:shutdown'
                else:
                    {'complete': stats.note_complete,
                     'cancelled': stats.note_cancel,
                     'error': lambda: stats.note_error('offline failure')}[terminal]()
                    trigger = 'print_stats:%s_printing' % terminal
                self.assertFalse(any(board.enabled for board in self.boards),
                                 'Native collection must stop at the boundary')
                self.assertEqual(self.log_messages, [],
                                 'Terminal formatting/logging must be deferred')
                self.assertTrue(all(not board.format_calls for board in self.boards))

                self.clock.now = 13.0
                for board in self.boards:
                    board.history = b'post-print activity'
                self.motion._advance_flush_time(13.0)
                self.printer.send_event('mcu:timer_too_close', 'mcu', 13.0)
                self.clock.now = 14.0
                self.clock.drain()
                emitted = self.captures()
                self.assertEqual(len(emitted), 1)
                frozen = emitted[0]
                self.assertEqual(frozen['trigger'], trigger)
                self.assertEqual(frozen['host_time'], 12.0)
                self.assertEqual([record['entry_time']
                                  for record in frozen['python_ring']], [10.0, 11.0])
                self.assertEqual(frozen['serial']['mcu'][0]['eventtime'], 12.0)
                self.assertEqual(frozen['first_warning']['host_time'], 10.0)
                self.assertEqual(frozen['latest_warning']['host_time'], 10.0)
                self.clock.now = 20.0
                self.assert_same_evidence(self.dump(), frozen)
                self.assert_same_evidence(self.dump(), frozen)
                before = len(self.captures())
                self.printer.send_event(trigger)
                self.printer.send_event('klippy:shutdown')
                self.clock.drain()
                self.assertEqual(len(self.captures()), before,
                                 'Duplicate terminal signals must be idempotent')

    def test_empty_print_is_frozen_and_automatically_dumped(self):
        self.enable()
        stats = self.start_print('empty.gcode')
        self.clock.now = 11.0
        stats.note_complete()
        self.clock.now = 12.0
        self.clock.drain()
        self.assertEqual(len(self.captures()), 1)
        frozen = self.captures()[0]
        self.assertEqual(frozen['host_time'], 11.0)
        self.assertEqual(frozen['python_ring'], [])
        self.assertIsNone(frozen['first_warning'])
        self.assertIsNone(frozen['latest_warning'])
        self.assert_same_evidence(self.dump(), frozen)

    def test_next_print_rearms_fresh_but_pause_resume_keeps_same_capture(self):
        self.enable()
        stats = self.start_print('first.gcode')
        self.motion._advance_flush_time(10.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 10.0)
        self.clock.now = 11.0
        stats.note_complete()
        self.clock.drain()
        self.clock.now = 12.0
        self.start_print('second.gcode')
        fresh = self.dump()
        self.assertEqual(fresh['python_ring'], [])
        self.assertIsNone(fresh['first_warning'])
        self.assertIsNone(fresh['latest_warning'])
        self.assertTrue(all(board.enabled for board in self.boards),
                        'Print completion must retain the logical enable flag')
        self.motion._advance_flush_time(12.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 12.0)
        stats.note_pause()
        self.clock.now = 13.0
        self.motion._advance_flush_time(13.0)
        stats.note_start()
        self.clock.now = 14.0
        self.motion._advance_flush_time(14.0)
        resumed = self.dump()
        self.assertEqual([record['entry_time'] for record in resumed['python_ring']],
                         [12.0, 13.0, 14.0])
        self.assertEqual(resumed['first_warning']['host_time'], 12.0)
        self.assertTrue(all(board.enabled for board in self.boards))

    def test_idle_reset_and_file_selection_retain_completed_capture(self):
        self.enable()
        stats = self.start_print()
        self.motion._advance_flush_time(10.0)
        self.clock.now = 11.0
        stats.note_complete()
        self.clock.drain()
        frozen = self.captures()[-1]
        self.clock.now = 12.0
        stats.reset()
        stats.set_current_file('not-started.gcode')
        self.motion._advance_flush_time(12.0)
        self.assert_same_evidence(self.dump(), frozen)
        self.assertEqual(len([capture for capture in self.captures()
                              if capture['trigger'] != 'explicit']), 1)

    def test_explicit_disable_retains_capture_and_new_prints_stay_disabled(self):
        self.enable()
        self.motion._advance_flush_time(10.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 10.0)
        self.clock.now = 11.0
        self.dispatch('SET_MOTION_TIMING_DIAGNOSTICS', 0)
        frozen = self.dump()
        self.assertEqual(frozen['host_time'], 11.0)
        self.clock.now = 12.0
        stats = self.start_print('disabled.gcode')
        self.motion._advance_flush_time(12.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 12.0)
        self.assertFalse(any(board.enabled for board in self.boards))
        self.clock.now = 13.0
        stats.note_complete()
        self.clock.drain()
        self.assert_same_evidence(self.dump(), frozen)
        self.assertFalse(any(capture['trigger'] != 'explicit'
                             for capture in self.captures()))
        self.clock.now = 14.0
        self.enable()
        fresh = self.dump()
        self.assertEqual(fresh['python_ring'], [])
        self.assertIsNone(fresh['first_warning'])
        self.assertIsNone(fresh['latest_warning'])
        self.motion._advance_flush_time(14.0)
        self.assertEqual([record['entry_time']
                          for record in self.dump()['python_ring']], [14.0])

    def test_pending_terminal_dump_owns_old_capture_across_new_session(self):
        for replacement in ('new-print', 'explicit-enable'):
            with self.subTest(replacement=replacement):
                self.clock.now = 10.0
                self.enable()
                stats = self.start_print('first.gcode')
                self.motion._advance_flush_time(10.0)
                self.printer.send_event('mcu:timer_too_close', 'mcu', 10.0)
                self.clock.now = 11.0
                stats.note_complete()
                self.log_messages.clear()
                self.clock.now = 12.0
                if replacement == 'new-print':
                    self.start_print('second.gcode')
                else:
                    # Do not drain: the old automatic dump is still pending.
                    self.printer.gcode.commands[
                        'SET_MOTION_TIMING_DIAGNOSTICS'](Command(1))
                self.motion._advance_flush_time(12.0)
                self.clock.now = 13.0
                self.clock.drain()
                self.assertEqual(len(self.captures()), 1)
                old = self.captures()[0]
                self.assertEqual(old['host_time'], 11.0)
                self.assertEqual([record['entry_time']
                                  for record in old['python_ring']], [10.0])
                self.assertEqual(old['first_warning']['host_time'], 10.0)
                fresh = self.dump()
                self.assertEqual([record['entry_time']
                                  for record in fresh['python_ring']], [12.0])
                self.assertIsNone(fresh['first_warning'])
                self.assertTrue(all(board.enabled for board in self.boards),
                                'Stale terminal callbacks must not stop a new capture')
                self.clock.now = 14.0
                self.motion._advance_flush_time(14.0)
                self.assertEqual([record['entry_time']
                                  for record in self.dump()['python_ring']],
                                 [12.0, 14.0])
                if replacement == 'new-print':
                    stats.note_complete()
                    self.clock.drain()
                    automatic = [capture for capture in self.captures()
                                 if capture['trigger'] != 'explicit']
                    self.assertEqual(len(automatic), 2)
                    self.assertEqual(automatic[-1]['host_time'], 14.0)
                    self.assertEqual([record['entry_time']
                                      for record in automatic[-1]['python_ring']],
                                     [12.0, 14.0])
                    self.assertIsNone(automatic[-1]['first_warning'])

    def test_old_received_warning_cannot_contaminate_next_print(self):
        self.enable()
        stats = self.start_print('first.gcode')
        self.clock.now = 11.0
        stats.note_complete()
        self.clock.drain()
        self.clock.now = 12.0
        self.start_print('second.gcode')
        self.clock.now = 13.0
        self.printer.send_event('mcu:timer_too_close', 'mcu', 10.5)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 11.5)
        capture = self.dump()
        self.assertIsNone(capture['first_warning'])
        self.assertIsNone(capture['latest_warning'])
        self.printer.send_event('mcu:timer_too_close', 'mcu', 12.5)
        self.assertIsNotNone(self.dump()['first_warning'])

    def test_delayed_active_receipts_use_capture_clock_and_throttle_real_copies(self):
        self.enable()
        self.start_print()
        self.clock.now = 20.0
        self.printer.send_event('mcu:timer_too_close', 'mcu', 11.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', 13.0)
        self.assertEqual([board.snapshot_reads for board in self.boards], [1, 1],
                         'Spread-out receipts processed together need one snapshot')
        capture = self.dump()
        self.assertEqual(capture['first_warning']['host_time'], 20.0)
        self.assertEqual(capture['latest_warning']['host_time'], 20.0)
        self.assertEqual(capture['first_warning']['serial']['mcu'][0]['eventtime'],
                         20.0)
        before = [board.snapshot_reads for board in self.boards]
        self.clock.now = 20.5
        self.printer.send_event('mcu:timer_too_close', 'mcu', 19.0)
        self.assertEqual([board.snapshot_reads for board in self.boards], before)
        self.clock.now = 21.01
        self.printer.send_event('mcu:timer_too_close', 'mcu', 20.0)
        self.assertEqual([board.snapshot_reads for board in self.boards],
                         [value + 1 for value in before])
        capture = self.dump()
        self.assertEqual(capture['first_warning']['host_time'], 20.0)
        self.assertEqual(capture['latest_warning']['host_time'], 21.01)

    def test_real_reactor_preserves_queued_serial_warning_before_terminal_dump(self):
        clock = reactor.Reactor()
        self.addCleanup(clock.finalize)
        clock.now = clock.monotonic()
        board = OfflineMCU('mcu', clock)
        printer = OfflinePrinter(clock, [board])
        board.get_printer = lambda: printer
        config = SimpleNamespace(get_printer=lambda: printer)
        motion = motion_queuing.PrinterMotionQueuing(config)
        motion.steppersyncmgr_gen_steps = lambda *args: 0
        stats = print_stats.PrintStats(config)
        printer.stats = stats
        reader = serialhdl.SerialReader.__new__(serialhdl.SerialReader)
        reader.reactor = clock
        reader.mcu = board
        reader.warn_prefix = ''
        printer.gcode.commands['SET_MOTION_TIMING_DIAGNOSTICS'](Command(1))

        def finish_with_queued_receipt(eventtime):
            stats.set_current_file('real-reactor.gcode')
            stats.note_start()
            clock.now = clock.monotonic()
            motion._advance_flush_time(clock.now)
            # The actual serial entrypoint queues asynchronously. Its FIFO
            # does not become reactor timers until the pipe is serviced.
            reader.handle_output({'#name': 'output', '#msg': 'Timer too close',
                                  '#receive_time': clock.now})
            self.assertEqual(printer.warning_receipts, [])
            stats.note_complete()
            clock.register_async_callback(lambda eventtime: clock.end())

        self.log_messages.clear()
        clock.register_callback(finish_with_queued_receipt)
        clock.run()
        emitted = self.captures()
        self.assertEqual(len(emitted), 1)
        capture = emitted[0]
        self.assertEqual(len(printer.warning_receipts), 1)
        self.assertIsNotNone(capture['first_warning'],
                             'Queued pre-end serial receipt must precede publication')
        warning = capture['first_warning']
        self.assertEqual(warning['host_time'], capture['host_time'])
        self.assertEqual(warning['python_ring'], capture['python_ring'])
        self.assertEqual(warning['serial'], capture['serial'])

    def test_pre_end_delayed_warning_uses_frozen_boundary_evidence(self):
        self.enable()
        stats = self.start_print()
        self.motion._advance_flush_time(10.0)
        self.clock.register_async_callback(
            lambda eventtime: self.printer.send_event(
                'mcu:timer_too_close', 'mcu', 11.0))
        self.clock.now = 12.0
        self.boards[0].history = b'boundary history'
        stats.note_complete()
        self.clock.now = 13.0
        self.boards[0].history = b'late history'
        self.motion._advance_flush_time(13.0)
        self.clock.drain()
        self.assertEqual(len(self.captures()), 1)
        frozen = self.captures()[0]
        warning = frozen['first_warning']
        self.assertIsNotNone(warning)
        self.assertEqual(warning['host_time'], frozen['host_time'])
        self.assertEqual(warning['python_ring'], frozen['python_ring'])
        self.assertEqual(warning['serial'], frozen['serial'])
        self.assertEqual(warning['raw_history']['mcu']['sent'][0]['data'],
                         b'boundary history'.hex())
        self.printer.send_event('mcu:timer_too_close', 'mcu', 12.5)
        self.assert_same_evidence(self.dump(), frozen)

    def test_pre_end_warning_after_publication_attaches_only_frozen_evidence(self):
        self.enable()
        stats = self.start_print()
        self.motion._advance_flush_time(10.0)
        self.clock.now = 12.0
        self.boards[0].history = b'boundary history'
        stats.note_complete()
        self.clock.drain()
        frozen = self.captures()[0]
        self.assertIsNone(frozen['first_warning'])
        self.clock.now = 13.0
        self.boards[0].history = b'post-publication history'
        before = [board.snapshot_reads for board in self.boards]
        self.printer.send_event('mcu:timer_too_close', 'mcu', 11.0)
        updated = self.dump()
        self.assertEqual([board.snapshot_reads for board in self.boards], before,
                         'Late receipt must not recapture live idle evidence')
        self.assertIsNone(self.captures()[0]['first_warning'],
                          'Already-published log must remain historical')
        warning = updated['first_warning']
        self.assertIsNotNone(warning)
        self.assertEqual(warning['host_time'], frozen['host_time'])
        self.assertEqual(warning['python_ring'], frozen['python_ring'])
        self.assertEqual(warning['serial'], frozen['serial'])
        self.assertEqual(warning['raw_history']['mcu']['sent'][0]['data'],
                         b'boundary history'.hex())
        for field in ('host_time', 'python_ring', 'serial'):
            self.assertEqual(updated[field], frozen[field])
        self.assert_same_evidence(self.dump(), updated)

    def test_reentrant_generation_cannot_append_old_record_to_new_print(self):
        self.enable()
        stats = self.start_print('first.gcode')

        def finish_and_start_during_generation(*args):
            self.clock.now = 11.0
            stats.note_complete()
            self.clock.now = 12.0
            self.start_print('second.gcode')
            return 0

        self.motion.steppersyncmgr_gen_steps = finish_and_start_during_generation
        self.motion._advance_flush_time(10.0)
        self.clock.drain()  # Emit the old print's pending automatic dump first.
        self.assertEqual(self.dump()['python_ring'], [],
                         'A yielding old generation must not record into the new print')
        self.motion.steppersyncmgr_gen_steps = lambda *args: 0
        self.clock.now = 13.0
        self.motion._advance_flush_time(13.0)
        self.assertEqual([record['entry_time']
                          for record in self.dump()['python_ring']], [13.0])

    def test_callback_and_generation_delays_have_separate_real_boundaries(self):
        self.enable()
        self.motion.need_step_gen_time = 14.0
        self.motion.last_flush_time = 11.0
        self.motion.last_step_gen_time = 12.0
        calls = []

        def slow_callback(flush_time, step_time):
            calls.append(('callback', flush_time, step_time))
            self.clock.now += .125

        def slow_generation(manager, flush_time, step_time, history_time):
            calls.append(('generation', flush_time, step_time))
            self.clock.now += .375
            return 0

        self.motion.register_flush_callback(slow_callback)
        self.motion.steppersyncmgr_gen_steps = slow_generation
        self.motion._advance_flush_time(9.0, 13.0)
        record = self.records('advance_flush_time')[-1]
        self.assertEqual(calls, [('callback', 12.95, 13.0),
                                 ('generation', 12.95, 13.0)])
        self.assertEqual(record['entry_time'], 10.0)
        self.assertEqual(record['callbacks_time'], 10.125)
        self.assertEqual(record['gen_steps_time'], 10.5)
        self.assertEqual(record['want_flush_time'], 9.0)
        self.assertEqual(record['want_step_gen_time'], 13.0)
        self.assertEqual(record['flush_time'], 12.95)
        self.assertEqual(record['step_gen_time'], 13.0)
        self.assertEqual(record['need_step_gen_time'], 14.0)
        self.assertEqual(record['estimated_print_time'], 10.0)

    def test_timer_deadline_measures_late_invocation_and_keeps_never_exit(self):
        self.enable()
        self.motion.need_flush_time = 20.0
        first_deadline = self.motion._flush_handler(10.0)
        self.assertAlmostEqual(first_deadline, 10.2)
        self.clock.now = 10.275
        second_deadline = self.motion._flush_handler(10.275)
        records = self.records('flush_handler')
        self.assertIsNone(records[0]['expected_time'])
        self.assertAlmostEqual(records[1]['expected_time'], 10.2)
        self.assertEqual(records[1]['entry_time'], 10.275)
        self.assertEqual(records[1]['deadline'], second_deadline)
        self.assertAlmostEqual(records[1]['entry_time'] - records[1]['expected_time'], .075)
        self.motion.need_flush_time = 0.0
        deadline = self.motion._flush_handler(self.clock.now)
        self.assertEqual(deadline, self.clock.NEVER)
        self.assertEqual(self.records('flush_handler')[-1]['deadline'], self.clock.NEVER)

    def test_external_movequeue_kick_invalidates_expected_deadline(self):
        self.enable()
        self.motion.need_flush_time = 20.0
        self.motion._flush_handler(10.0)
        self.motion.do_kick_flush_timer = True
        self.clock.now = 10.1
        self.motion.note_mcu_movequeue_activity(21.0, is_step_gen=False)
        self.assertEqual(self.clock.timer_updates[-1], self.clock.NOW)
        self.motion._flush_handler(10.1)
        self.assertIsNone(self.records('flush_handler')[-1]['expected_time'])

    def test_drip_timer_disable_and_restore_invalidate_expected_deadline(self):
        self.enable()
        self.motion.need_flush_time = 20.0
        self.motion._flush_handler(10.0)
        completion = SimpleNamespace(test=lambda: True)
        self.motion.drip_update_time(10.0, 10.0, completion)
        self.assertEqual(self.clock.timer_updates[-2:],
                         [self.clock.NEVER, self.clock.NOW])
        self.motion._flush_handler(10.0)
        self.assertIsNone(self.records('flush_handler')[-1]['expected_time'])

    def test_disabled_collection_stays_empty_and_enable_is_fresh(self):
        self.motion._advance_flush_time(1.0)
        self.printer.send_event('mcu:timer_too_close', 'mcu', self.clock.now)
        disabled = self.dump()
        self.assertEqual(disabled['python_ring'], [])
        self.assertIsNone(disabled['first_warning'])
        self.assertIsNone(disabled['latest_warning'])
        self.enable()
        self.assertTrue(all(board.enabled for board in self.boards))
        self.motion._advance_flush_time(2.0)
        self.dispatch('SET_MOTION_TIMING_DIAGNOSTICS', 0)
        retained = self.dump()['python_ring']
        self.motion._advance_flush_time(3.0)
        self.assertEqual(self.dump()['python_ring'], retained)
        self.assertFalse(any(board.enabled for board in self.boards))
        self.enable()
        self.assertEqual(self.dump()['python_ring'], [])

    def test_ring_retains_exactly_newest_128_callback_boundary_records(self):
        self.enable()
        for index in range(140):
            self.clock.now = 10.0 + index
            self.motion._advance_flush_time(float(index))
        records = self.dump()['python_ring']
        self.assertEqual(len(records), 128)
        self.assertEqual([record['entry_time'] for record in records],
                         [float(value) for value in range(22, 150)])

    def test_active_print_rejects_enable_and_disable_but_allows_dump(self):
        self.enable()
        for state in ('printing', 'paused'):
            self.printer.stats.state = state
            for enable in (0, 1):
                with self.subTest(state=state, enable=enable):
                    with self.assertRaisesRegex(RuntimeError,
                            'Cannot change motion timing diagnostics during a print'):
                        self.dispatch('SET_MOTION_TIMING_DIAGNOSTICS', enable)
            self.assertIsInstance(self.dump()['serial'], dict)
        self.assertTrue(self.printer.gcode.options[
            'DUMP_MOTION_TIMING_DIAGNOSTICS'].get('when_not_ready'))

    def test_warning_throttle_preserves_first_latest_and_all_receipts(self):
        self.enable()
        stats = print_stats.PrintStats(self.config)
        stats.set_current_file('timing.gcode')
        stats.note_start()
        for host_time, marker in ((10.0, b'first-history'),
                                  (10.2, b'throttled-history'),
                                  (10.99, b'still-throttled'),
                                  (11.01, b'latest-history')):
            self.clock.now = host_time
            self.boards[0].history = marker
            self.motion._advance_flush_time(host_time)
            self.printer.send_event('mcu:timer_too_close', 'mcu', host_time)
        self.assertEqual(len(self.printer.warning_receipts), 4)
        status = stats.get_status(self.clock.now)
        self.assertIn('timer_too_close', status, 'Missing per-print warning statistics')
        self.assertEqual(status['timer_too_close']['total'], 4,
                         'Snapshot throttling must never throttle warning counts')
        self.assertEqual(self.log_messages, [])
        self.assertEqual(self.boards[0].format_calls, [])
        self.assertEqual(self.boards[0].live_dumps, 0)
        self.assertEqual([board.snapshot_reads for board in self.boards], [2, 2],
                         'Throttled warnings must not recopy bulk serial history')
        self.clock.now = 12.0
        self.boards[0].history = b'overwritten-live-history'
        capture = self.dump()
        first, latest = capture['first_warning'], capture['latest_warning']
        self.assertEqual(first['host_time'], 10.0)
        self.assertEqual(latest['host_time'], 11.01)
        self.assertEqual(first['raw_history']['mcu']['sent'][0]['data'],
                         b'first-history'.hex())
        self.assertEqual(latest['raw_history']['mcu']['sent'][0]['data'],
                         b'latest-history'.hex())
        self.assertEqual(first['serial']['mcu'][0]['eventtime'], 10.0)
        self.assertEqual(latest['serial']['mcu'][0]['eventtime'], 11.01)
        self.assertEqual([board.live_dumps for board in self.boards], [1, 1])

    def test_terminal_events_emit_deferred_history_only_at_termination(self):
        for event in ('print_stats:complete_printing',
                      'print_stats:cancelled_printing',
                      'print_stats:error_printing', 'klippy:shutdown'):
            with self.subTest(event=event):
                self.log_messages.clear()
                self.enable()
                self.printer.send_event('mcu:timer_too_close', 'eboard', self.clock.now)
                self.assertEqual(self.log_messages, [])
                self.printer.send_event(event)
                self.clock.drain()
                captures = [json.loads(message.split(' ', 1)[1])
                            for message in self.log_messages
                            if message.startswith('MOTION_TIMING_DIAGNOSTICS ')]
                self.assertEqual(len(captures), 1)
                self.assertIsNotNone(captures[0]['first_warning'])
                self.assertIn('eboard', captures[0]['serial'])

    def test_virtual_sd_active_and_paused_resets_capture_cancellation_once(self):
        stats = print_stats.PrintStats(self.config)
        self.printer.stats = stats
        self.enable()
        sd = virtual_sdcard.VirtualSD.__new__(virtual_sdcard.VirtualSD)
        sd.printer = self.printer
        sd.reactor = self.clock
        sd.gcode = self.printer.gcode
        sd.print_stats = stats
        sd.auto_creator5_start = False
        sd.cmd_from_sd = False
        sd.work_timer = None
        sd.current_file = None
        # A reactor yield services the real worker's pending pause; no G-code
        # is dispatched because do_pause has already set must_pause_work.
        self.clock.pause = lambda eventtime: sd.work_handler(eventtime)
        self.clock.unregister_timer = lambda timer: None
        sd.gcode.get_mutex = lambda: SimpleNamespace(test=lambda: False)

        def captures():
            return [json.loads(message.split(' ', 1)[1])
                    for message in self.log_messages
                    if message.startswith('MOTION_TIMING_DIAGNOSTICS ')]

        # An idle reset, including the reset used before file selection,
        # must not be mistaken for a cancelled print.
        sd._reset_file()
        self.clock.drain()
        self.assertEqual(captures(), [])
        for paused in (False, True):
            with self.subTest(paused=paused):
                stats.set_current_file('reset.gcode')
                sd.current_file = io.StringIO('G1 X10\n')
                self.addCleanup(sd.current_file.close)
                sd.file_position = sd.next_file_position = 0
                sd.file_size = len(sd.current_file.getvalue())
                sd.must_pause_work = False
                sd.work_timer = self.clock.register_timer(sd.work_handler)
                stats.note_start()
                self.motion._advance_flush_time(self.clock.now)
                self.printer.send_event(
                    'mcu:timer_too_close', 'mcu', self.clock.now)
                if paused:
                    sd.do_pause()
                self.assertEqual(stats.state, 'paused' if paused else 'printing')
                before = len(captures())
                open_file = sd.current_file
                sd.cmd_SDCARD_RESET_FILE(Command())
                self.assertTrue(open_file.closed)
                self.assertEqual(stats.state, 'standby')
                report = stats.get_status(self.clock.now)['timer_too_close']
                self.assertEqual(report['state'], 'cancelled')
                self.assertEqual(report['total'], 1)
                self.assertEqual(len(captures()), before,
                                 'Cancellation capture must be deferred')
                self.assertFalse(any(board.enabled for board in self.boards),
                                 'SD reset must stop native capture before dispatch')
                self.clock.drain()
                emitted = captures()
                self.assertEqual(len(emitted), before + 1,
                                 'SD reset must capture active cancellation')
                self.assertEqual(emitted[-1]['trigger'],
                                 'print_stats:cancelled_printing')
                self.assertTrue(emitted[-1]['python_ring'])
                self.assertIsNotNone(emitted[-1]['first_warning'])

                # The cancelled report is retained through these resets.
                # Its state alone must not cause another terminal capture.
                sd._reset_file()
                stats.set_current_file('selected-but-not-started.gcode')
                sd._reset_file()
                self.clock.drain()
                self.assertEqual(len(captures()), before + 1)
                self.assert_same_evidence(self.dump(), emitted[-1])
                self.assertFalse(any(board.enabled for board in self.boards))
                self.clock.now += 2.0

    def test_virtual_sd_reset_does_not_capture_when_diagnostics_disabled(self):
        stats = print_stats.PrintStats(self.config)
        self.printer.stats = stats
        stats.set_current_file('disabled-reset.gcode')
        stats.note_start()
        sd = virtual_sdcard.VirtualSD.__new__(virtual_sdcard.VirtualSD)
        sd.printer = self.printer
        sd.print_stats = stats
        sd.current_file = None
        sd._reset_file()
        self.clock.drain()
        self.assertEqual(
            stats.get_status(self.clock.now)['timer_too_close']['state'],
            'cancelled')
        self.assertFalse(any(message.startswith('MOTION_TIMING_DIAGNOSTICS ')
                             for message in self.log_messages))


class NativeQueue:
    """A real serial worker plus an in-process protocol peer, not an MCU."""
    def __init__(self, case):
        self.case = case
        self.ffi, self.lib = chelper.get_ffi()
        for name in ('serialqueue_set_timing_enabled',
                     'serialqueue_extract_timing', 'serialqueue_snapshot_old'):
            case.assertTrue(hasattr(self.lib, name), 'Missing native feature: ' + name)
        try:
            self.ffi.sizeof('struct pull_queue_timing')
        except (TypeError, self.ffi.error):
            case.fail('Missing native feature: struct pull_queue_timing CFFI declaration')
        self.worker, self.peer = socket.socketpair()
        self.peer.settimeout(2.0)
        self.queue = self.lib.serialqueue_alloc(self.worker.fileno(), b'u', 0,
                                                b'timing-test')
        case.assertNotEqual(self.queue, self.ffi.NULL)
        self.cq = self.lib.serialqueue_alloc_commandqueue()
        self.lib.serialqueue_set_wire_frequency(self.queue, 1000000000.0)
        self.base_time = self.lib.get_monotonic()
        self.base_clock = (1 << 32) - 16384
        self.lib.serialqueue_set_clock_est(self.queue, 1000000.0, self.base_time,
                                           self.base_clock, self.base_clock)
        self.parser = msgproto.MessageParser()
        self.parser.process_identify(json.dumps({
            'commands': {'queue_step oid=%c interval=%u count=%hu add=%hi': 2},
            'responses': {}, 'config': {},
        }).encode(), decompress=False)
        self.closed = False
        case.addCleanup(self.close)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.lib.serialqueue_exit(self.queue)
        self.lib.serialqueue_free(self.queue)
        self.lib.serialqueue_free_commandqueue(self.cq)
        self.worker.close()
        self.peer.close()

    def enable(self, enabled=True):
        self.lib.serialqueue_set_timing_enabled(self.queue, int(enabled))

    def timing(self, maximum=128):
        out = self.ffi.new('struct pull_queue_timing[128]')
        count = self.lib.serialqueue_extract_timing(self.queue, out, maximum)
        fields = ('kind', 'bytes', 'eventtime', 'conv_time', 'est_freq',
                  'req_clock', 'min_clock', 'conv_clock', 'send_seq',
                  'receive_seq', 'ready_bytes', 'upcoming_bytes', 'need_ack_bytes')
        return [{field: getattr(out[index], field) for field in fields}
                for index in range(count)]

    def raw(self, sent=True, maximum=100, consume=False):
        out = self.ffi.new('struct pull_queue_message[100]')
        function = (self.lib.serialqueue_extract_old if consume else
                    self.lib.serialqueue_snapshot_old)
        count = function(self.queue, int(sent), out, maximum)
        return [dict(data=bytes(out[index].msg[0:out[index].len]),
                     sent_time=out[index].sent_time,
                     receive_time=out[index].receive_time,
                     notify_id=int(out[index].notify_id))
                for index in range(count)]

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(.001)
        self.case.fail('Local serial worker did not reach the expected state')

    def packet(self, sequence, payload=()):
        data = [5 + len(payload), 0x10 | (sequence & 0x0f)] + list(payload)
        return bytes(data + msgproto.crc16_ccitt(data) + [0x7e])

    def read_exact(self, size):
        data = b''
        while len(data) < size:
            chunk = self.peer.recv(size - len(data))
            self.case.assertTrue(chunk, 'Unexpected local socket EOF')
            data += chunk
        return data

    def send(self, index=0, minimum=None, request=None, payload=None):
        if minimum is None:
            minimum = self.base_clock - 10000
        if request is None:
            request = self.base_clock + 20000
        if payload is None:
            payload = bytes(self.parser.create_command(
                'queue_step oid=7 interval=%d count=3 add=0' % (100 + index)))
        self.lib.serialqueue_send(self.queue, self.cq, payload, len(payload),
                                  minimum, request, 0)
        header = self.read_exact(2)
        block = header + self.read_exact(header[0] - 2)
        self.case.assertEqual(self.parser.check_packet(list(block)), len(block))
        self.last_ack = (block[1] + 1) & 0xf
        self.peer.sendall(self.packet(self.last_ack))
        self.wait_for(lambda: any(record['data'] == block for record in self.raw()))
        return block

    def receive_data(self):
        packet = self.packet(self.last_ack, self.parser.create_command(
            'queue_step oid=9 interval=123 count=1 add=0'))
        result = self.ffi.new('struct pull_queue_message *')
        finished = threading.Event()

        def pull():
            self.lib.serialqueue_pull(self.queue, result)
            finished.set()

        thread = threading.Thread(target=pull, daemon=True)
        thread.start()
        self.peer.sendall(packet)
        self.case.assertTrue(finished.wait(2.0), 'Native local receive timed out')
        thread.join(2.0)
        self.case.assertEqual(bytes(result.msg[0:result.len]), packet)
        return packet

    def reader(self, clock=None):
        reader = serialhdl.SerialReader.__new__(serialhdl.SerialReader)
        reader.reactor = clock or ControlledReactor()
        reader.ffi_main, reader.ffi_lib = self.ffi, self.lib
        reader.serialqueue = self.queue
        reader.msgparser = self.parser
        reader.warn_prefix = ''
        reader._timing_lock = threading.Lock()
        reader._timing_enabled = False
        reader._timing_debug_history = None
        reader.stats_buf = self.ffi.new('char[4096]')
        return reader


class NativePrintCaptureTests(unittest.TestCase):
    """Per-print lifecycle through real MCU, SerialReader, and socket queues."""
    dispatch = MotionTimingTests.dispatch
    enable = MotionTimingTests.enable
    dump = MotionTimingTests.dump
    captures = MotionTimingTests.captures
    collect_log = MotionTimingTests.collect_log
    start_print = MotionTimingTests.start_print
    assert_same_evidence = MotionTimingTests.assert_same_evidence

    def setUp(self):
        self.native = NativeQueue(self)
        self.clock = ControlledReactor()
        self.clock.now = self.native.lib.get_monotonic()
        self.boards = []
        self.printer = OfflinePrinter(self.clock, self.boards)
        self.reader = self.native.reader(self.clock)
        self.clock_evidence = 'print clock evidence'
        board = mcu.MCU.__new__(mcu.MCU)
        board._name = 'mcu'
        board._printer = self.printer
        board._serial = self.reader
        board._clocksync = SimpleNamespace(
            estimated_print_time=lambda eventtime: eventtime,
            dump_debug=lambda: self.clock_evidence)
        board._config_cmds = ['config_stepper oid=7']
        board._oid_count = 8
        self.reader.mcu = board
        self.boards.append(board)
        self.config = SimpleNamespace(get_printer=lambda: self.printer)
        options = mock.patch.object(motion_queuing, 'get_danger_options',
            return_value=SimpleNamespace(bgflush_extra_time=.250))
        options.start()
        self.addCleanup(options.stop)
        self.motion = motion_queuing.PrinterMotionQueuing(self.config)
        self.motion.steppersyncmgr_gen_steps = lambda *args: 0
        self.log_messages = []
        logger = mock.patch.object(motion_queuing.logging, 'info', self.collect_log)
        logger.start()
        self.addCleanup(logger.stop)

    def sync_clock(self):
        # Session bounds and serial events must use the same native clock.
        self.clock.now = self.native.lib.get_monotonic()
        return self.clock.now

    def test_no_warning_terminal_snapshot_survives_100_later_native_blocks(self):
        self.enable()
        stats = self.start_print()
        first = self.native.send(0)
        incoming = self.native.receive_data()
        self.sync_clock()
        self.motion._advance_flush_time(self.clock.now)
        expected_numeric = self.native.timing()
        boundary = self.sync_clock()
        stats.note_complete()
        self.assertEqual(self.log_messages, [],
                         'Bulk terminal logging must wait for the reactor turn')
        for index in range(1, 107):
            last = self.native.send(index)
            self.native.receive_data()
        self.assertEqual(len(self.native.raw()), 100)
        self.assertEqual(len(self.native.raw(sent=False)), 100)
        self.assertNotIn(first, [record['data'] for record in self.native.raw()])
        self.assertEqual(self.native.timing(), expected_numeric,
                         'Terminal must stop native numeric recording immediately')
        self.clock_evidence = 'post-print clock evidence'
        self.boards[0]._config_cmds = ['post_print_config oid=99']
        self.sync_clock()
        self.motion._advance_flush_time(self.clock.now)
        self.clock.drain()
        self.assertEqual(len(self.captures()), 1)
        frozen = self.captures()[0]
        self.assertEqual(frozen['host_time'], boundary)
        self.assertEqual(frozen['serial']['mcu'], expected_numeric)
        self.assertEqual(len(frozen['python_ring']), 1)
        self.assertIsNone(frozen['first_warning'])
        self.assertIsNone(frozen['latest_warning'])
        debug = [message for message in self.log_messages
                 if not message.startswith('MOTION_TIMING_DIAGNOSTICS ')]
        output = '\n'.join(debug)
        self.assertIn('[hex=%s]' % first.hex(), output)
        self.assertIn('[hex=%s]' % incoming.hex(), output)
        self.assertNotIn('[hex=%s]' % last.hex(), output)
        self.assertIn('Dumping sent queue 1 messages', output)
        self.assertIn('Dumping received queue 1 messages', output)
        self.assertIn('print clock evidence', output)
        self.assertNotIn('post-print clock evidence', output)
        self.assertIn('config_stepper oid=7', output)
        self.assertNotIn('post_print_config', output)
        for unused in range(2):
            before = len(self.log_messages)
            self.sync_clock()
            self.assert_same_evidence(self.dump(), frozen)
            self.assertEqual([message for message in self.log_messages[before:]
                              if not message.startswith('MOTION_TIMING_DIAGNOSTICS ')],
                             debug)

    def test_new_print_resets_native_ring_and_excludes_previous_raw_history(self):
        self.enable()
        old = self.native.send(0)
        self.native.receive_data()
        self.sync_clock()
        stats = self.start_print('first.gcode')
        self.assertEqual(self.native.timing(), [],
                         'A new print must clear the enabled idle native ring')
        first = self.native.send(1)
        self.sync_clock()
        stats.note_complete()
        self.clock.drain()
        self.sync_clock()
        self.start_print('second.gcode')
        self.assertEqual(self.native.timing(), [],
                         'Logical enable must rearm with an empty native ring')
        second = self.native.send(2, request=self.native.base_clock + 12345)
        incoming = self.native.receive_data()
        self.sync_clock()
        self.printer.send_event('mcu:timer_too_close', 'mcu', self.clock.now)
        before = len(self.log_messages)
        capture = self.dump()
        self.assertEqual([record['kind']
                          for record in capture['serial']['mcu']], [1, 2])
        self.assertEqual([record['req_clock']
                          for record in capture['serial']['mcu']],
                         [self.native.base_clock + 12345] * 2)
        for warning in ('first_warning', 'latest_warning'):
            raw = capture[warning]['raw_history']['mcu']
            self.assertEqual([record['data'] for record in raw['sent']],
                             [second.hex()])
            self.assertEqual([record['data'] for record in raw['received']],
                             [incoming.hex()])
        output = '\n'.join(self.log_messages[before:])
        self.assertIn('[hex=%s]' % second.hex(), output)
        self.assertNotIn('[hex=%s]' % first.hex(), output)
        self.assertNotIn('[hex=%s]' % old.hex(), output)
        # Session filtering must never destructively consume shutdown evidence.
        self.assertEqual([record['data'] for record in self.native.raw()],
                         [old, first, second])

    def test_queued_serial_warning_from_old_print_does_not_enter_new_capture(self):
        self.enable()
        stats = self.start_print('first.gcode')
        self.native.send(0)
        receipt = self.sync_clock()
        self.reader.handle_output({'#name': 'output',
                                   '#msg': 'Timer too close',
                                   '#receive_time': receipt})
        self.assertEqual(stats.get_status(self.clock.now)['timer_too_close']['total'], 0,
                         'Serial receipt must not mutate print stats off-reactor')
        stats.note_complete()
        self.sync_clock()
        self.start_print('second.gcode')
        self.native.send(1)
        self.sync_clock()
        self.clock.drain()
        fresh = self.dump()
        self.assertIsNone(fresh['first_warning'])
        self.assertIsNone(fresh['latest_warning'])
        self.assertEqual(stats.get_status(self.clock.now)['timer_too_close']['total'], 0)
        self.reader.handle_output({'#name': 'output',
                                   '#msg': 'Timer too close',
                                   '#receive_time': self.clock.now})
        self.assertEqual(stats.get_status(self.clock.now)['timer_too_close']['total'], 0)
        self.clock.drain()
        self.assertEqual(stats.get_status(self.clock.now)['timer_too_close']['total'], 1)
        self.assertIsNotNone(self.dump()['first_warning'])


class SerialTimingNativeTests(unittest.TestCase):
    def require_method(self, cls, name):
        self.assertTrue(hasattr(cls, name), 'Missing diagnostic feature: ' + name)

    def test_enqueue_build_clocks_survive_wrap_and_keep_adjusted_minimum(self):
        native = NativeQueue(self)
        native.enable()
        for index, (minimum, request) in enumerate((
                ((1 << 32) - 20000, (1 << 32) - 1000),
                ((1 << 32) - 10000, (1 << 32) + 1000),
                (0, (1 << 32) + 2000))):
            native.send(index, minimum, request)
        records = native.timing()
        self.assertEqual([record['kind'] for record in records], [1, 2, 1, 2, 1, 2])
        self.assertEqual([record['req_clock'] for record in records],
                         [4294966296, 4294966296, 4294968296,
                          4294968296, 4294969296, 4294969296])
        adjusted = 4294969296 - (3 << 29)
        self.assertEqual([record['min_clock'] for record in records],
                         [4294947296, 4294947296, 4294957296,
                          4294957296, adjusted, adjusted])
        self.assertEqual([record['eventtime'] for record in records],
                         sorted(record['eventtime'] for record in records))
        for enqueue, build in zip(records[::2], records[1::2]):
            self.assertGreater(enqueue['bytes'], 0)
            self.assertEqual(build['bytes'], enqueue['bytes'] + 5)
            self.assertEqual(build['conv_clock'], native.base_clock)
            self.assertEqual(build['conv_time'], native.base_time)
            self.assertEqual(build['est_freq'], 1000000.0)
            # Independently reconstruct the same 64-bit deadline on either side
            # of wrap; it is queue need time, NOT a physical step timestamp.
            deadline = (build['conv_time'] +
                        (build['req_clock'] - build['conv_clock']) / 1000000.0)
            self.assertAlmostEqual(deadline - native.base_time,
                (build['req_clock'] - native.base_clock) / 1000000.0)
            self.assertGreaterEqual(build['send_seq'], 1)
            for field in ('ready_bytes', 'upcoming_bytes', 'need_ack_bytes'):
                self.assertGreaterEqual(build[field], 0)

    def test_unscheduled_and_background_records_do_not_claim_deadline(self):
        native = NativeQueue(self)
        native.enable()
        native.send(0, 0, 0)
        native.send(1, 0, 0x7fffffff00000000)
        self.assertEqual([(record['req_clock'], record['min_clock'])
                          for record in native.timing()], [(0, 0)] * 4)

    def test_native_ring_bounds_nonconsuming_newest_and_enable_lifecycle(self):
        native = NativeQueue(self)
        native.send(0)
        self.assertEqual(native.timing(), [])
        native.enable()
        for index in range(70):
            native.send(index, request=native.base_clock + index + 1)
        records = native.timing()
        self.assertEqual(len(records), 128)
        self.assertEqual([record['req_clock'] for record in records[::2]],
                         [native.base_clock + index + 1 for index in range(6, 70)])
        self.assertEqual(native.timing(5), records[-5:])
        self.assertEqual(native.timing(0), [])
        self.assertEqual(native.timing(-1), [])
        self.assertEqual(native.timing(), records)
        native.enable(False)
        native.send(71)
        self.assertEqual(native.timing(), records)
        native.enable()
        self.assertEqual(native.timing(), [])
        native.send(72)
        self.assertEqual([record['kind'] for record in native.timing()], [1, 2])

    def test_raw_snapshots_do_not_consume_sent_or_received_debug_history(self):
        native = NativeQueue(self)
        blocks = [native.send(index) for index in range(3)]
        received = native.receive_data()
        sent = native.raw()
        incoming = native.raw(sent=False)
        self.assertEqual([record['data'] for record in sent], blocks)
        self.assertEqual([record['data'] for record in incoming], [received])
        self.assertEqual(native.raw(maximum=2), sent[-2:])
        self.assertEqual(native.raw(maximum=0), [])
        self.assertEqual(native.raw(maximum=-1), [])
        self.assertEqual(native.raw(), sent)
        self.assertEqual(native.raw(sent=False), incoming)
        self.assertTrue(all(record['notify_id'] == 0 for record in sent + incoming))
        for record in sent:
            self.assertGreaterEqual(record['receive_time'], record['sent_time'])
        # Existing destructive shutdown extraction must still see all messages.
        for sentq, expected in ((True, sent), (False, incoming)):
            extracted = native.raw(sent=sentq, consume=True)
            self.assertEqual([(r['data'], r['sent_time'], r['receive_time'])
                              for r in extracted],
                             [(r['data'], r['sent_time'], r['receive_time'])
                              for r in expected])
            self.assertEqual(native.raw(sent=sentq), [])

    def test_retained_raw_snapshot_decodes_after_live_history_overwrite(self):
        self.require_method(serialhdl.SerialReader, 'get_timing_debug_snapshot')
        self.require_method(serialhdl.SerialReader, 'format_timing_debug_snapshot')
        native = NativeQueue(self)
        first = native.send(0)
        reader = native.reader()
        saved = reader.get_timing_debug_snapshot()
        self.assertTrue(saved['connected'])
        self.assertEqual(saved['sent'][0]['data'], first)
        self.assertIsInstance(saved['sent'][0]['data'], bytes)
        original = copy.deepcopy(saved)
        for index in range(1, 107):
            native.send(index)
        current = reader.get_timing_debug_snapshot()
        self.assertEqual(len(current['sent']), 100)
        self.assertNotIn(first, [record['data'] for record in current['sent']])
        formatted = reader.format_timing_debug_snapshot(saved)
        self.assertEqual(saved, original, 'Formatting must not mutate retained evidence')
        record = formatted['sent'][0]
        self.assertEqual(record['data'], first.hex())
        self.assertEqual(record['sent_time'], original['sent'][0]['sent_time'])
        self.assertEqual(record['receive_time'], original['sent'][0]['receive_time'])
        self.assertIn('queue_step oid=7 interval=100 count=3 add=0',
                      json.dumps(record))

    def test_disconnected_wrappers_and_undecodable_records_preserve_good_evidence(self):
        for name in ('set_timing_diagnostics', 'get_timing_diagnostics',
                     'get_timing_debug_snapshot', 'format_timing_debug_snapshot'):
            self.require_method(serialhdl.SerialReader, name)
        self.require_method(mcu.MCU, 'dump_timing_debug')
        native = NativeQueue(self)
        valid = native.send(0)
        reader = native.reader()
        snapshot = reader.get_timing_debug_snapshot()
        snapshot['sent'].insert(0, dict(data=b'\x00', sent_time=1.25,
                                         receive_time=1.5))
        formatted = reader.format_timing_debug_snapshot(snapshot)
        self.assertEqual(formatted['sent'][0]['data'], '00')
        self.assertEqual(formatted['sent'][0]['sent_time'], 1.25)
        self.assertEqual(formatted['sent'][0]['receive_time'], 1.5)
        self.assertIn('undecodable', json.dumps(formatted['sent'][0]).lower())
        self.assertIn('queue_step oid=7', json.dumps(formatted['sent'][1]))
        self.assertEqual(formatted['sent'][1]['data'], valid.hex())
        disconnected = native.reader()
        disconnected.serialqueue = None
        disconnected.set_timing_diagnostics(True)
        self.assertEqual(disconnected.get_timing_diagnostics(), [])
        self.assertEqual(disconnected.get_timing_debug_snapshot(),
                         dict(connected=False, sent=[], received=[]))
        board = mcu.MCU.__new__(mcu.MCU)
        board._serial = disconnected
        board._clocksync = SimpleNamespace(dump_debug=lambda: 'clock sync evidence')
        board._config_cmds = ['config_stepper oid=7']
        board._oid_count = 8
        output = board.dump_timing_debug()
        self.assertIn('config_stepper oid=7', output)
        self.assertIn('clock sync evidence', output)
        self.assertIn('Serial timing history unavailable: MCU disconnected', output)

    def test_queued_warning_retains_history_consumed_by_real_mcu_shutdown(self):
        self._assert_queued_warning_survives_failure('shutdown')

    def test_queued_warning_retains_history_consumed_by_real_mcu_timeout(self):
        self._assert_queued_warning_survives_failure('timeout')

    def _assert_queued_warning_survives_failure(self, failure):
        native = NativeQueue(self)
        clock = ControlledReactor()
        clock.now = native.lib.get_monotonic()
        boards = []
        printer = OfflineShutdownPrinter(clock, boards)
        reader = native.reader(clock)
        board = mcu.MCU.__new__(mcu.MCU)
        board._name = 'mcu'
        board._printer = printer
        board._serial = reader
        board._is_shutdown = False
        board._is_timeout = False
        board.is_non_critical = False
        board._clocksync = SimpleNamespace(
            estimated_print_time=lambda eventtime: eventtime,
            is_active=lambda: False,
            dump_debug=lambda: 'clock sync evidence')
        board._config_cmds = ['config_stepper oid=7']
        board._oid_count = 8
        reader.mcu = board
        boards.append(board)
        config = SimpleNamespace(get_printer=lambda: printer)
        motion_queuing.PrinterMotionQueuing(config)
        printer.gcode.commands['SET_MOTION_TIMING_DIAGNOSTICS'](Command(1))
        sent = native.send(minimum=4294957296, request=4294968296)
        received = native.receive_data()
        clock.now = native.lib.get_monotonic()
        original = reader.get_timing_debug_snapshot()
        self.assertEqual([record['data'] for record in original['sent']], [sent])
        self.assertEqual([record['data'] for record in original['received']],
                         [received])

        with self.assertLogs(level='INFO') as logs:
            # Both callbacks originate on the serial thread in production.
            # Hold reactor dispatch until the real shutdown's default debug
            # logging has destructively extracted both native raw histories.
            reader.handle_output({'#name': 'output',
                                  '#msg': 'Timer too close',
                                  '#receive_time': clock.now})
            self.assertEqual(printer.warning_receipts, [])
            with mock.patch.object(mcu, 'get_danger_options',
                    return_value=SimpleNamespace(log_shutdown_info=True)):
                if failure == 'shutdown':
                    board._handle_shutdown({
                        '#name': 'shutdown',
                        'static_string_id': 'Stepper too far in past'})
                    self.assertTrue(board.is_shutdown())
                else:
                    board._check_timeout(clock.now)
                    self.assertTrue(board._is_timeout)
            self.assertEqual(native.raw(), [])
            self.assertEqual(native.raw(sent=False), [])
            self.assertEqual(printer.warning_receipts, [])
            self.assertEqual(len(printer.shutdown_messages),
                             int(failure == 'timeout'))
            clock.drain()  # queued warning, then klippy:shutdown
            clock.drain()  # deferred terminal capture

        self.assertEqual(printer.warning_receipts, [('mcu', clock.now)])
        self.assertEqual(len(printer.shutdown_messages), 1)
        output = '\n'.join(record.getMessage() for record in logs.records)
        if failure == 'shutdown':
            self.assertIn("MCU 'mcu' shutdown: Stepper too far in past", output)
        else:
            self.assertIn("MCU 'mcu' disconnected: Timeout", output)
        self.assertIn('Dumping send queue 1 messages', output)
        self.assertIn('Dumping receive queue 1 messages', output)
        captures = [json.loads(record.getMessage().split(' ', 1)[1])
                    for record in logs.records
                    if record.getMessage().startswith('MOTION_TIMING_DIAGNOSTICS ')]
        self.assertEqual(len(captures), 1)
        capture = captures[0]
        self.assertEqual(capture['trigger'], 'klippy:shutdown')
        for records in (capture['serial']['mcu'],
                        capture['first_warning']['serial']['mcu'],
                        capture['latest_warning']['serial']['mcu']):
            self.assertEqual([record['kind'] for record in records], [1, 2])
            self.assertEqual([record['req_clock'] for record in records],
                             [4294968296, 4294968296])
            self.assertEqual([record['min_clock'] for record in records],
                             [4294957296, 4294957296])
        for warning in ('first_warning', 'latest_warning'):
            retained = capture[warning]['raw_history']['mcu']
            self.assertTrue(retained['connected'])
            for side, block, decoded in (
                    ('sent', sent, 'queue_step oid=7 interval=100 count=3 add=0'),
                    ('received', received,
                     'queue_step oid=9 interval=123 count=1 add=0')):
                self.assertEqual(len(retained[side]), 1,
                    'Shutdown must preserve raw evidence before destructive debug extraction')
                record = retained[side][0]
                self.assertEqual(record['data'], block.hex())
                self.assertEqual(record['sent_time'], original[side][0]['sent_time'])
                self.assertEqual(record['receive_time'],
                                 original[side][0]['receive_time'])
                self.assertIn(decoded, json.dumps(record))
        self.assertIn('Dumping sent queue 1 messages', output)
        self.assertIn('[hex=%s]' % sent.hex(), output)
        self.assertIn('[hex=%s]' % received.hex(), output)
        # A second destructive extraction must not replace the retained failure
        # boundary with empty queues. Consumer mutation must not alter it either.
        retained = reader.get_timing_debug_snapshot()
        reader.dump_debug()
        self.assertEqual(reader.get_timing_debug_snapshot(), original)
        retained['sent'][0]['data'] = b'changed by consumer'
        retained['received'].clear()
        self.assertEqual(reader.get_timing_debug_snapshot(), original)
        reader.set_timing_diagnostics(False)
        reader.serialqueue = None
        self.assertEqual(reader.get_timing_debug_snapshot(),
                         dict(connected=False, sent=[], received=[]))
        reader.serialqueue = native.queue
        self.assertEqual(reader.get_timing_debug_snapshot(), original)
        reader.set_timing_diagnostics(True)
        self.assertEqual(reader.get_timing_debug_snapshot(),
                         dict(connected=True, sent=[], received=[]))

    def test_consumed_history_is_bounded_and_new_connection_clears_it(self):
        native = NativeQueue(self)
        reader = serialhdl.SerialReader(ControlledReactor())
        reader.serialqueue = native.queue
        reader.msgparser = native.parser
        reader.set_timing_diagnostics(True)
        blocks = [native.send(index) for index in range(130)]
        reader.dump_debug()
        snapshot = reader.get_timing_debug_snapshot()
        self.assertEqual([record['data'] for record in snapshot['sent']],
                         blocks[-100:])
        self.assertEqual(native.raw(), [])
        dictionary = json.dumps({
            'commands': {'queue_step oid=%c interval=%u count=%hu add=%hi': 2},
            'responses': {}, 'config': {},
        }).encode()
        with tempfile.TemporaryFile() as output:
            try:
                reader.connect_file(output, dictionary)
                self.assertEqual(reader.get_timing_debug_snapshot(),
                                 dict(connected=True, sent=[], received=[]))
                self.assertEqual(reader.get_timing_diagnostics(), [])
            finally:
                reader.disconnect()

    def test_disabled_shutdown_debug_does_not_retain_or_snapshot_history(self):
        native = NativeQueue(self)
        reader = native.reader()
        native.send()
        # Any native non-destructive snapshot here would be diagnostic work in
        # default-off mode. Normal destructive extraction must still run.
        reader.ffi_lib = mock.Mock(wraps=native.lib)
        reader.ffi_lib.serialqueue_snapshot_old.side_effect = AssertionError(
            'Disabled diagnostics must not snapshot history')
        output = reader.dump_debug()
        self.assertIn('Dumping send queue 1 messages', output)
        self.assertIn('queue_step oid=7 interval=100 count=3 add=0', output)
        self.assertEqual(native.raw(), [])
        reader.ffi_lib = native.lib
        self.assertEqual(reader.get_timing_debug_snapshot(),
                         dict(connected=True, sent=[], received=[]))

    def test_complete_dump_keeps_other_boards_and_bad_retained_record(self):
        for cls, names in (
                (serialhdl.SerialReader, ('get_timing_debug_snapshot',
                    'format_timing_debug_snapshot', 'get_timing_diagnostics',
                    'set_timing_diagnostics')),
                (mcu.MCU, ('dump_timing_debug', 'get_timing_debug_snapshot',
                    'format_timing_debug_snapshot', 'get_timing_diagnostics',
                    'set_timing_diagnostics'))):
            for name in names:
                self.require_method(cls, name)
        native = NativeQueue(self)
        clock = ControlledReactor()
        clock.now = native.lib.get_monotonic()
        connected = native.reader(clock)
        disconnected = native.reader(clock)
        disconnected.serialqueue = None
        boards = []
        printer = OfflinePrinter(clock, boards)
        for name, reader in (('mcu', connected), ('eboard', disconnected)):
            board = mcu.MCU.__new__(mcu.MCU)
            board._name = name
            board._printer = printer
            board._serial = reader
            board._clocksync = SimpleNamespace(
                estimated_print_time=lambda eventtime: eventtime,
                dump_debug=lambda: 'clock sync evidence')
            board._config_cmds = ['config_stepper oid=7']
            board._oid_count = 8
            boards.append(board)
        config = SimpleNamespace(get_printer=lambda: printer)
        motion_queuing.PrinterMotionQueuing(config)
        self.assertIn('SET_MOTION_TIMING_DIAGNOSTICS', printer.gcode.commands)
        self.assertIn('DUMP_MOTION_TIMING_DIAGNOSTICS', printer.gcode.commands)
        printer.gcode.commands['SET_MOTION_TIMING_DIAGNOSTICS'](Command(1))
        valid = native.send(0)
        malformed = native.send(payload=b'\x02')
        clock.now = native.lib.get_monotonic()
        printer.send_event('mcu:timer_too_close', 'mcu', clock.now)
        # The retained warning must survive loss of both good and bad live
        # blocks, rather than merely formatting the current serial history.
        for index in range(1, 107):
            native.send(index)
        self.assertNotIn(valid, [record['data'] for record in native.raw()])
        self.assertNotIn(malformed, [record['data'] for record in native.raw()])
        with self.assertLogs(level='INFO') as logs:
            printer.gcode.commands['DUMP_MOTION_TIMING_DIAGNOSTICS'](Command())
            clock.drain()
        captures = [json.loads(record.getMessage().split(' ', 1)[1])
                    for record in logs.records
                    if record.getMessage().startswith('MOTION_TIMING_DIAGNOSTICS ')]
        self.assertEqual(len(captures), 1)
        capture = captures[0]
        self.assertEqual(set(capture['serial']), {'mcu', 'eboard'})
        self.assertEqual(capture['serial']['eboard'], [])
        retained = capture['first_warning']['raw_history']
        self.assertFalse(retained['eboard']['connected'])
        by_hex = {record['data']: record for record in retained['mcu']['sent']}
        self.assertIn('queue_step oid=7 interval=100', json.dumps(by_hex[valid.hex()]))
        self.assertIn('undecodable', json.dumps(by_hex[malformed.hex()]).lower())
        output = '\n'.join(record.getMessage() for record in logs.records)
        self.assertIn('Serial timing history unavailable: MCU disconnected', output)


if __name__ == '__main__':
    unittest.main()
