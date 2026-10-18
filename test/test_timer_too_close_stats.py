# Per-print timer warning regression tests
#
# Copyright (C) 2026 wondercrash <155874349+wondercrash@users.noreply.github.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import importlib.util
import json
import queue
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

from klippy import gcode, serialhdl

SPEC = importlib.util.spec_from_file_location(
    'print_stats', Path(__file__).resolve().parents[1]
    / 'klippy' / 'extras' / 'print_stats.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ControlledReactor:
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 100.
        self.callbacks = []
        self.async_callbacks = queue.Queue()

    def monotonic(self):
        return self.now

    def mutex(self):
        return threading.RLock()

    def register_callback(self, callback, waketime=NOW):
        self.callbacks.append((callback, waketime))
        return object()

    def register_async_callback(self, callback, waketime=NOW):
        self.async_callbacks.put((callback, waketime))

    def turn(self):
        # A callback registered by another callback runs on a later turn.
        while not self.async_callbacks.empty():
            self.callbacks.append(self.async_callbacks.get_nowait())
        pending, self.callbacks = self.callbacks, []
        for callback, waketime in pending:
            if waketime <= self.now:
                callback(self.now)
            else:
                self.callbacks.append((callback, waketime))

    def drain(self):
        for _ in range(20):
            if not self.callbacks and self.async_callbacks.empty():
                return
            self.turn()
        raise AssertionError('reactor callbacks did not quiesce')


class ControlledMCU:
    def __init__(self, printer, name):
        self.printer = printer
        self.name = name

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def estimated_print_time(self, eventtime):
        # All motion markers use the main MCU's common print-time domain.
        return eventtime - 90.


class ControlledToolhead:
    def __init__(self, reactor, mcu):
        self.reactor = reactor
        self.mcu = mcu
        self.moves = []
        self.last_move_time_calls = 0

    def get_last_move_time(self):
        # Empty lookahead may prime motion 250ms ahead of host time. Startup
        # metadata must use the MCU estimate instead of this motion flush.
        self.last_move_time_calls += 1
        return self.mcu.estimated_print_time(self.reactor.now) + .250

    def register_lookahead_callback(self, callback):
        # Mirror ToolHead.register_lookahead_callback: use the last move's end
        # when queued, otherwise call synchronously with get_last_move_time().
        if not self.moves:
            callback(self.get_last_move_time())
        else:
            self.moves[-1].append(callback)

    def queue_move(self):
        callbacks = []
        self.moves.append(callbacks)
        return callbacks

    def resolve_move(self, callbacks, print_time):
        if callbacks in self.moves:
            self.moves.remove(callbacks)
        for callback in callbacks:
            callback(print_time)


class ControlledPrinter:
    command_error = config_error = gcode.CommandError

    def __init__(self):
        self.reactor = ControlledReactor()
        self.handlers = {}
        self.events = []
        self.objects = {}
        self.objects['mcu'] = ControlledMCU(self, 'mcu')
        self.objects['toolhead'] = ControlledToolhead(
            self.reactor, self.objects['mcu'])
        self.objects['gcode_move'] = SimpleNamespace(
            get_status=lambda eventtime=None: {
                'position': gcode.Coord(0., 0., 0., 0.),
                'extrude_factor': 1.,
            })
        self.objects['gcode'] = gcode.GCodeDispatch(self)
        self.output = []
        self.objects['gcode'].register_output_handler(self.output.append)

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def lookup_object(self, name, *default):
        if default:
            return self.objects.get(name, default[0])
        return self.objects[name]

    def load_object(self, config, name):
        return self.lookup_object(name)

    def register_event_handler(self, name, callback):
        self.handlers.setdefault(name, []).append(callback)

    def send_event(self, name, *args):
        self.events.append((name, args))
        return [callback(*args) for callback in self.handlers.get(name, ())]

    def invoke_shutdown(self, message):
        raise AssertionError('unexpected shutdown: ' + message)


def make_serial(printer, name='mcu', attached=True):
    # handle_output does not need a serial device, helper allocation, parser,
    # background worker or MCU firmware. Keep the real output implementation.
    serial = serialhdl.SerialReader.__new__(serialhdl.SerialReader)
    serial.reactor = printer.reactor
    serial.warn_prefix = name + ': '
    serial.mcu = ControlledMCU(printer, name) if attached else None
    return serial


def output_params(message='Timer too close', receive_time=100.125):
    return {'#name': '#output', '#msg': message,
            '#receive_time': receive_time, '#sent_time': receive_time - .01}


class SerialWarningBridgeTests(unittest.TestCase):
    def test_exact_warning_is_logged_then_forwarded_only_on_reactor(self):
        printer = ControlledPrinter()
        serial = make_serial(printer, 'eboard')
        params = output_params(receive_time=100.125)
        with self.assertLogs(level='INFO') as logs:
            serial.handle_output(params)
        self.assertTrue(any('eboard: #output: Timer too close' in line
                            for line in logs.output))
        self.assertEqual(printer.events, [])
        # Neither mutation of parser storage nor callback dispatch latency may
        # change the receipt time or the MCU identity captured in the worker.
        params['#receive_time'] = 777.
        serial.mcu.name = 'renamed'
        printer.reactor.now = 105.
        printer.reactor.drain()
        self.assertEqual(printer.events, [
            ('mcu:timer_too_close', ('eboard', 100.125))])

    def test_distinct_identical_warning_messages_are_not_deduplicated(self):
        printer = ControlledPrinter()
        serial = make_serial(printer)
        serial.handle_output(output_params())
        serial.handle_output(output_params())
        printer.reactor.drain()
        self.assertEqual(printer.events, [
            ('mcu:timer_too_close', ('mcu', 100.125)),
            ('mcu:timer_too_close', ('mcu', 100.125))])

    def test_unrelated_or_inexact_output_is_logged_without_forwarding(self):
        printer = ControlledPrinter()
        serial = make_serial(printer)
        for text in ('Other warning', 'Timer too close ',
                     'timer too close', 'prefix Timer too close'):
            with self.subTest(text=text), self.assertLogs(level='INFO') as logs:
                serial.handle_output(output_params(text))
            self.assertTrue(any(text in line for line in logs.output))
        printer.reactor.drain()
        self.assertEqual(printer.events, [])

    def test_unattached_reader_logs_warning_without_sending_event(self):
        printer = ControlledPrinter()
        with self.assertLogs(level='INFO') as logs:
            make_serial(printer, attached=False).handle_output(output_params())
        self.assertTrue(any('Timer too close' in line for line in logs.output))
        printer.reactor.drain()
        self.assertEqual(printer.events, [])

    def test_no_print_stats_listener_is_required(self):
        printer = ControlledPrinter()
        make_serial(printer).handle_output(output_params())
        printer.reactor.drain()
        self.assertEqual(printer.events, [
            ('mcu:timer_too_close', ('mcu', 100.125))])


class TimerTooCloseStatsTests(unittest.TestCase):
    def setUp(self):
        self.printer = ControlledPrinter()
        self.reactor = self.printer.reactor
        self.toolhead = self.printer.objects['toolhead']
        self.gcode = self.printer.objects['gcode']
        config = SimpleNamespace(get_printer=lambda: self.printer)
        self.stats = MODULE.PrintStats(config)
        self.printer.objects['print_stats'] = self.stats
        self.printer.send_event('klippy:ready')
        self.printer.output.clear()

    def start(self, filename='part.gcode', at=100.):
        self.reactor.now = at
        self.stats.set_current_file(filename)
        self.stats.note_start()

    def info(self, **params):
        self.gcode.run_script('SET_PRINT_STATS_INFO ' + ' '.join(
            '%s=%s' % (key, value) for key, value in params.items()))

    def warning(self, mcu='mcu', received=None):
        if received is None:
            received = self.reactor.now
        self.reactor.now = max(self.reactor.now, received)
        self.printer.send_event('mcu:timer_too_close', mcu, received)

    def bridge_warning(self, mcu='mcu', received=None):
        if received is None:
            received = self.reactor.now
        make_serial(self.printer, mcu).handle_output(
            output_params(receive_time=received))

    def report(self):
        status = self.stats.get_status(self.reactor.now)
        self.assertIn('timer_too_close', status,
                      'PrintStats must expose the retained warning report')
        report = status['timer_too_close']
        self.assertEqual(set(report), {
            'filename', 'state', 'total', 'attribution', 'by_layer'})
        self.assertEqual(report['attribution'], 'receive_time_scheduled_layer')
        # Serialization is part of the printer-status public API.
        json.dumps(report, allow_nan=False)
        return report

    def summaries(self):
        return [text for text in self.printer.output
                if 'Timer too close:' in text]

    def query(self):
        self.assertIn('GET_TIMER_TOO_CLOSE_STATS',
                      self.gcode.ready_gcode_handlers,
                      'warning-report query must be registered')
        self.assertIn('GET_TIMER_TOO_CLOSE_STATS',
                      self.gcode.gcode_handlers,
                      'warning-report query must work when not ready')
        self.printer.output.clear()
        self.gcode.run_script('GET_TIMER_TOO_CLOSE_STATS')
        return '\n'.join(self.printer.output)

    def test_before_any_print_warning_is_excluded_and_query_is_explicit(self):
        self.warning(received=99.)
        self.assertEqual(self.report(), {
            'filename': '', 'state': 'standby', 'total': 0,
            'attribution': 'receive_time_scheduled_layer', 'by_layer': []})
        self.assertIn('Timer too close: no print recorded', self.query())

    def test_startup_and_scheduled_layer_use_separate_mcu_buckets(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        self.warning(received=100.125)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.toolhead.resolve_move(move, 10.5)
        self.warning('eboard', 100.75)
        self.assertEqual(self.report(), {
            'filename': 'part.gcode', 'state': 'printing', 'total': 2,
            'attribution': 'receive_time_scheduled_layer', 'by_layer': [
                {'mcu': 'mcu', 'layer': 0, 'count': 1,
                 'first_elapsed': .125, 'last_elapsed': .125},
                {'mcu': 'eboard', 'layer': 1, 'count': 1,
                 'first_elapsed': .75, 'last_elapsed': .75}]})

    def test_initial_total_layer_uses_receipt_estimate_not_motion_priming(self):
        self.start()
        self.reactor.now = 100.5
        self.info(TOTAL_LAYER=3)
        self.warning(received=100.5)
        self.warning(received=100.625)
        self.assertEqual(self.toolhead.last_move_time_calls, 0)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 2,
             'first_elapsed': .5, 'last_elapsed': .625}])

    def test_first_metadata_does_not_relabel_earlier_unknown_warnings(self):
        self.start()
        self.warning(received=100.125)
        self.reactor.now = 100.5
        self.info(TOTAL_LAYER=3)
        # Even a callback delivered after metadata uses its earlier receipt.
        self.warning(received=100.25)
        self.warning(received=100.625)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 1,
             'first_elapsed': .625, 'last_elapsed': .625},
            {'mcu': 'mcu', 'layer': None, 'count': 2,
             'first_elapsed': .125, 'last_elapsed': .25}])

    def test_metadata_present_at_start_initializes_effective_layer(self):
        self.stats.set_current_file('prepared.gcode')
        self.info(TOTAL_LAYER=3, CURRENT_LAYER=2)
        self.stats.note_start()
        self.warning(received=100.125)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 2, 'count': 1,
             'first_elapsed': .125, 'last_elapsed': .125}])

    def test_delayed_delivery_uses_receipt_not_parser_current_layer(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.toolhead.resolve_move(move, 10.5)
        self.reactor.now = 102.
        self.warning(received=100.25)
        self.warning(received=100.75)
        self.assertEqual([(r['layer'], r['count'])
                          for r in self.report()['by_layer']], [(0, 1), (1, 1)])

    def test_warning_waits_for_pending_motion_marker_before_attribution(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.reactor.now = 101.
        self.warning(received=100.25)
        self.warning(received=100.75)
        # Query/status may not force lookahead generation to resolve a marker.
        self.report()
        self.query()
        self.assertEqual(self.toolhead.last_move_time_calls, 0)
        self.toolhead.resolve_move(move, 10.5)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 1,
             'first_elapsed': .25, 'last_elapsed': .25},
            {'mcu': 'mcu', 'layer': 1, 'count': 1,
             'first_elapsed': .75, 'last_elapsed': .75}])

    def test_equal_print_time_markers_use_last_applicable_layer(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.info(CURRENT_LAYER=2)
        self.toolhead.resolve_move(move, 10.5)
        self.warning(received=100.5)
        self.assertEqual(self.report()['by_layer'][0]['layer'], 2)

    def test_absent_or_removed_metadata_is_unknown_not_startup(self):
        self.start()
        self.warning(received=100.125)
        self.reactor.now = 100.25
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(TOTAL_LAYER=0)
        self.toolhead.resolve_move(move, 10.5)
        self.warning(received=100.75)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 2,
             'first_elapsed': .125, 'last_elapsed': .75}])

    def test_layer_and_mcu_buckets_have_deterministic_numeric_order(self):
        self.start()
        self.warning('unknown', 100.)
        self.info(TOTAL_LAYER=12)
        for layer, print_time in ((2, 10.5), (10, 11.)):
            move = self.toolhead.queue_move()
            self.info(CURRENT_LAYER=layer)
            self.toolhead.resolve_move(move, print_time)
        self.warning('zboard', 101.25)
        self.warning('eboard', 100.75)
        self.warning('mcu', 100.75)
        self.assertEqual([(r['layer'], r['mcu'])
                          for r in self.report()['by_layer']],
                         [(2, 'eboard'), (2, 'mcu'), (10, 'zboard'),
                          (None, 'unknown')])

    def test_pause_resume_keeps_one_session_and_includes_pause_elapsed(self):
        self.start()
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_pause()
        self.assertEqual(self.report()['state'], 'paused')
        self.warning(received=102.)
        self.reactor.now = 104.
        self.stats.note_start()
        self.warning(received=104.5)
        report = self.report()
        self.assertEqual(report['state'], 'printing')
        self.assertEqual(report['total'], 3)
        self.assertEqual(report['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 3,
             'first_elapsed': .5, 'last_elapsed': 4.5}])

    def test_out_of_order_delivery_preserves_earliest_and_latest_receipts(self):
        self.start()
        self.warning(received=100.75)
        self.warning(received=100.25)
        self.warning(received=100.5)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 3,
             'first_elapsed': .25, 'last_elapsed': .75}])

    def test_finishing_while_paused_retains_paused_interval_warnings(self):
        self.start()
        self.reactor.now = 101.
        self.stats.note_pause()
        self.warning(received=103.)
        self.reactor.now = 104.
        self.stats.note_cancel()
        self.reactor.drain()
        self.assertEqual(self.report()['state'], 'cancelled')
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 1,
             'first_elapsed': 3., 'last_elapsed': 3.}])

    def test_terminal_states_retain_counts_and_emit_deferred_summary(self):
        for state in ('complete', 'cancelled', 'error'):
            with self.subTest(state=state):
                self.start()
                self.warning(received=100.5)
                self.printer.output.clear()
                self.reactor.now = 101.
                if state == 'complete':
                    self.stats.note_complete()
                elif state == 'cancelled':
                    self.stats.note_cancel()
                else:
                    self.stats.note_error('original motion fault')
                    self.assertEqual(self.stats.error_message,
                                     'original motion fault')
                self.assertEqual(self.summaries(), [])
                self.reactor.drain()
                self.assertEqual(self.report()['state'], state)
                self.assertEqual(self.report()['total'], 1)
                self.assertEqual(len(self.summaries()), 1)
                self.assertIn('part.gcode [%s]' % state, self.summaries()[0])

    def test_reset_and_file_selection_retain_finished_report(self):
        self.start()
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.reactor.drain()
        expected = self.report()
        self.stats.reset()
        self.assertEqual(self.report(), expected)
        self.stats.set_current_file('selected-next.gcode')
        self.assertEqual(self.report(), expected)
        self.assertIn('part.gcode [complete]', self.query())

    def test_reset_active_session_finalizes_it_as_cancelled(self):
        self.start()
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.reset()
        self.reactor.drain()
        self.assertEqual(self.report()['state'], 'cancelled')
        self.assertEqual(self.report()['total'], 1)
        self.assertEqual(self.report()['filename'], 'part.gcode')
        self.assertEqual(len(self.summaries()), 1)

    def test_next_print_clears_old_report_only_on_start(self):
        self.start()
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.reactor.drain()
        self.start('next.gcode', 110.)
        self.assertEqual(self.report(), {
            'filename': 'next.gcode', 'state': 'printing', 'total': 0,
            'attribution': 'receive_time_scheduled_layer', 'by_layer': []})

    def test_receive_time_filters_closed_interval_and_includes_endpoints(self):
        self.start()
        self.warning(received=99.5)
        self.warning(received=100.)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.warning(received=101.5)
        self.warning(received=101.)
        self.reactor.drain()
        self.assertEqual(self.report()['total'], 2)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 2,
             'first_elapsed': 0., 'last_elapsed': 1.}])

    def test_old_serial_callback_cannot_contaminate_new_print(self):
        self.start()
        self.bridge_warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.start('new.gcode', 110.)
        self.reactor.drain()
        self.assertEqual(self.report()['total'], 0)
        self.assertEqual(self.report()['filename'], 'new.gcode')

    def test_stale_layer_callback_cannot_change_new_session(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        old_move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=2)
        self.reactor.now = 101.
        self.stats.note_cancel()
        self.reactor.drain()
        self.start('new.gcode', 110.)
        self.info(TOTAL_LAYER=3)
        # Even an old callback with a print time after the new start is stale.
        self.toolhead.resolve_move(old_move, 20.25)
        self.warning(received=110.5)
        self.assertEqual(self.report()['by_layer'][0]['layer'], 0)
        self.assertEqual(self.report()['total'], 1)

    def test_delayed_terminal_receipt_updates_retained_report_and_summary(self):
        self.start()
        with self.assertLogs(level='INFO') as logs:
            self.reactor.now = 101.
            self.stats.note_complete()
            self.reactor.drain()
            self.assertEqual(len(self.summaries()), 1)
            self.assertIn('0 warning(s)', self.summaries()[0])
            self.assertEqual(
                [record.getMessage() for record in logs.records
                 if record.getMessage().startswith('Timer too close:')],
                ['Timer too close: 0 warning(s) - part.gcode [complete]'])
            self.reactor.now = 105.
            self.bridge_warning('eboard', 100.75)
            self.reactor.drain()
        self.assertEqual(
            [record.getMessage() for record in logs.records
             if record.getMessage().startswith('Timer too close:')],
            ['Timer too close: 0 warning(s) - part.gcode [complete]',
             'Timer too close: 1 warning(s) - part.gcode [complete] updated\n'
             'MCU=eboard layer=unknown count=1 first=0.750000s last=0.750000s'])
        self.assertEqual(self.report()['state'], 'complete')
        self.assertEqual(self.report()['total'], 1)
        self.assertEqual(len(self.summaries()), 2)
        self.assertIn('updated', self.summaries()[-1].lower())
        self.assertIn('1 warning(s)', self.summaries()[-1])
        self.assertEqual(self.report()['by_layer'][0]['last_elapsed'], .75)

    def test_terminal_report_accounts_for_already_queued_serial_warning(self):
        self.start()
        self.bridge_warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.assertEqual(self.summaries(), [])
        self.reactor.drain()
        self.assertEqual(self.report()['total'], 1)
        self.assertTrue(self.summaries())
        self.assertTrue(all('1 warning(s)' in text for text in self.summaries()))

    def test_shutdown_summary_includes_queued_warning_without_motion_flush(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        with self.assertLogs(level='INFO') as logs:
            self.bridge_warning('eboard', 100.75)
            self.reactor.now = 101.
            self.printer.send_event('klippy:shutdown')
            self.assertEqual(self.report()['total'], 0)
            self.assertEqual(self.summaries(), [])
            self.reactor.turn()
            self.assertEqual(self.report(), {
                'filename': 'part.gcode', 'state': 'error', 'total': 1,
                'attribution': 'receive_time_scheduled_layer', 'by_layer': [
                    {'mcu': 'eboard', 'layer': 0, 'count': 1,
                     'first_elapsed': .75, 'last_elapsed': .75}]})
            expected = (
                'Timer too close: 1 warning(s) - part.gcode [error]\n'
                'MCU=eboard layer=startup count=1 '
                'first=0.750000s last=0.750000s')
            self.assertEqual(self.summaries(),
                             ['// ' + expected.replace('\n', '\n// ')])
            # Later motion resolution and virtual-SD cleanup cannot revise
            # the shutdown attribution or emit another unchanged summary.
            self.toolhead.resolve_move(move, 10.5)
            self.stats.note_error('virtual SD cleanup')
            self.stats.note_complete()
            self.printer.send_event('klippy:shutdown')
            self.reactor.drain()
        self.assertEqual(self.toolhead.last_move_time_calls, 0)
        self.assertEqual(self.report()['by_layer'][0]['layer'], 0)
        self.assertEqual(len(self.summaries()), 1)
        self.assertEqual(
            [record.getMessage() for record in logs.records
             if record.getMessage().startswith('Timer too close:')],
            [expected])

    def test_repeated_finish_and_shutdown_do_not_change_or_repeat_report(self):
        self.start()
        self.warning(received=100.5)
        with self.assertLogs(level='INFO') as logs:
            self.reactor.now = 101.
            self.stats.note_complete()
            self.reactor.drain()
            expected = self.report()
            summaries = self.summaries()
            self.stats.note_complete()
            self.stats.note_cancel()
            self.printer.send_event('klippy:shutdown')
            self.reactor.drain()
        self.assertEqual(self.summaries(), summaries)
        self.assertEqual(
            [record.getMessage() for record in logs.records
             if record.getMessage().startswith('Timer too close:')],
            ['Timer too close: 1 warning(s) - part.gcode [complete]\n'
             'MCU=mcu layer=unknown count=1 first=0.500000s last=0.500000s'])
        self.assertEqual(self.report(), expected)
        self.assertEqual(len(self.summaries()), 1)

    def test_shutdown_retains_error_and_does_not_force_pending_motion(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.warning(received=100.75)
        self.reactor.now = 101.
        self.printer.send_event('klippy:shutdown')
        self.reactor.drain()
        self.assertEqual(self.report()['state'], 'error')
        self.assertEqual(self.report()['by_layer'][0]['layer'], 0)
        self.assertEqual(self.toolhead.last_move_time_calls, 0)
        expected = self.report()
        self.stats.note_error('virtual SD cleanup')
        self.stats.note_complete()
        self.reactor.drain()
        self.assertEqual(self.report(), expected)
        self.assertEqual(len(self.summaries()), 1)
        self.assertIn('part.gcode [error]', self.query())

    def test_zero_warning_and_zero_duration_empty_prints_are_reports(self):
        for duration in (0., 1.):
            with self.subTest(duration=duration):
                self.start('empty.gcode')
                self.printer.output.clear()
                self.reactor.now += duration
                self.stats.note_complete()
                self.reactor.drain()
                self.assertEqual(self.report(), {
                    'filename': 'empty.gcode', 'state': 'complete', 'total': 0,
                    'attribution': 'receive_time_scheduled_layer',
                    'by_layer': []})
                self.assertEqual(len(self.summaries()), 1)
                self.assertIn('0 warning(s) - empty.gcode [complete]',
                              self.summaries()[0])

    def test_status_snapshots_are_deep_copies_and_query_does_not_reset(self):
        self.start()
        self.warning(received=100.5)
        snapshot = self.report()
        snapshot['filename'] = 'corrupt'
        snapshot['by_layer'][0]['count'] = 999
        snapshot['by_layer'].append({'mcu': 'injected'})
        self.query()
        self.assertEqual(self.report()['filename'], 'part.gcode')
        self.assertEqual(self.report()['total'], 1)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': None, 'count': 1,
             'first_elapsed': .5, 'last_elapsed': .5}])
        self.assertEqual(self.stats.get_status(self.reactor.now)['state'],
                         'printing')

    def test_serial_worker_cannot_mutate_statistics_before_reactor_dispatch(self):
        self.start()
        self.assertEqual(self.report()['total'], 0)
        failures = []

        def receive():
            try:
                self.bridge_warning('eboard', 100.5)
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=receive)
        worker.start()
        worker.join(timeout=5.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(self.report()['total'], 0)
        self.reactor.drain()
        self.assertEqual(self.report()['total'], 1)

    def test_real_dispatch_bridge_and_report_end_to_end(self):
        self.start('smoke.gcode')
        self.info(TOTAL_LAYER=3)
        self.bridge_warning('mcu', 100.125)
        self.reactor.drain()
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.toolhead.resolve_move(move, 10.5)
        self.bridge_warning('eboard', 100.625)
        self.bridge_warning('eboard', 100.875)
        self.reactor.now = 101.
        self.reactor.drain()
        with self.assertLogs(level='INFO') as logs:
            self.stats.note_complete()
            self.reactor.drain()
        self.assertEqual(
            [record.getMessage() for record in logs.records
             if record.getMessage().startswith('Timer too close:')],
            ['Timer too close: 3 warning(s) - smoke.gcode [complete]\n'
             'MCU=mcu layer=startup count=1 first=0.125000s last=0.125000s\n'
             'MCU=eboard layer=1 count=2 first=0.625000s last=0.875000s'])
        self.assertEqual(len(self.summaries()), 1)
        expected = {
            'filename': 'smoke.gcode', 'state': 'complete', 'total': 3,
            'attribution': 'receive_time_scheduled_layer', 'by_layer': [
                {'mcu': 'mcu', 'layer': 0, 'count': 1,
                 'first_elapsed': .125, 'last_elapsed': .125},
                {'mcu': 'eboard', 'layer': 1, 'count': 2,
                 'first_elapsed': .625, 'last_elapsed': .875}]}
        self.assertEqual(self.report(), expected)
        with self.assertLogs(level='INFO') as query_logs:
            text = self.query()
        self.assertEqual(
            [record.getMessage() for record in query_logs.records],
            [record.getMessage() for record in logs.records])
        self.assertIn('Timer too close: 3 warning(s) - smoke.gcode [complete]',
                      text)
        self.assertIn('MCU=mcu layer=startup count=1', text)
        self.assertIn('MCU=eboard layer=1 count=2', text)
        self.assertRegex(text, r'first=0\.125[0-9]*s last=0\.125[0-9]*s')
        self.assertRegex(text, r'first=0\.625[0-9]*s last=0\.875[0-9]*s')
        self.stats.reset()
        self.assertEqual(self.report(), expected)
        with self.assertLogs(level='INFO') as retained_query_logs:
            self.assertIn('3 warning(s)', self.query())
        self.assertEqual(
            [record.getMessage() for record in retained_query_logs.records],
            [record.getMessage() for record in logs.records])

    def test_unresolved_snapshot_rebuckets_once_after_all_markers_resolve(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        first = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        second = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=2)
        self.warning(received=100.75)
        self.assertEqual(self.report()['total'], 1)
        self.toolhead.resolve_move(second, 11.)
        self.assertEqual(self.report()['total'], 1)
        self.toolhead.resolve_move(first, 10.5)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 1, 'count': 1,
             'first_elapsed': .75, 'last_elapsed': .75}])
        self.assertEqual(self.report()['total'], 1)

    def test_late_motion_marker_cannot_rewrite_finalized_report(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        self.warning(received=100.75)
        self.reactor.now = 101.
        self.stats.note_cancel()
        self.reactor.drain()
        self.toolhead.resolve_move(move, 10.5)
        self.warning(received=100.875)
        self.reactor.drain()
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 2,
             'first_elapsed': .75, 'last_elapsed': .875}])
        self.assertEqual(len(self.summaries()), 2)

    def test_deferred_summary_keeps_its_session_after_next_print_starts(self):
        self.start('old.gcode')
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.note_complete()
        self.start('new.gcode', 110.)
        self.reactor.drain()
        self.assertEqual(len(self.summaries()), 1)
        self.assertIn('1 warning(s) - old.gcode [complete]', self.summaries()[0])
        self.assertEqual(self.report()['filename'], 'new.gcode')
        self.assertEqual(self.report()['total'], 0)

    def test_reset_before_deferred_summary_retains_cancelled_session(self):
        self.start()
        self.warning(received=100.5)
        self.reactor.now = 101.
        self.stats.set_current_file('selected.gcode')
        self.reactor.drain()
        self.assertEqual(len(self.summaries()), 1)
        self.assertIn('part.gcode [cancelled]', self.summaries()[0])
        self.assertEqual(self.stats.filename, 'selected.gcode')
        self.assertEqual(self.report()['filename'], 'part.gcode')

    def test_first_metadata_with_current_layer_keeps_startup_baseline(self):
        self.start()
        move = self.toolhead.queue_move()
        self.info(TOTAL_LAYER=3, CURRENT_LAYER=2)
        self.warning(received=100.125)
        self.warning(received=100.75)
        self.toolhead.resolve_move(move, 10.5)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 1,
             'first_elapsed': .125, 'last_elapsed': .125},
            {'mcu': 'mcu', 'layer': 2, 'count': 1,
             'first_elapsed': .75, 'last_elapsed': .75}])

    def test_reintroduced_metadata_uses_scheduled_not_startup_boundary(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        move = self.toolhead.queue_move()
        self.info(TOTAL_LAYER=0)
        self.toolhead.resolve_move(move, 10.25)
        move = self.toolhead.queue_move()
        self.reactor.now = 100.5
        self.info(TOTAL_LAYER=3)
        self.warning(received=100.625)
        self.warning(received=100.875)
        self.toolhead.resolve_move(move, 10.75)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 0, 'count': 1,
             'first_elapsed': .875, 'last_elapsed': .875},
            {'mcu': 'mcu', 'layer': None, 'count': 1,
             'first_elapsed': .625, 'last_elapsed': .625}])

    def test_equal_time_callbacks_use_command_order_not_resolution_order(self):
        self.start()
        self.info(TOTAL_LAYER=3)
        first = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=1)
        second = self.toolhead.queue_move()
        self.info(CURRENT_LAYER=2)
        self.warning(received=100.75)
        self.toolhead.resolve_move(second, 10.5)
        self.toolhead.resolve_move(first, 10.5)
        self.assertEqual(self.report()['by_layer'], [
            {'mcu': 'mcu', 'layer': 2, 'count': 1,
             'first_elapsed': .75, 'last_elapsed': .75}])

    def test_delayed_warning_updates_retained_report_after_file_selection(self):
        self.start()
        self.reactor.now = 101.
        self.stats.note_complete()
        self.reactor.drain()
        self.stats.set_current_file('selected.gcode')
        self.warning('eboard', 100.75)
        self.reactor.drain()
        self.assertEqual(self.report()['filename'], 'part.gcode')
        self.assertEqual(self.report()['state'], 'complete')
        self.assertEqual(self.report()['total'], 1)
        self.assertEqual(self.stats.filename, 'selected.gcode')
        self.assertIn('updated', self.summaries()[-1])

    def test_unknown_layer_report_is_labelled_unknown(self):
        self.start()
        self.warning(received=100.5)
        self.assertIn('MCU=mcu layer=unknown count=1', self.query())


if __name__ == '__main__':
    unittest.main()
