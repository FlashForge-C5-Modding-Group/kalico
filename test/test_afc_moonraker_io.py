# AFC Moonraker persistence and owner lifecycle contracts.
#
# Copyright (C) 2026 Armored Turtle
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Real-worker persistence contracts, using only an event-controlled loopback server."""

from __future__ import annotations

import ast
import io as stream_io
import json
import logging
import os
import threading
import traceback
import types
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from datetime import datetime
from enum import Enum
from functools import cached_property
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Empty, Queue
from unittest.mock import Mock, call, patch
from urllib.error import HTTPError
from urllib.parse import parse_qs, quote, urlencode, urljoin
from urllib.request import Request, urlopen

import pytest

# These dependencies are consumed by the AST-executed production classes.
AST_DEPENDENCIES = {
    "os": os,
    "ThreadPoolExecutor": ThreadPoolExecutor,
    "datetime": datetime,
    "Enum": Enum,
    "cached_property": cached_property,
    "HTTPError": HTTPError,
    "quote": quote,
    "urlencode": urlencode,
    "urljoin": urljoin,
    "Request": Request,
    "urlopen": urlopen,
}


def load_classes(filename, names, symbols=None):
    """Compile complete production classes, preserving source coverage filenames."""
    path = Path(__file__).resolve().parents[1] / 'klippy' / 'extras' / filename
    tree = ast.parse(path.read_text(encoding='utf-8'))
    nodes = [node for node in tree.body
             if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names]
    namespace = {**globals(), **AST_DEPENDENCIES}
    if symbols:
        namespace.update(symbols)
    future = ast.ImportFrom(module='__future__',
                            names=[ast.alias(name='annotations')], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, *nodes], type_ignores=[]))
    exec(compile(module, str(path), 'exec'), namespace)
    return types.SimpleNamespace(**{name: namespace[name] for name in names})


chelper = types.SimpleNamespace(get_ffi=Mock(return_value=(None, Mock())))
utils = load_classes('AFC_utils.py', ['AFC_moonraker', 'check_and_return'])
AFC_moonraker = utils.AFC_moonraker
stats_module = load_classes('AFC_stats.py', ['AFCStats_var', 'AFCStats'],
                           {'check_and_return': utils.check_and_return})
AFCStats_var = stats_module.AFCStats_var
AFCStats = stats_module.AFCStats


class MockReactor:
    NEVER = float('inf')

    def monotonic(self):
        return 0.

    def mutex(self):
        return threading.Lock()

    def register_timer(self, *args):
        return object()


class MockLogger:
    def __init__(self):
        self.messages = []

    def set_debug(self, value):
        pass



WAIT = 5


class DeferredReactor(MockReactor):
    def __init__(self):
        super().__init__()
        self.callbacks = Queue()

    def register_async_callback(self, callback, waketime=None):
        self.callbacks.put(callback)

    def run_callbacks(self):
        while True:
            try:
                callback = self.callbacks.get_nowait()
            except Empty:
                return
            callback(0.0)


class RecordHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append((record.levelname.lower(), record.getMessage(), record.thread))


class ThreadLogger(MockLogger):
    def __init__(self):
        super().__init__()
        self.threads = []
        self.details = []
        self.handler = RecordHandler()
        self.logger = logging.Logger("afc-loopback-test", logging.DEBUG)
        self.logger.addHandler(self.handler)

    def _record(self, level, message, kwargs):
        self.messages.append((level, message))
        self.threads.append(threading.get_ident())
        self.details.append(kwargs)

    def debug(self, message, **kwargs):
        self._record("debug", message, kwargs)

    def info(self, message, **kwargs):
        self._record("info", message, kwargs)

    def error(self, message, **kwargs):
        self._record("error", message, kwargs)


class LoopbackIO:
    def __init__(self):
        self.records = []
        self.requests = []
        self.contents = {}
        self.statuses = {}
        self.responses = {}
        self.gates = {}
        self.escaped = []
        self.condition = threading.Condition()
        self.reactor = DeferredReactor()
        self.logger = ThreadLogger()
        self.startup_values = None
        self.startup_reads = 0
        self.workers = []
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                self._respond()

            def do_DELETE(self):
                self._respond()

            def do_GET(self):
                self._respond()

            def log_message(self, *args):
                pass

            def _respond(self):
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length).decode()
                if self.headers.get("Content-Type") == "application/json":
                    form = {}
                    data = json.loads(body)
                else:
                    form = parse_qs(body)
                    data = None
                values = data if data is not None else {
                    name: entries[0] for name, entries in form.items()}
                namespace = values.get("namespace", "")
                key = values.get("key", "")
                value = values.get("value")
                with owner.condition:
                    index = len(owner.records)
                    owner.records.append((self.command, namespace, key, value))
                    owner.requests.append((self.path, form, data))
                    owner.condition.notify_all()
                gate = owner.gates.get(index)
                if gate is not None:
                    if not gate.wait(WAIT / 2):
                        owner.escaped.append(index)
                status = owner.statuses.get(index, 200)
                if status == 200:
                    if self.command == "POST":
                        owner.contents[(namespace, key)] = value
                    elif self.command == "DELETE":
                        owner.contents.pop((namespace, key), None)
                result = owner.responses.get(index, {"value": value})
                payload = json.dumps({"result": result}).encode()
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = False
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.client = self.new_client()
        self.logger.messages.clear()
        self.logger.threads.clear()
        self.logger.details.clear()

    def new_client(self):
        client = AFC_moonraker("http://127.0.0.1", str(self.server.server_port),
                              self.logger, self.reactor)
        self.workers.append(client)
        return client

    def startup_stats(self):
        self.startup_reads += 1
        return self.startup_values

    def wait_requests(self, count):
        with self.condition:
            assert self.condition.wait_for(lambda: len(self.records) >= count, timeout=WAIT)

    def wait_idle(self):
        completed = threading.Event()
        self.client._enqueue(completed.set, ())
        assert completed.wait(WAIT), "worker did not reach the completion barrier"

    def close(self):
        for gate in self.gates.values():
            gate.set()
        for client in self.workers:
            client.join_thread()
            assert not client._write_thread.is_alive()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(WAIT)
        assert not self.server_thread.is_alive()


@pytest.fixture
def moonraker_io():
    io = LoopbackIO()
    try:
        yield io
    finally:
        io.close()


@contextmanager
def joining(client):
    """Expose the point where shutdown admission has finished, without timing sleeps."""
    entered = threading.Event()
    completed = threading.Event()
    errors = []
    original_join = client._write_thread.join

    def observed_join(*args, **kwargs):
        entered.set()
        return original_join(*args, **kwargs)

    def close():
        try:
            client.join_thread()
        except BaseException as error:
            errors.append(error)
        finally:
            completed.set()

    client._write_thread.join = observed_join
    helper = threading.Thread(target=close)
    helper.start()
    try:
        assert entered.wait(WAIT)
        yield completed
    finally:
        # The caller must release its HTTP gate before this context exits.
        helper.join(WAIT + 1)
        client._write_thread.join = original_join
        assert not helper.is_alive()
        assert errors == []


class TestAFCMoonrakerInit:
    def test_constructor_owns_a_working_writer_until_explicit_join(self, moonraker_io):
        io = moonraker_io
        client = io.new_client()
        gate = io.gates[0] = threading.Event()
        client.update_afc_stats("constructed.count", 6)
        io.wait_requests(1)
        try:
            assert client._write_thread.is_alive()
            assert not gate.is_set()
            assert io.escaped == []
        finally:
            gate.set()
            client.join_thread()
        assert not client._write_thread.is_alive()
        assert io.records == [("POST", "afc_stats", "constructed.count", "6")]
        assert io.contents == {("afc_stats", "constructed.count"): "6"}
        assert io.logger.messages == [
            ("debug", f"Moonraker url: http://127.0.0.1:{io.server.server_port}"),
        ]


class TestAFCMoonrakerUpdateAfcStats:
    def test_increments_and_reset_return_before_response(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        stat = AFCStats_var("lane1", "load_count", None, io.client)
        stat.increase_count()
        io.wait_requests(1)
        stat.increase_count()
        stat.reset_count()
        stat.increase_count()
        try:
            assert not gate.is_set()
            assert io.escaped == []
            assert stat.value == 1
        finally:
            gate.set()
        io.wait_idle()
        assert io.records == [
            ("POST", "afc_stats", "lane1.load_count", "1"),
            ("POST", "afc_stats", "lane1.load_count", "2"),
            ("POST", "afc_stats", "lane1.load_count", "0"),
            ("POST", "afc_stats", "lane1.load_count", "1"),
        ]
        assert io.contents == {("afc_stats", "lane1.load_count"): "1"}
        assert io.logger.messages == []


class TestAFCMoonrakerRemoveDatabaseEntry:
    def test_migration_precedes_increment_and_removes_legacy_key(self, moonraker_io):
        io = moonraker_io
        io.contents[("afc_stats", "legacy.count")] = "7"
        stat = AFCStats_var("legacy", "count", {"legacy": {"count": 7}},
                            io.client, "lane1")
        stat.increase_count()
        io.wait_idle()
        io.reactor.run_callbacks()
        assert stat.value == 8
        assert io.records == [
            ("POST", "afc_stats", "lane1.count", "7"),
            ("DELETE", "afc_stats", "legacy.count", None),
            ("POST", "afc_stats", "lane1.count", "8"),
        ]
        assert io.contents == {("afc_stats", "lane1.count"): "8"}
        assert io.logger.messages == [("debug", "Removing legacy.count from afc_stats")]
        assert io.logger.threads == [threading.get_ident()]


class TestAFCMoonrakerWriteWorker:
    def test_http_error_does_not_stop_later_persistence(self, moonraker_io):
        io = moonraker_io
        io.statuses[0] = 500
        io.client.update_afc_stats("failed", 1)
        io.client.update_afc_stats("later", 2)
        io.wait_idle()
        assert io.client._write_thread.is_alive()
        assert io.contents == {("afc_stats", "later"): "2"}
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("error", io.client.ERROR_STRING),
            ("error", "Error when trying to update failed in moonraker, see AFC.log for more info"),
        ]
        assert "HTTP Error 500" in io.logger.details[0]["traceback"]
        assert io.logger.threads == [threading.get_ident(), threading.get_ident()]

    def test_callable_exception_is_contained(self, moonraker_io):
        io = moonraker_io

        def fail():
            raise ValueError("controlled worker failure")

        with patch.object(traceback, "format_exc",
                   return_value="ValueError: controlled worker failure"):
            io.client._write_queue.put_nowait((fail, ()))
            io.client.update_afc_stats("survivor", 3)
            io.wait_idle()
        assert io.contents == {("afc_stats", "survivor"): "3"}
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("error", "Unexpected error in moonraker background writer"),
            ("debug", "ValueError: controlled worker failure"),
        ]
        assert io.logger.threads == [threading.get_ident(), threading.get_ident()]

    def test_native_thread_naming_failure_does_not_prevent_persistence(self, moonraker_io):
        io = moonraker_io
        completed = threading.Event()
        with patch.object(chelper, "get_ffi", side_effect=RuntimeError("native naming unavailable")):
            client = io.new_client()
            client.update_afc_stats("survivor", 9)
            client._write_queue.put_nowait((completed.set, ()))
            assert completed.wait(WAIT)
        assert io.contents == {("afc_stats", "survivor"): "9"}
        assert io.logger.messages == [
            ("debug", f"Moonraker url: http://127.0.0.1:{io.server.server_port}"),
        ]


class TestAFCMoonrakerJoinThread:
    def test_disconnect_attempts_all_admitted_operations_in_order(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.client.update_afc_stats("first", 1)
        io.wait_requests(1)
        io.client.update_afc_stats("second", 2)
        io.client.remove_database_entry("afc_stats", "legacy.count")
        io.client.update_afc_stats("last", 3)
        with joining(io.client) as completed:
            try:
                assert not completed.is_set()
            finally:
                gate.set()
        assert completed.is_set()
        assert io.records == [
            ("POST", "afc_stats", "first", "1"),
            ("POST", "afc_stats", "second", "2"),
            ("DELETE", "afc_stats", "legacy.count", None),
            ("POST", "afc_stats", "last", "3"),
        ]
        assert io.contents == {
            ("afc_stats", "first"): "1",
            ("afc_stats", "second"): "2",
            ("afc_stats", "last"): "3",
        }
        io.client.join_thread()
        assert not io.client._write_thread.is_alive()
        with pytest.raises(RuntimeError, match="^AFC Moonraker worker is closed$"):
            io.client.update_afc_stats("rejected", 4)
        assert len(io.records) == 4


class TestAFCMoonrakerEnqueue:
    @pytest.mark.parametrize("producer", [
        lambda client: client.update_afc_stats("key", 1),
        lambda client: client.remove_database_entry("afc_stats", "key"),
    ])
    def test_closed_worker_rejects_every_producer(self, moonraker_io, producer):
        io = moonraker_io
        io.client.join_thread()
        with pytest.raises(RuntimeError, match="^AFC Moonraker worker is closed$"):
            producer(io.client)
        assert io.records == []
        assert not io.client._write_thread.is_alive()

    def test_open_admission_executes_callable(self, moonraker_io):
        io = moonraker_io
        result = []
        io.client._enqueue(result.append, ("accepted",))
        io.wait_idle()
        assert result == ["accepted"]
        assert io.logger.messages == []


class TestAFCMoonrakerLogAsync:
    def test_logging_is_deferred_and_runs_on_reactor_thread(self, moonraker_io):
        io = moonraker_io
        io.client._log_async(io.logger.info, "deferred diagnostic")
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [("info", "deferred diagnostic")]
        assert io.logger.threads == [threading.get_ident()]
        io.reactor.run_callbacks()
        assert io.logger.messages == [("info", "deferred diagnostic")]


class TestAFCMoonrakerDrainLogs:
    def test_shutdown_preserves_diagnostics_without_ui_or_duplicates(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.statuses[0] = 500
        io.client._log_async(io.logger.info, "pending info")
        io.client._log_async(io.logger.debug, "pending debug", traceback="supplied traceback")
        io.client.update_afc_stats("failed", 1)
        io.wait_requests(1)
        io.client.update_afc_stats("later", 2)
        with joining(io.client):
            gate.set()
        assert io.logger.messages == []
        assert io.records == [
            ("POST", "afc_stats", "failed", "1"),
            ("POST", "afc_stats", "later", "2"),
        ]
        assert io.contents == {("afc_stats", "later"): "2"}
        records = io.logger.handler.records
        assert [level for level, _, _ in records] == ["info", "debug", "error", "error"]
        assert records[0][1] == "pending info"
        assert "pending debug" in records[1][1]
        assert "supplied traceback" in records[1][1]
        assert io.client.ERROR_STRING in records[2][1]
        assert "HTTP Error 500" in records[2][1]
        assert records[3][1] == (
            "Error when trying to update failed in moonraker, see AFC.log for more info")
        assert len({thread for _, _, thread in records}) == 1
        before = list(records)
        io.reactor.run_callbacks()
        io.client.join_thread()
        assert io.logger.messages == []
        assert records == before

    def test_empty_drain_is_harmless(self, moonraker_io):
        io = moonraker_io
        io.client._drain_logs()
        io.client._drain_logs(shutdown=True)
        assert io.logger.messages == []
        assert io.logger.handler.records == []


class TestAFCMoonrakerGetResults:
    def test_post_timeout_reports_error_and_next_request_persists(self, moonraker_io):
        io = moonraker_io
        io.client.REQUEST_TIMEOUT = 0.05
        gate = io.gates[0] = threading.Event()
        io.client.update_afc_stats("stalled", 1)
        io.wait_requests(1)
        io.client.update_afc_stats("later", 2)
        try:
            io.wait_idle()
            assert io.contents == {("afc_stats", "later"): "2"}
            assert io.escaped == []
        finally:
            gate.set()
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("error", io.client.ERROR_STRING),
            ("error", "Error when trying to update stalled in moonraker, see AFC.log for more info"),
        ]
        assert "timed out" in io.logger.details[0]["traceback"]
        assert io.logger.threads == [threading.get_ident(), threading.get_ident()]

    @pytest.mark.parametrize("status", [199, 200, 300, 301])
    @pytest.mark.parametrize("print_error", [False, True])
    def test_response_boundaries_and_logger_level(self, moonraker_io, status, print_error):
        io = moonraker_io
        response = stream_io.BytesIO(b'{"result": {"answer": 42}}')
        response.status = status
        response.reason = "controlled response"
        with patch.dict(AFC_moonraker._get_results.__globals__,
                        urlopen=lambda *args, **kwargs: response):
            result = io.client._get_results(io.client.database_url, print_error)
        assert response.closed
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        if status in (200, 300):
            assert result == {"answer": 42}
            assert io.logger.messages == []
        else:
            level = "error" if print_error else "debug"
            assert result is None
            assert io.logger.messages == [
                (level, io.client.ERROR_STRING),
                (level, f"Response: {status} Reason: controlled response"),
            ]
            assert io.logger.threads == [threading.get_ident()] * 2


class TestAFCMoonrakerRemoveDatabaseEntrySync:
    def test_delete_timeout_is_contained_and_later_write_persists(self, moonraker_io):
        io = moonraker_io
        io.client.REQUEST_TIMEOUT = 0.05
        gate = io.gates[0] = threading.Event()
        with patch.object(traceback, "format_exc", return_value="TimeoutError: timed out"):
            io.client.remove_database_entry("afc_stats", "stalled")
            io.wait_requests(1)
            io.client.update_afc_stats("later", 2)
            try:
                io.wait_idle()
                assert io.contents == {("afc_stats", "later"): "2"}
                assert io.escaped == []
            finally:
                gate.set()
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("error", "Unexpected error in moonraker background writer"),
            ("debug", "TimeoutError: timed out"),
        ]
        assert io.logger.threads == [threading.get_ident(), threading.get_ident()]

    def test_http_delete_error_does_not_stop_later_write(self, moonraker_io):
        io = moonraker_io
        io.statuses[0] = 500
        io.client.remove_database_entry("afc_stats", "failed")
        io.client.update_afc_stats("later", 2)
        io.wait_idle()
        assert io.contents == {("afc_stats", "later"): "2"}
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("debug", "Error occurred when trying to delete failed from afc_stats namespace"),
            ("debug", "HTTP Error 500: Internal Server Error"),
        ]
        assert io.logger.threads == [threading.get_ident(), threading.get_ident()]


class TestAFCMoonrakerUpdateAfcStatsSync:
    def test_form_payload_is_preserved(self, moonraker_io):
        io = moonraker_io
        io.client._update_afc_stats_sync("direct.count", 17)
        assert io.records == [("POST", "afc_stats", "direct.count", "17")]
        assert io.requests == [("/server/database/item", {
            "request_method": ["POST"], "namespace": ["afc_stats"],
            "key": ["direct.count"], "value": ["17"]}, None)]
        assert io.contents == {("afc_stats", "direct.count"): "17"}
        assert io.logger.messages == []


class TestAFCMoonrakerNamespaceDispatch:
    def test_nonstatistics_delete_blocks_until_cleanup_finishes(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.contents[("lane_data", "stale")] = "old"
        completed = threading.Event()
        errors = []

        def remove():
            try:
                io.client.remove_database_entry("lane_data", "stale")
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()

        helper = threading.Thread(target=remove)
        helper.start()
        try:
            io.wait_requests(1)
            assert not completed.is_set()
            assert io.contents == {("lane_data", "stale"): "old"}
            assert io.escaped == []
        finally:
            gate.set()
            helper.join(WAIT)
        assert not helper.is_alive()
        assert errors == []
        assert completed.is_set()
        assert io.contents == {}
        io.reactor.run_callbacks()
        assert io.records == [("DELETE", "lane_data", "stale", None)]
        assert io.logger.messages == [("debug", "Removing stale from lane_data")]
        assert io.logger.threads == [threading.get_ident()]

    def test_statistics_delete_returns_before_response_and_retains_fifo(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.client.remove_database_entry("afc_stats", "legacy")
        io.wait_requests(1)
        io.client.update_afc_stats("next", 4)
        try:
            assert not gate.is_set()
            assert io.escaped == []
            assert io.records == [("DELETE", "afc_stats", "legacy", None)]
        finally:
            gate.set()
        io.wait_idle()
        io.reactor.run_callbacks()
        assert io.records == [
            ("DELETE", "afc_stats", "legacy", None),
            ("POST", "afc_stats", "next", "4"),
        ]
        assert io.contents == {("afc_stats", "next"): "4"}
        assert io.logger.messages == [("debug", "Removing legacy from afc_stats")]

    def test_nonstatistics_delete_remains_available_after_worker_closes(self, moonraker_io):
        io = moonraker_io
        io.client.join_thread()
        io.client.remove_database_entry("lane_data", "stale")
        assert io.records == [("DELETE", "lane_data", "stale", None)]
        io.client._drain_logs(shutdown=True)
        assert io.logger.messages == []
        assert [(level, text) for level, text, _ in io.logger.handler.records] == [
            ("debug", "Removing stale from lane_data")]


assist = load_classes('AFC_assist.py', [
    'EspoolerDir', 'AFCEspoolerStats', 'Espooler_values', 'Espooler'])
AFCEspoolerStats = assist.AFCEspoolerStats
AFCExtruderStats = load_classes('AFC_extruder.py', ['AFCExtruderStats']).AFCExtruderStats
lane_types = load_classes('AFC_lane.py', [
    'AssistActive', 'SpeedMode', 'MoveDirection', 'AFCLaneState',
    'AFCHomingPoints', 'AFCMoveWarning', 'AFCLane'],
    {'AFC_assist': assist, 'EXCLUDE_TYPES': ['HTLF', 'Claymore', 'OpenAMS', 'ViViD']})
AFCLane = lane_types.AFCLane
AFC_PrintFileMetaData = load_classes('AFC_utils.py',
                                   ['AFC_PrintFileMetaData']).AFC_PrintFileMetaData
owner_types = load_classes('AFC.py', ['State', 'afc'], vars(lane_types))
AFC = owner_types.afc


class Config:
    def __init__(self, printer, name="AFC", **values):
        self.printer = printer
        self.name = name
        self.values = values

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, key, default=None, **kwargs):
        return self.values.get(key, default)

    getint = getfloat = getboolean = getlists = get

    def error(self, message):
        return ValueError(message)


class Printer:
    def __init__(self, reactor):
        self.reactor = reactor
        self.handlers = {}
        self.events = []
        self.start_args = {}
        self.objects = {}

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *args):
        self.events.append(event)
        for callback in self.handlers.get(event, []):
            callback(*args)

    def lookup_object(self, name, default=None):
        return self.objects.setdefault(name, Mock())

    def load_object(self, config, name):
        return self.lookup_object(name)

    def config_error(self, message):
        return ValueError(message)


@pytest.fixture
def afc_connection(moonraker_io):
    io = moonraker_io
    printer = Printer(io.reactor)
    config = Config(printer, track_filament_weight=False, trsync_update=False)
    namespace = AFC.__init__.__globals__
    with patch.dict(namespace, AFC_logger=lambda *args: io.logger,
                    afcDeltaTime=Mock()), patch.object(AFC, "print_version"):
        obj = AFC(config)
    printer.objects['AFC'] = obj
    obj.toolhead = printer.lookup_object('toolhead')
    io.client.get_afc_stats = io.startup_stats
    io.client.wait_for_moonraker = Mock(return_value=True)
    io.client.delete_lane_data = Mock()
    io.client.get_spoolman_server = Mock(return_value=None)
    io.client.check_for_td1 = Mock(return_value=(True, True, True))
    created = []

    def create(host, port, logger, reactor):
        client = io.client if not created else io.new_client()
        assert reactor is io.reactor
        created.append(client)
        return client

    with patch.dict(namespace, AFC_moonraker=create):
        try:
            yield obj, io
        finally:
            printer.send_event('klippy:disconnect')


def make_lane(obj):
    config = Config(obj.printer, name="AFC_stepper lane1", unit="unit:0")
    lane = AFCLane(config)
    lane.unit_obj = types.SimpleNamespace(type="BoxTurtle")
    lane.espooler.stats = AFCEspoolerStats(lane.name, lane.espooler)
    return lane


def make_extruder_stats(obj, name="extruder"):
    extruder = types.SimpleNamespace(afc=obj, logger=obj.logger)
    stats = AFCExtruderStats(name, extruder, 200)
    obj.printer.register_event_handler("afc:moonraker_connect", stats.handle_moonraker_stats)
    return stats


class TestAFCInit:
    def test_statistics_and_client_are_lazy_and_disconnect_is_registered(self, afc_connection):
        obj, io = afc_connection
        assert obj.afc_stats is None
        assert obj.moonraker is None
        assert io.startup_reads == 0
        assert io.records == []
        assert obj.printer.handlers["klippy:disconnect"] == [obj._handle_disconnect]
        assert io.logger.messages == []

    @pytest.mark.parametrize(("automatic", "tracking", "temps"), [
        (False, False, None), (True, True, ("PLA:210",)),
    ])
    def test_constructor_preserves_existing_configuration_branches(
            self, moonraker_io, automatic, tracking, temps):
        io = moonraker_io
        printer = Printer(io.reactor)
        printer.start_args['debuginput'] = 'test'
        config = Config(printer, auto_spool_switch=automatic,
                        track_filament_weight=tracking, default_material_temps=temps)
        with patch.dict(AFC.__init__.__globals__, AFC_logger=lambda *args: io.logger,
                        afcDeltaTime=Mock()), patch.object(AFC, "print_version"):
            obj = AFC(config)
        assert obj.moonraker is None
        assert obj.afc_stats is None
        assert obj.default_material_temps == (None if temps is None else list(temps))
        assert obj._in_klippy_test_env_ is True
        assert obj.auto_spool_switch is automatic
        assert io.logger.messages == []

    def test_automatic_switch_still_requires_tracking(self, moonraker_io):
        io = moonraker_io
        config = Config(Printer(io.reactor), auto_spool_switch=True, track_filament_weight=False)
        with patch.dict(AFC.__init__.__globals__, AFC_logger=lambda *args: io.logger):
            with pytest.raises(ValueError, match="auto_spool_switch requires track_filament_weight"):
                AFC(config)
        assert io.records == []
        assert io.logger.messages == []


class TestAFCHandleDisconnect:
    def test_disconnect_without_client_is_harmless(self, afc_connection):
        obj, io = afc_connection
        obj.printer.send_event("klippy:disconnect")
        assert obj.moonraker is None
        assert io.records == []
        assert io.logger.messages == []

    def test_registered_disconnect_drains_client_and_can_repeat(self, afc_connection):
        obj, io = afc_connection
        obj.handle_moonraker_connect()
        io.wait_idle()
        io.records.clear()
        io.contents.clear()
        obj.moonraker.update_afc_stats("disconnect", 6)
        obj.printer.send_event("klippy:disconnect")
        obj.printer.send_event("klippy:disconnect")
        assert io.records == [("POST", "afc_stats", "disconnect", "6")]
        assert io.contents == {("afc_stats", "disconnect"): "6"}
        assert not obj.moonraker._write_thread.is_alive()
        assert io.logger.messages == []


class TestAFCHandleMoonrakerConnect:
    @pytest.mark.parametrize("multiple_tools", [False, True])
    @pytest.mark.parametrize("td1_detection", [False, True])
    def test_repeated_prep_retains_statistics_and_unsent_runtime(
            self, afc_connection, multiple_tools, td1_detection):
        obj, io = afc_connection
        obj.enable_td1_detection = td1_detection
        obj.tools = {"extruder": object(), "extruder1": object()} if multiple_tools else {}
        lane = make_lane(obj)
        lane.td1_when_loaded = True
        extruder = make_extruder_stats(obj)
        assert obj.handle_moonraker_connect() is None
        client, stats, metadata = obj.moonraker, obj.afc_stats, obj.print_data_metadata
        for action in ("increase_count", "increase_count", "reset_count", "increase_count"):
            getattr(stats.tc_without_error, action)()
        lane_count, cut_count = lane.lane_load_count, extruder.cut_total
        lane_count.increase_count()
        cut_count.increase_count()
        cut_count.reset_count()
        runtime = lane.espooler.stats
        forward, reverse = runtime._n20_runtime_fwd, runtime._n20_runtime_rwd
        runtime.direction = assist.EspoolerDir.FWD
        runtime.start_time = 10.
        runtime.end_time = 12.5
        runtime.direction = assist.EspoolerDir.RWD
        runtime.start_time = 20.
        runtime.end_time = 24.
        reads = io.startup_reads
        assert obj.handle_moonraker_connect() is None
        assert obj.moonraker is client
        assert obj.afc_stats is stats
        assert stats.tc_without_error.value == 1
        assert stats.multiple_tools is multiple_tools
        assert obj.print_data_metadata is not metadata
        assert lane.lane_load_count is lane_count
        assert lane_count.value == 1
        assert extruder.cut_total is cut_count
        assert cut_count.value == 0
        assert runtime._n20_runtime_fwd is forward
        assert runtime._n20_runtime_rwd is reverse
        assert (forward.value, reverse.value) == (2.5, 4.)
        assert runtime._fwd_updated is True
        assert runtime._rwd_updated is True
        assert io.startup_reads == reads
        assert obj.printer.events == ["afc:moonraker_connect"] * 2
        assert io.client.wait_for_moonraker.call_count == 2
        assert io.client.delete_lane_data.call_count == 2
        assert io.client.get_spoolman_server.call_count == 2
        assert io.client.check_for_td1.call_count == (2 if td1_detection else 0)
        assert obj.td1_defined is td1_detection
        assert obj._td1_present is td1_detection
        assert obj.lane_data_enabled is td1_detection
        io.wait_idle()
        assert io.contents[("afc_stats", "error_stats.changes_without_error")] == "1"
        assert io.contents[("afc_stats", "lane1.load_count")] == "1"
        assert io.contents[("afc_stats", "extruder.cut.cut_total")] == "0"
        assert ("afc_stats", "lane1.n20_runtime_fwd") not in io.contents
        assert ("afc_stats", "lane1.n20_runtime_rwd") not in io.contents
        assert io.logger.messages == (
            [("debug", "Multiple tools detected for stats")] if multiple_tools else [])

    def test_readiness_failure_retries_same_worker(self, afc_connection):
        obj, io = afc_connection
        io.client.wait_for_moonraker.side_effect = [False, True]
        assert obj.handle_moonraker_connect() is False
        first = obj.moonraker
        assert obj.afc_stats is None
        assert obj.printer.events == []
        assert obj.handle_moonraker_connect() is None
        assert obj.moonraker is first
        assert io.workers == [first]
        assert obj.afc_stats.moonraker is first
        assert obj.printer.events == ["afc:moonraker_connect"]
        assert first._write_thread.is_alive()
        assert io.logger.messages == []

    def test_statistics_initialization_error_retries_same_client(self, afc_connection):
        obj, io = afc_connection
        attempts = []

        def initialize(*args):
            attempts.append(args[0])
            if len(attempts) == 1:
                raise ValueError("controlled statistics initialization failure")
            return AFCStats(*args)

        with patch.dict(AFC.__init__.__globals__, AFCStats=initialize), \
                patch.object(traceback, "format_exc", return_value="controlled traceback"):
            assert obj.handle_moonraker_connect() is None
            assert obj.afc_stats is None
            assert obj.spoolman is None
            assert obj.printer.events == []
            assert obj.handle_moonraker_connect() is None
        assert attempts == [io.client, io.client]
        assert obj.afc_stats.moonraker is io.client
        assert obj.printer.events == ["afc:moonraker_connect"]
        assert io.logger.messages == [(
            "debug", "Moonraker/Spoolman/afc_stats/td1 error\n"
            "Error: controlled statistics initialization failure\ncontrolled traceback")]

    @pytest.mark.parametrize("registered", [False, True])
    def test_spoolman_setup_repeats_on_each_prep(self, afc_connection, registered):
        obj, io = afc_connection
        io.client.get_spoolman_server.return_value = "http://spoolman.invalid"
        obj.spool.SPOOLMAN_REMOTE_METHOD = "spoolman_set_active_spool"
        obj.webhooks._remote_methods = (
            {"spoolman_set_active_spool": object()} if registered else {})
        obj.handle_moonraker_connect()
        stats = obj.afc_stats
        obj.handle_moonraker_connect()
        assert obj.afc_stats is stats
        assert io.client.get_spoolman_server.call_count == 2
        assert obj.spoolman == "http://spoolman.invalid"
        if registered:
            assert obj.toolhead.dwell.call_count == 0
            assert io.logger.messages == [
                ("debug", "spoolman_set_active_spool registered after 0s")] * 2
        else:
            assert obj.toolhead.dwell.call_args_list == [call(1)] * 60
            assert io.logger.messages == []

    def test_event_owner_failure_retries_without_reloading_initialized_owners(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        extruder = make_extruder_stats(obj)
        attempts = []

        def initialize(*args):
            attempts.append(args[1])
            if len(attempts) == 2:
                raise ValueError("controlled reverse initialization failure")
            return AFCStats_var(*args)

        with patch.dict(AFCEspoolerStats.__init__.__globals__, AFCStats_var=initialize), \
                patch.object(traceback, "format_exc", return_value="controlled traceback"):
            obj.handle_moonraker_connect()
            client, stats, counter = obj.moonraker, obj.afc_stats, lane.lane_load_count
            assert counter is not None
            assert not hasattr(extruder, "cut_total")
            counter.increase_count()
            obj.handle_moonraker_connect()
        assert obj.moonraker is client
        assert obj.afc_stats is stats
        assert lane.lane_load_count is counter
        assert counter.value == 1
        assert lane.espooler.stats._n20_runtime_rwd.value == 0
        assert extruder.cut_total.value == 0
        assert attempts == ["n20_runtime_fwd", "n20_runtime_rwd"] * 2
        assert io.startup_reads == 5
        assert obj.printer.events == ["afc:moonraker_connect"] * 2
        io.wait_idle()
        assert io.contents[("afc_stats", "lane1.load_count")] == "1"
        assert io.logger.messages == [(
            "debug", "Moonraker/Spoolman/afc_stats/td1 error\n"
            "Error: controlled reverse initialization failure\ncontrolled traceback")]


class TestAFCExtruderStatsInit:
    def test_constructor_is_lazy_and_preserves_warning_configuration(self, afc_connection):
        obj, io = afc_connection
        stats = make_extruder_stats(obj)
        assert stats.obj.afc is obj
        assert stats.cut_threshold_for_warning == 200
        assert stats.threshold_warning_sent is False
        assert stats.threshold_error_sent is False
        assert io.startup_reads == 0
        assert io.records == []
        assert io.logger.messages == []


class TestAFCExtruderStatsHandleMoonrakerStats:
    @pytest.mark.parametrize(("name", "count_key"), [
        ("extruder", "extruder.change_count.total"),
        ("extruder1", "extruder1.cut.total"),
    ])
    def test_repeat_retains_counts_and_resets(self, afc_connection, name, count_key):
        obj, io = afc_connection
        stats = make_extruder_stats(obj, name)
        obj.handle_moonraker_connect()
        reads = io.startup_reads
        cut, count = stats.cut_total, stats.tc_total
        cut.increase_count()
        cut.reset_count()
        count.increase_count()
        stats.handle_moonraker_stats()
        assert stats.cut_total is cut
        assert stats.tc_total is count
        assert (cut.value, count.value) == (0, 1)
        assert io.startup_reads == reads
        io.wait_idle()
        assert io.contents[("afc_stats", f"{name}.cut.cut_total")] == "0"
        assert io.contents[("afc_stats", count_key)] == "1"
        assert io.logger.messages == []

    def test_partial_initialization_retries(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        stats = make_extruder_stats(obj)
        attempts = []

        def initialize(*args, **kwargs):
            attempts.append(args[1])
            if len(attempts) == 2:
                raise ValueError("controlled partial initialization")
            return AFCStats_var(*args, **kwargs)

        with patch.dict(AFCExtruderStats.__init__.__globals__, AFCStats_var=initialize):
            with pytest.raises(ValueError, match="controlled partial initialization"):
                stats.handle_moonraker_stats()
            stats.handle_moonraker_stats()
            total = stats.cut_total
            total.increase_count()
            stats.handle_moonraker_stats()
        assert stats.cut_total is total
        assert total.value == 1
        assert io.startup_reads == 2
        io.wait_idle()
        assert io.contents == {
            ("afc_stats", "extruder.cut.last_blade_changed"): "N/A",
            ("afc_stats", "extruder.cut.cut_total"): "1",
        }
        assert io.logger.messages == []

    @pytest.mark.parametrize("name", ["extruder", "extruder1"])
    def test_saved_blade_date_is_not_replaced(self, afc_connection, name):
        obj, io = afc_connection
        io.startup_values = {name: {
            "cut": {"cut_total": 3, "cut_total_since_changed": 2,
                    "last_blade_changed": "2026-10-01 12:30", "total": 5,
                    "tool_unload": 2, "tool_load": 3},
            "change_count": {"total": 5, "tool_unload": 2, "tool_load": 3,
                             "tool_selected": 4, "tool_unselected": 4}}}
        obj.moonraker = io.client
        stats = make_extruder_stats(obj, name)
        stats.handle_moonraker_stats()
        blade = stats.last_blade_changed
        stats.handle_moonraker_stats()
        assert stats.last_blade_changed is blade
        assert blade.value == "2026-10-01 12:30"
        assert stats.cut_total.value == 3
        assert io.startup_reads == 1
        assert io.records == []
        assert io.logger.messages == []


class TestAFCEspoolerStatsInit:
    def test_constructor_is_lazy_then_tracks_runtime_for_owner(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        io.startup_values = {"lane1": {"n20_runtime_fwd": 5, "n20_runtime_rwd": 7}}
        stats = make_lane(obj).espooler.stats
        assert io.startup_reads == 0
        assert io.records == []
        assert stats._fwd_updated is False
        assert stats._rwd_updated is False
        stats.handle_moonraker_stats()
        stats.direction = assist.EspoolerDir.FWD
        stats.start_time = 10.
        stats.end_time = 12.
        stats.update_database()
        io.wait_idle()
        assert stats._n20_runtime_fwd.value == 7.
        assert stats._n20_runtime_rwd.value == 7
        assert stats._fwd_updated is False
        assert stats._rwd_updated is False
        assert io.startup_reads == 1
        assert io.records == [("POST", "afc_stats", "lane1.n20_runtime_fwd", "7.0")]
        assert io.contents == {("afc_stats", "lane1.n20_runtime_fwd"): "7.0"}
        assert io.logger.messages == []


class TestAFCEspoolerStatsHandleMoonrakerStats:
    def test_partial_initialization_retries(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        stats = make_lane(obj).espooler.stats
        attempts = []

        def initialize(*args):
            attempts.append(args[1])
            if len(attempts) == 2:
                raise ValueError("controlled reverse initialization failure")
            return AFCStats_var(*args)

        with patch.dict(AFCEspoolerStats.__init__.__globals__, AFCStats_var=initialize):
            with pytest.raises(ValueError, match="controlled reverse initialization failure"):
                stats.handle_moonraker_stats()
            stats.handle_moonraker_stats()
            forward = stats._n20_runtime_fwd
            forward.increase_count()
            stats.handle_moonraker_stats()
        assert stats._n20_runtime_fwd is forward
        assert forward.value == 1
        assert stats._n20_runtime_rwd.value == 0
        assert io.startup_reads == 2
        io.wait_idle()
        assert io.contents == {("afc_stats", "lane1.n20_runtime_fwd"): "1"}
        assert io.logger.messages == []

    def test_repeat_preserves_unsent_runtime_until_explicit_update(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        stats = make_lane(obj).espooler.stats
        stats.handle_moonraker_stats()
        forward, reverse = stats._n20_runtime_fwd, stats._n20_runtime_rwd
        for direction, start, end in [
                (assist.EspoolerDir.FWD, 10., 12.5),
                (assist.EspoolerDir.RWD, 20., 24.)]:
            stats.direction = direction
            stats.start_time = start
            stats.end_time = end
        stats.handle_moonraker_stats()
        assert stats._n20_runtime_fwd is forward
        assert stats._n20_runtime_rwd is reverse
        assert (forward.value, reverse.value) == (2.5, 4.)
        assert stats._fwd_updated is True
        assert stats._rwd_updated is True
        assert io.records == []
        assert io.startup_reads == 1
        stats.update_database()
        io.wait_idle()
        assert io.contents == {
            ("afc_stats", "lane1.n20_runtime_fwd"): "2.5",
            ("afc_stats", "lane1.n20_runtime_rwd"): "4.0"}
        assert stats._fwd_updated is False
        assert stats._rwd_updated is False
        assert io.logger.messages == []


class TestAFCLaneHandleMoonrakerConnect:
    @pytest.mark.parametrize(("unit_type", "fullname", "admitted"), [
        ("BoxTurtle", "AFC_stepper lane1", True),
        ("HTLF", "AFC_stepper lane1", False),
        ("HTLF", "AFC_lane lane1", True),
    ])
    @pytest.mark.parametrize("initial_td1", [False, True])
    def test_repeat_retains_count_and_repeats_other_setup(
            self, afc_connection, unit_type, fullname, admitted, initial_td1):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.td1_defined = True
        lane = make_lane(obj)
        lane.fullname = fullname
        lane.unit_obj.type = unit_type
        lane.td1_when_loaded = initial_td1
        with patch.object(lane.espooler, "handle_moonraker_connect",
                          wraps=lane.espooler.handle_moonraker_connect) as connect:
            lane.handle_moonraker_connect()
            if admitted:
                counter = lane.lane_load_count
                counter.increase_count()
                counter.increase_count()
                counter.reset_count()
                counter.increase_count()
            obj.td1_defined = False
            lane.handle_moonraker_connect()
        if admitted:
            assert lane.lane_load_count is counter
            assert counter.value == 1
            assert connect.call_count == 2
            assert lane.td1_when_loaded is False
            assert io.startup_reads == 2
            io.wait_idle()
            assert io.contents == {("afc_stats", "lane1.load_count"): "1"}
        else:
            assert lane.lane_load_count is None
            assert connect.call_count == 0
            assert lane.td1_when_loaded is initial_td1
            assert io.startup_reads == 0
            assert io.records == []
        assert io.logger.messages == []

    def test_failed_lane_stat_read_retries_without_replacing_initialized_count(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        lane = make_lane(obj)
        attempts = []

        def startup():
            attempts.append("read")
            if len(attempts) == 1:
                raise ValueError("controlled startup read failure")
            return None

        io.client.get_afc_stats = startup
        with pytest.raises(ValueError, match="controlled startup read failure"):
            lane.handle_moonraker_connect()
        assert lane.lane_load_count is None
        lane.handle_moonraker_connect()
        counter = lane.lane_load_count
        counter.increase_count()
        lane.handle_moonraker_connect()
        assert lane.lane_load_count is counter
        assert counter.value == 1
        assert attempts == ["read"] * 3
        io.wait_idle()
        assert io.contents == {("afc_stats", "lane1.load_count"): "1"}
        assert io.logger.messages == []


class TestCreator5InventoryOrdering:
    def test_connection_event_cannot_publish_before_lane_cleanup_finishes(
            self, afc_connection, tmp_path):
        obj, io = afc_connection
        sync_types = load_classes('creator5_filament_sync.py',
                                  ['empty_filament', 'Creator5FilamentSync'])
        config = Config(obj.printer, filename=str(tmp_path / "inventory.json"),
                        moonraker_url=io.client.host)
        publisher = sync_types.Creator5FilamentSync(config)
        io.client.delete_lane_data = types.MethodType(AFC_moonraker.delete_lane_data, io.client)
        io.responses[0] = {"value": {"stale": "old"}}
        io.contents[("lane_data", "stale")] = "old"
        gate = io.gates[1] = threading.Event()
        completed = threading.Event()
        errors = []

        def prep():
            try:
                obj.handle_moonraker_connect()
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()

        helper = threading.Thread(target=prep)
        helper.start()
        try:
            io.wait_requests(2)
            assert io.records == [
                ("GET", "", "", None), ("DELETE", "lane_data", "stale", None)]
            assert obj.printer.events == []
            assert not completed.is_set()
            assert io.contents == {("lane_data", "stale"): "old"}
            assert io.escaped == []
        finally:
            gate.set()
            helper.join(WAIT)
            publisher.executor.shutdown(wait=True)
        assert not helper.is_alive()
        assert errors == []
        assert completed.is_set()
        io.wait_idle()
        assert obj.printer.events == ["afc:moonraker_connect"]
        inventory_posts = [record for record in io.records if record[1] == "lane_data"]
        assert inventory_posts == [("DELETE", "lane_data", "stale", None)] + [
            ("POST", "lane_data", f"c5_tool{index}", {
                "material": "", "color": "", "nozzle_temp": 0, "bed_temp": 0,
                "spool_id": None, "lane": str(index)}) for index in range(4)]
        assert ("lane_data", "stale") not in io.contents
        assert {key for namespace, key in io.contents if namespace == "lane_data"} == {
            "c5_tool0", "c5_tool1", "c5_tool2", "c5_tool3"}
        io.reactor.run_callbacks()
        assert io.logger.messages == [("debug", "Removing stale from lane_data")]


class TestUnchangedSynchronousMoonrakerApis:
    @pytest.mark.parametrize(("operation", "response", "expected", "path"), [
        (lambda client: client.get_file_metadata("folder/file.gcode"),
         {"size": 15}, {"size": 15},
         "/server/files/metadata?filename=folder/file.gcode"),
        (lambda client: client.get_spool(7),
         {"id": 7}, {"id": 7}, "/server/spoolman/proxy"),
        (lambda client: client.get_td1_data(),
         {"devices": {"serial": {"color": "red"}}},
         {"serial": {"color": "red"}}, "/machine/td1/data"),
        (lambda client: client.send_lane_data({
            "namespace": "lane_data", "key": "lane1", "value": "loaded"}),
         {"value": "loaded"}, None, "/server/database/item"),
    ])
    def test_public_api_still_waits_for_response_and_returns_result(
            self, moonraker_io, operation, response, expected, path):
        io = moonraker_io
        io.client.join_thread()
        io.responses[0] = response
        gate = io.gates[0] = threading.Event()
        results, errors = [], []
        completed = threading.Event()

        def call_api():
            try:
                results.append(operation(io.client))
            except BaseException as exc:
                errors.append(exc)
            finally:
                completed.set()

        helper = threading.Thread(target=call_api)
        helper.start()
        try:
            io.wait_requests(1)
            assert not completed.is_set()
            assert io.escaped == []
        finally:
            gate.set()
            helper.join(WAIT)
        assert not helper.is_alive()
        assert errors == []
        assert completed.is_set()
        assert results == [expected]
        assert io.requests[0][0] == path
        assert io.logger.messages == []
