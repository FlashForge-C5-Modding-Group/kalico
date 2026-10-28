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


class EventCompletion:
    """Bounded unit bridge only; real scheduling is covered by reactor smoke."""
    def __init__(self):
        self.event = threading.Event()
        self.result = None
        self.wait_started = threading.Event()

    def complete(self, result):
        assert not self.event.is_set(), "completion delivered twice"
        self.result = result
        self.event.set()

    def test(self):
        return self.event.is_set()

    def wait(self):
        self.wait_started.set()
        assert self.event.wait(WAIT), "completion did not settle"
        return self.result


class DeferredReactor(MockReactor):
    def __init__(self):
        super().__init__()
        self.callbacks = Queue()

    def register_async_callback(self, callback, waketime=None):
        self.callbacks.put(callback)
    def completion(self):
        return EventCompletion()

    def async_complete(self, completion, result):
        completion.complete(result)


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
        self.raw_responses = {}
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
                    data = json.loads(body) if body else {}
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
                payload = owner.raw_responses.get(
                    index, json.dumps({"result": result}).encode())
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

    def test_nonstatistics_delete_rejects_closed_worker(self, moonraker_io):
        io = moonraker_io
        io.client.join_thread()
        with pytest.raises(RuntimeError, match="AFC Moonraker worker is closed"):
            io.client.remove_database_entry("lane_data", "stale")
        assert io.records == []


assist = load_classes('AFC_assist.py', [
    'EspoolerDir', 'AFCEspoolerStats', 'Espooler_values', 'Espooler'])
AFCEspoolerStats = assist.AFCEspoolerStats
AFCExtruderStats = load_classes('AFC_extruder.py', ['AFCExtruderStats']).AFCExtruderStats
lane_types = load_classes('AFC_lane.py', [
    'AssistActive', 'SpeedMode', 'MoveDirection', 'AFCLaneState',
    'AFCHomingPoints', 'AFCMoveWarning', 'AFCLane'],
    {'AFC_assist': assist, 'EXCLUDE_TYPES': ['HTLF', 'Claymore', 'OpenAMS', 'ViViD'],
     'VALID_DIRECT_HUB': ['direct', 'direct_load']})
AFCLane = lane_types.AFCLane
AFC_PrintFileMetaData = load_classes('AFC_utils.py',
                                   ['AFC_PrintFileMetaData']).AFC_PrintFileMetaData
owner_types = load_classes('AFC.py', ['State', 'afc'],
                           {**vars(lane_types), 'AFC_VERSION': 'test'})
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

    getint = getfloat = getboolean = getlist = getlists = get

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


class TestCooperativeMoonrakerResults:
    @pytest.mark.parametrize(("operation", "response", "expected", "path"), [
        (lambda client: client.get_spool(7),
         {"id": 7}, {"id": 7}, "/server/spoolman/proxy"),
        (lambda client: client.get_td1_data(),
         {"devices": {"serial": {"color": "red"}}},
         {"serial": {"color": "red"}}, "/machine/td1/data"),
    ])
    def test_public_api_still_waits_for_response_and_returns_result(
            self, moonraker_io, operation, response, expected, path):
        io = moonraker_io
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


class TestNetworkWorkerContracts:
    def test_result_read_cannot_overtake_admitted_statistics(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.client.update_afc_stats("first", 1)
        io.wait_requests(1)
        results = []
        helper = threading.Thread(target=lambda: results.append(io.client.get_spool(7)))
        helper.start()
        try:
            # A queue barrier cannot pass the held first request.
            assert not results
            assert io.records == [("POST", "afc_stats", "first", "1")]
        finally:
            gate.set()
            helper.join(WAIT)
        assert not helper.is_alive()
        assert len(results) == 1
        assert [record[0] for record in io.records] == ["POST", "POST"]
        assert [request[0] for request in io.requests] == [
            "/server/database/item", "/server/spoolman/proxy"]

    @pytest.mark.parametrize("body", [b"not json", b"{}", b"null", b"[]"])
    def test_bad_metadata_envelope_delivers_failure_then_next_callback(
            self, moonraker_io, body):
        io = moonraker_io
        io.raw_responses[0] = body
        io.responses[1] = {"size": 12}
        received = []
        owner = threading.get_ident()
        io.client.get_file_metadata("bad.gcode", lambda result: received.append(
            (result, threading.get_ident())))
        io.client.get_file_metadata("good.gcode", lambda result: received.append(
            (result, threading.get_ident())))
        io.wait_idle()
        assert received == []
        io.reactor.run_callbacks()
        assert received == [(None, owner), ({"size": 12}, owner)]
        assert io.escaped == []

    def test_lane_publication_snapshots_before_waiting_for_http(self, moonraker_io):
        io = moonraker_io
        gate = io.gates[0] = threading.Event()
        io.client.update_afc_stats("hold", 1)
        io.wait_requests(1)
        payload = {"namespace": "lane_data", "key": "lane1",
                   "value": {"material": "PLA"}}
        io.client.send_lane_data(payload)
        payload["value"]["material"] = "PETG"
        try:
            assert io.records == [("POST", "afc_stats", "hold", "1")]
            assert io.escaped == []
        finally:
            gate.set()
        io.wait_idle()
        assert io.requests[1][2] == {
            "namespace": "lane_data", "key": "lane1",
            "value": {"material": "PLA"}}

    def test_closed_client_drops_admitted_metadata_callback(self, moonraker_io):
        io = moonraker_io
        received = []
        io.client.get_file_metadata("old.gcode", received.append)
        io.wait_idle()
        io.client.join_thread()
        io.reactor.run_callbacks()
        assert received == []


class TestMetadataRequestGenerations:
    @pytest.mark.parametrize("restart_filename", ["same.gcode", "new.gcode"])
    @pytest.mark.parametrize("reset_before_restart", [False, True])
    def test_restart_rejects_old_reply_and_obsolete_continuation(
            self, restart_filename, reset_before_restart):
        callbacks = []
        client = types.SimpleNamespace(get_file_metadata=lambda filename, callback:
                                       callbacks.append(callback))
        metadata = AFC_PrintFileMetaData(client, ThreadLogger())
        finished = []
        metadata.query_filename("same.gcode", lambda: finished.append("old"))
        if reset_before_restart:
            metadata.reset()
        metadata.query_filename(restart_filename, lambda: finished.append("new"))
        callbacks[0]({"filament_change_count": 100, "filament_temps": [280]})
        assert metadata.tool_change_count == 0
        assert metadata.tool_temperatures == []
        assert finished == []
        callbacks[1]({"filament_change_count": 3, "filament_temps": [210]})
        assert metadata.tool_change_count == 3
        assert metadata.tool_temperatures == [210]
        assert finished == ["new"]

    @pytest.mark.parametrize("client, filename", [(None, "file.gcode"), (None, "")])
    def test_no_request_still_finishes_once(self, client, filename):
        metadata = AFC_PrintFileMetaData(client, ThreadLogger())
        finished = []
        metadata.query_filename(filename, lambda: finished.append(True))
        assert finished == [True]
        assert metadata.tool_change_count == 0


class TestDeferredTd1Presence:
    def test_status_reads_share_refresh_without_fetching_inline(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        assert obj.td1_present is False
        assert obj.td1_present is False
        io.client.check_for_td1.assert_not_called()
        io.reactor.run_callbacks()
        assert obj.td1_present is True
        assert io.client.check_for_td1.call_count == 1

    def test_replaced_client_cannot_apply_pending_presence(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        assert obj.td1_present is False
        obj.moonraker = io.new_client()
        io.reactor.run_callbacks()
        assert obj._td1_present is False
        io.client.check_for_td1.assert_not_called()


AFCSpool = load_classes('AFC_spool.py', ['AFCSpool']).AFCSpool


class TestSpoolAssignmentGenerations:
    def setup_spool(self, obj, lane):
        spool = AFCSpool.__new__(AFCSpool)
        spool.afc = obj
        spool.logger = obj.logger
        spool.disable_weight_check = False
        spool.creator5_tool_lanes = {}
        obj.spoolman = "configured"
        obj.lanes[lane.name] = lane
        lane.spool_id = 4
        lane._material = "PETG"
        lane.filament_density = 1.27
        lane.send_lane_data = Mock()
        lane.clear_lane_data = Mock()
        spool.set_snapmaker_filament_params = Mock()
        return spool

    @pytest.mark.parametrize("material,densities", [
        ({"not": "text"}, ["PLA:1.24"]), ("PLA", ["PLA:broken"])])
    def test_malformed_material_preserves_entire_lane(
            self, afc_connection, material, densities):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.common_density_values = densities
        obj.moonraker = io.client
        io.responses[0] = {
            "filament": {"material": material, "density": 1.24,
                         "color_hex": "FFFFFF"}, "remaining_weight": 100}
        assert spool.set_spoolID(lane, "8") is False
        assert (lane.spool_id, lane.material, lane.filament_density) == (
            4, "PETG", 1.27)
        lane.send_lane_data.assert_not_called()
        spool.set_snapmaker_filament_params.assert_not_called()

    def test_clear_during_lookup_wins_without_old_publication(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)

        def lookup(spool_id):
            spool.clear_values(lane)
            return {"filament": {"material": "PLA", "color_hex": "FFFFFF"},
                    "remaining_weight": 100}

        obj.moonraker = io.client
        io.client.get_spool = lookup
        assert spool.set_spoolID(lane, "8") is False
        assert (lane.spool_id, lane.material, lane.weight) == (None, "", 0)
        lane.send_lane_data.assert_not_called()
        spool.set_snapmaker_filament_params.assert_not_called()

    def test_duplicate_appearing_during_lookup_is_rejected(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        other = types.SimpleNamespace(spool_id=None)
        obj.lanes["other"] = other

        def lookup(spool_id):
            other.spool_id = 8
            return {"filament": {"material": "PLA", "color_hex": "FFFFFF"},
                    "remaining_weight": 100}

        obj.moonraker = io.client
        io.client.get_spool = lookup
        assert spool.set_spoolID(lane, 8) is False
        assert lane.spool_id == 4
        lane.send_lane_data.assert_not_called()

    def test_material_descriptor_validates_before_mutation(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        lane._material = "PETG"
        lane.filament_density = 1.27
        with pytest.raises(ValueError, match="Filament material must be text or None"):
            lane.material = {"truthy": True}
        assert (lane.material, lane.filament_density) == ("PETG", 1.27)


class TestPrintStartMetadataOwnership:
    def test_delayed_reply_preserves_progress_and_fault_reference(self, afc_connection):
        obj, io = afc_connection
        callbacks = []
        client = types.SimpleNamespace(get_file_metadata=lambda filename, callback:
                                       callbacks.append(callback))
        obj.moonraker = io.client
        obj.print_data_metadata = AFC_PrintFileMetaData(client, obj.logger)
        obj.function.in_print.return_value = (True, "file.gcode")
        buffer = Mock()
        obj.function.get_current_lane_obj.return_value = types.SimpleNamespace(
            buffer_obj=buffer)
        io.reactor.unregister_timer = Mock()
        obj.in_print_timer = object()
        obj.in_print_reactor_timer(0.)
        assert obj.in_print_timer is None
        buffer.update_filament_error_pos.assert_called_once()
        obj.current_toolchange = 2
        callbacks[0]({"filament_change_count": 7, "filament_temps": [220]})
        assert obj.current_toolchange == 2
        assert obj.number_of_toolchanges == 7
        assert obj.print_tool_temperatures == [220]
        buffer.update_filament_error_pos.assert_called_once()

    def test_replaced_metadata_owner_cannot_finish_current_print(self, afc_connection):
        obj, io = afc_connection
        old = AFC_PrintFileMetaData(None, obj.logger)
        obj.print_data_metadata = AFC_PrintFileMetaData(None, obj.logger)
        obj.function.in_print.return_value = (True, "same.gcode")
        obj.number_of_toolchanges = 12
        obj.current_toolchange = 4
        obj._finish_print_start(old, "same.gcode")
        assert obj.number_of_toolchanges == 12
        assert obj.current_toolchange == 4


class TestCooperativeBackupReset:
    @pytest.mark.parametrize("status", [200, 500])
    def test_reset_waits_for_backup_and_preserves_failure_policy(
            self, afc_connection, status):
        obj, io = afc_connection
        obj.moonraker = io.client
        gate = io.gates[0] = threading.Event()
        io.statuses[0] = status
        io.responses[0] = {"backup_path": "/backup/test.db"}
        counter = types.SimpleNamespace(reset_count=Mock())
        obj.lanes["lane1"] = types.SimpleNamespace(
            name="lane1", lane_load_count=counter)
        completed = threading.Event()
        command = types.SimpleNamespace(get=lambda key, default=None:
                                        "lane1" if key == "LANE" else default)
        helper = threading.Thread(target=lambda: (
            obj.cmd_AFC_RESET_STATS(command), completed.set()))
        helper.start()
        try:
            io.wait_requests(1)
            assert not completed.is_set()
            counter.reset_count.assert_not_called()
        finally:
            gate.set()
            helper.join(WAIT)
        assert not helper.is_alive()
        assert completed.is_set()
        assert counter.reset_count.call_count == (1 if status == 200 else 0)
        assert obj.db_backup is (status == 200)


class TestSpoolManualSupersession:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    @pytest.mark.parametrize("method, values, field, expected", [
        ("cmd_SET_COLOR", {"COLOR": "123456"}, "color", "#123456"),
        ("cmd_SET_WEIGHT", {"WEIGHT": 321.}, "weight", 321.),
        ("cmd_SET_MATERIAL", {"MATERIAL": "ABS"}, "material", "ABS"),
        ("cmd_AFC_SET_SPOOL_TEMP", {"BED_TEMP": 80, "EXTRUDER_TEMP": 240},
         "extruder_temp", 240),
    ])
    def test_manual_change_rejects_inflight_reply(
            self, afc_connection, method, values, field, expected):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.common_density_values = ["ABS:1.04"]
        command = types.SimpleNamespace(
            get=lambda key, default=None: {"LANE": lane.name, **values}.get(key, default),
            get_int=lambda key, default=None, **kwargs: values.get(key, default),
            get_float=lambda key, default=None: values.get(key, default))

        def lookup(spool_id):
            getattr(spool, method)(command)
            return {"filament": {"material": "PLA", "color_hex": "FFFFFF",
                                 "settings_extruder_temp": 200},
                    "remaining_weight": 100}

        obj.moonraker = io.client
        io.client.get_spool = lookup
        assert spool.set_spoolID(lane, "8") is False
        assert getattr(lane, field) == expected
        assert lane.spool_id == 4


class TestWorkerResultBridge:
    @pytest.mark.parametrize("body, expected", [
        (b'{"result": false}', False), (b'{"result": []}', []),
        (b'{"result": null}', None), (b'{}', None),
        (b'not json', None),
    ])
    def test_decode_and_close_run_only_on_worker(self, moonraker_io, body, expected):
        io = moonraker_io
        owner = threading.get_ident()
        threads = []

        class Response(stream_io.BytesIO):
            status = 200

            def read(self, *args):
                threads.append(threading.get_ident())
                return super().read(*args)

            def close(self):
                threads.append(threading.get_ident())
                return super().close()

        response = Response(body)
        with patch.dict(AFC_moonraker._get_results.__globals__,
                        urlopen=lambda *args, **kwargs: response):
            assert io.client._get_results(io.client.database_url) == expected
        assert response.closed
        assert threads and owner not in threads
        assert set(threads) == {io.client._write_thread.ident}

    def test_worker_exception_returns_to_waiter_and_fifo_keeps_running(self, moonraker_io):
        io = moonraker_io

        def fail():
            raise ValueError("controlled worker failure")

        with pytest.raises(ValueError, match="controlled worker failure"):
            io.client._call_worker(fail, ())
        io.client.update_afc_stats("later", 2)
        io.wait_idle()
        assert io.contents == {("afc_stats", "later"): "2"}


class TestTd1RefreshLifecycle:
    @pytest.mark.parametrize("ready, moving, enabled", [
        (False, False, True), (True, True, True), (True, False, False)])
    def test_ineligible_status_never_admits_request(
            self, afc_connection, ready, moving, enabled):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.enable_td1_detection = enabled
        obj.printer.state_message = "Printer is ready" if ready else "Not ready"
        obj.function.is_printing.return_value = moving
        io.reactor.monotonic = lambda: 31.
        assert obj.td1_present is False
        io.reactor.run_callbacks()
        io.client.check_for_td1.assert_not_called()

    def test_operational_presence_waits_for_status_refresh(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        assert obj.td1_present is False
        pending = obj._td1_query_completion
        results = []
        helper = threading.Thread(target=lambda: results.append(obj.check_td1_present()))
        helper.start()
        assert not pending.test()
        assert results == []
        io.reactor.run_callbacks()
        helper.join(WAIT)
        assert not helper.is_alive()
        assert results == [True]
        assert obj._td1_query_completion is None
        assert io.client.check_for_td1.call_count == 1

    def test_exception_retains_cache_and_settles_waiter_once(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj._td1_present = True
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        io.client.check_for_td1.side_effect = ValueError("controlled refresh failure")
        assert obj.td1_present is True
        pending = obj._td1_query_completion
        io.reactor.run_callbacks()
        assert pending.wait() is True
        assert obj._td1_query_completion is None
        assert obj._td1_present is True
        assert io.logger.messages == [("error", "TD-1 presence refresh failed")]
        assert "controlled refresh failure" in io.logger.details[0]["traceback"]


class TestMetadataActiveFailures:
    def test_active_failure_clears_prior_metadata_and_finishes_once(self):
        callbacks = []
        client = types.SimpleNamespace(get_file_metadata=lambda filename, callback:
                                       callbacks.append(callback))
        metadata = AFC_PrintFileMetaData(client, ThreadLogger())
        metadata.query_filename("old.gcode")
        callbacks[0]({"filament_change_count": 5, "filament_temps": [230]})
        finished = []
        metadata.query_filename("next.gcode", lambda: finished.append(True))
        assert metadata.tool_change_count == 0
        assert metadata.tool_temperatures == []
        callbacks[1](None)
        assert metadata.tool_change_count == 0
        assert metadata.tool_temperatures == []
        assert finished == [True]


class TestSpoolAtomicCommit:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    def test_valid_spool_preserves_density_precedence_and_mapping(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.moonraker = io.client
        obj.ignore_spoolman_material_temps = False
        obj.common_density_values = ["PLA:1.24"]
        io.responses[0] = {
            "filament": {"material": "PLA", "density": 1.3, "diameter": 1.75,
                         "name": "Silk", "vendor": {"name": "Vendor"},
                         "multi_color_hexes": "AABBCC,112233",
                         "settings_extruder_temp": 215, "settings_bed_temp": 60},
            "remaining_weight": 420, "initial_weight": 1050, "spool_weight": 195}
        assert spool.set_spoolID(lane, 8) is True
        assert (lane.spool_id, lane.material, lane.filament_density,
                lane.filament_diameter, lane.empty_spool_weight, lane.weight,
                lane.color, lane.multi_color, lane.filament_name, lane.spool_vendor,
                lane.extruder_temp, lane.bed_temp) == (
                    8, "PLA", 1.3, 1.75, 195, 420, "#AABBCC",
                    ["AABBCC", "112233"], "Silk", "Vendor", 215, 60)
        assert lane.espooler.espooler_values.full_weight == 1050
        lane.send_lane_data.assert_called_once()
        spool.set_snapmaker_filament_params.assert_called_once_with(lane)

    @pytest.mark.parametrize("response", [None, {}, {"filament": None},
        {"filament": {"material": "PLA", "vendor": 1}, "remaining_weight": 100},
        {"filament": {"material": "PLA", "multi_color_hexes": 5},
         "remaining_weight": 100}])
    def test_malformed_lookup_cannot_partially_commit(self, afc_connection, response):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.moonraker = io.client
        io.responses[0] = response
        assert spool.set_spoolID(lane, 8) is False
        assert (lane.spool_id, lane.material, lane.filament_density) == (
            4, "PETG", 1.27)
        lane.send_lane_data.assert_not_called()
        spool.set_snapmaker_filament_params.assert_not_called()

    def test_invalid_weight_is_completed_clear_not_supersession(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.moonraker = io.client
        io.responses[0] = {"filament": {"material": "PLA"}, "remaining_weight": 0}
        assert spool.set_spoolID(lane, 8) is True
        assert (lane.spool_id, lane.material, lane.weight) == (None, "", 0)
        lane.clear_lane_data.assert_called_once()
        lane.send_lane_data.assert_not_called()


class TestResultFailureSettlement:
    @pytest.mark.parametrize("failure", ["http", "timeout", "connection"])
    def test_failed_result_settles_and_later_work_succeeds(self, moonraker_io, failure):
        io = moonraker_io
        if failure == "http":
            io.statuses[0] = 500
        if failure == "timeout":
            io.client.REQUEST_TIMEOUT = 0.05
            io.gates[0] = threading.Event()
        if failure == "connection":
            with patch.dict(AFC_moonraker._get_results.__globals__,
                            urlopen=Mock(side_effect=ConnectionRefusedError("refused"))):
                assert io.client._get_results(io.client.database_url) is None
        else:
            try:
                assert io.client._get_results(io.client.database_url) is None
            finally:
                if failure == "timeout":
                    io.gates[0].set()
        io.responses[len(io.records)] = {"answer": 42}
        assert io.client._get_results(io.client.database_url) == {"answer": 42}
        io.reactor.run_callbacks()
        errors = [message for level, message in io.logger.messages if level == "error"]
        assert errors == [io.client.ERROR_STRING]

    def test_closed_result_request_is_rejected_without_http(self, moonraker_io):
        io = moonraker_io
        io.client.join_thread()
        with pytest.raises(RuntimeError, match="AFC Moonraker worker is closed"):
            io.client.get_spool(8)
        assert io.records == []


class TestSpoolCommandOwnership:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    def test_duplicate_rejected_after_lookup_does_not_update_active_spool(
            self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.function.get_current_lane.return_value = lane.name
        other = types.SimpleNamespace(spool_id=None)
        obj.lanes["other"] = other
        spool.set_active_spool = Mock()

        def lookup(spool_id):
            other.spool_id = 8
            return {"filament": {"material": "PLA", "color_hex": "FFFFFF"},
                    "remaining_weight": 100}

        obj.moonraker = io.client
        io.client.get_spool = lookup
        command = types.SimpleNamespace(get=lambda key, default=None:
                                        {"LANE": lane.name, "SPOOL_ID": "8"}.get(
                                            key, default))
        spool.cmd_SET_SPOOL_ID(command)
        assert lane.spool_id == 4
        spool.set_active_spool.assert_not_called()
        lane.send_lane_data.assert_not_called()


class TestStatusPresenceBoundary:
    def test_webhook_and_status_share_immediate_cached_presence(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        request = Mock()
        with patch.object(obj, "get_bypass_state", return_value=False), \
                patch.object(obj, "_get_quiet_mode", return_value=False):
            assert obj.get_status()["td1_present"] is False
        obj._webhooks_status(request)
        assert request.send.call_args.args[0]["status:"]["AFC"]["system"][
            "td1_present"] is False
        io.client.check_for_td1.assert_not_called()
        io.reactor.run_callbacks()
        assert io.client.check_for_td1.call_count == 1


class TestTd1DeliveryRechecks:
    @pytest.mark.parametrize("change", ["closed", "moving", "not_ready", "disabled"])
    def test_pending_refresh_rechecks_runtime_guards(self, afc_connection, change):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.
        assert obj.td1_present is False
        pending = obj._td1_query_completion
        if change == "closed":
            io.client.join_thread()
        elif change == "moving":
            obj.function.is_printing.return_value = True
        elif change == "not_ready":
            obj.printer.state_message = "Not ready"
        else:
            obj.enable_td1_detection = False
        io.reactor.run_callbacks()
        assert pending.wait() is False
        assert obj._td1_query_completion is None
        assert obj._td1_present is False
        io.client.check_for_td1.assert_not_called()


class TestPrintTimerReplacement:
    def test_file_reset_removes_previous_timer_before_registering_new(
            self, afc_connection):
        obj, io = afc_connection
        old, replacement = object(), object()
        events = []
        obj.in_print_timer = old
        io.reactor.unregister_timer = lambda timer: events.append(("unregister", timer))
        io.reactor.register_timer = lambda callback, when: (
            events.append(("register", callback)), replacement)[1]
        obj._reset_file_callback()
        assert events == [
            ("unregister", old), ("register", obj.in_print_reactor_timer)]
        assert obj.in_print_timer is replacement


class TestSpoolClearAndNoop:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    @pytest.mark.parametrize("configured, remember", [
        (True, False), (True, True), (False, False), (False, True)])
    def test_explicit_clear_or_remembered_noop_completes_without_lookup(
            self, afc_connection, configured, remember):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.spoolman = "configured" if configured else None
        obj.moonraker = io.client
        io.client.get_spool = Mock(side_effect=AssertionError("unexpected lookup"))
        lane.remember_spool = remember
        assert spool.set_spoolID(lane, "") is True
        assert lane.spool_id == (4 if remember else None)
        assert lane.clear_lane_data.call_count == (0 if remember else 1)

    def test_unknown_material_retains_density_without_partial_update(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        lane._material = "PETG"
        lane.filament_density = 1.27
        lane.material = "Unlisted"
        assert (lane.material, lane.filament_density) == ("Unlisted", 1.27)


class TestDetachedSpoolOwner:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    def test_replaced_lane_cannot_receive_old_assignment(self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        replacement = types.SimpleNamespace(spool_id=None)

        def lookup(spool_id):
            obj.lanes[lane.name] = replacement
            return {"filament": {"material": "PLA", "color_hex": "FFFFFF"},
                    "remaining_weight": 100}

        obj.moonraker = io.client
        io.client.get_spool = lookup
        assert spool.set_spoolID(lane, 8) is False
        assert lane.spool_id == 4
        assert replacement.spool_id is None
        lane.send_lane_data.assert_not_called()
        spool.set_snapmaker_filament_params.assert_not_called()


class TestSpoolConfiguredPrecedence:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    def test_ignored_temperatures_and_missing_density_keep_existing_precedence(
            self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.moonraker = io.client
        obj.ignore_spoolman_material_temps = True
        spool.disable_weight_check = True
        lane.extruder_temp = 260
        io.responses[0] = {"filament": {"settings_extruder_temp": 190,
                                      "color_hex": "FFFFFF"},
                           "remaining_weight": 0}
        assert spool.set_spoolID(lane, 8, save_vars=False) is True
        assert lane.material is None
        assert lane.filament_density is None
        assert lane.extruder_temp == 260
        assert lane.weight == 0
        assert lane.color == "#FFFFFF"


class TestRemainingPresenceBoundaries:
    def test_disabled_operational_and_scheduler_return_without_request(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.enable_td1_detection = False
        assert obj._schedule_td1_refresh() is None
        assert obj.check_td1_present() is False
        io.client.check_for_td1.assert_not_called()

    @pytest.mark.parametrize("cached", [False, True])
    def test_operational_cache_without_eligible_refresh(self, afc_connection, cached):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        obj._td1_present = cached
        assert obj.check_td1_present() is cached
        io.client.check_for_td1.assert_not_called()

    @pytest.mark.parametrize("close_client", [False, True])
    def test_owner_change_during_query_discards_returned_presence(
            self, afc_connection, close_client):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        io.reactor.monotonic = lambda: 31.

        def query():
            if close_client:
                io.client.join_thread()
            else:
                obj.moonraker = io.new_client()
            return True, True, True

        io.client.check_for_td1.side_effect = query
        assert obj.td1_present is False
        pending = obj._td1_query_completion
        io.reactor.run_callbacks()
        assert pending.wait() is False
        assert obj._td1_present is False

    def test_obsolete_completion_does_not_clear_new_pending_slot(self, afc_connection):
        obj, io = afc_connection
        obj.moonraker = io.new_client()
        newer = io.reactor.completion()
        obj._td1_query_completion = newer
        obsolete = io.reactor.completion()
        # Deliver an abandoned callback against the replaced client.
        obj._refresh_td1_present(0., obsolete, io.client)
        assert obsolete.wait() is False
        assert obj._td1_query_completion is newer
        assert not newer.test()
        io.client.check_for_td1.assert_not_called()


class TestRemainingMetadataBoundaries:
    @pytest.mark.parametrize("printing", [False, True])
    @pytest.mark.parametrize("lane", [None, types.SimpleNamespace(buffer_obj=None)])
    def test_timer_without_metadata_owner_finishes_without_http(
            self, afc_connection, printing, lane):
        obj, io = afc_connection
        obj.moonraker = None
        obj.print_data_metadata = None
        obj.function.in_print.return_value = (printing, "file.gcode")
        obj.function.get_current_lane_obj.return_value = lane
        obj.number_of_toolchanges = 3
        obj.current_toolchange = 2
        io.reactor.unregister_timer = Mock()
        obj.in_print_timer = object()
        assert obj.in_print_reactor_timer(0.) == io.reactor.NEVER
        assert obj.in_print_timer is None
        assert obj.current_toolchange == 2
        assert obj.number_of_toolchanges == (0 if printing else 3)
        assert io.records == []

    def test_current_owner_different_print_does_not_apply_counts(self, afc_connection):
        obj, io = afc_connection
        metadata = AFC_PrintFileMetaData(None, obj.logger)
        obj.print_data_metadata = metadata
        obj.function.in_print.return_value = (True, "other.gcode")
        obj.number_of_toolchanges = 4
        obj._finish_print_start(metadata, "old.gcode")
        assert obj.number_of_toolchanges == 4

    def test_no_client_without_continuation_clears_old_cache(self):
        metadata = AFC_PrintFileMetaData(None, ThreadLogger())
        metadata._metadata = {"filament_change_count": 7}
        metadata.query_filename("file.gcode")
        assert metadata.filename == "file.gcode"
        assert metadata.tool_change_count == 0


class TestLanePublicationFailure:
    def test_http_failure_reports_on_owner_and_does_not_stop_fifo(self, moonraker_io):
        io = moonraker_io
        io.statuses[0] = 500
        io.client.send_lane_data({"namespace": "lane_data", "key": "lane1",
                                  "value": {"material": "PLA"}})
        io.client.update_afc_stats("later", 2)
        io.wait_idle()
        assert io.logger.messages == []
        io.reactor.run_callbacks()
        assert io.logger.messages == [
            ("error", io.client.ERROR_STRING),
            ("error", "Error sending lane data, check AFC.log for more information")]
        assert io.logger.threads == [threading.get_ident()] * 2
        assert io.contents == {("afc_stats", "later"): "2"}

    def test_invalid_json_payload_does_not_admit_partial_request(self, moonraker_io):
        io = moonraker_io
        with pytest.raises(TypeError):
            io.client.send_lane_data({"value": object()})
        io.client.update_afc_stats("later", 2)
        io.wait_idle()
        assert io.records == [("POST", "afc_stats", "later", "2")]


afcFunction = load_classes(
    'AFC_functions.py', ['afcFunction'],
    {'AFCprompt': lambda *args: Mock()}).afcFunction
afcPrep = load_classes('AFC_prep.py', ['afcPrep']).afcPrep


class TestOperationalPresenceConsumers:
    @pytest.mark.parametrize("present", [False, True])
    def test_lane_capture_uses_operational_presence(self, afc_connection, present):
        obj, io = afc_connection
        lane = make_lane(obj)
        obj.moonraker = io.client
        obj._td1_present = present
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        lane.td1_device_id = "serial"
        obj.function.check_for_td1_id.return_value = (True, None)
        lane.unit_obj.get_td1_data = Mock()
        lane.get_td1_data_load()
        assert lane.unit_obj.get_td1_data.call_count == int(present)

    @pytest.mark.parametrize("present", [False, True])
    def test_prep_capture_requires_operational_presence(self, afc_connection, present):
        obj, io = afc_connection
        lane = make_lane(obj)
        obj.lanes[lane.name] = lane
        obj.moonraker = io.client
        obj._td1_present = present
        obj.printer.state_message = "Printer is ready"
        obj.function.is_printing.return_value = False
        obj.function.get_current_lane_obj.return_value = None
        obj.function.check_for_td1_error.return_value = (False, None)
        lane.td1_device_id = "serial"
        lane.prep_state = True
        lane.get_td1_data = Mock(return_value=(True, "captured"))
        prep = afcPrep.__new__(afcPrep)
        prep.afc, prep.logger, prep.get_td1_data = obj, io.logger, True
        prep._td1_prep(True)
        assert lane.get_td1_data.call_count == int(present)
        assert obj.function.check_for_td1_error.call_count == int(present)

    @pytest.mark.parametrize("present", [False, True])
    def test_calibration_accepts_fresh_presence_without_moving_fixture(
            self, afc_connection, present):
        obj, io = afc_connection
        lane = make_lane(obj)
        obj.lanes[lane.name] = lane
        lane.hub_obj = types.SimpleNamespace(state=False)
        lane.unit_obj.calibrate_td1 = Mock(return_value=(True, "", 0.))
        obj.moonraker = io.client
        obj.function.get_current_lane.return_value = None
        obj.function.is_printing.return_value = False
        obj.printer.state_message = "Printer is ready"
        io.reactor.monotonic = lambda: 31.
        io.client.check_for_td1.return_value = (True, present, True)
        function = afcFunction.__new__(afcFunction)
        function.afc, function.logger = obj, io.logger
        function._afc_cali_comp = Mock()
        command = types.SimpleNamespace(
            get=lambda key, default=None: lane.name if key == "TD1" else default,
            get_float=lambda key, default=None: default)
        assert obj.td1_present is False
        pending = obj._td1_query_completion
        with ThreadPoolExecutor(max_workers=1) as executor:
            result = executor.submit(function.cmd_CALIBRATE_AFC, command)
            assert pending.wait_started.wait(WAIT)
            lane.unit_obj.calibrate_td1.assert_not_called()
            io.reactor.run_callbacks()
            result.result(timeout=WAIT)
        assert lane.unit_obj.calibrate_td1.call_count == int(present)
        assert function._afc_cali_comp.call_count == int(present)
        assert io.client.check_for_td1.call_count == 1


class TestSpoolCommandCompletionBoundaries:
    setup_spool = TestSpoolAssignmentGenerations.setup_spool

    def test_loaded_lane_success_updates_led_and_active_spool_after_lookup(
            self, afc_connection):
        obj, io = afc_connection
        lane = make_lane(obj)
        spool = self.setup_spool(obj, lane)
        obj.moonraker = io.client
        obj.function.get_current_lane.return_value = lane.name
        obj.units[lane.unit] = lane.unit_obj
        lane.unit_obj.lane_loaded = Mock()
        spool.set_active_spool = Mock()
        io.responses[0] = {"filament": {"material": "PLA", "color_hex": "FFFFFF"},
                           "remaining_weight": 100}
        command = types.SimpleNamespace(get=lambda key, default=None:
                                        {"LANE": lane.name, "SPOOL_ID": "8"}.get(
                                            key, default))
        spool.cmd_SET_SPOOL_ID(command)
        assert lane.spool_id == 8
        lane.unit_obj.lane_loaded.assert_called_once_with(lane, force=True)
        spool.set_active_spool.assert_called_once_with(8)
        lane.send_lane_data.assert_called_once()

    @pytest.mark.parametrize("response", [None, {"filament": None}])
    def test_creator5_failed_real_lookup_preserves_record_and_gcode_error(
            self, afc_connection, response):
        obj, io = afc_connection
        obj.moonraker = io.client
        obj.spoolman = "configured"
        spool = AFCSpool.__new__(AFCSpool)
        spool.afc = obj
        spool.creator5_tool_lanes = {"T2": 2}
        records = [{"material": "PETG", "color": "#112233"} for _ in range(4)]
        spool.creator5_filament_sync = Mock(filaments=records)
        command = types.SimpleNamespace(
            get=lambda key, default=None: {"LANE": "T2", "SPOOL_ID": "8"}.get(
                key, default), error=ValueError)
        io.responses[0] = response
        with pytest.raises(ValueError, match="Cannot assign spool to T2: Spool ID not found"):
            spool.cmd_SET_SPOOL_ID(command)
        assert io.requests[0][0] == "/server/spoolman/proxy"
        assert records[2] == {"material": "PETG", "color": "#112233"}
        spool.creator5_filament_sync.set_tool_fields.assert_not_called()
