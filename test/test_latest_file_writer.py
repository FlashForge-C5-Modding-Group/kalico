import os
import threading
import time

from klippy.extras import latest_file_writer as lfw


def test_submit_does_not_wait_for_disk_and_newest_text_wins(tmp_path, monkeypatch):
    path = str(tmp_path / "state.txt")
    release = threading.Event()
    real_fsync = os.fsync

    def slow_fsync(fd):
        release.wait(5.)
        real_fsync(fd)

    monkeypatch.setattr(lfw.os, "fsync", slow_fsync)
    writer = lfw.LatestFileWriter(path, "test_writer")
    start = time.monotonic()
    for text in ("first", "second", "third"):
        writer.submit(text)
    assert time.monotonic() - start < 1.  # fsync is blocked for up to 5s
    release.set()
    writer.close()
    with open(path) as f:
        assert f.read() == "third"
    assert os.listdir(tmp_path) == ["state.txt"]  # no leftover temp file


def test_write_failure_is_reported_not_raised(tmp_path):
    errors = []
    writer = lfw.LatestFileWriter(str(tmp_path / "missing" / "x.txt"),
                                  "test_fail", on_error=errors.append)
    writer.submit("data")
    writer.close()  # must not raise
    assert len(errors) == 1 and isinstance(errors[0], OSError)


def test_failing_error_handler_does_not_stop_later_writes(tmp_path):
    handled = threading.Event()

    def broken_handler(exc):
        handled.set()
        raise RuntimeError("handler bug")

    target = tmp_path / "later.txt"
    writer = lfw.LatestFileWriter(str(target), "test_handler", on_error=broken_handler)
    (tmp_path / ".later.txt.tmp").mkdir()  # first write cannot open its tmp file
    writer.submit("first")
    assert handled.wait(5.)
    (tmp_path / ".later.txt.tmp").rmdir()
    writer.submit("second")
    writer.close()
    assert target.read_text() == "second"


def test_unexpected_write_error_does_not_kill_writer_thread(tmp_path):
    target = tmp_path / "state.txt"
    errors = []
    writer = lfw.LatestFileWriter(str(target), "test_type", on_error=errors.append)
    writer.submit(123)  # not text: f.write raises TypeError on the thread
    writer._thread.join(0.5)
    assert writer._thread.is_alive()
    writer.submit("ok")
    writer.close()
    assert target.read_text() == "ok"
    assert len(errors) == 1 and isinstance(errors[0], TypeError)


def test_submit_after_close_writes_synchronously(tmp_path):
    path = str(tmp_path / "late.txt")
    writer = lfw.LatestFileWriter(path, "test_late")
    writer.close()
    writer.submit("late")
    with open(path) as f:
        assert f.read() == "late"
