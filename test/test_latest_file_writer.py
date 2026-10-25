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


def test_write_failure_is_reported_and_later_writes_still_succeed(tmp_path):
    target = tmp_path / "missing" / "state.txt"
    errors = []
    failed = threading.Event()

    def on_error(exc):
        errors.append(exc)
        failed.set()

    writer = lfw.LatestFileWriter(str(target), "test_fail", on_error=on_error)
    writer.submit("first")  # directory does not exist yet
    assert failed.wait(5.)
    target.parent.mkdir()
    writer.submit("second")
    writer.close()
    assert target.read_text() == "second"
    assert len(errors) == 1 and isinstance(errors[0], OSError)
