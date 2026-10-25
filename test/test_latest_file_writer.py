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
    assert time.monotonic() - start < 0.05
    release.set()
    writer.close()
    with open(path) as f:
        assert f.read() == "third"
    assert not os.path.exists(path + ".tmp")


def test_write_failure_is_logged_not_raised(tmp_path):
    writer = lfw.LatestFileWriter(str(tmp_path / "missing" / "x.txt"), "test_fail")
    writer.submit("data")
    writer.close()  # must not raise


def test_submit_after_close_writes_synchronously(tmp_path):
    path = str(tmp_path / "late.txt")
    writer = lfw.LatestFileWriter(path, "test_late")
    writer.close()
    writer.submit("late")
    with open(path) as f:
        assert f.read() == "late"
