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


def test_failed_write_does_not_stop_later_writes(tmp_path, monkeypatch):
    target = tmp_path / "state.txt"
    real_replace = os.replace
    failed = threading.Event()

    def replace_fails_once(src, dst):
        if not failed.is_set():
            failed.set()
            raise OSError("disk full")
        real_replace(src, dst)

    monkeypatch.setattr(lfw.os, "replace", replace_fails_once)
    writer = lfw.LatestFileWriter(str(target), "test_fail")
    writer.submit("first")
    assert failed.wait(5.)  # failed on the writer thread
    writer.submit("second")
    writer.close()
    assert target.read_text() == "second"
    assert os.listdir(tmp_path) == ["state.txt"]  # failed temp file removed
