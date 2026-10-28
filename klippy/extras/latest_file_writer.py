# Write the newest text for a file from a background thread
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import threading


class LatestFileWriter:
    """Persist the newest submitted text for one file off the reactor thread.

    eMMC writes, renames and fsyncs on the Creator 5 host can stall for
    hundreds of ms; on the reactor thread that starves step generation.
    Only the newest text matters, so it replaces any unwritten older text.
    Each write goes to a hidden '.<name>.tmp' beside the target, is fsynced,
    then atomically replaces the target, so readers never see a truncated file.

    Failures are logged and never raised.
    """

    def __init__(self, path, name):
        self.path = path
        self._lock = threading.Lock()  # guards _pending, _seq, _closed
        self._file_lock = threading.Lock()  # serializes writes to path
        self._wanted = threading.Event()
        self._pending = None  # (seq, text) not yet written
        self._seq = 0
        self._written_seq = 0
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name=name, daemon=True
        )
        self._thread.start()

    def submit(self, text):
        """Queue text without blocking; a newer submit supersedes it."""
        with self._lock:
            self._seq += 1
            item = (self._seq, text)
            if not self._closed:
                self._pending = item
        if self._closed:
            # After close (shutdown teardown) write synchronously.
            self._write(*item)
            return
        self._wanted.set()

    def flush(self):
        """Synchronously write any pending text (blocks the caller)."""
        with self._lock:
            item, self._pending = self._pending, None
        if item is not None:
            self._write(*item)

    def close(self):
        """Write pending text and stop the thread."""
        with self._lock:
            self._closed = True
        self.flush()
        self._wanted.set()
        self._thread.join()

    def _run(self):
        while True:
            self._wanted.wait()
            self._wanted.clear()
            with self._lock:
                item, self._pending = self._pending, None
                closed = self._closed
            if item is not None:
                self._write(*item)
            if closed:
                return

    def _write(self, seq, text):
        with self._file_lock:
            if seq <= self._written_seq:
                return  # a newer text was already written
            self._written_seq = seq
            directory, name = os.path.split(self.path)
            tmp_path = os.path.join(directory, "." + name + ".tmp")
            try:
                with open(tmp_path, "w") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, self.path)
            except Exception:  # a dead writer thread would lose saves
                logging.exception("Could not write %s", self.path)
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
