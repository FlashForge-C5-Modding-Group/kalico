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
        self._pending = None  # (seq, text, backups) not yet written
        self._latest = None  # newest submitted text
        self._seq = 0
        self._written_seq = 0
        self._done_seq = 0  # newest seq that reached the disk
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name=name, daemon=True
        )
        self._thread.start()

    def submit(self, text, backup=None):
        """Queue text without blocking; a newer submit supersedes it.

        backup, a (path, text) pair, is written before the file. Backups of
        superseded texts are still written.
        """
        with self._lock:
            self._seq += 1
            backups = list(self._pending[2]) if self._pending else []
            if backup is not None:
                backups.append(backup)
            item = (self._seq, text, backups)
            self._latest = text
            if not self._closed:
                self._pending = item
        if self._closed:
            # After close (shutdown teardown) write synchronously.
            self._write(*item)
            return
        self._wanted.set()

    def pending(self):
        """The newest submitted text while it isn't on disk yet, else None.

        Read it instead of the file, which may still hold an older text.
        """
        with self._lock:
            return self._latest if self._seq > self._done_seq else None

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

    def _write(self, seq, text, backups=()):
        with self._file_lock:
            for path, backup_text in backups:
                try:
                    with open(path, "w") as f:
                        f.write(backup_text)
                except Exception:
                    logging.exception("Could not write %s", path)
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
                with self._lock:
                    self._done_seq = max(self._done_seq, seq)
            except Exception:  # a dead writer thread would lose saves
                logging.exception("Could not write %s", self.path)
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
