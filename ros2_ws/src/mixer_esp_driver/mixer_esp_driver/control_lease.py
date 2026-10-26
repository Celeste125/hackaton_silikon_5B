"""Single-controller lease for the web panel.

Any number of browsers may open the panel and watch, but only one at a time may
command the robot. Without this, every open tab posted its own motor and drive
setpoints and the last writer won, so two tabs made the motors alternate.

The holder keeps the lease alive by talking to the server (its state poll and
its control requests). If it goes silent for ``timeout_s`` the lease is free.
Another client can take it over deliberately with ``force=True``.

This is protection against collisions between people, not security: client ids
are not secret and the panel has no login.
"""

import threading
import time


class ControlLease:
    def __init__(self, timeout_s=6.0, clock=time.monotonic):
        self.timeout_s = float(timeout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._holder = None
        self._label = ''
        self._seen = 0.0

    def _live(self, now):
        return self._holder is not None and now - self._seen <= self.timeout_s

    def claim(self, client, label='', force=False):
        """Try to become the holder. Returns (ok, changed_hands)."""
        with self._lock:
            now = self._clock()
            if self._live(now) and self._holder != client and not force:
                return False, False
            changed = self._holder != client
            self._holder = client
            self._label = label
            self._seen = now
            return True, changed

    def touch(self, client):
        """True if ``client`` holds a live lease; refreshes it."""
        with self._lock:
            now = self._clock()
            if self._holder == client and self._live(now):
                self._seen = now
                return True
            return False

    def release(self, client):
        """Give the lease up. True if ``client`` was the holder."""
        with self._lock:
            if self._holder != client:
                return False
            self._holder = None
            self._label = ''
            return True

    def snapshot(self, client=None):
        with self._lock:
            live = self._live(self._clock())
            return {
                'held': live,
                'you': live and client is not None and self._holder == client,
                'holder': self._label if live else None,
            }
