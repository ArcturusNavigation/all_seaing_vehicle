"""
Live run-log stream for the control panel (issues #480, #501): receives the JSON-lines entries
that every RunLogger publishes on /run_log, **through foxglove_bridge's WebSocket** (not DDS),
so the laptop adds no DDS subscriber over Wi-Fi. It keeps a rolling window of recent entries,
on which the run-log analyzers (all_seaing_utility/run_log_analyzers) run live.

Connection: bridge.py.
"""

import collections
import json
import struct
import threading
import time

from all_seaing_control_panel.bridge import BridgeClient, decode_string_cdr

TOPIC = "/run_log"


class LiveLog:
    def __init__(self, url, window_s=300.0, max_entries=200_000):
        self.url = url
        self.window_s = window_s
        self._entries = collections.deque(maxlen=max_entries)  # (receive wall time, entry)
        self._lock = threading.Lock()
        self.received = 0
        self._client = BridgeClient(url, lambda t: t == TOPIC, self._on_message, name="live_log")
        self._client.start()

    def _on_message(self, channel, receive_ns, payload):
        now = time.time()
        try:
            text = decode_string_cdr(payload)
        except (struct.error, IndexError):
            return
        parsed = []
        for line in text.splitlines():
            try:
                parsed.append((now, json.loads(line)))
            except json.JSONDecodeError:
                pass
        with self._lock:
            self._entries.extend(parsed)
            self.received += len(parsed)
            while self._entries and now - self._entries[0][0] > self.window_s:
                self._entries.popleft()

    def status(self):
        with self._lock:
            n = len(self._entries)
            span = self._entries[-1][0] - self._entries[0][0] if n > 1 else 0.0
        return {"url": self.url, "connected": self._client.connected, "subscribed": self._client.connected and self.received > 0,
                "error": self._client.error,
                "entries": n, "received_total": self.received, "span_s": round(span, 1)}

    def window(self, seconds):
        """Copies of the entries received in the last `seconds` (wall time)."""
        cutoff = time.time() - seconds
        with self._lock:
            return [dict(e) for t, e in self._entries if t >= cutoff]
