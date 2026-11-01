"""
Minimal foxglove_bridge WebSocket client shared by the control panel's live features (live run log,
recorder, live frame view). Everything comes **through the bridge** (not DDS), so the laptop adds no
DDS subscriber over Wi-Fi.

Foxglove WebSocket protocol, the subset used here. Newer bridges (foxglove_bridge >= 0.8, built on
the Foxglove SDK) speak "foxglove.sdk.v1", older ones "foxglove.websocket.v1"; both are offered and the
bridge picks one (same messages):
  server -> {"op": "advertise", "channels": [{"id", "topic", "encoding", "schemaName", "schema", ...}]}
  server -> {"op": "unadvertise", "channelIds": [...]}
  client -> {"op": "subscribe", "subscriptions": [{"id": <sub id>, "channelId": <channel id>}]}
  server -> binary: 0x01 | u32 subscription id | u64 receive timestamp [ns] | payload (CDR)

Usage:
    client = BridgeClient(url, wanted=lambda topic: topic in TOPICS, on_message=cb, name="recorder")
    client.start()   # background thread, reconnects every 3 s while started
    ...              # cb(channel dict, bridge receive time [ns], CDR payload bytes) on that thread
    client.stop()    # closes the connection; start() again opens a new one
"""

import json
import struct
import threading
import time

from websockets.sync.client import connect


def decode_string_cdr(cdr):
    """std_msgs/String in CDR: 4-byte encapsulation header | u32 length (incl. NUL) | UTF-8 bytes."""
    (n,) = struct.unpack_from("<I", cdr, 4)
    return cdr[8:8 + n].rstrip(b"\x00").decode(errors="replace")


def header_stamp_ns(cdr):
    """Header stamp [ns] of a CDR (little endian) message starting with a std_msgs/Header."""
    sec, nanosec = struct.unpack_from("<iI", cdr, 4)
    return sec * 1_000_000_000 + nanosec


class BridgeClient:
    def __init__(self, url, wanted, on_message, name="bridge"):
        self.url = url
        self.wanted = wanted  # topic -> bool
        self.on_message = on_message  # (channel, receive time ns, payload)
        self.name = name
        self.enabled = False
        self.connected = False
        self.error = ""
        self._generation = 0  # each start() gets its own; a thread exits once it's not current
        self._ws = None
        self._lock = threading.Lock()

    def start(self):
        with self._lock:
            if self.enabled:
                return
            self.enabled = True
            self._generation += 1
            gen = self._generation
        threading.Thread(target=self._run, args=(gen,), daemon=True, name=f"{self.name} {self.url}").start()

    def stop(self):
        """Disconnects now; the connection thread exits (a later start() opens a new connection)."""
        with self._lock:
            self.enabled = False
            self._generation += 1
            ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def _current(self, gen):
        return self.enabled and gen == self._generation

    def _run(self, gen):
        while self._current(gen):
            try:
                with connect(self.url, subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"], open_timeout=5,
                             max_size=None) as ws:
                    with self._lock:
                        if not self._current(gen):
                            break
                        self._ws = ws
                    self.connected, self.error = True, ""
                    subs, next_id = {}, 1  # subscription id -> channel
                    for msg in ws:
                        if not self._current(gen):
                            break
                        if isinstance(msg, str):
                            d = json.loads(msg)
                            if d.get("op") == "advertise":
                                new = []
                                subscribed = {c["id"] for c in subs.values()}
                                for ch in d["channels"]:
                                    if self.wanted(ch["topic"]) and ch["id"] not in subscribed:
                                        subs[next_id] = ch
                                        new.append({"id": next_id, "channelId": ch["id"]})
                                        next_id += 1
                                if new:
                                    ws.send(json.dumps({"op": "subscribe", "subscriptions": new}))
                            elif d.get("op") == "unadvertise":
                                gone = set(d.get("channelIds", []))
                                subs = {k: v for k, v in subs.items() if v["id"] not in gone}
                        elif msg and msg[0] == 0x01:
                            sub_id, ts = struct.unpack_from("<IQ", msg, 1)
                            ch = subs.get(sub_id)
                            if ch is not None:
                                self.on_message(ch, ts, bytes(msg[13:]))
            except Exception as e:  # bridge down, network, closed by stop()...: retry while current
                if self._current(gen):
                    self.error = str(e)
            if gen == self._generation:
                self.connected = False
            if self._current(gen):
                time.sleep(3.0)
