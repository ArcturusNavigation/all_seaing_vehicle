from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from report_pb2 import *
import serial
import socket, struct
import time

HEADER = b"$R"
FOOTER = b"!!"

class FrameParser:
    def __init__(self, max_len=255):
        self.buf = bytearray()
        self.max_len = max_len

    def feed(self, data: bytes):
        """Feed raw bytes; returns a list of valid payloads."""
        self.buf.extend(data)
        payloads = []

        while True:
            # 1) Find header
            start = self.buf.find(HEADER)
            if start < 0:
                # Keep at most 1 byte in case it's b'$' starting a header
                if len(self.buf) > 1:
                    self.buf[:] = self.buf[-1:]
                break
            if start > 0:
                del self.buf[:start]

            if len(self.buf) < 4:
                break
            length = self.buf[2]
            recv_cksum = self.buf[3]

            if length > self.max_len:
                del self.buf[0:1]
                continue

            frame_len = 2 + 1 + 1 + length + 2  # $R + len + cksum + payload + !!
            if len(self.buf) < frame_len:
                break  # wait for more bytes

            # 3) Check footer
            if self.buf[frame_len - 2: frame_len] != FOOTER:
                # misaligned/corrupt; skip one byte and rescan
                del self.buf[0:1]
                continue

            # 4) Extract payload and verify checksum
            payload = bytes(self.buf[4: 4 + length])
            calc_cksum = sum(payload) % 256

            if calc_cksum != recv_cksum:
                # Bad frame (payload corrupted or wrong length). Resync.
                # Drop the leading '$' and rescan for next header.
                del self.buf[0:1]
                continue

            # 5) Accept frame
            payloads.append(payload)
            del self.buf[:frame_len]

        return payloads

# Setup serial connection
# serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serial_port = serial.Serial('/dev/tty.usbmodem2080345752471', 115200, timeout=0.2)
parser = FrameParser()
time.sleep(2)  # Allow serial port to stabilize
sock = socket.create_connection(("localhost", 50000))
try:
    while True:
        chunk = serial_port.read(1024)
        if not chunk:
            continue

        for payload in parser.feed(chunk):
            print('payload')
            try:
                msg = Report()
                msg.ParseFromString(payload)
                print(msg)

                ts = Timestamp(); ts.FromDatetime(datetime.now(timezone.utc))
                msg.sent_at.CopyFrom(ts)
                # Serialize (binary protobuf) and send with 4-byte big-endian length prefix,
                wire = msg.SerializeToString()
                frame = HEADER + struct.pack("!B", len(payload)) + bytes(payload) + FOOTER
                sock.sendall(frame)
                print('Sent message')
            except DecodeError as e:
                # payload passed checksum but still not valid protobuf (rare but possible)
                print("protobuf decode error:", e)
except KeyboardInterrupt:
    print("\n Ctrl-C received - exiting gracefully")
finally:
    serial_port.close()
    sock.close()