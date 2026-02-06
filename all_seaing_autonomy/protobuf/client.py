from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from report_pb2 import *
import serial
import socket, struct
import time

def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise RuntimeError("Serial connection closed or timeout")
        buf.extend(chunk)
    return bytes(buf)


def read_frame(ser):
    # Read 4-byte length header
    try:
        header = read_exact(ser, 4)
        (size,) = struct.unpack("!I", header)
        if size > 2000: #TODO: make a better check here
            raise Exception
        print(f"Received header data, size {size}")
    except:
        serial_port.reset_input_buffer()
        print('No data yet')
        return None

    # Read the payload of that length
    payload = read_exact(ser, size)
    return payload

# Setup serial connection
# serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
serial_port = serial.Serial('/dev/tty.usbmodem2080345752471', 115200, timeout=1)
time.sleep(2)  # Allow serial port to stabilize

while True:
    if serial_port.in_waiting == 0:
        continue

    msg_str = read_frame(serial_port)

    if msg_str == None:
        continue

    msg = Report()
    msg.ParseFromString(msg_str)
    print(msg)

    ts = Timestamp(); ts.FromDatetime(datetime.now(timezone.utc))
    msg.sent_at.CopyFrom(ts)

    # Serialize (binary protobuf) and send with 4-byte big-endian length prefix,
    wire = msg.SerializeToString()
    frame = struct.pack("!I", len(wire)) + wire + "test".encode()

    with socket.create_connection(("localhost", 12345)) as s:
        s.sendall(frame)


serial_port.close()