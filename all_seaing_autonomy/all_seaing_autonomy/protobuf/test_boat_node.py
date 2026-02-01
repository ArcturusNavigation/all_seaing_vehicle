from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from report_pb2 import *
import socket, struct
import serial
import time


serial_port = serial.Serial("/dev/tty.usbmodem2089346352471", 57600, timeout=1)
time.sleep(2)  # Allow serial port to stabilize

# Build message,
msg = Report(
    team_id="MITB",
    vehicle_id="Fish-n-Ships",
    seq=42,
    gate_pass=GatePass(
        type=GateType.GATE_ENTRY,
        position=LatLng(latitude=27.374736, longitude=-82.452767),
    ),
)
ts = Timestamp(); ts.FromDatetime(datetime.now(timezone.utc))
msg.sent_at.CopyFrom(ts)

# Serialize (binary protobuf) and send with 4-byte big-endian length prefix,
wire = msg.SerializeToString()
frame = struct.pack("!I", len(wire)) + wire + "test".encode()

serial_port.write(frame)
serial_port.flush() 
serial_port.close()

