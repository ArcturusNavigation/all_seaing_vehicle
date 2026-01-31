from datetime import datetime, timezone
from google.protobuf.timestamp_pb2 import Timestamp
from report_pb2 import *
import socket, struct

# Build message,
msg = Report(
    team_id="GOAT",
    vehicle_id="Bob-01",
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

with socket.create_connection(("localhost", 12345)) as s:
    s.sendall(frame)