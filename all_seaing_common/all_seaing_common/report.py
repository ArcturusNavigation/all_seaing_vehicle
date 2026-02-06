from enum import Enum
from all_seaing_common.report_pb2 import Report, Heartbeat, ObjectDetected, GatePass, ObjectDelivery, Docking, SoundSignal
from google.protobuf.timestamp_pb2 import Timestamp
from datetime import datetime, timezone

TEAM_ID = "MITB"
VEHICLE_ID = "Fish-n-Ships"
__report_seq = [0]

def report_factory(data):
    ret = None
    match data:
        case Heartbeat():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                heartbeat=data
            )
        case ObjectDetected():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                object_detected=data
            )
        case GatePass():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                object_detected=data
            )
        case ObjectDelivery():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                object_delivery=data
            )
        case Docking():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                docking=data
            )
        case SoundSignal():
            ret = Report(
                team_id=TEAM_ID,
                vehicle_id=VEHICLE_ID,
                seq=__report_seq[0],
                sound_signal=data
            )
    __report_seq[0] += 1
    ts = Timestamp()
    ts.FromDatetime(datetime.now(timezone.utc))
    ret.sent_at.CopyFrom(ts)
    return ret