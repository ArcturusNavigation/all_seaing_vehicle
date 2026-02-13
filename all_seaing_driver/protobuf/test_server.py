"""
Created on 2025 Dec 24
@author: Daveman

Server that waits for TCP connection and parses ProtoBuf report.
"""

import argparse
import asyncio
import signal
from google.protobuf.timestamp_pb2 import Timestamp

import TCP_library
import report_pb2


def csv(val):
    return "" if val is None else str(val)


def parse_protobuf(payload:bytes, log_fn:callable):
    report = report_pb2.Report()
    report.ParseFromString(payload)

    print(report)
    
    body = report.WhichOneof("body")
    ts = TCP_library.make_timestring(report.sent_at)
    
    header = ",".join([
        csv(report.team_id),
        csv(report.vehicle_id),
        csv(report.seq),
        csv(ts),
        csv(body),
    ])
    
    if body == 'heartbeat':
        hb = report.heartbeat
        csv_line = ",".join([
            header,
            csv(hb.state),
            csv(hb.position.latitude),
            csv(hb.position.longitude),
            csv(hb.spd_mps),
            csv(hb.heading_deg),
            csv(hb.current_task),
        ])
    elif body == 'object_detected':
        od = report.object_detected
        csv_line = ",".join([
            header,
            csv(od.object_type),
            csv(od.color),
            csv(od.position.latitude),
            csv(od.position.longitude),
            csv(od.object_id),
            csv(od.task_context),
        ])
    elif body == 'gate_pass':
        gp = report.gate_pass
        csv_line = ",".join([
            header,
            csv(gp.type),
            csv(gp.position.latitude),
            csv(gp.position.longitude),
        ])
    elif body == 'object_delivery':
        od = report.object_delivery
        csv_line = ",".join([
            header,
            csv(od.vessel_color),
            csv(od.position.latitude),
            csv(od.position.longitude),
            csv(od.delivery_type),
        ])
    elif body == 'docking':
        d = report.docking
        csv_line = ",".join([
            header,
            csv(d.dock),
            csv(d.slip),
        ])
    elif body == 'sound_signal':
        ss = report.sound_signal
        csv_line = ",".join([
            header,
            csv(ss.signal_type),
            csv(ss.frequency_hz),
            csv(ss.assigned_task),
        ])

    log_fn(csv_line)


async def main(port):
    server = TCP_library.TCPServer(
        port=port,
        log_file="Boat",
    )
    
    server.set_executor_callback(
        parse_protobuf,
        server.log
        )
    
    loop = asyncio.get_running_loop()
    
    # Install signal handlers (Linux/macOS)
    try:
        loop.add_signal_handler(
            signal.SIGINT,
            lambda: asyncio.create_task(server.shutdown())
        )

        loop.add_signal_handler(
            signal.SIGTERM,
            lambda: asyncio.create_task(server.shutdown())
        )
    except NotImplementedError:
        # Windows fallback
        pass
    
    try:
        await server.start()
    except asyncio.CancelledError:
        pass
    finally:
        await server.shutdown()
        
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--port",
                        "-p",
                        help="TCP Port",
                        type=int,
                        default=TCP_library.PORT,
                        )
    
    args = parser.parse_args()
    
    try:
        asyncio.run(main(
            port=args.port,
        ))
    except KeyboardInterrupt:
        print("\n Ctrl-C received - exiting gracefully")
    