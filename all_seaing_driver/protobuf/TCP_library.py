"""
Created on 2025 Dec 04
@author: Daveman, Matthew Helms

Created a library for TCP server and client.

Server:
Uses asynchronous I/O and thread pools to read in the TCP data stream.

Client:
When run from the command line, user input is sent to server.
The "bad" argument modifies the message.

Using send_HLF_message(): A random number of bytes is added before the
header and after the footer.


Message:
Header ($R) + payload_length + payload + footer (!!)
$RHello world!!
Thread safe logging is performed, if desired.


Update:
2025 Dec 12
Changed the message to have a header ($R) and footer (!!) to make the system more robust.

2025 Dec 27
Added nanos to Timestamp.
Extracted nanos from Timestamp for the log file.
Moved random byte/char functions inside TCPClient class.

2026 Jan 28
Added Header Length Footer(HLF) packet format.
Made random byte/char functions static.
Update default port to 50000.

"""

import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import random
import signal
import socket
import string
import struct
import threading
import time
from google.protobuf.timestamp_pb2 import Timestamp

PORT = 50000
HEADER = b"$R"
FOOTER = b"!!"
LENGTH_SIZE = 1


def make_timestamp():
    now = time.time()
    ts = Timestamp()
    ts.seconds = int(now)
    ts.nanos = int((now - ts.seconds) * 1e9)
    return ts


def make_timestring(ts):
    return( f"{ts.seconds}.{ts.nanos:09d}" )


class TCPClient:
    def __init__(self, host:str="127.0.0.1", port:int=PORT, debug:bool=False, bad:bool=False):
        self.host = host
        self.port = port
        self.bad = bad
        self.debug = debug
        self.sock = None


    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

        if self.debug:
            print(f"[+] Connected to {self.host}:{self.port}")

    @staticmethod
    def random_bytes(min_len=3, max_len=10):
        length = random.randint(min_len, max_len)
        return bytes(random.getrandbits(8) for _ in range(length))

    @staticmethod
    def random_chars(min_len=3, max_len=10):
        length = random.randint(min_len, max_len)
        chars = string.ascii_letters + string.digits
        return ''.join(random.choice(chars) for _ in range(length))
            
    def send_HLF_message(self, payload:bytes):
        if not self.sock:
            raise RuntimeError("Client not connected")
        
        r1 = r2 = b""
        if self.bad:
            r1 = self.random_bytes()
            r2 = self.random_bytes()
        packet = r1 + HEADER + struct.pack("!B", len(payload)) + payload + FOOTER + r2
        print(packet)
        self.sock.sendall(packet)
        if self.debug:
            print(f"[>] Sent {packet!r}")


    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

            if self.debug:
                print("[*] Connection closed")


class TCPServer:
    def __init__(self, host:str="0.0.0.0", port:int=PORT, workers:int=8,  log_file:str=None, debug:bool=False):
        self.host = host
        self.port = port
        self.workers = workers
        self.log_file = log_file
        self.debug = debug
        
        # YYYMMDD_HHMMSS
        timestamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
        
        if self.log_file:
            self.log_file = f"{log_file}_{timestamp}.csv"

        self.executor = ThreadPoolExecutor(max_workers=workers)
        self.log_lock = threading.Lock()
        self.executor_fn = None
        self.executor_args = ()

        self.server = None
        self.shutdown_event = asyncio.Event()

        if self.debug: 
            print(f"[+] host:{self.host}:{self.port} workers:{self.workers}")
            print(f"[+] log:{self.log_file}")
            print(f"[+] debug:{self.debug}")
            
            
    def set_executor_callback(self, fn, *args):
        self.executor_fn = fn
        self.executor_args = args



    # Thread-safe logging
    def log(self, msg:str):
        #timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H:%M:%S.%f")
        timestamp = make_timestamp()
        
        payload = f"{make_timestring(timestamp)},{msg}"
        
        if self.log_file:
            with self.log_lock:
                with open(self.log_file, "a", encoding='utf-8') as f:
                    f.write(f"{payload}\n")

        if self.debug:
            print(f"[>] {payload}")


    # Handle each client connection
    async def handle_client(self, reader:asyncio.StreamReader, writer:asyncio.StreamWriter):
        addr = writer.get_extra_info('peername')
        if self.debug:
            print(f"[+] Connection from {addr}")

        buffer = bytearray()
        loop = asyncio.get_running_loop()

        try:
            while not self.shutdown_event.is_set():
                # Read whatever data is available
                chunk = await reader.read(4096)

                # True EOF means client closed connection
                if chunk == b"":
                    if self.debug:
                        print(f"[!] Client {addr} disconnected")
                    break

                if chunk is None:
                    continue

                buffer.extend(chunk)

                # Extract framed messages
                messages = self.extract_messages(buffer)

                for payload in messages:
                    if self.executor_fn:
                        await loop.run_in_executor(
                            self.executor,
                            self.executor_fn,
                            payload,
                            *self.executor_args
                        )
                    else:
                        print(f"Message from {addr}")
                        await loop.run_in_executor(
                            self.executor,
                            self.log,
                            f"{addr} | {payload!r}" 
                        )

        except Exception as e:
            if self.debug:
                print(f"[!] Error with {addr}: {e}")
        finally:
            writer.close()
            await writer.wait_closed()
            if self.debug:
                print(f"[*] Connection closed for {addr}")


    """
    Generator that yields complete messages framed by HEADER/FOOTER.
    Removes consumed bytes from buffer.
    """
    @staticmethod
    def extract_messages(buffer:bytearray):
        
        messages=[]

        while True:
            header_start = buffer.find(HEADER)
            if header_start == -1:
                # No header yet, discard junk before possible partial header
                if len(buffer) > len(HEADER):
                    del buffer[:-len(HEADER)]
                break

            length_start = header_start + len(HEADER)
            if length_start + LENGTH_SIZE > len(buffer):
                # Buffer not long enough to read length, discard bytes before header
                if header_start > 0:
                    del buffer[: header_start]
                break

            payload_length = buffer[length_start : length_start + LENGTH_SIZE][0]
            payload_start = length_start + LENGTH_SIZE
            footer_start = payload_start + payload_length

            if footer_start + len(FOOTER) > len(buffer):
                # Buffer not long enough to read payload and footer, discard bytes before header
                if header_start > 0:
                    del buffer[: header_start]
                break

            payload = bytes(buffer[payload_start : payload_start + payload_length])

            footer = buffer[footer_start : footer_start + len(FOOTER)]
            if footer != FOOTER:
                # Footer doesn't match expected value, discard current header and repeat loop
                del buffer[: length_start]
                continue

            messages.append(payload)

            # Removed processed bytes
            del buffer[: footer_start + len(FOOTER)]

        return messages


    # Start server
    async def start(self):
        self.server = await asyncio.start_server(self.handle_client, self.host, self.port)

        addr = ", ".join(str(sock.getsockname()) for sock in self.server.sockets)
        if self.debug:
            print(f"[+] Server started on {addr}")

        async with self.server:
            await self.shutdown_event.wait()
            if self.debug:
                print("[*] Shutdown event triggered - stopping server...")


    # Graceful shutdown
    async def shutdown(self):
        self.shutdown_event.set()
        self.executor.shutdown(wait=True)

        if self.debug:
            print("[*] Thread pool shut down.")

        if self.server:
            self.server.close()
            await self.server.wait_closed()

            if self.debug:
                print("[*] Server socket closed.")


async def main(port, log_file, debug):
    server = TCPServer(
        port=port,
        log_file=log_file,
        debug=debug
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

    sub = parser.add_subparsers(dest="mode", required=True)
    
    # Subparser server
    server = sub.add_parser("server",
                            help="Run in server mode"
                            )

    server.add_argument("--debug",
                        "-d",
                        help="Print server debugging statements",
                        action='store_true',
                        )
    server.add_argument("--log",
                        "-l",
                        help="Name of server log file",
                        default=None)
    server.add_argument("--port",
                        "-p",
                        help="TCP Port",
                        type=int,
                        default=PORT,
                        )

    # Subparser client
    client = sub.add_parser("client",
                            help="Run in client mode"
                            )
    
    client.add_argument("--bad",
                        "-b",
                        help="Add junk bytes to the message (pre/post header/footer)",
                        action='store_true',
                        )
    client.add_argument("--debug",
                        "-d",
                        help="Print client debugging statements",
                        action='store_true',
                        )
    client.add_argument("--TCP",
                        "-t",
                        help="TCP Host",
                        type=str,
                        default="127.0.0.1",
                        )
    client.add_argument("--port",
                        "-p",
                        help="TCP Port",
                        type=int,
                        default=PORT,
                        )

    args = parser.parse_args()

    if args.mode == "server":
        print("Running server")
        try:
            asyncio.run(main(
                port=args.port,
                log_file=args.log,
                debug=args.debug
            ))
        except KeyboardInterrupt:
            print("\n[!] CTRL-C received - exiting gracefully")

    elif args.mode == "client":
        print("Running client")

        client = TCPClient(args.TCP, args.port, args.bad)

        try:
            client.connect()

            while True:
                msg = input("Enter message ('quit') to exit): ").strip()
                if msg.lower() in ('quit', 'exit'):
                    break

                client.send_HLF_message(msg.encode('utf-8'))
        except KeyboardInterrupt:
            print("\n[!] Interrupted by user")

        finally:
            client.close()



