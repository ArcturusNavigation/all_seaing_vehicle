import report_pb2
import socketserver

class PbHandler(socketserver.BaseRequestHandler):

    def handle(self):
        msg_len = int.from_bytes(self.request.recv(4), byteorder='big')
        msg_str = self.request.recv(msg_len)
        msg = report_pb2.Report()
        msg.ParseFromString(msg_str)
        print(msg)


if __name__ == "__main__":
    HOST, PORT = "localhost", 12345

    with socketserver.TCPServer((HOST, PORT), PbHandler) as server:
        server.serve_forever()