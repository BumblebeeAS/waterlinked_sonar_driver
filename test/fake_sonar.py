"""Fake Sonar 3D-15 for integration tests.

Answers the integration HTTP API on localhost and, while acoustics is enabled
and UDP output is unicast, replays a .sonar recording to the configured
destination.

Usage: fake_sonar.py RECORDING HTTP_PORT
"""

import json
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

state = {
    "acoustics": False,
    "range": {"min": 0.3, "max": 15.0},
    "speed_of_sound": 0.0,
    "mode": "low-frequency",
    "salinity": "salt",
    "udp": {
        "mode": "multicast",
        "unicast_destination_ip": "",
        "unicast_destination_port": 0,
    },
    "imu": False,
    "ntp": "auto",
}
lock = threading.Lock()

STATUS_ENTRY = {"id": "x", "message": "ok", "status": "ok", "operational": True}
ABOUT = {
    "chipid": "0xfake",
    "hardware_revision": 6,
    "is_ready": True,
    "product_id": 21045,
    "product_name": "Sonar 3D-15",
    "variant": "",
    "version": "1.8.0 (fake)",
    "version_short": "1.8.0",
}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, body=None):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        if body is not None:
            self.wfile.write(json.dumps(body).encode())

    def do_GET(self):
        path = self.path.replace("/api/v1/integration", "", 1)
        with lock:
            routes = {
                "/about": ABOUT,
                "/status": {
                    "api": STATUS_ENTRY,
                    "temperature": STATUS_ENTRY,
                    "systems_check": STATUS_ENTRY,
                    "time": STATUS_ENTRY,
                },
                "/temperature": 21.5,
                "/acoustics/enabled": state["acoustics"],
                "/acoustics/range": state["range"],
                "/acoustics/speed_of_sound": state["speed_of_sound"],
                "/acoustics/mode": state["mode"],
                "/acoustics/salinity": state["salinity"],
                "/udp": state["udp"],
                "/output/imu-batch/enabled": state["imu"],
                "/time/status": {
                    "system_time": "2026-01-01T00:00:00Z",
                    "ntp_synced": False,
                    "ntp_synced_to": "",
                    "ntp_seconds_since_last_sync": None,
                },
                "/time/ntp": {"ntp_address": state["ntp"]},
            }
        if path in routes:
            self._send(200, routes[path])
        else:
            self._send(404)

    def do_POST(self):
        path = self.path.replace("/api/v1/integration", "", 1)
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length)) if length else None
        keys = {
            "/acoustics/enabled": "acoustics",
            "/acoustics/range": "range",
            "/acoustics/speed_of_sound": "speed_of_sound",
            "/acoustics/mode": "mode",
            "/acoustics/salinity": "salinity",
            "/udp": "udp",
            "/output/imu-batch/enabled": "imu",
        }
        with lock:
            if path in keys:
                state[keys[path]] = body
            elif path == "/time/ntp":
                state["ntp"] = body["ntp_address"]
            else:
                return self._send(404)
        self._send(204)


def read_packets(recording):
    with open(recording, "rb") as f:
        data = f.read()
    packets = []
    offset = 0
    while offset + 8 <= len(data):
        total = struct.unpack_from("<I", data, offset + 4)[0]
        packets.append(data[offset : offset + total])
        offset += total
    return packets


def replay(packets):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        with lock:
            streaming = state["acoustics"] and state["udp"]["mode"] == "unicast"
            destination = (
                state["udp"]["unicast_destination_ip"],
                state["udp"]["unicast_destination_port"],
            )
        if streaming:
            for packet in packets:
                sock.sendto(packet, destination)
                time.sleep(0.02)
        else:
            time.sleep(0.1)


def main():
    recording, http_port = sys.argv[1], int(sys.argv[2])
    packets = read_packets(recording)
    threading.Thread(target=replay, args=(packets,), daemon=True).start()
    server = HTTPServer(("127.0.0.1", http_port), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
