#!/usr/bin/env python3
"""
Mock of the Franka Desk REST API that franka_lock_unlock uses (test only).

The endpoints come from the client code, not from a real Desk. FR3 joint paths only.

Usage: mock_desk.py <host> <port> <certfile> <keyfile> [--fail-unlock] [--in-use]
  --fail-unlock  the unlock request fails (HTTP 500)
  --in-use       another owner holds the control token
"""

import json
import ssl
import sys
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST, PORT, CERT, KEY = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
FAIL_UNLOCK = "--fail-unlock" in sys.argv
IN_USE = "--in-use" in sys.argv  # another user holds the control token
STATE = {"token": None, "token_id": None, "locked": True, "fci": False, "logged_in": False}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body="", ctype="text/plain"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(n) if n else b""

    def log_message(self, fmt, *args):
        print(f"[desk {HOST}:{PORT}] {self.command} {self.path}", flush=True)

    def do_GET(self):
        if self.path == "/":
            return self._send(200, "<html>Desk</html>", "text/html")
        if self.path == "/admin/api/control-token":
            active = None if STATE["token_id"] is None else {"id": STATE["token_id"]}
            if IN_USE:
                active = {"id": 99, "ownedBy": "desk-user", "token": "secret"}
            return self._send(200, json.dumps({"activeToken": active}), "application/json")
        self._send(404, "not found")

    def do_POST(self):
        self._body()
        p = self.path
        if p == "/admin/api/login":
            STATE["logged_in"] = True
            return self._send(200, "auth-cookie")
        if p == "/admin/api/logout":
            STATE["logged_in"] = False
            return self._send(200)
        if p.startswith("/admin/api/control-token/request"):
            STATE["token"], STATE["token_id"] = str(uuid.uuid4()), 1
            STATE["pending"] = IN_USE
            return self._send(
                200, json.dumps({"token": STATE["token"], "id": 1}), "application/json"
            )
        if p == "/admin/api/control-token/fci":
            STATE["fci"] = True
            return self._send(200)
        if p.startswith("/admin/api/safety/recoverable-safety-errors/acknowledge"):
            return self._send(424, "NoAckRequired")
        if p == "/admin/api/safety/td2-tests/execute":
            return self._send(200)
        if p in ("/desk/api/joints/unlock", "/desk/api/joints/lock"):
            unlock = p.endswith("unlock")
            if unlock and FAIL_UNLOCK:
                return self._send(500, "mock unlock failure")
            STATE["locked"] = not unlock
            if not unlock:
                STATE["fci"] = False
            print(
                f"[desk {HOST}:{PORT}] STATE locked={STATE['locked']} fci={STATE['fci']}",
                flush=True,
            )
            return self._send(200)
        self._send(404, "not found")

    def do_DELETE(self):
        self._body()
        if self.path == "/admin/api/control-token":
            STATE["token"], STATE["token_id"] = None, None
            return self._send(200)
        self._send(404, "not found")


server = ThreadingHTTPServer((HOST, PORT), Handler)
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(CERT, KEY)
server.socket = ctx.wrap_socket(server.socket, server_side=True)
print(f"mock desk on {HOST}:{PORT}", flush=True)
server.serve_forever()
