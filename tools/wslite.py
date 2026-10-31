"""Minimal WebSocket (RFC 6455) framing over plain sockets, stdlib only.

Just enough for the AUTOBOT tools: text frames, ping/pong, close. Used by
tools/fake_robot.py (server side) and tools/link_stress.py (client side).
"""

from __future__ import annotations

import base64
import hashlib
import os
import socket
import struct

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_TEXT = 0x1
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


class WsClosed(Exception):
    pass


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise WsClosed("connection closed")
        buf += chunk
    return buf


def send_frame(sock: socket.socket, payload: bytes, opcode=OP_TEXT, mask=False):
    head = bytes([0x80 | opcode])
    n = len(payload)
    mbit = 0x80 if mask else 0
    if n < 126:
        head += bytes([mbit | n])
    elif n < 65536:
        head += bytes([mbit | 126]) + struct.pack(">H", n)
    else:
        head += bytes([mbit | 127]) + struct.pack(">Q", n)
    if mask:
        key = os.urandom(4)
        payload = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
        head += key
    sock.sendall(head + payload)


def recv_frame(sock: socket.socket):
    """Returns (opcode, payload bytes). Raises WsClosed / socket.timeout."""
    b0, b1 = _recv_exact(sock, 2)
    opcode = b0 & 0x0F
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack(">H", _recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack(">Q", _recv_exact(sock, 8))[0]
    key = _recv_exact(sock, 4) if b1 & 0x80 else None
    payload = _recv_exact(sock, n) if n else b""
    if key:
        payload = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
    return opcode, payload


class WsClient:
    """Blocking client. send_text() is safe from another thread than recv()."""

    def __init__(self, host: str, port: int = 80, path: str = "/ws", timeout=3.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
               f"Sec-WebSocket-Version: 13\r\n\r\n")
        self.sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise WsClosed("closed during handshake")
            resp += chunk
            if len(resp) > 8192:
                raise WsClosed("handshake reply too long")
        head = resp.split(b"\r\n\r\n", 1)[0].decode(errors="replace")
        if " 101 " not in head.split("\r\n", 1)[0]:
            raise WsClosed("handshake refused: " + head.split("\r\n", 1)[0])
        if accept_key(key) not in head:
            raise WsClosed("bad Sec-WebSocket-Accept")

    def send_text(self, text: str):
        send_frame(self.sock, text.encode(), OP_TEXT, mask=True)

    def recv_text(self):
        """Next text message (answers pings itself). None on close."""
        while True:
            op, payload = recv_frame(self.sock)
            if op == OP_TEXT:
                return payload.decode(errors="replace")
            if op == OP_PING:
                send_frame(self.sock, payload, OP_PONG, mask=True)
            elif op == OP_CLOSE:
                return None

    def close(self):
        try:
            send_frame(self.sock, b"", OP_CLOSE, mask=True)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
