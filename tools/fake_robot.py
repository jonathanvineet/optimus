#!/usr/bin/env python3
"""A fake AUTOBOT robot (firmware 2.0 protocol) for testing without hardware.

Serves the same endpoints as the ESP32: / (the real phone page, read from the
.ino), /ws, /api/samples, /api/telemetry, /api/log, /api/diag. The data comes
from the mapper's SimulatedRobot driving around a room.

Chaos modes (to prove the Mac side and the phone page survive failures):
  drop     refuse every connection, close open WebSockets   (robot off / out of range)
  stall    accept, then answer nothing for the whole window  (robot frozen)
  reset    one-shot reboot: new boot id, clock and buffers restart
  garbage  malformed replies and corrupt sample rows
  slow     replies trickle out in small pieces

  python tools/fake_robot.py --port 8080
  python tools/fake_robot.py --port 8080 --chaos drop --at 10 --for 5
  curl 'http://127.0.0.1:8080/chaos?mode=stall&secs=4'
"""

from __future__ import annotations

import argparse
import json
import random
import re
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import wslite  # noqa: E402
from mac_mapper import SimulatedRobot  # noqa: E402

SAMPLE_RING = 512
LOG_SIZE = 3000
COLS = ["seq", "millis", "mode", "distance_cm", "us_status", "us_seq", "us_age_ms",
        "imu_ok", "yaw_unwrapped_deg", "roll_deg", "pitch_deg", "gyro_z_dps",
        "gyro_z_raw_dps", "gyro_bias_dps", "gyro_cal", "left_cmd", "right_cmd",
        "left_cmd_int", "right_cmd_int"]


def phone_page():
    ino = (ROOT / "autobot_mapping" / "autobot_mapping.ino").read_text()
    m = re.search(r'R"HTML\((.*?)\)HTML"', ino, re.S)
    return m.group(1).encode() if m else b"<h1>no page</h1>"


class FakeRobot:
    def __init__(self, host="127.0.0.1", port=0):
        self.lock = threading.Lock()
        self.chaos_mode = "none"
        self.chaos_until = 0.0
        self.ws_socks = set()
        self.drive_frames = []           # (seq, throttle, steering, mono)
        self.stops = []
        self.phone_logs = []
        self.requests = 0
        self.page = phone_page()
        self._boot()
        self._stop = threading.Event()

        robot = self

        class Handler(_Handler):
            fake = robot

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self.url = f"http://{host}:{self.port}"

    # ----- state -----
    def _boot(self, prev_events=None):
        self.boot_id = random.randint(1, 2**31)
        self.t0 = time.monotonic()
        self.sim = SimulatedRobot(seed=random.randint(0, 999))
        self.samples = deque(maxlen=SAMPLE_RING)
        self.sample_seq = 0
        self.events = []
        self.event_base = 0
        self.mode = "M"
        if prev_events:
            self._event("PREV_BOOT", "begin", len(prev_events), 0, 0)
            for e in prev_events:
                self._event("PREV:" + e[2], e[3], e[4], e[5], e[6], ms=e[1])
            self._event("PREV_BOOT", "end", 0, 0, 0)
        self._event("BOOT", "software" if prev_events else "power_on", 0, 0, 0)

    def millis(self):
        return int((time.monotonic() - self.t0) * 1000)

    def _event(self, etype, tag, a, b, f, ms=None):
        seq = self.event_base + len(self.events)
        self.events.append((seq, self.millis() if ms is None else ms, etype, tag, a, b, f))
        if len(self.events) > LOG_SIZE:
            drop = len(self.events) - LOG_SIZE
            self.events = self.events[drop:]
            self.event_base += drop

    def reboot(self):
        with self.lock:
            self._boot(prev_events=self.events[-20:])
        for s in list(self.ws_socks):
            try:
                s.close()
            except OSError:
                pass

    def chaos(self, mode, secs):
        if mode == "reset":
            self.reboot()
            return
        with self.lock:
            self.chaos_mode = mode
            self.chaos_until = time.monotonic() + secs
        if mode == "drop":
            for s in list(self.ws_socks):
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def active_chaos(self):
        with self.lock:
            if self.chaos_mode != "none" and time.monotonic() < self.chaos_until:
                return self.chaos_mode
            self.chaos_mode = "none"
            return "none"

    def _sample_row(self):
        d = self.sim.telemetry_dict()
        dist = "" if d["distance_cm"] is None else f"{d['distance_cm']:.1f}"
        vals = [self.sample_seq, d["millis"], self.mode, dist, d["us_status"], d["us_seq"],
                d["us_age_ms"], 1, f"{d['yaw_unwrapped_deg']:.3f}", "0.00", "0.00",
                f"{d['gyro_z_dps']:.3f}", f"{d['gyro_z_raw_dps']:.3f}",
                f"{d['gyro_bias_dps']:.4f}", "ok", d["left_cmd"], d["right_cmd"],
                f"{d['left_cmd_int']:.3f}", f"{d['right_cmd_int']:.3f}"]
        return ",".join(str(v) for v in vals)

    def _tick(self):
        last_motor = None
        while not self._stop.wait(0.04):
            with self.lock:
                self.sim.run_until(time.monotonic() - self.t0)
                self.samples.append((self.sample_seq, self._sample_row()))
                self.sample_seq += 1
                d = self.sim.telemetry_dict()
                if d["distance_cm"] is not None:
                    self._event("RANGE", "ok", 0, 0, d["distance_cm"])
                motor = (d["left_cmd"], d["right_cmd"])
                if motor != last_motor:
                    self._event("MOTOR", "auto", motor[0], motor[1], 0)
                    last_motor = motor

    def _push_status(self):
        while not self._stop.wait(0.1):
            with self.lock:
                d = self.sim.telemetry_dict()
                seq = self.drive_frames[-1][0] if self.drive_frames else 0
                dist = "-" if d["distance_cm"] is None else str(int(d["distance_cm"]))
                msg = f"s,{seq},{dist},0,{self.mode},,"
            if self.active_chaos() in ("drop", "stall"):
                continue
            for s in list(self.ws_socks):
                try:
                    wslite.send_frame(s, msg.encode())
                except OSError:
                    self.ws_socks.discard(s)

    def start(self):
        for fn in (self.httpd.serve_forever, self._tick, self._push_status):
            threading.Thread(target=fn, daemon=True).start()
        return self

    def stop(self):
        self._stop.set()
        self.httpd.shutdown()
        self.httpd.server_close()
        for s in list(self.ws_socks):
            try:
                s.close()
            except OSError:
                pass


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"     # keep-alive, like esp_http_server
    fake: FakeRobot = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="text/plain"):
        chaos = self.fake.active_chaos()
        if chaos == "garbage" and ctype == "application/json":
            body = b"{not json"
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if chaos == "slow":
            for i in range(0, len(body), 64):
                self.wfile.write(body[i:i + 64])
                self.wfile.flush()
                time.sleep(0.05)
        else:
            self.wfile.write(body)

    def do_GET(self):
        fake = self.fake
        fake.requests += 1
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}

        if url.path == "/chaos":       # control endpoint, never affected by chaos
            fake.chaos(q.get("mode", "none"), float(q.get("secs", "3")))
            return self._send(200, b"ok")

        chaos = fake.active_chaos()
        if chaos == "drop":
            self.close_connection = True
            self.connection.close()
            return
        if chaos == "stall":
            while fake.active_chaos() == "stall":
                time.sleep(0.05)
            self.close_connection = True
            self.connection.close()
            return

        if url.path == "/":
            return self._send(200, fake.page, "text/html")
        if url.path == "/ws":
            return self._websocket()
        if url.path == "/api/samples":
            return self._samples(int(q.get("since", "0")), chaos == "garbage")
        if url.path == "/api/telemetry":
            with fake.lock:
                d = dict(fake.sim.telemetry_dict(), boot_id=fake.boot_id,
                         millis=fake.millis(), fw="fake-2.0")
            return self._send(200, json.dumps(d).encode(), "application/json")
        if url.path == "/api/log":
            return self._log(int(q.get("since", "0")))
        if url.path == "/api/diag":
            d = {"fw": "fake-2.0", "boot_id": fake.boot_id, "reset_reason": "power_on",
                 "uptime_ms": fake.millis(), "min_free_heap": 150000, "loop_max_us_boot": 900,
                 "wifi_disconnects": 0, "wifi_last_reason": 0, "ws_clients": len(fake.ws_socks),
                 "boots": [{"boot_no": 1, "reason": "power_on", "fw": "fake-2.0",
                            "prev_boot_ran_ms": 0, "events_before_reset": []}]}
            return self._send(200, json.dumps(d).encode(), "application/json")
        return self._send(404, b"Not found")

    def _samples(self, since, garbage):
        fake = self.fake
        with fake.lock:
            end = fake.sample_seq
            rows = [r for (seq, r) in fake.samples if seq >= since]
            oldest = fake.samples[0][0] if fake.samples else end
            boot = fake.boot_id
        lost = oldest - since if 0 < since < oldest else 0
        rows = rows[:512]
        nxt = since + len(rows) if since >= oldest else oldest + len(rows)
        if garbage:
            rows = [r if random.random() > 0.3 else r[: len(r) // 2] + ",x,y" for r in rows]
        body = (f"#boot={boot},next={nxt},oldest={oldest},lost={lost},"
                f"now={fake.millis()},fw=fake-2.0\n#cols={','.join(COLS)}\n"
                + "".join(r + "\n" for r in rows))
        assert end >= nxt
        self._send(200, body.encode())

    def _log(self, since):
        fake = self.fake
        with fake.lock:
            end = fake.event_base + len(fake.events)
            oldest = fake.event_base
            start = max(since, oldest)
            stop = min(end, start + 400)
            evs = fake.events[start - oldest: stop - oldest]
            boot = fake.boot_id
        body = (f"#boot={boot},next={stop},oldest={oldest},more={int(stop < end)},"
                f"now={fake.millis()},fw=fake-2.0\n"
                + "".join(f"{e[0]},{e[1]},{e[2]},{e[3]},{e[4]},{e[5]},{e[6]:.2f}\n" for e in evs))
        self._send(200, body.encode())

    def _websocket(self):
        fake = self.fake
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            return self._send(400, b"not a websocket request")
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", wslite.accept_key(key))
        self.end_headers()
        self.wfile.flush()
        sock = self.connection
        sock.settimeout(None)
        fake.ws_socks.add(sock)
        try:
            while True:
                op, payload = wslite.recv_frame(sock)
                if op == wslite.OP_CLOSE:
                    break
                if op == wslite.OP_PING:
                    wslite.send_frame(sock, payload, wslite.OP_PONG)
                    continue
                if op != wslite.OP_TEXT:
                    continue
                self._ws_text(sock, payload.decode(errors="replace"))
        except (wslite.WsClosed, OSError):
            pass
        finally:
            fake.ws_socks.discard(sock)
            self.close_connection = True

    def _ws_text(self, sock, msg):
        fake = self.fake
        if fake.active_chaos() in ("drop", "stall"):
            return
        if msg.startswith("ping,"):
            wslite.send_frame(sock, ("pong," + msg[5:]).encode())
        elif msg.startswith("d,"):
            parts = msg.split(",")
            with fake.lock:
                fake.drive_frames.append((int(parts[1]), int(parts[2]), int(parts[3]),
                                          time.monotonic()))
        elif msg.startswith("stop,"):
            fake.stops.append(msg[5:])
            with fake.lock:
                fake.mode = "M"
        elif msg.startswith("mode,"):
            with fake.lock:
                fake.mode = "A" if msg[5:6] == "A" else "M"
        elif msg.startswith("log,"):
            fake.phone_logs.append(msg[4:])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--chaos", choices=["drop", "stall", "reset", "garbage", "slow"])
    ap.add_argument("--at", type=float, default=10.0, help="chaos starts after this many s")
    ap.add_argument("--for", dest="secs", type=float, default=5.0, help="chaos duration (s)")
    args = ap.parse_args()

    fake = FakeRobot(args.host, args.port).start()
    print(f"Fake robot at {fake.url}  (phone page: {fake.url}/)")
    print(f"Mapper:  python mac_mapper.py --url {fake.url}")
    try:
        if args.chaos:
            time.sleep(args.at)
            print(f"chaos: {args.chaos} for {args.secs} s")
            fake.chaos(args.chaos, args.secs)
        while True:
            time.sleep(5)
            print(f"requests {fake.requests}, ws clients {len(fake.ws_socks)}, "
                  f"drive frames {len(fake.drive_frames)}, samples {fake.sample_seq}")
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()


if __name__ == "__main__":
    main()
