#!/usr/bin/env python3
"""Stress-test the link to the REAL robot (firmware 2.0+) and judge it.

Acts like the phone and the mapper at the same time, for N minutes:
  - phone:  one WebSocket sending drive frames 20x/s with throttle 0 and
            steering 0 (the robot does NOT move), pinging 4x/s for the
            round-trip time, reading the robot's 10 Hz status pushes;
  - mapper: GET /api/samples every 0.2 s and /api/log every 0.5 s over
            keep-alive connections, checking every sample arrives;
  - --clients N: extra idle WebSockets (more phones / browser tabs).

Then reads /api/diag and prints a PASS/FAIL report:
  0 robot resets, 0 lost samples, round trip p95 < 60 ms,
  no gap between robot status messages > 300 ms, no WebSocket drops.

  python tools/link_stress.py                    # 5 minutes at 192.168.1.200
  python tools/link_stress.py --minutes 1 --host 192.168.4.1 --clients 2

Run it once with nothing else connected, then again with the phone page
open and mac_mapper.py running. The report goes to maps/logs/stress_*/.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import requests

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wslite  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


class Phone(threading.Thread):
    """One WebSocket client behaving like the phone page."""

    def __init__(self, host, port, active, stop):
        super().__init__(daemon=True, name="phone" if active else "idle-client")
        self.host, self.port, self.active, self.stop_ev = host, port, active, stop
        self.rtts = []
        self.status_gaps = []
        self.max_status_gap = 0.0
        self.reconnects = 0
        self.errors = []
        self.frames_sent = 0
        self.lock = threading.Lock()

    def run(self):
        while not self.stop_ev.is_set():
            try:
                self._session()
            except Exception as e:      # noqa: BLE001 - any failure is a data point
                self.errors.append(f"{time.strftime('%H:%M:%S')} {type(e).__name__}: {e}")
            if not self.stop_ev.is_set():
                self.reconnects += 1
                time.sleep(0.25)

    def _session(self):
        ws = wslite.WsClient(self.host, self.port, timeout=3.0)
        ws.sock.settimeout(2.0)
        sent = {}
        done = threading.Event()

        def sender():
            seq, ping_id, t_ping = 0, 0, 0.0
            try:
                while not self.stop_ev.is_set() and not done.is_set():
                    now = time.monotonic()
                    if self.active:
                        seq += 1
                        ws.send_text(f"d,{seq},0,0")
                        self.frames_sent += 1
                    if now - t_ping >= 0.25:
                        t_ping = now
                        ping_id += 1
                        sent[ping_id] = now
                        ws.send_text(f"ping,{ping_id}")
                    time.sleep(0.05)
            except OSError:
                done.set()

        threading.Thread(target=sender, daemon=True).start()
        last_status = None
        try:
            while not self.stop_ev.is_set():
                msg = ws.recv_text()       # socket timeout 2 s -> raises
                if msg is None:
                    raise wslite.WsClosed("robot closed the WebSocket")
                now = time.monotonic()
                if msg.startswith("pong,"):
                    t = sent.pop(int(msg[5:]), None)
                    if t is not None:
                        self.rtts.append((now - t) * 1000)
                elif msg.startswith("s,"):
                    if last_status is not None:
                        gap = now - last_status
                        self.max_status_gap = max(self.max_status_gap, gap)
                        if gap > 0.3:
                            self.status_gaps.append(gap)
                    last_status = now
        finally:
            done.set()
            ws.close()


class MapperSim(threading.Thread):
    def __init__(self, base, stop):
        super().__init__(daemon=True, name="mapper")
        self.base, self.stop_ev = base, stop
        self.samples = 0
        self.lost = 0
        self.gaps = 0
        self.errors = []
        self.boots = set()
        self.log_events = 0
        self.latencies = []

    def run(self):
        s_samples, s_log = requests.Session(), requests.Session()
        since, since_log, t_log, boot = 0, 0, 0.0, None
        last_seq = None
        while not self.stop_ev.is_set():
            t0 = time.monotonic()
            try:
                r = s_samples.get(f"{self.base}/api/samples", params={"since": since},
                                  timeout=(1.0, 2.0))
                r.raise_for_status()
                self.latencies.append((time.monotonic() - t0) * 1000)
                hdr = {}
                rows = []
                for line in r.text.splitlines():
                    if line.startswith("#") and not line.startswith("#cols"):
                        hdr.update(kv.split("=", 1) for kv in line[1:].split(",") if "=" in kv)
                    elif line and not line.startswith("#"):
                        rows.append(line)
                if boot is not None and hdr.get("boot") != boot:
                    self.errors.append(f"{time.strftime('%H:%M:%S')} ROBOT REBOOTED")
                    since, last_seq = 0, None
                boot = hdr.get("boot")
                self.boots.add(boot)
                self.lost += int(hdr.get("lost", 0))
                for row in rows:
                    seq = int(row.split(",", 1)[0])
                    if last_seq is not None and seq != last_seq + 1:
                        self.gaps += 1
                    last_seq = seq
                    self.samples += 1
                since = int(hdr.get("next", since))
            except Exception as e:      # noqa: BLE001
                self.errors.append(f"{time.strftime('%H:%M:%S')} samples: {type(e).__name__}: {e}")
                s_samples = requests.Session()
            if time.monotonic() - t_log >= 0.5:
                t_log = time.monotonic()
                try:
                    r = s_log.get(f"{self.base}/api/log", params={"since": since_log},
                                  timeout=(1.0, 2.0))
                    lines = r.text.splitlines()
                    nxt = [kv.split("=")[1] for kv in lines[0][1:].split(",") if kv.startswith("next=")]
                    self.log_events += len(lines) - 1
                    since_log = int(nxt[0]) if nxt else since_log
                except Exception as e:  # noqa: BLE001
                    self.errors.append(f"{time.strftime('%H:%M:%S')} log: {type(e).__name__}: {e}")
                    s_log = requests.Session()
            self.stop_ev.wait(max(0.0, 0.2 - (time.monotonic() - t0)))


def pct(values, p):
    if not values:
        return float("nan")
    v = sorted(values)
    return v[min(len(v) - 1, int(round(p / 100 * (len(v) - 1))))]


def get_diag(base):
    try:
        return requests.get(f"{base}/api/diag", timeout=(1.0, 3.0)).json()
    except Exception as e:      # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.1.200")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--clients", type=int, default=0, help="extra idle WebSocket clients")
    args = ap.parse_args()

    base = f"http://{args.host}:{args.port}" if args.port != 80 else f"http://{args.host}"
    before = get_diag(base)
    if "error" in before:
        sys.exit(f"Robot not reachable at {base}: {before['error']}")
    print(f"Robot {before.get('fw')} at {base}, up {before.get('uptime_ms', 0) / 1000:.0f} s, "
          f"reset reason {before.get('reset_reason')}")
    print(f"Running {args.minutes:g} min: phone at 20 Hz + mapper polling"
          + (f" + {args.clients} extra clients" if args.clients else "") + ". Robot will NOT move.\n")

    stop = threading.Event()
    phone = Phone(args.host, args.port, True, stop)
    idle = [Phone(args.host, args.port, False, stop) for _ in range(args.clients)]
    mapper = MapperSim(base, stop)
    for t in [phone, mapper, *idle]:
        t.start()

    t_end = time.monotonic() + args.minutes * 60
    try:
        while time.monotonic() < t_end:
            time.sleep(min(10.0, max(0.0, t_end - time.monotonic())))
            print(f"{time.strftime('%H:%M:%S')}  rtt p95 {pct(phone.rtts[-200:], 95):5.1f} ms  "
                  f"max {max(phone.rtts[-200:], default=0):6.1f} ms  status gaps>300ms "
                  f"{len(phone.status_gaps)}  ws reconnects {phone.reconnects}  "
                  f"samples {mapper.samples} (lost {mapper.lost}, gaps {mapper.gaps})")
    except KeyboardInterrupt:
        print("stopped early")
    stop.set()
    time.sleep(0.5)
    after = get_diag(base)

    resets = (after.get("boot_id") != before.get("boot_id")) or "error" in after
    rtt95 = pct(phone.rtts, 95)
    checks = [
        ("no robot reset", not resets, f"boot {before.get('boot_id')} -> {after.get('boot_id')}"),
        ("no lost samples", mapper.lost == 0 and mapper.gaps == 0,
         f"lost {mapper.lost}, sequence gaps {mapper.gaps}"),
        ("round trip p95 < 60 ms", rtt95 < 60, f"p50 {pct(phone.rtts, 50):.1f}, p95 {rtt95:.1f}, "
                                               f"max {max(phone.rtts, default=0):.1f} ms"),
        ("no status gap > 300 ms", not phone.status_gaps,
         f"{len(phone.status_gaps)} gaps, worst {phone.max_status_gap * 1000:.0f} ms"),
        ("no WebSocket drops", phone.reconnects == 0, f"{phone.reconnects} reconnects"),
        ("no HTTP errors", not mapper.errors, f"{len(mapper.errors)} errors"),
        ("robot dropped no commands", after.get("cmd_dropped", 0) == 0,
         f"{after.get('cmd_dropped')} dropped"),
    ]
    lines = [f"AUTOBOT link stress test {datetime.now().isoformat(timespec='seconds')}",
             f"robot {base}, fw {before.get('fw')}, {args.minutes:g} min, "
             f"{1 + args.clients} WebSocket client(s)", ""]
    for name, ok, detail in checks:
        lines.append(f"  [{'PASS' if ok else 'FAIL'}] {name:28s} {detail}")
    lines += ["",
              f"  phone drive frames sent {phone.frames_sent}, pings answered {len(phone.rtts)}",
              f"  samples received {mapper.samples} "
              f"(~{mapper.samples / max(1, args.minutes * 60):.1f}/s), "
              f"/api/samples p95 {pct(mapper.latencies, 95):.0f} ms, log events {mapper.log_events}",
              f"  robot: slowest loop pass {after.get('loop_max_us_boot', 0) / 1000:.1f} ms, "
              f"min free heap {after.get('min_free_heap')} B, Wi-Fi drops "
              f"{after.get('wifi_disconnects')} (last reason {after.get('wifi_last_reason')}), "
              f"RSSI {after.get('rssi')} dBm",
              f"  robot stack free: loop {after.get('loop_stack_free')} B, "
              f"server {after.get('httpd_stack_free')} B"]
    errs = phone.errors + mapper.errors + [e for c in idle for e in c.errors]
    if errs:
        lines += ["", "  errors (first 20):"] + [f"    {e}" for e in errs[:20]]
    if resets and after.get("boots"):
        lines += ["", "  robot boot history:"] + [
            f"    #{b['boot_no']} {b['reason']}: {' | '.join(b['events_before_reset'][-6:])}"
            for b in after["boots"]]
    verdict = all(ok for _, ok, _ in checks)
    lines += ["", "RESULT: " + ("PASS" if verdict else "FAIL")]
    report = "\n".join(lines)
    print("\n" + report)

    out = ROOT / "maps" / "logs" / f"stress_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.txt").write_text(report + "\n")
    (out / "diag_before.json").write_text(json.dumps(before, indent=2))
    (out / "diag_after.json").write_text(json.dumps(after, indent=2))
    (out / "rtt_ms.json").write_text(json.dumps([round(r, 2) for r in phone.rtts]))
    print(f"\nSaved to {out}/")
    sys.exit(0 if verdict else 1)


if __name__ == "__main__":
    main()
