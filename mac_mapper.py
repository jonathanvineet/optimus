#!/usr/bin/env python3
"""
AUTOBOT live 2D occupancy-grid mapper (macOS, Python 3).

Reads every 25 Hz sample from the ESP32 (/api/samples, firmware 2.0+; older
firmware is polled at /api/telemetry) while you drive the robot with the
phone controller, and builds an approximate occupancy grid from the HC-SR04
range and the MPU6050 gyro heading.

Every run writes a session folder maps/logs/<YYYYmmdd_HHMMSS>/ with
mapper.log (every link error, outage, reconnect, robot reboot reason,
thread crash and stall), telemetry.jsonl, robotlog.csv, diag_*.json and
summary.txt.

READ-ONLY: this program never sends motor commands.

What is MEASURED vs INFERRED
----------------------------
  measured : ultrasonic range (cm), gyro Z rate (deg/s), the PWM commands the
             firmware applied to each motor, ESP32 timestamps.
  inferred : heading (integrated gyro; drifts, no magnetometer, relative to
             the start pose), position (dead reckoning from motor COMMANDS -
             there are no wheel encoders, so wheel slip, stalls, battery sag
             and floor type are all invisible), the occupancy grid.

This is a baseline mapper with pose-uncertainty tracking. It does NOT do
scan matching, loop closure or any global correction. Errors accumulate.

Usage
-----
  python mac_mapper.py                 # live, connect to the robot
  python mac_mapper.py --simulate      # no hardware: simulated room
  python mac_mapper.py --selftest      # headless algorithm checks

Keys (map window): space/p = pause/resume map updates, r = reset map + pose,
s = save snapshot, q/Esc = quit.
"""

from __future__ import annotations

import argparse
import csv
import faulthandler
import json
import logging
import math
import os
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import requests

# =============================================================================
# USER CONFIGURATION - edit values in this section only
# =============================================================================

# --- Connection --------------------------------------------------------------
# Tried in order: the robot's fixed home-network address, then the robot's
# own BotController AP (only used if secrets.h has no home Wi-Fi). While the
# link is down the mapper keeps re-trying all of them.
ROBOT_HOSTS = ["192.168.1.200", "192.168.4.1"]
TELEMETRY_URL = "http://192.168.1.200/api/telemetry"  # used if discovery fails
POLL_HZ = 8.0                    # only for pre-2.0 firmware (/api/telemetry polling)
SAMPLES_POLL_S = 0.2             # /api/samples: fetch everything new every 0.2 s
HTTP_CONNECT_TIMEOUT_S = 0.8
HTTP_READ_TIMEOUT_S = 1.5
STALE_AFTER_S = 1.5              # no fresh telemetry for this long -> "STALE"
REDISCOVER_AFTER_S = 3.0         # link down this long -> look for the robot again
STILL_DOWN_LOG_S = 5.0           # while down, log a "still down" line this often
STALL_DUMP_AFTER_S = 5.0         # a thread silent this long -> dump all stacks

# --- Heading (gyro Z integrated on the ESP32) --------------------------------
# +1: firmware yaw increases when the robot turns LEFT (counter-clockwise seen
# from above). That is the case when the MPU6050 is mounted chip-side UP.
# If turning left makes yaw_deg DECREASE, set this to -1.0.
YAW_SIGN = +1.0
GYRO_RANDOM_WALK_DEG_PER_SQRT_S = 0.05   # heading noise growth (1 sigma)
GYRO_SCALE_ERROR = 0.02                  # 2 % scale-factor uncertainty per turn

# --- Approximate motion model (NO wheel encoders) ----------------------------
# Wheel speed is guessed from the applied PWM:
#   |pwm| <= PWM_DEADBAND               -> wheel assumed stopped
#   otherwise speed = MAX_SPEED_MPS * (|pwm| - DEADBAND) / (255 - DEADBAND)
# Forward motion = mean of the two signed wheel speeds (spin-in-place -> 0).
# Heading comes from the gyro, never from the command difference.
MAX_SPEED_MPS = 0.40             # straight-line speed at PWM 255   <-- CALIBRATE
PWM_DEADBAND = 70                # PWM where wheels start turning    <-- CALIBRATE
LEFT_SPEED_SCALE = 1.00          # extra per-side correction (firmware trim already applied)
RIGHT_SPEED_SCALE = 1.00
REVERSE_SPEED_SCALE = 1.00       # reverse speed relative to forward
MOTION_REL_SIGMA = 0.25          # 1-sigma relative error of each distance step
MOTION_ABS_SIGMA_MPS = 0.02      # extra 1-sigma speed error while moving
# Stall detection: commanded forward, but the measured range to whatever is
# ahead has not shrunk by STALL_MIN_CHANGE_M in STALL_WINDOW_S -> the robot is
# assumed NOT to be moving (pushing an obstacle, wheels spinning).
STALL_WINDOW_S = 1.5
STALL_MIN_CHANGE_M = 0.03

# --- HC-SR04 ultrasonic sensor -----------------------------------------------
SENSOR_OFFSET_X_M = 0.08         # sensor ahead of the wheel axle centre
SENSOR_OFFSET_Y_M = 0.00         # sensor left (+) of centre line
MIN_RANGE_M = 0.05
MAX_MAPPING_RANGE_M = 2.0        # longer readings: free space up to here, no obstacle mark
MAX_US_AGE_S = 0.25              # older range samples are not used for mapping
MAX_TURN_RATE_FOR_MAPPING_DPS = 45.0   # skip ranges while spinning fast
MEDIAN_WINDOW = 3                # median of last N compatible readings (1 = off)
MEDIAN_MAX_SPAN_S = 0.6
MEDIAN_MAX_YAW_SPREAD_DEG = 5.0
MEDIAN_MAX_POS_SPREAD_M = 0.05
# A missing echo means "nothing reflected back", which can be open space OR a
# soft / angled surface. 0.0 = missing echoes never clear cells (safe default).
NO_ECHO_FREE_RANGE_M = 0.0

# --- Beam model (HC-SR04 has a broad cone, roughly +/-15 degrees) -----------
BEAM_HALF_ANGLE_OCC_DEG = 15.0   # obstacle could be anywhere on this arc
BEAM_HALF_ANGLE_FREE_DEG = 10.0  # clear only the more reliable inner cone
OBSTACLE_THICKNESS_M = 0.08      # radial width of the "occupied" band
FREE_FAR_WEIGHT = 0.4            # free-space evidence at max range (1.0 at sensor)

# --- Occupancy grid ----------------------------------------------------------
RESOLUTION_M = 0.05              # 5 cm cells
INITIAL_MAP_SIZE_M = 6.0         # square, centred on the start pose
MAX_MAP_SIZE_M = 20.0            # the grid grows up to this; beyond is dropped
GROW_STEP_M = 2.0
L_OCC = 0.85                     # log-odds added at the beam arc centre
L_FREE = -0.30                   # log-odds added for free cells
L_MIN, L_MAX = -4.0, 4.0         # clamp: p in [0.018, 0.982], never "certain"
P_OCC_THRESHOLD = 0.65
P_FREE_THRESHOLD = 0.35
POSE_SIGMA_REF_M = 0.30          # map updates are down-weighted as pose sigma grows
MIN_UPDATE_WEIGHT = 0.2
STATIONARY_REPEAT_WEIGHT = 0.25  # same pose + same range again -> weaker update

# --- Display / output --------------------------------------------------------
GUI_REFRESH_MS = 150
HIT_POINTS_MAX = 3000
OUTPUT_DIR = "maps"              # snapshots go to maps/YYYYmmdd_HHMMSS/
AUTOSAVE_ON_EXIT = False
RECORD_TELEMETRY = True          # raw telemetry -> maps/logs/*.jsonl (for --replay)
SENSOR_BLIND_WARN_S = 3.0        # banner if no valid echo for this long
ROBOT_LOG = True                 # download the robot's full event log (/api/log)
ROBOT_LOG_POLL_S = 0.5
KEEP_MAC_AWAKE = True            # run `caffeinate` while mapping (macOS)

# =============================================================================
# END OF USER CONFIGURATION
# =============================================================================

MILLIS_MOD = 2 ** 32
MAX_GAP_S = 1.0


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_180(a):
    return (a + 180.0) % 360.0 - 180.0


# =============================================================================
# 0. SESSION LOGGING (everything that goes wrong ends up in mapper.log)
# =============================================================================

log = logging.getLogger("autobot")
log.addHandler(logging.NullHandler())
_fault_file = None


def start_session_logging(kind="map"):
    """Create maps/logs/<stamp>/ and send all logging there. Returns the dir."""
    global _fault_file
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    d = Path(OUTPUT_DIR) / "logs" / stamp
    d.mkdir(parents=True, exist_ok=True)

    fh = logging.FileHandler(d / "mapper.log")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s [%(threadName)s] %(message)s",
        "%Y-%m-%d %H:%M:%S"))
    ch = logging.StreamHandler()
    ch.setLevel(logging.WARNING)
    ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S"))
    log.setLevel(logging.DEBUG)
    log.addHandler(fh)
    log.addHandler(ch)

    # Hard crashes (segfault in a C extension, fatal errors) land here.
    _fault_file = open(d / "faults.log", "a")
    faulthandler.enable(_fault_file, all_threads=True)

    def excepthook(t, v, tb):
        log.critical("uncaught exception - mapper is exiting", exc_info=(t, v, tb))

    def thread_excepthook(args):
        log.critical("thread %s DIED", args.thread.name if args.thread else "?",
                     exc_info=(args.exc_type, args.exc_value, args.exc_traceback))

    sys.excepthook = excepthook
    threading.excepthook = thread_excepthook

    log.info("session %s started (%s): python %s, numpy %s, requests %s, argv %s",
             stamp, kind, sys.version.split()[0], np.__version__, requests.__version__,
             " ".join(sys.argv))
    print(f"Session logs: {d}/")
    return d


def describe_exc(e, url=""):
    """One line naming the error class and message, for the log."""
    text = f"{type(e).__name__}: {e}"
    return f"{text} [{url}]" if url else text


def base_url(url):
    """http://host/api/telemetry -> http://host ; http://host -> http://host"""
    url = url.rstrip("/")
    return url.split("/api/", 1)[0] if "/api/" in url else url


# =============================================================================
# 1. TELEMETRY ACQUISITION AND PARSING
# =============================================================================

REQUIRED_FIELDS = ("millis", "distance_cm", "yaw_deg", "gyro_z_dps", "imu_ok",
                   "left_cmd", "right_cmd", "mode")


@dataclass
class Telemetry:
    recv_time: float              # Mac time.monotonic() at receipt
    wall_time: float              # Mac time.time() at receipt
    latency_s: float
    millis: int                   # ESP32 millis() (wraps at 2^32)
    boot_id: Optional[int]
    mode: str
    distance_m: Optional[float]   # None = no valid echo
    us_status: str
    us_seq: Optional[int]
    us_age_s: Optional[float]
    imu_ok: bool
    yaw_deg: float
    yaw_unwrapped_deg: Optional[float]
    gyro_z_dps: float
    gyro_z_raw_dps: Optional[float]
    gyro_bias_dps: Optional[float]
    gyro_cal: str
    left_cmd: float
    right_cmd: float
    left_cmd_int: Optional[float]
    right_cmd_int: Optional[float]
    raw: dict


def _finite(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def parse_telemetry(obj, recv_time, wall_time, latency_s=0.0):
    """Validate one telemetry JSON object. Returns (Telemetry | None, error)."""
    if not isinstance(obj, dict):
        return None, "response is not a JSON object"

    missing = [k for k in REQUIRED_FIELDS if k not in obj]
    if missing:
        return None, "missing fields: " + ", ".join(missing)

    millis = _finite(obj["millis"])
    yaw = _finite(obj["yaw_deg"])
    gz = _finite(obj["gyro_z_dps"])
    lc = _finite(obj["left_cmd"])
    rc = _finite(obj["right_cmd"])
    if None in (millis, yaw, gz, lc, rc) or millis < 0:
        return None, "non-numeric core field"

    # Range: explicitly invalid unless the value is a sane positive number
    # and the firmware did not flag it as invalid.
    dist_cm = _finite(obj.get("distance_cm"))
    flagged_valid = obj.get("distance_valid", True) is True
    status = str(obj.get("us_status", "ok" if dist_cm is not None else "invalid"))
    distance_m = None
    if dist_cm is not None and flagged_valid and 0.0 < dist_cm <= 450.0:
        distance_m = dist_cm / 100.0
    elif dist_cm is not None and status == "ok":
        status = "rejected"

    us_seq = _finite(obj.get("us_seq"))
    us_age = _finite(obj.get("us_age_ms"))
    boot = _finite(obj.get("boot_id"))

    return Telemetry(
        recv_time=recv_time,
        wall_time=wall_time,
        latency_s=latency_s,
        millis=int(millis) % MILLIS_MOD,
        boot_id=int(boot) if boot is not None else None,
        mode=str(obj["mode"]),
        distance_m=distance_m,
        us_status=status,
        us_seq=int(us_seq) if us_seq is not None else None,
        us_age_s=us_age / 1000.0 if us_age is not None else None,
        imu_ok=obj["imu_ok"] is True,
        yaw_deg=yaw,
        yaw_unwrapped_deg=_finite(obj.get("yaw_unwrapped_deg")),
        gyro_z_dps=gz,
        gyro_z_raw_dps=_finite(obj.get("gyro_z_raw_dps")),
        gyro_bias_dps=_finite(obj.get("gyro_bias_dps")),
        gyro_cal=str(obj.get("gyro_cal", "unknown")),
        left_cmd=lc,
        right_cmd=rc,
        left_cmd_int=_finite(obj.get("left_cmd_int")),
        right_cmd_int=_finite(obj.get("right_cmd_int")),
        raw=obj,
    ), ""


class SourceStatus:
    """Thread-safe connection bookkeeping shared with the GUI."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ok_count = 0
        self.err_count = 0
        self.consecutive_errors = 0
        self.last_ok_time = None
        self.last_error = ""
        self.latency_s = None

    def ok(self, latency):
        with self.lock:
            self.ok_count += 1
            self.consecutive_errors = 0
            self.last_ok_time = time.monotonic()
            self.latency_s = latency

    def error(self, msg):
        with self.lock:
            self.err_count += 1
            self.consecutive_errors += 1
            self.last_error = msg

    def state(self):
        with self.lock:
            now = time.monotonic()
            if self.last_ok_time is None:
                s = "CONNECTING" if self.consecutive_errors < 3 else "DISCONNECTED"
            elif now - self.last_ok_time > STALE_AFTER_S:
                s = "DISCONNECTED" if self.consecutive_errors >= 3 else "STALE"
            else:
                s = "CONNECTED"
            return {
                "state": s,
                "ok": self.ok_count,
                "errors": self.err_count,
                "last_error": self.last_error,
                "latency_s": self.latency_s,
                "age_s": None if self.last_ok_time is None else now - self.last_ok_time,
            }


def parse_samples_response(text):
    """Parse /api/samples. Returns (header dict, column names, row strings)."""
    header, cols, rows = {}, [], []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#cols="):
            cols = line[6:].split(",")
        elif line.startswith("#"):
            for kv in line[1:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    header[k] = v
        else:
            rows.append(line)
    return header, cols, rows


def sample_row_to_dict(cols, row, boot_id):
    """One /api/samples CSV row -> the same dict /api/telemetry returns."""
    vals = row.split(",")
    if len(vals) != len(cols):
        return None
    d = dict(zip(cols, vals))
    try:
        yaw_u = float(d["yaw_unwrapped_deg"])
        dist = d["distance_cm"]
        return {
            "boot_id": int(boot_id) if boot_id else None,
            "sample_seq": int(d["seq"]),
            "millis": int(d["millis"]),
            "mode": "auto" if d["mode"] == "A" else "manual",
            "distance_cm": float(dist) if dist else None,
            "distance_valid": bool(dist),
            "us_status": d["us_status"],
            "us_seq": int(d["us_seq"]),
            "us_age_ms": int(d["us_age_ms"]),
            "imu_ok": d["imu_ok"] == "1",
            "yaw_deg": wrap_180(yaw_u),
            "yaw_unwrapped_deg": yaw_u,
            "roll_deg": float(d["roll_deg"]),
            "pitch_deg": float(d["pitch_deg"]),
            "gyro_z_dps": float(d["gyro_z_dps"]),
            "gyro_z_raw_dps": float(d["gyro_z_raw_dps"]),
            "gyro_bias_dps": float(d["gyro_bias_dps"]),
            "gyro_cal": d["gyro_cal"],
            "left_cmd": int(d["left_cmd"]),
            "right_cmd": int(d["right_cmd"]),
            "left_cmd_int": float(d["left_cmd_int"]),
            "right_cmd_int": float(d["right_cmd_int"]),
        }
    except (KeyError, ValueError):
        return None


class TelemetryClient:
    """The robot link, in a background thread; never blocks the GUI.

    Firmware 2.0+: fetches every 25 Hz sample from /api/samples over ONE
    keep-alive connection, so nothing is lost even when Wi-Fi stutters (the
    robot keeps ~20 s). Older firmware: polls /api/telemetry. Every error,
    outage, reconnect and robot reboot goes to mapper.log; while the link is
    down, all ROBOT_HOSTS are tried again. Nothing in here can kill the thread.
    """

    def __init__(self, url=TELEMETRY_URL, poll_hz=POLL_HZ, session_dir=None):
        self.base = base_url(url)
        self.legacy_period = 1.0 / max(0.5, poll_hz)
        self.samples = queue.Queue(maxsize=20000)
        self.status = SourceStatus()
        self.session_dir = Path(session_dir) if session_dir else None
        self.mode = None                 # "samples" | "telemetry"
        self.since = 0
        self.boot = None
        self.received = 0
        self.lost_samples = 0
        self.dropped_local = 0
        self.bad_rows = 0
        self.outages = []                # durations (s)
        self.reboots = 0
        self.ever_up = False
        self.down_since = None
        self.errors_in_outage = 0
        self.last_still_down_log = 0.0
        self.last_discovery = 0.0
        self.want_diag = None            # reason string -> fetch /api/diag
        self.beat = time.monotonic()     # for the stall watchdog
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="link", daemon=True)

    @property
    def url(self):
        return self.base

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                log.error("link thread did not stop within 3 s")

    def summary(self):
        return (f"samples received {self.received}, lost on robot {self.lost_samples}, "
                f"dropped on Mac {self.dropped_local}, bad rows {self.bad_rows}, "
                f"outages {len(self.outages)} (total {sum(self.outages):.1f} s, "
                f"longest {max(self.outages, default=0):.1f} s), robot reboots {self.reboots}")

    @staticmethod
    def _new_session():
        # Keep-alive: one TCP connection for the whole run. (The 2.0 firmware
        # serves several clients at once; the phone is not starved.)
        return requests.Session()

    def _run(self):
        log.info("link thread started, robot %s", self.base)
        session = self._new_session()
        backoff = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.beat = t0
            try:
                if self.mode == "telemetry":
                    self._poll_telemetry(session)
                    period = self.legacy_period
                    self._link_ok(time.monotonic() - t0)
                else:
                    period = SAMPLES_POLL_S
                    if self._poll_samples(session):
                        self._link_ok(time.monotonic() - t0)
                    else:
                        period = 0.0       # switched to /api/telemetry: poll now
                backoff = 0.0
                if self.want_diag:
                    self._fetch_diag(session, self.want_diag)
                    self.want_diag = None
            except Exception as e:      # anything: log it, back off, carry on
                self._link_error(e)
                session.close()
                session = self._new_session()
                # The robot answering garbage is less likely to be overloaded
                # than one that does not answer at all.
                backoff = min(1.0, backoff + (0.1 if isinstance(e, ValueError) else 0.2))
                period = self.legacy_period if self.mode == "telemetry" else SAMPLES_POLL_S
                if time.monotonic() - (self.down_since or t0) >= REDISCOVER_AFTER_S:
                    self._rediscover()
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)) + backoff)
        session.close()
        log.info("link thread stopped: %s", self.summary())

    def _get(self, session, path, **kw):
        url = self.base + path
        try:
            return session.get(url, timeout=(HTTP_CONNECT_TIMEOUT_S, HTTP_READ_TIMEOUT_S), **kw)
        except requests.exceptions.RequestException as e:
            raise ConnectionError(describe_exc(e, url)) from e

    def _poll_samples(self, session):
        r = self._get(session, "/api/samples", params={"since": self.since})
        if r.status_code == 404:
            log.warning("robot firmware has no /api/samples (older than 2.0): "
                        "polling /api/telemetry instead - upload the new firmware")
            self.mode = "telemetry"
            return False
        if r.status_code != 200:
            raise ValueError(f"HTTP {r.status_code} from /api/samples: {r.text[:80]!r}")
        hdr, cols, rows = parse_samples_response(r.text)
        if "next" not in hdr or not cols:
            raise ValueError(f"bad /api/samples reply: {r.text[:80]!r}")
        self.mode = "samples"

        boot = hdr.get("boot")
        if self.boot is not None and boot != self.boot:
            self.reboots += 1
            log.warning("ROBOT REBOOTED (boot id %s -> %s) - reading its reset reason",
                        self.boot, boot)
            self.boot, self.since = boot, 0
            self.want_diag = "reboot"
            return True
        self.boot = boot

        lost = int(hdr.get("lost", 0))
        if lost:
            self.lost_samples += lost
            log.warning("LOST %d samples (link was down longer than the robot's "
                        "~20 s buffer)", lost)

        now_ms = int(hdr.get("now", 0))
        mono, wall = time.monotonic(), time.time()
        latency = r.elapsed.total_seconds()
        for row in rows:
            obj = sample_row_to_dict(cols, row, boot)
            if obj is None:
                self.bad_rows += 1
                log.debug("bad sample row: %r", row[:120])
                continue
            # Arrival time reconstructed from the robot clock (keeps spacing).
            age = ((now_ms - obj["millis"]) % MILLIS_MOD) / 1000.0
            if age > 60.0:
                age = 0.0
            s, err = parse_telemetry(obj, mono - age, wall - age, latency)
            if s is None:
                self.bad_rows += 1
                log.debug("sample rejected (%s): %r", err, row[:120])
                continue
            self._put(s)
            self.received += 1
        self.since = int(hdr["next"])
        return True

    def _poll_telemetry(self, session):
        r = self._get(session, "/api/telemetry")
        latency = r.elapsed.total_seconds()
        if r.status_code != 200:
            raise ValueError(f"HTTP {r.status_code} from /api/telemetry")
        if not r.content:
            raise ValueError("empty response")
        sample, err = parse_telemetry(r.json(), time.monotonic(), time.time(), latency)
        if sample is None:
            raise ValueError(err)
        if self.boot is not None and sample.boot_id is not None and str(sample.boot_id) != self.boot:
            self.reboots += 1
            log.warning("ROBOT REBOOTED (boot id %s -> %s)", self.boot, sample.boot_id)
        self.boot = str(sample.boot_id) if sample.boot_id is not None else None
        self._put(sample)
        self.received += 1

    def _link_ok(self, latency):
        now = time.monotonic()
        if self.down_since is not None:
            dur = now - self.down_since
            self.outages.append(dur)
            log.warning("link UP again after %.1f s down (%d errors), robot %s",
                        dur, self.errors_in_outage, self.base)
            self.down_since = None
            self.errors_in_outage = 0
            self.want_diag = self.want_diag or "reconnect"
        elif not self.ever_up:
            log.info("link UP, robot %s, %s mode", self.base, self.mode)
            self.want_diag = self.want_diag or "start"
        self.ever_up = True
        self.status.ok(latency)

    def _link_error(self, e):
        msg = str(e) if isinstance(e, ConnectionError) else describe_exc(e, self.base)
        self.status.error(msg[:120])
        now = time.monotonic()
        if self.down_since is None:
            self.down_since = now
            self.errors_in_outage = 0
            self.last_still_down_log = now
            log.warning("link DOWN: %s", msg)
        else:
            log.debug("link error: %s", msg)
            if now - self.last_still_down_log >= STILL_DOWN_LOG_S:
                self.last_still_down_log = now
                log.warning("link still DOWN after %.0f s (%d errors); last: %s",
                            now - self.down_since, self.errors_in_outage, msg)
        self.errors_in_outage += 1
        if not isinstance(e, (ConnectionError, ValueError)):
            log.error("unexpected error in the link thread", exc_info=e)

    def _rediscover(self):
        now = time.monotonic()
        if now - self.last_discovery < REDISCOVER_AFTER_S:
            return
        self.last_discovery = now
        current = self.base.split("//", 1)[-1]
        hosts = [current] + [h for h in ROBOT_HOSTS if h != current]
        for host in hosts:
            if self._stop.is_set():
                return
            url = f"http://{host}"
            try:
                r = requests.get(url + "/api/telemetry", timeout=(0.5, 0.8))
                fw = r.json().get("fw", "?")
            except (requests.exceptions.RequestException, ValueError) as e:
                log.debug("rediscovery: %s no answer (%s)", host, describe_exc(e))
                continue
            if url != self.base:
                log.warning("robot found at %s (%s) - switching from %s", url, fw, self.base)
                self.base = url
                self.since = 0
                self.boot = None
            return
        log.debug("rediscovery: robot not found at %s", ", ".join(hosts))

    def _fetch_diag(self, session, why):
        try:
            r = self._get(session, "/api/diag")
        except ConnectionError as e:
            log.warning("could not read /api/diag after %s: %s", why, e)
            return
        if r.status_code == 404:
            log.info("robot has no /api/diag (firmware older than 2.0)")
            return
        try:
            d = r.json()
        except ValueError:
            log.warning("bad /api/diag reply: %r", r.text[:120])
            return
        if self.session_dir:
            path = self.session_dir / f"diag_{datetime.now().strftime('%H%M%S')}_{why}.json"
            path.write_text(json.dumps(d, indent=2))
        boots = d.get("boots", [])
        # Routine at start; worth shouting about after a reboot or outage.
        lvl = logging.INFO if why == "start" else logging.WARNING
        log.log(lvl, "robot diag (%s): fw %s, this boot reset reason %s, up %.0f s, "
                    "min free heap %s B, slowest loop pass %.1f ms, Wi-Fi drops %s "
                    "(last reason %s), WS clients %s",
                    why, d.get("fw"), d.get("reset_reason"), d.get("uptime_ms", 0) / 1000,
                    d.get("min_free_heap"), d.get("loop_max_us_boot", 0) / 1000,
                    d.get("wifi_disconnects"), wifi_reason_name(d.get("wifi_last_reason")),
                    d.get("ws_clients"))
        for b in boots:
            log.log(lvl, "  boot #%s reason %-12s fw %s; previous boot ran %.1f s; "
                        "last events before that reset: %s",
                        b.get("boot_no"), b.get("reason"), b.get("fw"),
                        b.get("prev_boot_ran_ms", 0) / 1000,
                        " | ".join(b.get("events_before_reset", [])[-6:]) or "-")

    def _put(self, sample):
        try:
            self.samples.put_nowait(sample)
        except queue.Full:
            self.dropped_local += 1
            if self.dropped_local in (1, 10, 100) or self.dropped_local % 1000 == 0:
                log.error("mapper worker is not keeping up: %d samples dropped on the Mac",
                          self.dropped_local)


# Station disconnect reason codes (esp_wifi_types.h).
WIFI_REASONS = {
    1: "unspecified", 2: "auth expired", 3: "auth leave (AP kicked us)",
    4: "assoc expired (idle)", 5: "AP too many clients", 6: "not authed",
    7: "not associated", 8: "assoc leave", 15: "4-way handshake timeout (password?)",
    23: "802.1x auth failed", 200: "beacon timeout (weak signal / AP gone)",
    201: "no AP found (SSID / 2.4 GHz?)", 202: "auth fail (password)",
    203: "assoc fail", 204: "handshake timeout", 205: "connection fail",
    206: "AP TSF reset", 207: "roaming", 208: "assoc comeback too long",
}


def wifi_reason_name(code):
    if code in (None, 0):
        return "-"
    return f"{code} {WIFI_REASONS.get(int(code), '')}".strip()


# =============================================================================
# 2. SENSOR VALIDATION AND FILTERING
# =============================================================================

class RangeFilter:
    """Median of the last few readings taken from (nearly) the same pose.

    A median of 3 removes single-sample spikes (multipath, crosstalk) but also
    delays a genuine change - a person stepping into the beam - by one sample
    and can hide a one-sample-wide object. Only readings taken within a short
    time, small heading change and small displacement are combined, so the
    filter never mixes two different viewing directions.
    """

    def __init__(self, window=MEDIAN_WINDOW):
        self.buf = deque(maxlen=max(1, window))

    def reset(self):
        self.buf.clear()

    def add(self, t, r, x, y, theta):
        self.buf.append((t, r, x, y, theta))
        group = [r]
        for (ti, ri, xi, yi, thi) in list(self.buf)[:-1]:
            if (t - ti <= MEDIAN_MAX_SPAN_S
                    and abs(math.degrees(wrap_pi(thi - theta))) <= MEDIAN_MAX_YAW_SPREAD_DEG
                    and math.hypot(xi - x, yi - y) <= MEDIAN_MAX_POS_SPREAD_M):
                group.append(ri)
        return float(np.median(group)), len(group)


# =============================================================================
# 3. GYRO HEADING
# =============================================================================

class HeadingEstimator:
    """Relative heading from the firmware's integrated gyro yaw.

    Works on yaw DIFFERENCES only, so a firmware reboot, a map reset, or yaw
    wrapping at +/-180 never causes a heading jump. theta = 0 is the heading at
    start/reset. Counter-clockwise (left turn) is positive.
    """

    def __init__(self):
        self.prev_yaw = None
        self.prev_unwrapped = None

    def rebase(self):
        """Forget the reference (after a reboot); next sample becomes it."""
        self.prev_yaw = None
        self.prev_unwrapped = None

    def delta(self, s: Telemetry):
        """Heading change in radians since the previous sample."""
        if self.prev_yaw is None:
            d_deg = 0.0
        elif s.yaw_unwrapped_deg is not None and self.prev_unwrapped is not None:
            d_deg = s.yaw_unwrapped_deg - self.prev_unwrapped
        else:
            d_deg = wrap_180(s.yaw_deg - self.prev_yaw)
        self.prev_yaw = s.yaw_deg
        self.prev_unwrapped = s.yaw_unwrapped_deg
        return math.radians(YAW_SIGN * d_deg)

    @staticmethod
    def variance_increment(dtheta, dt, imu_ok):
        if not imu_ok:
            # Heading is frozen on the robot while the IMU is down, but the
            # robot may still turn: grow the uncertainty fast.
            return math.radians(30.0) ** 2 * dt
        rw = math.radians(GYRO_RANDOM_WALK_DEG_PER_SQRT_S) ** 2 * dt
        return rw + (GYRO_SCALE_ERROR * dtheta) ** 2


# =============================================================================
# 4. MOTION PREDICTION (command-based dead reckoning)
# =============================================================================

class CommandMotionModel:
    """Forward distance guessed from motor PWM. NOT a measurement.

    The firmware exports the time integral of each applied PWM, so the
    average command between two polls is exact even if the joystick moved in
    between. Turning that into metres still relies on the calibration
    constants above and is wrong whenever wheels slip, stall or the battery
    sags.
    """

    @staticmethod
    def wheel_speed(pwm, scale):
        a = abs(pwm)
        if a <= PWM_DEADBAND:
            return 0.0
        frac = min(1.0, (a - PWM_DEADBAND) / max(1.0, 255.0 - PWM_DEADBAND))
        v = MAX_SPEED_MPS * frac * scale
        return v if pwm > 0 else -v * REVERSE_SPEED_SCALE

    def forward_step(self, avg_left, avg_right, dt):
        """Returns (distance_m, sigma_m) for one interval."""
        vl = self.wheel_speed(avg_left, LEFT_SPEED_SCALE)
        vr = self.wheel_speed(avg_right, RIGHT_SPEED_SCALE)
        d = 0.5 * (vl + vr) * dt
        moving = vl != 0.0 or vr != 0.0
        sigma = MOTION_REL_SIGMA * abs(d) + (MOTION_ABS_SIGMA_MPS * dt if moving else 0.0)
        return d, sigma


class PoseEstimator:
    """Planar pose (x, y, theta) with a 3x3 covariance (EKF-style prediction)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.P = np.zeros((3, 3))
        self.odometer_m = 0.0

    def predict(self, d, sigma_d, dtheta, var_theta):
        mid = self.theta + 0.5 * dtheta
        c, s = math.cos(mid), math.sin(mid)
        self.x += d * c
        self.y += d * s
        self.theta = wrap_pi(self.theta + dtheta)
        self.odometer_m += d

        F = np.array([[1.0, 0.0, -d * s],
                      [0.0, 1.0, d * c],
                      [0.0, 0.0, 1.0]])
        G = np.array([[c], [s], [0.0]])
        Q = (G @ G.T) * sigma_d ** 2
        Q[2, 2] += var_theta
        self.P = F @ self.P @ F.T + Q

    def inflate(self, pos_sigma, theta_sigma):
        self.P[0, 0] += pos_sigma ** 2
        self.P[1, 1] += pos_sigma ** 2
        self.P[2, 2] += theta_sigma ** 2

    def sigma_xy(self):
        """1-sigma radius along the worst direction."""
        ev = np.linalg.eigvalsh(self.P[:2, :2])
        return float(math.sqrt(max(0.0, ev[-1])))

    def sigma_theta(self):
        return float(math.sqrt(max(0.0, self.P[2, 2])))

    def sensor_pose(self):
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (self.x + c * SENSOR_OFFSET_X_M - s * SENSOR_OFFSET_Y_M,
                self.y + s * SENSOR_OFFSET_X_M + c * SENSOR_OFFSET_Y_M,
                self.theta)


class SpeedCalibrationHelper:
    """Compares ultrasonic range change with estimated forward distance.

    Drive straight at a wall: the change in measured range is a real distance
    measurement, the odometer is the model's guess. Their ratio suggests a
    corrected MAX_SPEED_MPS.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.start = None     # (range, odometer, theta)
        self.result = None

    def update(self, r, pose: PoseEstimator):
        if r is None:
            return
        if self.start is None:
            self.start = (r, pose.odometer_m, pose.theta)
            return
        r0, odo0, th0 = self.start
        if abs(math.degrees(wrap_pi(pose.theta - th0))) > 5.0:
            self.start = (r, pose.odometer_m, pose.theta)   # not straight; restart
            self.result = None
            return
        measured = r0 - r
        estimated = pose.odometer_m - odo0
        if estimated > 0.3 and measured > 0.1:
            self.result = (measured, estimated, MAX_SPEED_MPS * measured / estimated)


# =============================================================================
# 5. OCCUPANCY GRID
# =============================================================================

class OccupancyGrid:
    """Log-odds occupancy grid.

    Array orientation: logodds[iy, ix]; row 0 is the MINIMUM y, column 0 the
    minimum x. Cell (ix, iy) covers x in [x_min + ix*res, x_min + (ix+1)*res).
    Plot with imshow(origin="lower", extent=self.extent()).
    """

    def __init__(self, resolution=RESOLUTION_M, size_m=INITIAL_MAP_SIZE_M):
        self.res = resolution
        self.initial_size = size_m
        self.reset()

    def reset(self):
        n = int(round(self.initial_size / self.res))
        self.L = np.zeros((n, n), dtype=np.float32)
        self.x_min = -n * self.res / 2.0
        self.y_min = -n * self.res / 2.0
        self.dropped_updates = 0

    @property
    def nx(self):
        return self.L.shape[1]

    @property
    def ny(self):
        return self.L.shape[0]

    def extent(self):
        return (self.x_min, self.x_min + self.nx * self.res,
                self.y_min, self.y_min + self.ny * self.res)

    def probabilities(self):
        return 1.0 / (1.0 + np.exp(-self.L.astype(np.float64)))

    def classes(self):
        """0 = unknown, 1 = free, 2 = occupied."""
        p = self.probabilities()
        c = np.zeros(p.shape, dtype=np.int8)
        c[p < P_FREE_THRESHOLD] = 1
        c[p > P_OCC_THRESHOLD] = 2
        return c

    def ensure_contains(self, x0, x1, y0, y1):
        """Grow the grid (up to MAX_MAP_SIZE_M) so the box is inside it.
        Returns True if the grid changed shape."""
        ext = self.extent()
        step = int(round(GROW_STEP_M / self.res))
        max_cells = int(round(MAX_MAP_SIZE_M / self.res))
        grow = [0, 0, 0, 0]  # left, right, bottom, top (cells)
        if x0 < ext[0]:
            grow[0] = int(math.ceil((ext[0] - x0) / self.res / step)) * step
        if x1 > ext[1]:
            grow[1] = int(math.ceil((x1 - ext[1]) / self.res / step)) * step
        if y0 < ext[2]:
            grow[2] = int(math.ceil((ext[2] - y0) / self.res / step)) * step
        if y1 > ext[3]:
            grow[3] = int(math.ceil((y1 - ext[3]) / self.res / step)) * step
        # Respect the size cap.
        room_x = max(0, max_cells - self.nx)
        room_y = max(0, max_cells - self.ny)
        if grow[0] + grow[1] > room_x:
            scale = room_x / (grow[0] + grow[1])
            grow[0], grow[1] = int(grow[0] * scale), int(grow[1] * scale)
        if grow[2] + grow[3] > room_y:
            scale = room_y / (grow[2] + grow[3])
            grow[2], grow[3] = int(grow[2] * scale), int(grow[3] * scale)
        if not any(grow):
            return False
        self.L = np.pad(self.L, ((grow[2], grow[3]), (grow[0], grow[1])), constant_values=0.0)
        self.x_min -= grow[0] * self.res
        self.y_min -= grow[2] * self.res
        return True

    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.x_min) / self.res)),
                int(math.floor((y - self.y_min) / self.res)))

    def integrate_beam(self, sx, sy, theta, r, weight=1.0, mark_hit=True):
        """Cone-shaped HC-SR04 update.

        Free:     cells inside the inner cone and closer than r - thickness/2.
                  Weight falls with angle and range.
        Occupied: cells on the arc |d - r| <= thickness/2 across the wider cone,
                  Gaussian-weighted toward the beam axis. The echo came from
                  SOMEWHERE on that arc; repeated views from other angles let
                  the true obstacle accumulate while wrong arc cells get
                  cleared by free-space updates.
        Returns True if the grid changed shape.
        """
        hit = mark_hit and r < MAX_MAPPING_RANGE_M
        r_eff = min(r, MAX_MAPPING_RANGE_M)
        reach = r_eff + OBSTACLE_THICKNESS_M
        grown = self.ensure_contains(sx - reach, sx + reach, sy - reach, sy + reach)

        ix0, iy0 = self.world_to_cell(sx - reach, sy - reach)
        ix1, iy1 = self.world_to_cell(sx + reach, sy + reach)
        cx0, cx1 = max(ix0, 0), min(ix1, self.nx - 1)
        cy0, cy1 = max(iy0, 0), min(iy1, self.ny - 1)
        if (cx0, cy0, cx1, cy1) != (ix0, iy0, ix1, iy1):
            self.dropped_updates += 1   # part of the beam is beyond MAX_MAP_SIZE_M
        if cx0 > cx1 or cy0 > cy1:
            return grown

        xs = self.x_min + (np.arange(cx0, cx1 + 1) + 0.5) * self.res
        ys = self.y_min + (np.arange(cy0, cy1 + 1) + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys)
        dx, dy = X - sx, Y - sy
        dist = np.hypot(dx, dy)
        bearing = np.abs((np.arctan2(dy, dx) - theta + np.pi) % (2 * np.pi) - np.pi)

        half_occ = math.radians(BEAM_HALF_ANGLE_OCC_DEG)
        half_free = math.radians(BEAM_HALF_ANGLE_FREE_DEG)
        half_t = OBSTACLE_THICKNESS_M / 2.0

        free = (bearing <= half_free) & (dist < r_eff - half_t) & (dist >= MIN_RANGE_M * 0.5)
        free_w = np.exp(-0.5 * (bearing / (half_free * 0.7)) ** 2)
        free_w *= 1.0 - (1.0 - FREE_FAR_WEIGHT) * np.clip(dist / MAX_MAPPING_RANGE_M, 0.0, 1.0)

        block = self.L[cy0:cy1 + 1, cx0:cx1 + 1]
        block[free] += np.float32(L_FREE * weight) * free_w[free].astype(np.float32)

        if hit:
            occ = (bearing <= half_occ) & (np.abs(dist - r) <= half_t)
            occ_w = np.exp(-0.5 * (bearing / (half_occ * 0.5)) ** 2)
            block[occ] += np.float32(L_OCC * weight) * occ_w[occ].astype(np.float32)

        np.clip(block, L_MIN, L_MAX, out=block)
        return grown


# =============================================================================
# 6. MAPPER (orchestrates the pieces; no GUI code)
# =============================================================================

class StallDetector:
    """Uses the ultrasonic as a sanity check on the command-based motion.

    If both wheels are commanded forward but the measured range to what is
    ahead stays the same (within STALL_MIN_CHANGE_M) for STALL_WINDOW_S while
    the heading is steady, the robot is almost certainly blocked - e.g.
    pushing an object too low for the beam. Forward motion is then set to 0.
    With no valid echo there is no evidence either way: state is kept.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.ref = None          # (t, range, theta)
        self.stalled = False

    def update(self, t, r, commanding_forward, theta):
        if not commanding_forward:
            self.ref = None
            self.stalled = False
            return self.stalled
        if r is None:
            return self.stalled
        if (self.ref is None or abs(r - self.ref[1]) >= STALL_MIN_CHANGE_M
                or abs(wrap_pi(theta - self.ref[2])) > math.radians(5.0)):
            self.ref = (t, r, theta)
            self.stalled = False
        elif t - self.ref[0] >= STALL_WINDOW_S:
            self.stalled = True
        return self.stalled


TRAJ_FIELDS = ["wall_time_iso", "esp_millis", "x_m", "y_m", "heading_deg",
               "sigma_xy_m", "sigma_heading_deg", "odometer_m",
               "range_raw_m", "range_filtered_m", "us_status", "range_used",
               "left_cmd", "right_cmd", "gyro_z_dps", "imu_ok", "mode", "stalled",
               "map_status"]


class Mapper:
    def __init__(self):
        self.grid = OccupancyGrid()
        self.pose = PoseEstimator()
        self.heading = HeadingEstimator()
        self.motion = CommandMotionModel()
        self.range_filter = RangeFilter()
        self.speed_cal = SpeedCalibrationHelper()
        self.stall = StallDetector()
        # The worker thread processes samples; the GUI only reads. Hold this
        # lock for any access to mapper state from another thread.
        self.lock = threading.RLock()
        self.esp_time_s = 0.0          # ESP32 time since start (rollover-safe)
        self.last_valid_echo_s = None
        self.imu_resets_seen = None
        self.paused = False
        self.last: Optional[Telemetry] = None
        self.trajectory = []
        self.hits = deque(maxlen=HIT_POINTS_MAX)
        self.events = deque(maxlen=8)
        self.map_status = "waiting for telemetry"
        self.last_used_us_seq = None
        self.last_beam = None          # (sx, sy, theta, r) for display
        self.last_filtered = None
        self.last_integrated = None    # (x, y, theta, r) for repeat detection
        self.grid_changed_shape = True
        self.counts = {"samples": 0, "duplicates": 0, "reboots": 0, "gaps": 0,
                       "beams": 0, "no_echo": 0, "skipped": 0, "stalls": 0,
                       "imu_resets": 0}

    def log(self, msg):
        self.events.append(time.strftime("%H:%M:%S ") + msg)
        print("[mapper]", msg)

    def reset(self):
        """Clear the map and set the current pose to (0, 0, 0 deg)."""
        self.grid.reset()
        self.pose.reset()
        self.range_filter.reset()
        self.speed_cal.reset()
        self.stall.reset()
        self.trajectory.clear()
        self.hits.clear()
        self.last_beam = None
        self.last_integrated = None
        self.grid_changed_shape = True
        for k in self.counts:
            self.counts[k] = 0
        # Keep self.last and the heading reference so the next sample's
        # deltas are measured from NOW.
        self.log("map and pose reset")

    def process(self, s: Telemetry):
        self.counts["samples"] += 1

        if self.last is None:
            self.last = s
            self.heading.delta(s)   # set the reference
            self.last_used_us_seq = s.us_seq
            self.map_status = "started"
            self._record(s, None, False)
            return

        prev = self.last
        dms = (s.millis - prev.millis) % MILLIS_MOD
        rebooted = (s.boot_id is not None and prev.boot_id is not None
                    and s.boot_id != prev.boot_id) or dms > MILLIS_MOD // 2

        if rebooted:
            # Keep the current pose; re-anchor heading and command integrals.
            self.counts["reboots"] += 1
            self.heading.rebase()
            self.heading.delta(s)
            self.pose.inflate(0.10, math.radians(10.0))
            self.range_filter.reset()
            self.last = s
            self.last_used_us_seq = s.us_seq
            self.imu_resets_seen = None
            why = s.raw.get("reset_reason", "unknown")
            self.log(f"ESP32 REBOOTED (reason: {why}) - pose kept, uncertainty inflated")
            if why == "brownout":
                self.log("brownout = supply dipped (motor current). Check power wiring.")
            return

        if dms == 0:
            self.counts["duplicates"] += 1
            return

        dt = dms / 1000.0
        self.esp_time_s += dt
        if s.distance_m is not None:
            self.last_valid_echo_s = self.esp_time_s
        if dt > MAX_GAP_S:
            self.counts["gaps"] += 1
            self.log(f"telemetry gap {dt:.1f} s (motion still integrated)")

        # Heading (gyro) -------------------------------------------------------
        dtheta = self.heading.delta(s)
        var_theta = HeadingEstimator.variance_increment(dtheta, dt, s.imu_ok)
        resets = s.raw.get("imu_resets")
        if isinstance(resets, int) and not isinstance(resets, bool):
            if self.imu_resets_seen is not None and resets > self.imu_resets_seen:
                # Rotation during the IMU outage was missed.
                self.counts["imu_resets"] += resets - self.imu_resets_seen
                var_theta += math.radians(15.0) ** 2
                self.log("IMU was reset on the robot (power dip?) - heading less certain")
            self.imu_resets_seen = resets

        # Forward motion (commands) -------------------------------------------
        if (s.left_cmd_int is not None and prev.left_cmd_int is not None
                and s.right_cmd_int is not None and prev.right_cmd_int is not None):
            avg_l = (s.left_cmd_int - prev.left_cmd_int) / dt
            avg_r = (s.right_cmd_int - prev.right_cmd_int) / dt
        else:
            # Older firmware: assume the command changed linearly between polls.
            avg_l = 0.5 * (s.left_cmd + prev.left_cmd)
            avg_r = 0.5 * (s.right_cmd + prev.right_cmd)
        avg_l = max(-255.0, min(255.0, avg_l))
        avg_r = max(-255.0, min(255.0, avg_r))
        d, sigma_d = self.motion.forward_step(avg_l, avg_r, dt)

        was_stalled = self.stall.stalled
        forward = avg_l > PWM_DEADBAND and avg_r > PWM_DEADBAND
        if self.stall.update(self.esp_time_s, s.distance_m, forward, self.pose.theta):
            d, sigma_d = 0.0, 0.01 * dt
            if not was_stalled:
                self.counts["stalls"] += 1
                self.log("robot STALLED (range not shrinking) - motion frozen")
        elif was_stalled:
            self.log("robot moving again")

        self.pose.predict(d, sigma_d, dtheta, var_theta)

        used = self._maybe_map(s)
        self._record(s, self.last_filtered, used)
        self.last = s

    def _maybe_map(self, s: Telemetry):
        new_range = s.us_seq is None or s.us_seq != self.last_used_us_seq
        self.last_used_us_seq = s.us_seq
        self.last_filtered = None

        if self.paused:
            self.map_status = "PAUSED (pose still tracked)"
            return False
        if not new_range:
            return False
        if not s.imu_ok:
            return self._skip("SKIP: IMU not OK - heading unknown")
        if s.gyro_cal == "running":
            return self._skip("SKIP: gyro calibrating")
        if s.us_age_s is not None and s.us_age_s > MAX_US_AGE_S:
            return self._skip("SKIP: range sample stale")
        if abs(s.gyro_z_dps) > MAX_TURN_RATE_FOR_MAPPING_DPS:
            return self._skip("SKIP: turning too fast")

        sx, sy, th = self.pose.sensor_pose()

        if s.distance_m is None:
            self.counts["no_echo"] += 1
            if NO_ECHO_FREE_RANGE_M > 0 and s.us_status == "no_echo":
                self.grid_changed_shape |= self.grid.integrate_beam(
                    sx, sy, th, NO_ECHO_FREE_RANGE_M, self._pose_weight(), mark_hit=False)
                self.map_status = f"no echo -> weak free to {NO_ECHO_FREE_RANGE_M:.1f} m"
            else:
                self.map_status = f"no valid echo ({s.us_status}) - not used"
            self.last_beam = None
            return False

        if s.distance_m < MIN_RANGE_M:
            return self._skip("SKIP: range below minimum")

        r, n = self.range_filter.add(s.recv_time, s.distance_m, sx, sy, th)
        self.last_filtered = r
        self.speed_cal.update(r, self.pose)

        weight = self._pose_weight()
        if self.last_integrated is not None:
            lx, ly, lth, lr = self.last_integrated
            if (math.hypot(sx - lx, sy - ly) < 0.02
                    and abs(math.degrees(wrap_pi(th - lth))) < 2.0
                    and abs(r - lr) < 0.03):
                weight *= STATIONARY_REPEAT_WEIGHT

        self.grid_changed_shape |= self.grid.integrate_beam(sx, sy, th, r, weight)
        self.last_integrated = (sx, sy, th, r)
        self.last_beam = (sx, sy, th, min(r, MAX_MAPPING_RANGE_M))
        self.counts["beams"] += 1
        if r < MAX_MAPPING_RANGE_M:
            self.hits.append((sx + r * math.cos(th), sy + r * math.sin(th)))
            self.map_status = f"MAPPING  r={r:.2f} m (median of {n})  w={weight:.2f}"
        else:
            self.map_status = f"MAPPING  r={r:.2f} m > max: free space only"
        return True

    def warnings(self, link_state):
        """Short human-readable reasons why the map is not updating."""
        w = []
        if link_state not in ("CONNECTED",):
            w.append(f"NO TELEMETRY ({link_state}) - map frozen. Robot on? Same Wi-Fi? "
                     "(details in mapper.log)")
        s = self.last
        if s is not None:
            if not s.imu_ok:
                w.append("IMU FAIL on robot - mapping paused until it recovers")
            if (self.last_valid_echo_s is None and self.counts["samples"] > 20) or (
                    self.last_valid_echo_s is not None
                    and self.esp_time_s - self.last_valid_echo_s > SENSOR_BLIND_WARN_S):
                since = self.esp_time_s - (self.last_valid_echo_s or 0.0)
                w.append(f"SENSOR BLIND {since:.0f} s ({s.us_status}) - nothing to map")
        if self.stall.stalled:
            w.append("ROBOT STALLED - pushing something? Position frozen")
        if self.paused:
            w.append("PAUSED (space/p to resume)")
        return w

    def _skip(self, why):
        self.counts["skipped"] += 1
        self.map_status = why
        return False

    def _pose_weight(self):
        return max(MIN_UPDATE_WEIGHT, 1.0 / (1.0 + self.pose.sigma_xy() / POSE_SIGMA_REF_M))

    def _record(self, s: Telemetry, filtered, used):
        self.trajectory.append([
            datetime.fromtimestamp(s.wall_time).isoformat(timespec="milliseconds"),
            s.millis,
            round(self.pose.x, 4), round(self.pose.y, 4),
            round(math.degrees(self.pose.theta), 3),
            round(self.pose.sigma_xy(), 4),
            round(math.degrees(self.pose.sigma_theta()), 3),
            round(self.pose.odometer_m, 4),
            "" if s.distance_m is None else round(s.distance_m, 4),
            "" if filtered is None else round(filtered, 4),
            s.us_status, int(used),
            s.left_cmd, s.right_cmd, s.gyro_z_dps, int(s.imu_ok), s.mode,
            int(self.stall.stalled), self.map_status,
        ])


# =============================================================================
# 7. FILE EXPORT
# =============================================================================

def save_snapshot(mapper: Mapper, fig=None, out_root=OUTPUT_DIR, source_desc=""):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(out_root) / stamp
    n = 1
    while out.exists():
        out = Path(out_root) / f"{stamp}_{n}"
        n += 1
    out.mkdir(parents=True)

    g = mapper.grid
    p = g.probabilities()
    cls = g.classes()
    ext = g.extent()
    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "source": source_desc,
        "resolution_m": g.res,
        "array_shape_rows_cols": [g.ny, g.nx],
        "array_orientation": "grid[iy, ix]; row 0 = minimum y; column 0 = minimum x",
        "x_min_m": ext[0], "x_max_m": ext[1], "y_min_m": ext[2], "y_max_m": ext[3],
        "frame": "origin = start pose; +x = initial forward; +y = initial left; "
                 "heading CCW positive",
        "p_free_threshold": P_FREE_THRESHOLD, "p_occ_threshold": P_OCC_THRESHOLD,
        "pose_x_m": mapper.pose.x, "pose_y_m": mapper.pose.y,
        "pose_heading_deg": math.degrees(mapper.pose.theta),
        "pose_sigma_xy_m": mapper.pose.sigma_xy(),
        "pose_sigma_heading_deg": math.degrees(mapper.pose.sigma_theta()),
        "motion_model": {"MAX_SPEED_MPS": MAX_SPEED_MPS, "PWM_DEADBAND": PWM_DEADBAND,
                         "LEFT_SPEED_SCALE": LEFT_SPEED_SCALE,
                         "RIGHT_SPEED_SCALE": RIGHT_SPEED_SCALE},
        "counts": dict(mapper.counts),
        "note": "Approximate dead-reckoning map. No scan matching or loop closure.",
    }

    if fig is not None:
        fig.savefig(out / "map.png", dpi=150)
    else:
        _render_png(mapper, out / "map.png")

    np.save(out / "occupancy_grid.npy", p.astype(np.float32))

    with open(out / "occupancy_grid.csv", "w", newline="") as f:
        for k, v in meta.items():
            if not isinstance(v, dict):
                f.write(f"# {k}: {v}\n")
        w = csv.writer(f)
        w.writerow(["ix", "iy", "x_center_m", "y_center_m", "log_odds",
                    "probability", "state"])
        names = {0: "unknown", 1: "free", 2: "occupied"}
        xs = ext[0] + (np.arange(g.nx) + 0.5) * g.res
        ys = ext[2] + (np.arange(g.ny) + 0.5) * g.res
        for iy in range(g.ny):
            for ix in range(g.nx):
                w.writerow([ix, iy, f"{xs[ix]:.3f}", f"{ys[iy]:.3f}",
                            f"{g.L[iy, ix]:.4f}", f"{p[iy, ix]:.4f}", names[int(cls[iy, ix])]])

    with open(out / "trajectory.csv", "w", newline="") as f:
        f.write(f"# created: {meta['created']}\n# source: {source_desc}\n")
        f.write(f"# frame: {meta['frame']}\n")
        f.write("# x/y/heading are INFERRED (gyro + motor-command model), not measured\n")
        w = csv.writer(f)
        w.writerow(TRAJ_FIELDS)
        w.writerows(mapper.trajectory)

    with open(out / "metadata.json", "w") as f:
        json.dump(meta, f, indent=2)

    return out


def _render_png(mapper: Mapper, path):
    import matplotlib
    matplotlib.use("Agg", force=False)
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(mapper.grid.probabilities(), origin="lower", extent=mapper.grid.extent(),
              cmap="gray_r", vmin=0, vmax=1, interpolation="nearest")
    if mapper.trajectory:
        t = np.array([[r[2], r[3]] for r in mapper.trajectory])
        ax.plot(t[:, 0], t[:, 1], color="tab:blue", lw=1)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.savefig(path, dpi=150)
    plt.close(fig)


# =============================================================================
# 7b. ROBOT EVENT LOG (download, decode, analyse)
# =============================================================================

# Event types printed live in the terminal / map panel. RANGE, GYRO, STATS,
# REQS and JOY are high-rate and only go to the CSV.
LOUD_EVENTS = {"BOOT", "PREV_BOOT", "WATCHDOG", "ESTOP", "MODE", "GUARD", "AUTO",
               "IMU", "WIFI", "LINKGAP", "MAC", "STALL", "LINK"}
ROBOT_LOG_FIELDS = ["mac_time_iso", "boot_id", "seq", "esp_ms", "type", "tag",
                    "a", "b", "f", "description"]
NORMAL_STOP_SOURCES = {"phone", "mode_change", "boot", "speed_slider"}

LINK_TEXT = {
    "ws_open": "phone/client CONNECTED (WebSocket)",
    "ws_close": "phone/client DISCONNECTED (WebSocket closed)",
    "driver_changed": "a different client took over driving",
    "phone_reconnected": "phone reconnected its WebSocket",
    "phone_link_restored": "phone sees the link again",
    "phone_rtt_spike": "phone measured a slow round trip",
    "phone_page_hidden": "phone page hidden (screen lock / app switch)",
    "cmd_queue_full": "command queue FULL - commands dropped",
    "httpd_start_failed": "HTTP server FAILED to start",
    "sample_ring_alloc_failed": "no memory for the sample ring",
    "telemetry_overflow": "telemetry JSON overflowed its buffer",
    "ws_silent_closed": "robot closed a SILENT client (phone gone / half-open link)",
    "httpd_stalled": "robot HTTP server task STALLED",
    "httpd_recovered": "robot HTTP server task running again",
}


def describe_event(etype, tag, a, b, f):
    """Human-readable text for one robot log event."""
    prev = etype.startswith("PREV:")
    t = etype[5:] if prev else etype
    pre = "[BEFORE RESET] " if prev else ""
    if t == "RANGE":
        if tag == "ok":
            return f"{pre}range {f:.1f} cm"
        raw = f", raw {f:.1f} cm" if f >= 0 else ""
        return f"{pre}range INVALID: {tag}{raw}"
    if t == "GYRO":
        return f"{pre}yaw {f:.1f} deg, rate {a / 10:+.1f} dps, raw {b / 10:+.1f} dps ({tag})"
    if t == "MOTOR":
        what = "STOP" if a == 0 and b == 0 else f"L={a} R={b}"
        return f"{pre}motors {what}  <- {tag}"
    if t == "JOY":
        return f"{pre}phone joystick throttle={a} steering={b}"
    if t == "LINKGAP":
        return f"{pre}phone drive commands paused {a} ms while driving (Wi-Fi lag)"
    if t == "WATCHDOG":
        return (f"{pre}SAFETY STOP: {tag} heartbeat lost "
                f"({a} ms since last command)")
    if t == "ESTOP":
        expl = {"estop_button": "STOP button pressed",
                "estop_page_blur": "phone page lost focus (notification / app switch)",
                "estop_page_hidden": "phone page hidden (screen lock / app switch)",
                "estop_link_lost": "phone lost the link"}
        return f"{pre}EMERGENCY STOP: {expl.get(tag, tag)}" + (" (was AUTO)" if a else "")
    if t == "MODE":
        return f"{pre}mode -> {tag}"
    if t == "SPEED":
        return f"{pre}speed slider {b} -> {a}"
    if t == "GUARD":
        if tag == "latch":
            return (f"{pre}FORWARD BLOCKED by guard at {f:.1f} cm "
                    f"({a}/3 readings close, throttle {b})")
        return f"{pre}forward guard {tag.replace('_', ' ')} (range {f:.1f} cm)"
    if t == "AUTO":
        return f"{pre}AUTO: {tag.replace('_', ' ')} (range {f:.1f} cm, a={a}, b={b})"
    if t == "IMU":
        return f"{pre}IMU: {tag.replace('_', ' ')} (a={a}, b={b}, value={f:.3f})"
    if t == "WIFI":
        if tag == "sta_disconnected":
            return (f"{pre}Wi-Fi DISCONNECTED from router: reason {wifi_reason_name(a)}, "
                    f"drop #{b}, RSSI {f:.0f} dBm")
        if tag == "sta_got_ip":
            return f"{pre}Wi-Fi connected, got IP .{a}"
        if tag in ("ap_join", "ap_leave"):
            return f"{pre}Wi-Fi device {tag.replace('ap_', '')} (MAC ..{a:02x}), {b} connected"
        return f"{pre}Wi-Fi device {tag}: {a} connected"
    if t == "STATS":
        return f"{pre}loop {a * 10} Hz, slowest pass {b} ms, free heap {f:.0f} B ({tag})"
    if t == "REQS":
        return f"{pre}per second: WebSocket frames in {a} / out {f:.0f}, HTTP requests {b}"
    if t == "LINK":
        text = LINK_TEXT.get(tag, tag.replace("_", " "))
        if tag in ("ws_open", "ws_close"):
            return f"{pre}{text}: client .{a}, socket {b}, {f:.0f} open"
        if tag in ("phone_rtt_spike",):
            return f"{pre}{text}: {a} ms"
        if tag in ("phone_reconnected", "phone_link_restored"):
            return f"{pre}{text} (attempt {a})"
        return f"{pre}{text} (a={a}, b={b})"
    if t == "HEALTH":
        return (f"{pre}health: min free heap {a} KB, loop stack free {b} B, "
                f"server stack free {f:.0f} B")
    if t == "BOOT":
        return f"ROBOT BOOTED, reset reason: {tag}"
    if t == "PREV_BOOT":
        return (f"--- {a} events recovered from before the reset ---" if tag == "begin"
                else "--- end of pre-reset events ---")
    if t == "MAC":
        return f"[Mac] {tag}"
    if t == "STALL":
        return (f"{pre}ROBOT PROGRAM FROZE {a} ms in {tag}"
                + (" - dead-man CUT THE MOTORS" if b else ""))
    return f"{pre}{t} {tag} a={a} b={b} f={f}"


def parse_log_response(text):
    """Returns (header dict, [(seq, ms, type, tag, a, b, f), ...])."""
    header, rows = {}, []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            for kv in line[1:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    header[k] = v
            continue
        parts = line.split(",")
        if len(parts) != 7:
            continue
        try:
            rows.append((int(parts[0]), int(parts[1]), parts[2], parts[3],
                         int(parts[4]), int(parts[5]), float(parts[6])))
        except ValueError:
            continue
    return header, rows


class RobotLogClient:
    """Continuously downloads /api/log into a CSV (everything the robot did).

    With `link` (a TelemetryClient) it follows the link's current robot
    address, so it moves along when the robot is rediscovered elsewhere."""

    def __init__(self, url, out_path, on_loud=None, period=ROBOT_LOG_POLL_S, link=None):
        self._url = url
        self.link = link
        self.out_path = Path(out_path)
        self.on_loud = on_loud
        self.period = period
        self.events = 0
        self.errors = 0
        self.lost = 0
        self.beat = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="robotlog", daemon=True)

    @property
    def url(self):
        return self.link.base + "/api/log" if self.link else self._url

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                log.error("robot-log thread did not stop within 3 s")

    def _note(self, w, boot, msg):
        self._write(w, boot, ("", "", "MAC", msg, 0, 0, 0.0))

    def _write(self, w, boot, row):
        seq, ms, etype, tag, a, b, f = row
        desc = describe_event(etype, tag, a, b, f)
        w.writerow([datetime.now().isoformat(timespec="milliseconds"), boot, seq, ms,
                    etype, tag, a, b, f, desc])
        self.events += 1
        base = etype[5:] if etype.startswith("PREV:") else etype
        loud = base in LOUD_EVENTS or (base == "MOTOR" and a == 0 and b == 0
                                       and tag not in NORMAL_STOP_SOURCES)
        if loud and self.on_loud:
            self.on_loud(desc)

    def _run(self):
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        session = requests.Session()   # keep-alive: one connection
        since, boot, reachable, down_since, last_note = 0, None, None, None, 0.0
        log.info("robot-log thread started -> %s", self.out_path)
        with open(self.out_path, "w", newline="") as f:
            f.write(f"# AUTOBOT robot event log, started {datetime.now().isoformat()}\n")
            f.write(f"# source: {self.url}\n")
            w = csv.writer(f)
            w.writerow(ROBOT_LOG_FIELDS)
            while not self._stop.is_set():
                self.beat = time.monotonic()
                more = False
                url = self.url
                try:
                    r = session.get(url, params={"since": since},
                                    timeout=(HTTP_CONNECT_TIMEOUT_S, 2.0))
                    if r.status_code == 404:
                        raise ValueError("robot firmware has no /api/log (upload v1.3+)")
                    if r.status_code != 200:
                        raise ValueError(f"HTTP {r.status_code}")
                    hdr, rows = parse_log_response(r.text)
                    if "next" not in hdr:
                        raise ValueError(f"bad log header: {r.text[:60]!r}")
                    if reachable is False:
                        gap = time.monotonic() - down_since
                        self._note(w, boot, f"robot reachable again after {gap:.1f} s "
                                            f"({self.errors} errors so far)")
                    reachable = True
                    if boot is not None and hdr.get("boot") != boot:
                        self._note(w, hdr.get("boot"), "robot REBOOTED - new log session")
                        log.warning("robot log: robot rebooted (boot %s -> %s)",
                                    boot, hdr.get("boot"))
                        boot, since = hdr.get("boot"), 0
                        continue
                    boot = hdr.get("boot")
                    oldest = int(hdr.get("oldest", 0))
                    if since and oldest > since:
                        self.lost += oldest - since
                        self._note(w, boot, f"LOST {oldest - since} events "
                                            "(Mac could not download them in time)")
                        log.warning("robot log: %d events lost", oldest - since)
                    for row in rows:
                        self._write(w, boot, row)
                    nxt = int(hdr["next"])
                    more = hdr.get("more") == "1"
                    since = nxt
                except Exception as e:   # anything: note it, back off, carry on
                    self.errors += 1
                    msg = describe_exc(e, url)
                    now = time.monotonic()
                    if reachable is not False:
                        down_since = now
                        last_note = now
                        self._note(w, boot, f"robot UNREACHABLE: {msg}")
                    elif now - last_note >= STILL_DOWN_LOG_S:
                        last_note = now
                        self._note(w, boot, f"robot still unreachable after "
                                            f"{now - down_since:.0f} s: {msg}")
                    log.debug("robot log: %s", msg)
                    if not isinstance(e, (requests.exceptions.RequestException, ValueError)):
                        log.error("unexpected error in the robot-log thread", exc_info=e)
                    reachable = False
                    session.close()
                    session = requests.Session()
                f.flush()
                # Catching up a backlog: still leave the robot a short breather.
                self._stop.wait(0.1 if more else self.period)
        log.info("robot-log thread stopped: %d events, %d errors, %d lost",
                 self.events, self.errors, self.lost)


def read_robot_log(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(line for line in f if not line.startswith("#"))
        for r in reader:
            try:
                r["a"] = int(r["a"])
                r["b"] = int(r["b"])
                r["f"] = float(r["f"])
                r["esp_ms"] = int(r["esp_ms"]) if r["esp_ms"] else None
                r["mac_t"] = datetime.fromisoformat(r["mac_time_iso"]).timestamp()
            except (ValueError, KeyError, TypeError):
                continue
            rows.append(r)
    # Continuous time axis: robot ms, aligned per boot to Mac clock.
    offsets = {}
    for r in rows:
        if r["esp_ms"] is not None and not r["type"].startswith("PREV:") and r["boot_id"]:
            offsets.setdefault(r["boot_id"], []).append(r["mac_t"] - r["esp_ms"] / 1000.0)
    offsets = {k: float(np.median(v)) for k, v in offsets.items()}
    for r in rows:
        off = offsets.get(r["boot_id"])
        if r["esp_ms"] is not None and off is not None and not r["type"].startswith("PREV:"):
            r["t"] = r["esp_ms"] / 1000.0 + off
        else:
            r["t"] = r["mac_t"]
    return rows


def analyze_robot_log(path, make_plot=True):
    """Print a 'why did it stop' report and save a timeline PNG. Returns text."""
    rows = read_robot_log(path)
    out = []
    p = out.append
    if not rows:
        return "log is empty"
    t0 = min(r["t"] for r in rows)
    ts = lambda r: f"{r['t'] - t0:7.1f}s"
    by = {}
    for r in rows:
        by.setdefault(r["type"], []).append(r)
    dur = max(r["t"] for r in rows) - t0
    p(f"ROBOT LOG REPORT  {path}")
    p(f"duration {dur:.0f} s, {len(rows)} events")

    # Boots / resets
    p("\n== Resets ==")
    boots = by.get("BOOT", [])
    if not boots:
        p("  no boot in this log (robot was already running)")
    for r in boots:
        p(f"  {ts(r)}  booted, reset reason: {r['tag']}"
          + ("   <-- POWER DIP from motors: fix power wiring" if r["tag"] == "brownout" else "")
          + ("   <-- FIRMWARE CRASH/HANG" if r["tag"] in ("crash", "watchdog") else ""))
    prev = [r for r in rows if r["type"].startswith("PREV:")]
    if prev:
        p(f"  {len(prev)} events from just BEFORE a reset were recovered; last ones:")
        for r in prev[-8:]:
            p(f"      {r['description']}")

    # Ultrasonic
    p("\n== Ultrasonic ==")
    rng = [r for r in by.get("RANGE", [])]
    if rng:
        ok = [r for r in rng if r["tag"] == "ok"]
        rate = len(rng) / max(1.0, rng[-1]["t"] - rng[0]["t"])
        p(f"  {len(rng)} readings ({rate:.1f}/s), valid {100 * len(ok) / len(rng):.0f}%")
        counts = {}
        for r in rng:
            counts[r["tag"]] = counts.get(r["tag"], 0) + 1
        p("  status: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items(), key=lambda x: -x[1])))
        # Motor state at each reading
        motor = [r for r in by.get("MOTOR", [])]
        mi, on = 0, False
        spikes = {True: 0, False: 0}
        invalid = {True: 0, False: 0}
        total = {True: 0, False: 0}
        vals = [(r["t"], r["f"] if r["tag"] == "ok" else None) for r in rng]
        for i, r in enumerate(rng):
            while mi < len(motor) and motor[mi]["t"] <= r["t"]:
                on = motor[mi]["a"] != 0 or motor[mi]["b"] != 0
                mi += 1
            total[on] += 1
            if r["tag"] != "ok":
                invalid[on] += 1
            if 0 < i < len(rng) - 1:
                a, b, c = vals[i - 1][1], vals[i][1], vals[i + 1][1]
                if None not in (a, b, c) and abs(a - c) < 10 and abs(b - a) > 30 and abs(b - c) > 30:
                    spikes[on] += 1
        for on in (False, True):
            if total[on]:
                p(f"  motors {'ON ' if on else 'OFF'}: {total[on]} readings, "
                  f"invalid {100 * invalid[on] / total[on]:.0f}%, "
                  f"isolated spikes {spikes[on]} ({100 * spikes[on] / total[on]:.1f}%)")
        if total[True] and total[False]:
            on_rate = (invalid[True] + spikes[True]) / total[True]
            off_rate = (invalid[False] + spikes[False]) / total[False]
            if on_rate > 2 * off_rate + 0.05:
                p("  --> readings are much worse while the motors run: ELECTRICAL NOISE "
                  "from the motors on the sensor wiring.")
        close = [r for r in ok if r["f"] < 10]
        if close:
            p(f"  {len(close)} valid readings under 10 cm (real contact or noise)")
    else:
        p("  no range readings in log")

    # Why it stopped
    p("\n== Why the motors stopped ==")
    stops = {}
    unexpected = []
    last_joy = None
    prev_motor = None
    for r in rows:
        if r["type"] == "JOY":
            last_joy = r
        if r["type"] != "MOTOR":
            continue
        was_moving = prev_motor is not None and (prev_motor["a"] or prev_motor["b"])
        if was_moving and r["a"] == 0 and r["b"] == 0:
            stops[r["tag"]] = stops.get(r["tag"], 0) + 1
            user_released = r["tag"] == "phone" and last_joy and last_joy["a"] == 0 and last_joy["b"] == 0
            if not user_released:
                unexpected.append(r)
        prev_motor = r
    if stops:
        for k, v in sorted(stops.items(), key=lambda x: -x[1]):
            expl = {"phone": "you released the joystick",
                    "watchdog_manual": "phone commands stopped arriving for 0.7 s (Wi-Fi)",
                    "watchdog_auto_link": "phone heartbeat lost in AUTO",
                    "guard": "forward guard: obstacle under 20 cm",
                    "estop_button": "STOP button",
                    "estop_page_blur": "phone page lost focus",
                    "estop_page_hidden": "phone screen locked / app switched",
                    "deadman": "robot program FROZE; dead-man cut the motors",
                    "auto_backoff": "AUTO obstacle reaction",
                    "auto_invalid_range": "AUTO: no valid echo",
                    "auto_close_unconfirmed": "AUTO: one close reading, waiting to confirm",
                    "auto_sensor_stale": "AUTO: ultrasonic stopped producing samples",
                    "mode_change": "mode switched"}.get(k, "")
            p(f"  {v:4d} x {k:24s} {expl}")
    else:
        p("  no stop events")
    if unexpected:
        p(f"\n  Unexpected stops (not a joystick release), up to 25, with context:")
        for r in unexpected[:25]:
            ctx = [x for x in rows if x["type"] == "RANGE" and r["t"] - 0.3 <= x["t"] <= r["t"]]
            rtxt = " ".join(f"{x['f']:.0f}" if x["tag"] == "ok" else x["tag"] for x in ctx[-4:])
            p(f"    {ts(r)}  {r['tag']:22s} ranges before: [{rtxt}]")

    # Safety and links
    p("\n== Safety / link ==")
    stalls = by.get("STALL", [])
    if stalls:
        p(f"  PROGRAM FREEZES: {len(stalls)}, worst {max(r['a'] for r in stalls)} ms; where: "
          + ", ".join(f"{k} x{v}" for k, v in
                      {t: sum(1 for r in stalls if r['tag'] == t)
                       for t in {r['tag'] for r in stalls}}.items()))
        cut = sum(1 for r in stalls if r["b"])
        if cut:
            p(f"    dead-man cut the motors {cut} time(s)")
    for name in ("WATCHDOG", "ESTOP", "GUARD", "AUTO", "IMU", "WIFI", "LINK"):
        ev = by.get(name, [])
        if ev:
            counts = {}
            for r in ev:
                counts[r["tag"]] = counts.get(r["tag"], 0) + 1
            p(f"  {name:8s} " + ", ".join(f"{k} x{v}" for k, v in counts.items()))
    drops = [r for r in by.get("WIFI", []) if r["tag"] == "sta_disconnected"]
    if drops:
        reasons = {}
        for r in drops:
            k = wifi_reason_name(r["a"])
            reasons[k] = reasons.get(k, 0) + 1
        p("  Wi-Fi drops by reason: " + ", ".join(f"{k} x{v}" for k, v in reasons.items()))
    gaps = [r["a"] for r in by.get("LINKGAP", [])]
    if gaps:
        p(f"  phone command gaps while driving: {len(gaps)}, worst {max(gaps)} ms, "
          f"over 500 ms (watchdog): {sum(g > 500 for g in gaps)}")
    stats = by.get("STATS", [])
    if stats:
        p(f"  loop rate min {min(r['a'] for r in stats) * 10} Hz, "
          f"slowest pass {max(r['b'] for r in stats)} ms, "
          f"free heap min {min(r['f'] for r in stats):.0f} B")
    health = by.get("HEALTH", [])
    if health:
        p(f"  min free heap {min(r['a'] for r in health)} KB, "
          f"stack free: loop {min(r['b'] for r in health)} B, "
          f"server {min(r['f'] for r in health):.0f} B")
    macs = by.get("MAC", [])
    if macs:
        p(f"  Mac-side notes: {len(macs)}")
        for r in macs[:15]:
            p(f"    {ts(r)}  {r['tag']}")

    report = "\n".join(out)
    if make_plot:
        try:
            png = _plot_robot_log(rows, t0, Path(path).with_suffix(".timeline.png"))
            report += f"\n\nTimeline chart: {png}"
        except Exception as e:
            report += f"\n\n(timeline chart failed: {e})"
    return report


def _plot_robot_log(rows, t0, png_path):
    # A bare Figure (Agg canvas): independent of pyplot and of whatever GUI
    # backend the map window used.
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    live = [r for r in rows if not r["type"].startswith("PREV:")]
    sel = lambda typ: [r for r in live if r["type"] == typ]
    fig = Figure(figsize=(16, 10))
    FigureCanvasAgg(fig)
    ax = fig.subplots(4, 1, sharex=True, gridspec_kw={"height_ratios": [3, 2, 2, 1.4]})
    rng = sel("RANGE")
    ok = [r for r in rng if r["tag"] == "ok"]
    bad = [r for r in rng if r["tag"] != "ok"]
    ax[0].plot([r["t"] - t0 for r in ok], [r["f"] for r in ok], ".", ms=3,
               color="tab:green", label="valid range")
    ax[0].plot([r["t"] - t0 for r in bad], [-10] * len(bad), "x", ms=4,
               color="tab:red", label="invalid (no/garbage echo)")
    ax[0].axhline(20, color="tab:orange", lw=0.8, ls="--", label="manual guard 20 cm")
    ax[0].axhline(35, color="tab:purple", lw=0.8, ls=":", label="AUTO obstacle 35 cm")
    ax[0].set_ylabel("ultrasonic [cm]")
    ax[0].set_ylim(-20, max([150] + [min(r["f"], 400) for r in ok]) + 10)
    ax[0].legend(loc="upper right", fontsize=8)

    mot = sel("MOTOR")
    if mot:
        tt = [r["t"] - t0 for r in mot]
        ax[1].step(tt, [r["a"] for r in mot], where="post", label="left PWM")
        ax[1].step(tt, [r["b"] for r in mot], where="post", label="right PWM")
    joy = sel("JOY")
    if joy:
        ax[1].step([r["t"] - t0 for r in joy], [r["a"] * 2.55 for r in joy], where="post",
                   color="0.6", lw=0.8, label="joystick throttle (scaled)")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("motor PWM")
    ax[1].legend(loc="upper right", fontsize=8)

    gy = sel("GYRO")
    if gy:
        ax[2].plot([r["t"] - t0 for r in gy], [r["f"] for r in gy], color="tab:blue",
                   label="yaw [deg]")
        ax2 = ax[2].twinx()
        ax2.plot([r["t"] - t0 for r in gy], [r["a"] / 10 for r in gy], color="tab:gray",
                 lw=0.6, label="rate [dps]")
        ax2.set_ylabel("rate [dps]")
    ax[2].set_ylabel("yaw [deg]")

    colors = {"STALL": "magenta", "WATCHDOG": "orange", "ESTOP": "red", "GUARD": "goldenrod", "AUTO": "purple",
              "IMU": "brown", "BOOT": "black", "WIFI": "teal", "MAC": "gray",
              "LINKGAP": "pink", "LINK": "navy"}
    level = {k: i for i, k in enumerate(colors)}
    for r in live:
        if r["type"] in colors:
            x = r["t"] - t0
            for a in ax[:3]:
                a.axvline(x, color=colors[r["type"]], lw=0.6, alpha=0.5)
            ax[3].plot(x, level[r["type"]], "|", color=colors[r["type"]], ms=12, mew=2)
    ax[3].set_yticks(list(level.values()))
    ax[3].set_yticklabels(list(level.keys()), fontsize=8)
    ax[3].set_xlabel("time [s]")
    ax[0].set_title("AUTOBOT robot log timeline (vertical lines = events, see bottom row)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    return png_path


def run_log_only(telemetry_url, log_url, rate, session_dir):
    """Headless: record telemetry + robot log, print events, analyse at exit."""
    tel = TelemetryClient(telemetry_url, rate, session_dir=session_dir)
    mapper = Mapper()
    worker = MapperWorker(mapper, tel, TelemetryRecorder(session_dir / "telemetry.jsonl"))
    robot_log_path = session_dir / "robotlog.csv"

    def loud(d):
        print(time.strftime("%H:%M:%S "), d)
        log.info("robot: %s", d)

    rl = RobotLogClient(log_url, robot_log_path, on_loud=loud, link=tel)
    wd = StallWatchdog({"mapper worker": lambda: worker.beat, "link": lambda: tel.beat,
                        "robot log": lambda: rl.beat}, {"link": 10.0, "robot log": 10.0})
    keep_mac_awake()
    print(f"Logging to {session_dir}/")
    print("Drive the robot normally. Press Ctrl-C to stop and get the report.\n")
    tel.start()
    worker.start()
    rl.start()
    wd.start()
    try:
        while True:
            time.sleep(5)
            st = tel.status.state()
            with mapper.lock:
                s = mapper.last
                rng = "-" if s is None or s.distance_m is None else f"{s.distance_m * 100:.0f} cm"
                info = "" if s is None else (f"mode {s.mode}  range {rng} ({s.us_status})  "
                                            f"L/R {s.left_cmd:.0f}/{s.right_cmd:.0f}  "
                                            f"imu {'OK' if s.imu_ok else 'FAIL'}")
            print(f"{time.strftime('%H:%M:%S')}  link {st['state']}  log events "
                  f"{rl.events}  {info}")
    except KeyboardInterrupt:
        print("\nstopping ...")
    finally:
        rl.stop()
        tel.stop()
        worker.stop()
        wd.stop()
    text = "LINK  " + tel.summary() + "\n" + analyze_robot_log(robot_log_path)
    print()
    print(text)
    log.info("summary:\n%s", text)
    (session_dir / "summary.txt").write_text(text + "\n")


# =============================================================================
# 8. SIMULATED ROBOT (for --simulate and --selftest; no hardware needed)
# =============================================================================

class SimulatedRobot:
    """A fake robot in a 4 x 3 m room with a box, producing the same JSON as the
    firmware. True speed differs from the model by 8 % and the gyro has a small
    residual bias, so the map shows realistic dead-reckoning error."""

    WALLS = [((-0.8, -1.0), (3.2, -1.0)), ((3.2, -1.0), (3.2, 2.0)),
             ((3.2, 2.0), (-0.8, 2.0)), ((-0.8, 2.0), (-0.8, -1.0)),
             ((1.6, 0.4), (2.0, 0.4)), ((2.0, 0.4), (2.0, 0.9)),
             ((2.0, 0.9), (1.6, 0.9)), ((1.6, 0.9), (1.6, 0.4))]

    def __init__(self, seed=1):
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.t = 0.0
        self.yaw_deg = 0.0
        self.l_int = self.r_int = 0.0
        self.left = self.right = 0
        self.state, self.state_t, self.turn_goal = "fwd", 0.0, 0.0
        self.us_dist, self.us_status, self.us_seq, self.us_t = None, "none", 0, 0.0
        self.next_ping = 0.0
        self.boot_id = 12345

    def _raycast(self, x, y, th):
        best = None
        dx, dy = math.cos(th), math.sin(th)
        for (ax, ay), (bx, by) in self.WALLS:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((ax - x) * ey - (ay - y) * ex) / den
            u = ((ax - x) * dy - (ay - y) * dx) / den
            if t > 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        return best

    def _ping(self):
        sx = self.x + SENSOR_OFFSET_X_M * math.cos(self.th)
        sy = self.y + SENSOR_OFFSET_X_M * math.sin(self.th)
        rays = [self._raycast(sx, sy, self.th + math.radians(a)) for a in (-8, -4, 0, 4, 8)]
        rays = [r for r in rays if r is not None]
        self.us_seq += 1
        self.us_t = self.t
        if not rays or min(rays) > 4.0 or self.rng.random() < 0.05:
            self.us_dist, self.us_status = None, "no_echo"
        else:
            self.us_dist, self.us_status = min(rays) + self.rng.normal(0, 0.01), "ok"

    def _controller(self):
        r = self.us_dist
        if self.state == "fwd":
            self.left, self.right = 160, 160
            if r is not None and r < 0.45:
                self.state, self.state_t = "turn", self.t
                self.turn_goal = self.th + math.radians(90 + self.rng.normal(0, 15))
        elif self.state == "turn":
            self.left, self.right = -150, 150
            if self.th >= self.turn_goal:
                self.state = "fwd"

    def step(self, dt):
        self._controller()
        true_max = MAX_SPEED_MPS * 1.08

        def wheel(p):
            a = abs(p)
            if a <= PWM_DEADBAND:
                return 0.0
            return math.copysign(true_max * (a - PWM_DEADBAND) / (255 - PWM_DEADBAND), p)

        vl, vr = wheel(self.left), wheel(self.right)
        v = 0.5 * (vl + vr)
        omega = (vr - vl) / 0.13                    # 13 cm track width
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += omega * dt
        gyro = math.degrees(omega) + 0.01 + self.rng.normal(0, 0.05)   # residual bias
        self.yaw_deg += gyro * dt
        self.gyro = gyro
        self.l_int += self.left * dt
        self.r_int += self.right * dt
        self.t += dt
        if self.t >= self.next_ping:
            self._ping()
            self.next_ping = self.t + 0.06

    def telemetry_dict(self):
        ms = int(self.t * 1000) % MILLIS_MOD
        return {
            "fw": "simulator", "boot_id": self.boot_id, "millis": ms, "mode": "manual",
            "auto_state": "driving",
            "distance_cm": None if self.us_dist is None else round(self.us_dist * 100, 1),
            "distance_valid": self.us_dist is not None, "us_status": self.us_status,
            "us_seq": self.us_seq, "us_age_ms": int((self.t - self.us_t) * 1000),
            "us_valid_count": 0, "us_invalid_count": 0,
            "imu_ok": True, "imu_who_am_i": 104, "imu_errors": 0, "imu_gaps": 0,
            "yaw_deg": round(wrap_180(self.yaw_deg), 3),
            "yaw_unwrapped_deg": round(self.yaw_deg, 3),
            "gyro_z_dps": round(getattr(self, "gyro", 0.0), 3),
            "gyro_z_raw_dps": round(getattr(self, "gyro", 0.0) - 1.2, 3),
            "gyro_bias_dps": -1.2, "gyro_stationary": False, "gyro_cal": "ok",
            "gyro_cal_stddev_dps": 0.05, "gyro_samples": int(self.t * 200),
            "left_cmd": self.left, "right_cmd": self.right,
            "left_cmd_int": round(self.l_int, 3), "right_cmd_int": round(self.r_int, 3),
            "throttle": 0, "steering": 0, "motor_speed": 180,
            "watchdog_stops": 0, "fwd_blocked": False, "auto_backoffs": 0,
            "auto_reason": "none", "imu_resets": 0, "reset_reason": "power_on", "loop_hz": 1000,
            "loop_max_us": 900, "ws_clients": 1, "sample_seq": int(self.t * 25),
            "roll_deg": 0.0, "pitch_deg": 0.0,
            "home_ip": "", "home_rssi": 0, "free_heap": 200000,
        }

    def run_until(self, t_end, dt=0.005):
        while self.t < t_end:
            self.step(dt)


class SimulatedSource:
    """Same interface as TelemetryClient, driven by SimulatedRobot in real time."""

    def __init__(self, poll_hz=POLL_HZ):
        self.robot = SimulatedRobot()
        self.period = 1.0 / poll_hz
        self.samples = queue.Queue(maxsize=200)
        self.status = SourceStatus()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        t0 = time.monotonic()
        while not self._stop.is_set():
            self.robot.run_until(time.monotonic() - t0)
            s, _ = parse_telemetry(self.robot.telemetry_dict(), time.monotonic(), time.time(), 0.0)
            if not self.samples.full():
                self.samples.put_nowait(s)
            self.status.ok(0.0)
            self._stop.wait(self.period)


class TelemetryRecorder:
    """Writes every raw telemetry sample as one JSON line, for --replay."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "a")
        self.last_flush = time.monotonic()

    def write(self, s: Telemetry):
        self.f.write(json.dumps({"wall": s.wall_time, "mono": s.recv_time,
                                 "lat": s.latency_s, "data": s.raw}) + "\n")
        self.flush_if_due()

    def flush_if_due(self):
        """Called after every write AND by the worker when idle, so the file
        is never more than ~1 s behind even if samples stop arriving."""
        if time.monotonic() - self.last_flush > 1.0:
            self.f.flush()
            self.last_flush = time.monotonic()

    def close(self):
        self.f.flush()
        self.f.close()


class ReplaySource:
    """Same interface as TelemetryClient; plays back a recorded .jsonl file."""

    def __init__(self, path, speed=1.0):
        self.path = Path(path)
        self.speed = max(0.01, speed)
        self.samples = queue.Queue(maxsize=200)
        self.status = SourceStatus()
        self.finished = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="replay", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        prev_mono = None
        with open(self.path) as f:
            for line in f:
                if self._stop.is_set():
                    return
                try:
                    rec = json.loads(line)
                except ValueError:
                    self.status.error("bad line in recording")
                    continue
                if prev_mono is not None:
                    self._stop.wait(max(0.0, rec["mono"] - prev_mono) / self.speed)
                prev_mono = rec["mono"]
                s, err = parse_telemetry(rec["data"], time.monotonic(), rec["wall"],
                                         rec.get("lat", 0.0))
                if s is None:
                    self.status.error(err)
                    continue
                self.samples.put(s)
                self.status.ok(rec.get("lat", 0.0))
        self.finished = True
        print("[replay] end of recording")


class MapperWorker:
    """Processes telemetry in the background so mapping keeps running even when
    macOS throttles or hides the plot window. The GUI only draws."""

    def __init__(self, mapper: Mapper, source, recorder: Optional[TelemetryRecorder] = None):
        self.mapper = mapper
        self.source = source
        self.recorder = recorder
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mapper", daemon=True)
        self.link_state = None
        self.beat = time.monotonic()
        self.errors = 0

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            # Closing the file under a running writer would only add errors.
            log.error("mapper worker did not stop within 3 s - telemetry file left open")
        elif self.recorder:
            self.recorder.close()

    def _run(self):
        last_check = 0.0
        while not self._stop.is_set():
            self.beat = time.monotonic()
            try:
                s = self.source.samples.get(timeout=0.2)
            except queue.Empty:
                s = None
                if self.recorder:
                    self.recorder.flush_if_due()
            if s is not None:
                try:
                    with self.mapper.lock:
                        self.mapper.process(s)
                    if self.recorder:
                        self.recorder.write(s)
                except Exception:
                    self.errors += 1
                    log.exception("mapper failed on sample millis=%s (worker continues)",
                                  getattr(s, "millis", "?"))
            if time.monotonic() - last_check > 0.5:
                last_check = time.monotonic()
                st = self.source.status.state()
                if st["state"] != self.link_state:
                    with self.mapper.lock:
                        extra = f" ({st['last_error']})" if st["state"] != "CONNECTED" else ""
                        self.mapper.log(f"link {st['state']}{extra}")
                    log.info("link state %s%s", st["state"],
                             f" ({st['last_error']})" if st["state"] != "CONNECTED" else "")
                    self.link_state = st["state"]


class StallWatchdog:
    """Reports any thread that stops making progress: logs how long, and the
    full stack of EVERY thread (that shows exactly where a deadlock sits)."""

    def __init__(self, beats, limits=None):
        self.beats = beats              # name -> callable returning a monotonic time
        self.limits = limits or {}
        self.stalled = {}
        self.message = ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="watchdog", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop.wait(1.0):
            now = time.monotonic()
            msgs = []
            for name, get in self.beats.items():
                age = now - get()
                limit = self.limits.get(name, STALL_DUMP_AFTER_S)
                if age > limit:
                    msgs.append(f"{name} stalled {age:.0f} s")
                    if name not in self.stalled:
                        self.stalled[name] = now
                        log.error("STALL: %s made no progress for %.1f s - stacks of all "
                                  "threads follow", name, age)
                        dump_all_stacks()
                elif name in self.stalled:
                    log.warning("%s recovered after a %.1f s stall", name,
                                now - self.stalled.pop(name))
            self.message = "; ".join(msgs)


def dump_all_stacks():
    names = {t.ident: t.name for t in threading.enumerate()}
    for tid, frame in sys._current_frames().items():
        log.error("--- thread %s ---\n%s", names.get(tid, tid),
                  "".join(traceback.format_stack(frame)))


def find_robot(hosts=ROBOT_HOSTS, attempts=2):
    """Return the telemetry URL of the first host that answers, else None.
    (The link keeps looking afterwards, so a miss here is not fatal.)"""
    for attempt in range(attempts):
        for host in hosts:
            try:
                ip = socket.gethostbyname(host)
            except OSError:
                print(f"  {host}: not found")
                continue
            url = f"http://{ip}/api/telemetry"
            try:
                r = requests.get(url, timeout=(1.0, 2.0))
                obj = r.json()
                print(f"  {host} -> {ip}: robot found ({obj.get('fw', '?')}, "
                      f"home_ip {obj.get('home_ip') or '-'}, rssi {obj.get('home_rssi', '-')})")
                log.info("discovery: robot at %s (%s)", ip, obj.get("fw", "?"))
                return url
            except (requests.exceptions.RequestException, ValueError) as e:
                print(f"  {host} -> {ip}: no answer")
                log.info("discovery: %s no answer (%s)", host, describe_exc(e))
    return None


def keep_mac_awake():
    """Stop macOS sleeping the display/system (and dropping Wi-Fi) while mapping.
    caffeinate exits by itself when this process ends."""
    if KEEP_MAC_AWAKE and sys.platform == "darwin" and shutil.which("caffeinate"):
        subprocess.Popen(["caffeinate", "-dims", "-w", str(os.getpid())])
        print("caffeinate: Mac will stay awake while the mapper runs")


# =============================================================================
# 9. VISUALISATION (matplotlib)
# =============================================================================

class MapperApp:
    def __init__(self, source, source_desc, record_path=None, robot_log=None,
                 session_dir=None):
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
        from matplotlib.patches import Circle, Patch, Polygon, Wedge

        # Free our keys from matplotlib's default shortcuts.
        for name in ("keymap.save", "keymap.quit", "keymap.pan", "keymap.home",
                     "keymap.fullscreen", "keymap.grid"):
            if name in matplotlib.rcParams:
                matplotlib.rcParams[name] = [k for k in matplotlib.rcParams[name]
                                             if k not in ("s", "q", "p", "r", " ", "escape")]

        self.plt = plt
        self.source = source
        self.source_desc = source_desc
        self.mapper = Mapper()
        self.worker = MapperWorker(self.mapper, source,
                                   TelemetryRecorder(record_path) if record_path else None)
        if record_path:
            print(f"Recording raw telemetry to {record_path}")
        self.robot_log = None
        if robot_log:
            url, path = robot_log
            link = source if isinstance(source, TelemetryClient) else None
            self.robot_log = RobotLogClient(url, path, on_loud=self._robot_event, link=link)
            print(f"Recording robot event log to {path}")
        self.session_dir = Path(session_dir) if session_dir else None
        self.closed = False
        self.gui_beat = time.monotonic()
        beats = {"mapper worker": lambda: self.worker.beat,
                 "GUI": lambda: self.gui_beat}
        limits = {"GUI": 10.0, "link": 10.0, "robot log": 10.0}
        if hasattr(source, "beat"):
            beats["link"] = lambda: source.beat
        if self.robot_log:
            beats["robot log"] = lambda: self.robot_log.beat
        self.watchdog = StallWatchdog(beats, limits)

        self.fig = plt.figure(figsize=(13, 8))
        self.fig.canvas.manager.set_window_title("AUTOBOT mapper")
        gs = self.fig.add_gridspec(1, 2, width_ratios=[2.3, 1])
        self.ax = self.fig.add_subplot(gs[0])
        self.info_ax = self.fig.add_subplot(gs[1])
        self.info_ax.axis("off")

        g = self.mapper.grid
        self.img = self.ax.imshow(g.probabilities(), origin="lower", extent=g.extent(),
                                  cmap="gray_r", vmin=0.0, vmax=1.0,
                                  interpolation="nearest", zorder=0)
        cb = self.fig.colorbar(self.img, ax=self.ax, fraction=0.035, pad=0.02)
        cb.set_label("P(occupied)")

        self.traj_line, = self.ax.plot([], [], color="tab:blue", lw=1.2, zorder=3)
        self.hits_line, = self.ax.plot([], [], "o", color="tab:orange", ms=2.0,
                                       alpha=0.6, zorder=4)
        self.ax.plot([0], [0], marker="+", color="tab:green", ms=12, mew=2, zorder=5)
        self.robot = Polygon(np.zeros((3, 2)), closed=True, fc="tab:red", ec="k", zorder=6)
        self.ax.add_patch(self.robot)
        self.sigma_circle = Circle((0, 0), 0.0, fill=False, ec="tab:red", ls="--",
                                   lw=1, zorder=5)
        self.ax.add_patch(self.sigma_circle)
        self.beam = Wedge((0, 0), 0.01, 0, 1, fc="gold", alpha=0.25, ec="none", zorder=2)
        self.ax.add_patch(self.beam)

        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x [m]  (start heading = +x)")
        self.ax.set_ylabel("y [m]  (left of start = +y)")
        self.ax.grid(True, alpha=0.25)
        self.ax.set_title("AUTOBOT approximate occupancy grid (dead reckoning, no SLAM)")
        self.ax.legend(handles=[
            Patch(fc="white", ec="0.5", label=f"likely free (p<{P_FREE_THRESHOLD})"),
            Patch(fc="0.5", ec="0.5", label="unknown (p~0.5)"),
            Patch(fc="black", ec="0.5", label=f"likely occupied (p>{P_OCC_THRESHOLD})"),
            Line2D([], [], color="tab:blue", label="estimated trajectory"),
            Line2D([], [], marker="o", ls="", color="tab:orange", label="ultrasonic returns"),
            Patch(fc="tab:red", label="robot (heading)"),
            Line2D([], [], ls="--", color="tab:red", label="2-sigma position"),
            Patch(fc="gold", alpha=0.4, label="current beam"),
            Line2D([], [], marker="+", ls="", color="tab:green", mew=2, label="start"),
        ], loc="upper left", fontsize=7, framealpha=0.85)

        self.info = self.info_ax.text(0.0, 1.0, "", va="top", ha="left",
                                      family="monospace", fontsize=8.5,
                                      transform=self.info_ax.transAxes)
        self.banner = self.ax.text(0.5, 0.02, "", ha="center", va="bottom", fontsize=10,
                                   color="white", fontweight="bold", zorder=10,
                                   transform=self.ax.transAxes,
                                   bbox=dict(boxstyle="round", fc="tab:red", alpha=0.85))
        self.banner.set_visible(False)

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.timer = self.fig.canvas.new_timer(interval=GUI_REFRESH_MS)
        self.timer.add_callback(self.tick)
        self.fig.tight_layout()

    def run(self):
        self.source.start()
        self.worker.start()
        if self.robot_log:
            self.robot_log.start()
        self.watchdog.start()
        self.timer.start()
        try:
            self.plt.show()
        except KeyboardInterrupt:
            log.info("Ctrl-C")
        finally:
            self.shutdown()

    def shutdown(self):
        """Runs once, after the window is gone (never inside a GUI callback
        and never holding mapper.lock): stop threads in order, then report."""
        if self.closed:
            return
        self.closed = True
        t0 = time.monotonic()
        log.info("shutting down")
        try:
            self.timer.stop()
        except Exception:
            pass
        self.source.stop()
        self.worker.stop()          # joins before closing the telemetry file
        if self.robot_log:
            self.robot_log.stop()
        self.watchdog.stop()
        log.info("threads stopped in %.2f s", time.monotonic() - t0)

        summary = []
        if hasattr(self.source, "summary"):
            summary.append("LINK  " + self.source.summary())
        summary.append(f"MAPPER  worker errors {self.worker.errors}, samples "
                       f"{self.mapper.counts.get('samples', 0)}")
        if self.robot_log and self.robot_log.events:
            summary.append(analyze_robot_log(self.robot_log.out_path))
        text = "\n".join(summary)
        print()
        print(text)
        log.info("summary:\n%s", text)
        if self.session_dir:
            (self.session_dir / "summary.txt").write_text(text + "\n")
        if AUTOSAVE_ON_EXIT and self.mapper.trajectory:
            print("Autosaved to", save_snapshot(self.mapper, None, OUTPUT_DIR, self.source_desc))
        print("Mapper stopped." + (f" Logs: {self.session_dir}/" if self.session_dir else ""))

    def _robot_event(self, desc):
        with self.mapper.lock:
            self.mapper.log("robot: " + desc)
        log.info("robot: %s", desc)

    def on_close(self, _event):
        # Only stop drawing here; shutdown() runs after plt.show() returns.
        try:
            self.timer.stop()
        except Exception:
            pass

    def on_key(self, event):
        k = (event.key or "").lower()
        if k in ("q", "escape"):
            self.plt.close(self.fig)   # outside the lock: closing joins nothing now
            return
        with self.mapper.lock:
            self._handle_key(k)
        self.redraw()

    def _handle_key(self, k):
        if k in (" ", "p"):
            self.mapper.paused = not self.mapper.paused
            self.mapper.log("map updates PAUSED" if self.mapper.paused else "map updates RESUMED")
        elif k == "r":
            self.mapper.reset()
        elif k == "s":
            try:
                self.redraw()
                out = save_snapshot(self.mapper, self.fig, OUTPUT_DIR, self.source_desc)
                self.mapper.log(f"saved {out}")
            except Exception as e:
                self.mapper.log(f"SAVE FAILED: {e}")

    def tick(self):
        self.gui_beat = time.monotonic()
        if self.closed:
            return
        try:
            self.redraw()
        except Exception:
            log.exception("redraw failed (GUI continues)")

    def redraw(self):
        with self.mapper.lock:
            self._redraw_locked()
        self.fig.canvas.draw_idle()

    def _redraw_locked(self):
        m = self.mapper
        g = m.grid
        self.img.set_data(g.probabilities())
        if m.grid_changed_shape:
            self.img.set_extent(g.extent())
            e = g.extent()
            self.ax.set_xlim(e[0], e[1])
            self.ax.set_ylim(e[2], e[3])
            m.grid_changed_shape = False

        if m.trajectory:
            step = max(1, len(m.trajectory) // 5000)
            t = m.trajectory[::step] + [m.trajectory[-1]]
            self.traj_line.set_data([r[2] for r in t], [r[3] for r in t])
        else:
            self.traj_line.set_data([], [])
        if m.hits:
            h = np.array(m.hits)
            self.hits_line.set_data(h[:, 0], h[:, 1])
        else:
            self.hits_line.set_data([], [])

        p = m.pose
        c, s = math.cos(p.theta), math.sin(p.theta)
        size = 0.12
        pts = np.array([[size, 0.0], [-size * 0.6, size * 0.5], [-size * 0.6, -size * 0.5]])
        rot = np.array([[c, -s], [s, c]])
        self.robot.set_xy(pts @ rot.T + [p.x, p.y])
        self.sigma_circle.set_center((p.x, p.y))
        self.sigma_circle.set_radius(2.0 * p.sigma_xy())

        if m.last_beam is not None:
            sx, sy, th, r = m.last_beam
            self.beam.set_center((sx, sy))
            self.beam.set_radius(max(0.01, r))
            self.beam.set_theta1(math.degrees(th) - BEAM_HALF_ANGLE_OCC_DEG)
            self.beam.set_theta2(math.degrees(th) + BEAM_HALF_ANGLE_OCC_DEG)
            self.beam.set_visible(True)
        else:
            self.beam.set_visible(False)

        self.info.set_text(self.status_text())
        warn = m.warnings(self.source.status.state()["state"])
        if self.watchdog.message:
            warn = warn + [f"STALL: {self.watchdog.message} (stacks in mapper.log)"]
        self.banner.set_text("\n".join(warn))
        self.banner.set_visible(bool(warn))

    def status_text(self):
        m = self.mapper
        st = self.source.status.state()
        s = m.last
        lines = ["AUTOBOT MAPPER  (read-only, no motor control)",
                 f"source : {self.source_desc}", ""]
        lat = "-" if st["latency_s"] is None else f"{st['latency_s'] * 1000:.0f} ms"
        age = "-" if st["age_s"] is None else f"{st['age_s']:.1f} s"
        lines += [f"LINK   {st['state']}",
                  f"  ok {st['ok']}  err {st['errors']}  latency {lat}  age {age}"]
        if st["last_error"] and st["state"] != "CONNECTED":
            lines.append(f"  last error: {st['last_error'][:40]}")
        if isinstance(self.source, TelemetryClient):
            src = self.source
            lines.append(f"  {src.mode or '-'}: rx {src.received}  lost {src.lost_samples}"
                         f"  outages {len(src.outages)}  reboots {src.reboots}")
        lines.append("")
        if s is None:
            lines.append("waiting for first telemetry sample ...")
        else:
            dist = "none" if s.distance_m is None else f"{s.distance_m * 100:6.1f} cm"
            filt = "-" if m.last_filtered is None else f"{m.last_filtered * 100:6.1f} cm"
            bias = "-" if s.gyro_bias_dps is None else f"{s.gyro_bias_dps:+.3f}"
            raw = "-" if s.gyro_z_raw_dps is None else f"{s.gyro_z_raw_dps:+.2f}"
            lines += [
                "MEASURED (from robot)",
                f"  mode      {s.mode}",
                f"  IMU       {'OK' if s.imu_ok else 'FAIL'}   cal: {s.gyro_cal}",
                f"  gyro z    {s.gyro_z_dps:+7.2f} dps  raw {raw}  bias {bias}",
                f"  robot yaw {s.yaw_deg:+8.2f} deg",
                f"  range     {dist}  ({s.us_status})",
                f"  filtered  {filt}",
                f"  cmd L/R   {s.left_cmd:+4.0f} / {s.right_cmd:+4.0f} PWM",
                "",
                "INFERRED (approximate)",
                f"  x, y      {m.pose.x:+6.2f}, {m.pose.y:+6.2f} m",
                f"  heading   {math.degrees(m.pose.theta):+7.1f} deg (vs start)",
                f"  1-sigma   pos {m.pose.sigma_xy() * 100:5.1f} cm  "
                f"hdg {math.degrees(m.pose.sigma_theta()):4.1f} deg",
                f"  odometer  {m.pose.odometer_m:+6.2f} m",
            ]
            if m.speed_cal.result:
                meas, est, sug = m.speed_cal.result
                lines += [f"  speed cal range {meas:.2f} m vs est {est:.2f} m",
                          f"            -> MAX_SPEED_MPS ~ {sug:.3f}",
                          "            (valid only driving straight,",
                          "             square-on at a flat wall)"]
        c = m.counts
        occ = int((m.grid.classes() == 2).sum())
        free = int((m.grid.classes() == 1).sum())
        lines += ["",
                  f"MAP    {'PAUSED' if m.paused else 'LIVE'}",
                  f"  {m.map_status[:44]}",
                  f"  beams {c['beams']}  no-echo {c['no_echo']}  skipped {c['skipped']}",
                  f"  cells occ {occ}  free {free}",
                  f"  grid {m.grid.nx}x{m.grid.ny} @ {m.grid.res * 100:.0f} cm",
                  f"  reboots {c['reboots']}  gaps {c['gaps']}  dropped {m.grid.dropped_updates}",
                  f"  stalls {c['stalls']}  IMU resets {c['imu_resets']}",
                  "",
                  "KEYS  space/p pause   r reset   s save   q quit",
                  "", "EVENTS"]
        lines += ["  " + e[:46] for e in list(m.events)[-6:]]
        return "\n".join(lines)


# =============================================================================
# 10. SELF-TEST (headless; no hardware)
# =============================================================================

def run_selftest():
    import http.server
    import tempfile

    import matplotlib
    matplotlib.use("Agg")

    results = []

    def check(name, cond, detail=""):
        results.append((name, bool(cond)))
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    print("Self-test")
    now = time.monotonic()
    good = SimulatedRobot().telemetry_dict()

    # --- parsing ---
    s, _ = parse_telemetry(good, now, time.time())
    check("parse valid sample", s is not None)
    s, err = parse_telemetry({"millis": 1}, now, time.time())
    check("missing fields rejected", s is None and "missing" in err, err)
    s, _ = parse_telemetry([1, 2], now, time.time())
    check("non-object rejected", s is None)
    bad = dict(good, yaw_deg="NaN")
    check("non-numeric yaw rejected", parse_telemetry(bad, now, time.time())[0] is None)
    nul = dict(good, distance_cm=None, distance_valid=False, us_status="no_echo")
    s, _ = parse_telemetry(nul, now, time.time())
    check("null distance -> invalid, not free", s is not None and s.distance_m is None
          and s.us_status == "no_echo")
    s, _ = parse_telemetry(dict(good, distance_cm=-5, distance_valid=True), now, time.time())
    check("negative distance rejected", s is not None and s.distance_m is None)

    # --- firmware / python field agreement ---
    ino = Path(__file__).resolve().parent / "autobot_mapping" / "autobot_mapping.ino"
    if ino.exists():
        import re
        text = ino.read_text()
        start = text.index("void renderTelemetry()")
        body = text[start:text.index("FW_VERSION,", start)]   # the snprintf format only
        fw_keys = set(re.findall(r'\\"(\w+)\\":', body))
        check("firmware emits all REQUIRED_FIELDS", set(REQUIRED_FIELDS) <= fw_keys,
              str(sorted(set(REQUIRED_FIELDS) - fw_keys)))
        check("simulator JSON keys == firmware JSON keys", set(good) == fw_keys,
              f"only fw: {sorted(fw_keys - set(good))}, only sim: {sorted(set(good) - fw_keys)}")

        # /api/samples: the firmware's column list must convert to a sample
        # that parses exactly like the /api/telemetry JSON.
        cstart = text.index('"#cols=')
        cend = text.index('\\n"', cstart)
        fw_cols = re.sub(r'"\s*"', "", text[cstart + 1:cend])[len("#cols="):].split(",")
        row_vals = {"seq": "7", "millis": "123456", "mode": "M", "distance_cm": "87.5",
                    "us_status": "ok", "us_seq": "42", "us_age_ms": "12", "imu_ok": "1",
                    "yaw_unwrapped_deg": "370.5", "roll_deg": "1.00", "pitch_deg": "-2.00",
                    "gyro_z_dps": "3.000", "gyro_z_raw_dps": "1.800", "gyro_bias_dps": "-1.2000",
                    "gyro_cal": "ok", "left_cmd": "100", "right_cmd": "-100",
                    "left_cmd_int": "12.500", "right_cmd_int": "-12.500"}
        row = ",".join(row_vals.get(c, "?") for c in fw_cols)
        obj = sample_row_to_dict(fw_cols, row, "99")
        ps, perr = parse_telemetry(obj, now, time.time()) if obj else (None, "row not converted")
        check("firmware /api/samples columns parse like telemetry",
              ps is not None and abs(ps.yaw_deg - 10.5) < 1e-9 and ps.distance_m == 0.875
              and ps.left_cmd_int == 12.5 and ps.boot_id == 99,
              f"cols {fw_cols}" if ps is None else perr)
        empty = dict(row_vals, distance_cm="", us_status="no_echo")
        obj = sample_row_to_dict(fw_cols, ",".join(empty.get(c, "?") for c in fw_cols), "99")
        ps, _ = parse_telemetry(obj, now, time.time())
        check("sample without echo -> invalid range, not free",
              ps is not None and ps.distance_m is None and ps.us_status == "no_echo")
        check("short sample row rejected", sample_row_to_dict(fw_cols, "1,2,3", "1") is None)
    else:
        print("  [SKIP] firmware file not found for field comparison")

    # --- grid beam geometry ---
    g = OccupancyGrid()
    g.integrate_beam(0.0, 0.0, 0.0, 1.0)
    ci = lambda x, y: g.L[g.world_to_cell(x, y)[1], g.world_to_cell(x, y)[0]]
    check("cell at the range is occupied-ish", ci(1.0, 0.0) > 0, f"L={ci(1.0, 0.0):.2f}")
    check("cell before the range is free-ish", ci(0.5, 0.0) < 0, f"L={ci(0.5, 0.0):.2f}")
    check("cell behind the obstacle untouched", ci(1.3, 0.0) == 0)
    check("cell outside the cone untouched", ci(0.5, 0.6) == 0)
    g2 = OccupancyGrid()
    g2.integrate_beam(0.0, 0.0, math.pi / 2, 1.0)
    check("beam at +90 deg hits +y (array orientation)",
          g2.L[g2.world_to_cell(0, 1.0)[1], g2.world_to_cell(0, 1.0)[0]] > 0
          and g2.L[g2.world_to_cell(1.0, 0)[1], g2.world_to_cell(1.0, 0)[0]] == 0)
    for _ in range(2000):
        g.integrate_beam(0.0, 0.0, 0.0, 1.0)
    check("log-odds stay clamped and finite",
          np.isfinite(g.L).all() and g.L.max() <= L_MAX and g.L.min() >= L_MIN,
          f"[{g.L.min():.2f}, {g.L.max():.2f}]")
    g3 = OccupancyGrid(size_m=2.0)
    g3.integrate_beam(0.8, 0.0, 0.0, 1.5)
    check("grid grows for out-of-bounds beam", g3.extent()[1] >= 0.8 + 1.5, str(g3.extent()))

    # --- mapper behaviours ---
    m = Mapper()
    base = dict(good, millis=MILLIS_MOD - 100, left_cmd_int=0.0, right_cmd_int=0.0,
                left_cmd=200, right_cmd=200)
    m.process(parse_telemetry(base, now, time.time())[0])
    nxt = dict(base, millis=100, left_cmd_int=40.0, right_cmd_int=40.0, us_seq=base["us_seq"] + 1)
    m.process(parse_telemetry(nxt, now, time.time())[0])
    expect = CommandMotionModel().forward_step(200, 200, 0.2)[0]
    check("millis rollover handled (dt = 0.2 s)", abs(m.pose.x - expect) < 1e-6,
          f"x={m.pose.x:.4f} expected {expect:.4f}")
    beams_before = m.counts["beams"]
    m.process(parse_telemetry(dict(nxt, millis=300, us_seq=nxt["us_seq"] + 1, distance_cm=None,
                                   distance_valid=False, us_status="no_echo"),
                              now, time.time())[0])
    check("invalid echo does not touch grid", m.counts["beams"] == beams_before)
    x_before = m.pose.x
    m.process(parse_telemetry(dict(nxt, millis=300, boot_id=999, left_cmd_int=0.0,
                                   right_cmd_int=0.0), now, time.time())[0])
    check("reboot detected, pose kept", m.counts["reboots"] == 1 and m.pose.x == x_before)
    m.paused = True
    lb = m.grid.L.copy()
    m.process(parse_telemetry(dict(nxt, millis=500, boot_id=999, us_seq=999), now, time.time())[0])
    check("pause freezes grid", np.array_equal(lb, m.grid.L))
    m.paused = False
    m.reset()
    check("reset clears grid, pose, trajectory",
          not m.grid.L.any() and m.pose.x == 0 and not m.trajectory)

    # --- yaw wrap without yaw_unwrapped_deg ---
    h = HeadingEstimator()
    a = parse_telemetry(dict(good, yaw_deg=179.0, yaw_unwrapped_deg=None), now, 0)[0]
    b = parse_telemetry(dict(good, yaw_deg=-179.0, yaw_unwrapped_deg=None), now, 0)[0]
    h.delta(a)
    d = math.degrees(h.delta(b))
    check("yaw wrap +179 -> -179 is +2 deg", abs(d - 2.0) < 1e-6, f"{d:.3f}")

    # --- simulated room ---
    robot = SimulatedRobot(seed=3)
    m = Mapper()
    t = 0.0
    while t < 180.0:
        t += 1.0 / POLL_HZ
        robot.run_until(t)
        m.process(parse_telemetry(robot.telemetry_dict(), t, time.time())[0])
    occ_cells = np.argwhere(m.grid.classes() == 2)
    xs = m.grid.x_min + (occ_cells[:, 1] + 0.5) * m.grid.res
    ys = m.grid.y_min + (occ_cells[:, 0] + 0.5) * m.grid.res

    def seg_dist(px, py, a, b):
        ax, ay = a
        bx, by = b
        ex, ey = bx - ax, by - ay
        u = np.clip(((px - ax) * ex + (py - ay) * ey) / (ex * ex + ey * ey), 0, 1)
        return np.hypot(px - ax - u * ex, py - ay - u * ey)

    dmin = np.min([seg_dist(xs, ys, a, b) for a, b in SimulatedRobot.WALLS], axis=0)
    frac = float(np.mean(dmin < 0.25)) if len(dmin) else 0.0
    pos_err = math.hypot(m.pose.x - robot.x, m.pose.y - robot.y)
    print(f"        sim 180 s: {len(occ_cells)} occupied cells, {frac:.0%} within 25 cm "
          f"of a true wall; final position error {pos_err:.2f} m, "
          f"model 2-sigma {2 * m.pose.sigma_xy():.2f} m")
    check("simulated walls appear in the map", len(occ_cells) > 50 and frac > 0.6)
    check("no NaN anywhere", np.isfinite(m.grid.L).all() and math.isfinite(m.pose.x))

    # --- save ---
    with tempfile.TemporaryDirectory() as tmp:
        out = save_snapshot(m, None, tmp, "selftest")
        files = {p.name for p in out.iterdir()}
        need = {"map.png", "occupancy_grid.npy", "occupancy_grid.csv", "trajectory.csv"}
        check("snapshot files written", need <= files, str(sorted(files)))
        arr = np.load(out / "occupancy_grid.npy")
        check("npy shape matches grid", arr.shape == m.grid.L.shape)
        with open(out / "trajectory.csv") as f:
            rows = [r for r in csv.reader(line for line in f if not line.startswith("#"))]
        check("trajectory.csv rows", rows[0] == TRAJ_FIELDS and len(rows) == len(m.trajectory) + 1)

    # --- HTTP client against a local fake robot ---
    replies = deque([("200", json.dumps(good)), ("500", "oops"), ("200", "not json"),
                     ("200", ""), ("200", json.dumps({"millis": 5}))])

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith(("/api/samples", "/api/diag")):   # pre-2.0 firmware
                self.send_response(404)
                self.end_headers()
                return
            code, body = replies.popleft() if replies else ("200", json.dumps(good))
            self.send_response(int(code))
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body.encode())

        def log_message(self, *a):
            pass

    srv = http.server.HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    client = TelemetryClient(f"http://127.0.0.1:{srv.server_address[1]}/api/telemetry", 20)
    client.start()
    time.sleep(2.5)
    client.stop()
    srv.shutdown()
    st = client.status.state()
    check("HTTP client survives 500 / bad JSON / empty / missing fields",
          st["ok"] >= 2 and st["errors"] >= 4, f"ok={st['ok']} errors={st['errors']}")
    client = TelemetryClient("http://127.0.0.1:9/api/telemetry", 10)
    client.start()
    time.sleep(1.0)
    client.stop()
    check("unreachable robot -> errors counted, no crash", client.status.state()["errors"] >= 1)

    # --- stall detection: commanded forward, range constant ---
    m = Mapper()
    st = dict(good, left_cmd=200, right_cmd=200, distance_cm=80.0, distance_valid=True,
              us_status="ok", yaw_unwrapped_deg=0.0, yaw_deg=0.0, gyro_z_dps=0.0)
    for i in range(40):                                   # 5 s at 8 Hz
        smp = dict(st, millis=1000 + i * 125, us_seq=100 + i,
                   left_cmd_int=200 * i * 0.125, right_cmd_int=200 * i * 0.125)
        m.process(parse_telemetry(smp, i * 0.125, time.time())[0])
    free_run = CommandMotionModel().forward_step(200, 200, 1.5)[0]
    check("stall: position frozen when range does not shrink",
          m.stall.stalled and m.pose.x < free_run + 0.02,
          f"x={m.pose.x:.2f} m (unstalled model would give {39 * 0.125 * free_run / 1.5:.2f})")
    check("stall: banner explains it", any("STALLED" in w for w in m.warnings("CONNECTED")))
    check("link loss shows in banner", any("NO TELEMETRY" in w for w in m.warnings("STALE")))

    # --- background worker + record/replay round trip ---
    with tempfile.TemporaryDirectory() as tmp:
        rec_path = Path(tmp) / "rec.jsonl"
        src = SimulatedSource(10)
        mw = Mapper()
        worker = MapperWorker(mw, src, TelemetryRecorder(rec_path))
        src.start()
        worker.start()
        time.sleep(2.0)
        src.stop()
        time.sleep(0.4)
        worker.stop()
        n_live = mw.counts["samples"]
        check("worker maps without any GUI", n_live >= 10 and mw.counts["beams"] > 0,
              f"samples={n_live} beams={mw.counts['beams']}")
        rp = ReplaySource(rec_path, speed=50.0)
        mr = Mapper()
        w2 = MapperWorker(mr, rp)
        rp.start()
        w2.start()
        deadline = time.monotonic() + 5
        while not rp.finished and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.5)
        w2.stop()
        rp.stop()
        check("replay reproduces the recorded run",
              mr.counts["samples"] == n_live and abs(mr.pose.x - mw.pose.x) < 1e-6,
              f"{mr.counts['samples']} vs {n_live} samples")

    # --- robot event log: decode, download (fake robot), analyse ---
    if ino.exists():
        import re
        names = re.search(r"EV_NAMES\[EV_COUNT\] = \{(.*?)\};", ino.read_text(), re.S)
        fw_types = re.findall(r'"(\w+)"', names.group(1)) if names else []
        undecoded = [t for t in fw_types
                     if describe_event(t, "x", 0, 0, 0.0).startswith(f"{t} x")]
        check("every firmware event type has a decoder", fw_types and not undecoded,
              f"{len(fw_types)} types, undecoded {undecoded}")

    def make_events(noisy_spikes):
        ev, seq = [], 0
        for i in range(160):                      # 10 s of ranges at 16 Hz
            ms = 1000 + i * 62
            motors_on = 2000 < ms < 7000
            r = 80.0 - (ms - 2000) / 1000 * 10 if motors_on else 80.0
            tag = "ok"
            if motors_on and noisy_spikes and i % 5 == 0:
                r = 6.0                           # noise spike
            if motors_on and i % 9 == 0:
                tag, r = "no_echo", -1.0
            ev.append([ms, "RANGE", tag, i, 0, r])
        ev += [[1990, "JOY", "phone", 60, 0, 0], [2000, "MOTOR", "phone", 110, 118, 0],
               [3000, "GUARD", "latch", 2, 60, 6.0], [3000, "MOTOR", "guard", 0, 0, 0],
               [3500, "GUARD", "release_unconfirmed", 0, 60, 70.0],
               [3500, "MOTOR", "phone", 110, 118, 0],
               [6900, "LINKGAP", "drive", 850, 0, 0],
               [7000, "WATCHDOG", "manual", 760, 60, 0], [7000, "MOTOR", "watchdog_manual", 0, 0, 0],
               [7100, "ESTOP", "estop_page_hidden", 0, 0, 0],
               [7200, "STALL", "http_log", 4800, 1, 0], [7200, "MOTOR", "deadman", 0, 0, 0]]
        ev.sort(key=lambda e: e[0])
        return [[seq] + e for seq, e in enumerate(ev)]

    boot1 = make_events(True)
    boot2 = [[0, 0, "PREV_BOOT", "begin", 2, 0, 0], [1, 9000, "PREV:MOTOR", "phone", 255, 255, 0],
             [2, 9010, "PREV:RANGE", "ok", 0, 0, 50.0], [3, 0, "PREV_BOOT", "end", 2, 0, 0],
             [4, 5, "BOOT", "brownout", 0, 0, 0]]
    state = {"boot": "111", "calls": 0}

    class LogHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            from urllib.parse import parse_qs, urlparse
            state["calls"] += 1
            if state["calls"] == 6:
                state["boot"] = "222"
            evs = boot1 if state["boot"] == "111" else boot2
            since = int(parse_qs(urlparse(self.path).query).get("since", ["0"])[0])
            # pretend the robot ring only holds events from seq 3 on (some lost)
            oldest = 3 if state["boot"] == "111" else 0
            start = max(since, oldest)
            chunk = evs[start:start + 60]
            nxt = start + len(chunk)
            body = f"#boot={state['boot']},next={nxt},oldest={oldest},now=1,fw=test\n"
            body += "".join(",".join(str(x) for x in e[:6]) + f",{e[6]:.2f}\n" for e in chunk)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(body.encode())

        def log_message(self, *a):
            pass

    with tempfile.TemporaryDirectory() as tmp:
        srv = http.server.HTTPServer(("127.0.0.1", 0), LogHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        loud = []
        rl = RobotLogClient(f"http://127.0.0.1:{srv.server_address[1]}/api/log",
                            Path(tmp) / "robotlog.csv", on_loud=loud.append, period=0.1)
        rl.start()
        time.sleep(2.0)
        rl.stop()
        srv.shutdown()
        logged = read_robot_log(Path(tmp) / "robotlog.csv")
        types = {r["type"] for r in logged}
        check("robot log downloaded across a reboot",
              {"RANGE", "MOTOR", "WATCHDOG", "BOOT", "PREV:MOTOR"} <= types,
              f"{len(logged)} rows, types {sorted(types)}")
        check("important events reported live",
              any("SAFETY STOP" in d for d in loud) and any("BOOTED" in d for d in loud))
        report = analyze_robot_log(Path(tmp) / "robotlog.csv")
        check("report: names the stop causes",
              "watchdog_manual" in report and "guard" in report and "brownout" in report)
        check("report: program freeze reported with location",
              "PROGRAM FREEZES" in report and "http_log" in report)
        check("report: detects motor noise on the sensor", "ELECTRICAL NOISE" in report)
        check("report: timeline chart written",
              (Path(tmp) / "robotlog.timeline.png").exists())
        if "-v" in sys.argv:
            print(report)

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    return 0 if not failed else 1


# =============================================================================
# MAIN
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="AUTOBOT live 2D occupancy-grid mapper")
    ap.add_argument("--url", default=None,
                    help="robot address, e.g. http://192.168.1.200 "
                         "(default: try 192.168.1.200, then 192.168.4.1)")
    ap.add_argument("--rate", type=float, default=POLL_HZ,
                    help="poll rate in Hz for pre-2.0 firmware (5-10)")
    ap.add_argument("--simulate", action="store_true", help="use a simulated robot")
    ap.add_argument("--selftest", action="store_true", help="run headless checks and exit")
    ap.add_argument("-v", action="store_true", help="verbose self-test")
    ap.add_argument("--replay", metavar="FILE.jsonl", help="replay a recorded session")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--no-record", action="store_true", help="do not record raw telemetry")
    ap.add_argument("--log-only", action="store_true",
                    help="no map window: record everything, print events, report at Ctrl-C")
    ap.add_argument("--analyze", metavar="ROBOTLOG.csv", help="report on a saved robot log")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(run_selftest())
    if args.analyze:
        print(analyze_robot_log(args.analyze))
        return

    live = not (args.simulate or args.replay)
    session_dir = start_session_logging("log-only" if args.log_only else "map") if live else None
    if args.url is None and live:
        print("Looking for the robot ...")
        args.url = find_robot()
        if args.url is None:
            args.url = TELEMETRY_URL
            print(f"Robot not found yet - will keep looking ({', '.join(ROBOT_HOSTS)}).\n"
                  "Is the robot on, and is this Mac on the same Wi-Fi "
                  "(home network or BotController)?")
            log.warning("robot not found at start; the link keeps searching %s", ROBOT_HOSTS)
    args.url = args.url or TELEMETRY_URL
    log_url = base_url(args.url) + "/api/log"
    if args.log_only:
        run_log_only(args.url, log_url, min(10.0, max(1.0, args.rate)), session_dir)
        return

    rate = min(10.0, max(1.0, args.rate))
    record = None
    robot_log = None
    if args.replay:
        source, desc = ReplaySource(args.replay, args.speed), f"REPLAY {args.replay}"
    elif args.simulate:
        source, desc = SimulatedSource(rate), "SIMULATOR (no hardware)"
    else:
        source, desc = TelemetryClient(args.url, rate, session_dir=session_dir), base_url(args.url)
        if RECORD_TELEMETRY and not args.no_record:
            record = session_dir / "telemetry.jsonl"
        if ROBOT_LOG and not args.no_record:
            robot_log = (log_url, session_dir / "robotlog.csv")
    if not args.replay:
        keep_mac_awake()
    print(f"AUTOBOT mapper - source: {desc}")
    print("Keys in the map window: space/p pause, r reset, s save, q quit")
    MapperApp(source, desc, record, robot_log, session_dir).run()


if __name__ == "__main__":
    main()
