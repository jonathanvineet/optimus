"""Chaos tests: the Mac link must survive every kind of robot failure.

Runs the real TelemetryClient + MapperWorker + RobotLogClient (no GUI)
against tools/fake_robot.py, injects a failure, and checks that:
  - no thread dies and nothing hangs (faulthandler kills a hung run),
  - the link comes back within a few seconds of the robot recovering,
  - samples arrive in order, never twice, and without gaps (the robot
    buffers ~20 s, so short outages lose nothing),
  - every outage and reboot is written to the log,
  - shutdown is quick even in the middle of a failure.

    pytest tests/ -v
"""

from __future__ import annotations

import faulthandler
import logging
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import mac_mapper as mm  # noqa: E402
from fake_robot import FakeRobot  # noqa: E402

HANG_LIMIT_S = 60


@pytest.fixture(autouse=True)
def hang_guard():
    # A deadlock fails loudly with every thread's stack instead of hanging CI.
    faulthandler.dump_traceback_later(HANG_LIMIT_S, exit=True)
    yield
    faulthandler.cancel_dump_traceback_later()


@pytest.fixture
def logfile(tmp_path):
    path = tmp_path / "mapper.log"
    h = logging.FileHandler(path)
    h.setLevel(logging.DEBUG)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(threadName)s] %(message)s"))
    mm.log.setLevel(logging.DEBUG)
    mm.log.addHandler(h)
    yield path
    mm.log.removeHandler(h)
    h.close()


@pytest.fixture
def robot():
    fake = FakeRobot().start()
    yield fake
    fake.stop()


class Rig:
    """Link + worker + robot log, exactly as the map window runs them."""

    def __init__(self, url, tmp_path):
        self.link = mm.TelemetryClient(url, session_dir=tmp_path)
        self.mapper = mm.Mapper()
        self.seen = []                      # sample_seq values in processing order
        orig = self.mapper.process

        def spy(s):
            self.seen.append((s.boot_id, s.raw.get("sample_seq")))
            return orig(s)

        self.mapper.process = spy
        self.worker = mm.MapperWorker(self.mapper, self.link,
                                      mm.TelemetryRecorder(tmp_path / "telemetry.jsonl"))
        self.rlog = mm.RobotLogClient(None, tmp_path / "robotlog.csv", link=self.link,
                                      period=0.2)

    def start(self):
        self.link.start()
        self.worker.start()
        self.rlog.start()
        return self

    def threads_alive(self):
        return all(t._thread.is_alive() for t in (self.link, self.worker, self.rlog))

    def stop(self):
        t0 = time.monotonic()
        self.link.stop()
        self.worker.stop()
        self.rlog.stop()
        return time.monotonic() - t0

    def wait_connected(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.link.status.state()["state"] == "CONNECTED":
                return time.monotonic() - t0
            time.sleep(0.05)
        return None


def assert_in_order_no_dupes(seen):
    by_boot = {}
    for boot, seq in seen:
        by_boot.setdefault(boot, []).append(seq)
    for boot, seqs in by_boot.items():
        assert seqs == sorted(seqs), f"boot {boot}: samples out of order"
        assert len(seqs) == len(set(seqs)), f"boot {boot}: duplicate samples"
    return by_boot


def test_lossless_in_order(robot, tmp_path, logfile):
    rig = Rig(robot.url, tmp_path).start()
    assert rig.wait_connected(3.0) is not None
    time.sleep(3.0)
    assert rig.link.mode == "samples"
    assert rig.stop() < 3.0
    by_boot = assert_in_order_no_dupes(rig.seen)
    seqs = by_boot[robot.boot_id]
    assert seqs[-1] - seqs[0] + 1 == len(seqs), "gap in the sample sequence"
    assert len(seqs) > 60                    # ~25 Hz for >3 s
    assert rig.link.lost_samples == 0 and rig.link.dropped_local == 0
    assert rig.worker.errors == 0


@pytest.mark.parametrize("mode", ["drop", "stall", "slow", "garbage"])
def test_survives_chaos(robot, tmp_path, logfile, mode):
    rig = Rig(robot.url, tmp_path).start()
    assert rig.wait_connected(3.0) is not None
    time.sleep(1.0)

    robot.chaos(mode, 3.0)
    time.sleep(3.2)
    assert rig.threads_alive(), f"a thread died during {mode}"

    back = rig.wait_connected(6.0)
    assert back is not None, f"link did not recover after {mode}"
    assert back < 4.0, f"recovery took {back:.1f} s"
    time.sleep(1.0)

    assert rig.threads_alive()
    assert rig.stop() < 4.0
    by_boot = assert_in_order_no_dupes(rig.seen)
    seqs = by_boot[robot.boot_id]

    text = logfile.read_text()
    if mode in ("drop", "stall"):
        # Short outage: the robot's buffer covers it, nothing lost.
        assert seqs[-1] - seqs[0] + 1 == len(seqs), "samples lost during a short outage"
        assert "link DOWN" in text and "link UP again" in text
    if mode == "garbage":
        assert rig.link.bad_rows > 0 or "link DOWN" in text
    assert rig.worker.errors == 0


def test_reboot_detected_and_logged(robot, tmp_path, logfile):
    rig = Rig(robot.url, tmp_path).start()
    assert rig.wait_connected(3.0) is not None
    time.sleep(1.0)
    old_boot = robot.boot_id
    robot.chaos("reset", 0)
    time.sleep(2.5)
    assert rig.threads_alive()
    assert rig.stop() < 3.0

    assert rig.link.reboots == 1
    by_boot = assert_in_order_no_dupes(rig.seen)
    assert old_boot in by_boot and robot.boot_id in by_boot
    assert by_boot[robot.boot_id][0] == 0, "new boot not read from its first sample"
    assert rig.mapper.counts["reboots"] >= 1
    text = logfile.read_text()
    assert "ROBOT REBOOTED" in text
    assert "robot diag (reboot)" in text
    assert list(tmp_path.glob("diag_*_reboot.json"))
    rows = mm.read_robot_log(tmp_path / "robotlog.csv")
    assert any(r["type"].startswith("PREV:") for r in rows), "pre-reset events not saved"


def test_robot_unreachable_from_start(tmp_path, logfile):
    # Nothing listens on this port: errors are counted and logged, no crash.
    rig = Rig("http://127.0.0.1:9", tmp_path).start()
    time.sleep(4.5)
    assert rig.threads_alive()
    assert rig.link.status.state()["errors"] >= 2
    assert rig.stop() < 3.0
    text = logfile.read_text()
    assert "link DOWN" in text
    assert "rediscovery" in text            # it went looking for the robot


def test_shutdown_during_stall_is_quick(robot, tmp_path, logfile):
    rig = Rig(robot.url, tmp_path).start()
    assert rig.wait_connected(3.0) is not None
    robot.chaos("stall", 20.0)
    time.sleep(0.5)
    took = rig.stop()
    assert took < 4.0, f"shutdown took {took:.1f} s while the robot was frozen"
    assert not any(t.name in ("link", "mapper", "robotlog") and t.is_alive()
                   for t in threading.enumerate())


def test_stall_watchdog_dumps_stacks(logfile):
    gate = threading.Event()
    stuck_since = time.monotonic()
    t = threading.Thread(target=gate.wait, name="stuck-worker", daemon=True)
    t.start()
    wd = mm.StallWatchdog({"stuck": lambda: stuck_since}, {"stuck": 1.0})
    wd.start()
    time.sleep(2.5)
    wd.stop()
    gate.set()
    text = logfile.read_text()
    assert "STALL: stuck" in text
    assert "--- thread stuck-worker ---" in text   # where it is stuck
    assert "stuck stalled" in wd.message


def test_phone_websocket_protocol(robot):
    import wslite
    c = wslite.WsClient("127.0.0.1", robot.port)
    c.send_text("ping,7")
    got = [c.recv_text() for _ in range(5)]
    assert "pong,7" in got
    assert any(m and m.startswith("s,") for m in got)
    for i in range(1, 6):
        c.send_text(f"d,{i},10,0")
    c.send_text("stop,button")
    time.sleep(0.3)
    c.close()
    assert [f[0] for f in robot.drive_frames] == [1, 2, 3, 4, 5]
    assert robot.stops == ["button"]
