"""The real phone page (from the .ino) in headless Chrome, against the fake robot.

Checks the link behaviour that used to fail: stick input is sent at a steady
20 Hz (not per touch event), release sends stops, the ONLINE/RECONNECTING
indicator follows the real link, the socket reconnects by itself after an
outage, and E-STOP goes out. Skipped when Chrome is not installed.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
import requests

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import wslite  # noqa: E402
from fake_robot import FakeRobot  # noqa: E402

CHROME_PATHS = ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                shutil.which("google-chrome") or "", shutil.which("chromium") or ""]
CHROME = next((p for p in CHROME_PATHS if p and Path(p).exists()), None)
DEBUG_PORT = 9333

pytestmark = pytest.mark.skipif(CHROME is None, reason="Chrome not installed")


class Page:
    """Tiny Chrome DevTools Protocol client over wslite."""

    def __init__(self, ws_url):
        hostport, path = ws_url[len("ws://"):].split("/", 1)
        host, port = hostport.split(":")
        self.ws = wslite.WsClient(host, int(port), "/" + path, timeout=10)
        self.ws.sock.settimeout(10)
        self.next_id = 0

    def call(self, method, **params):
        self.next_id += 1
        mid = self.next_id
        self.ws.send_text(json.dumps({"id": mid, "method": method, "params": params}))
        while True:
            msg = json.loads(self.ws.recv_text())
            if msg.get("id") == mid:
                if "error" in msg:
                    raise RuntimeError(msg["error"])
                return msg.get("result", {})

    def js(self, expr):
        r = self.call("Runtime.evaluate", expression=expr, returnByValue=True)
        if "exceptionDetails" in r:
            raise RuntimeError(r["exceptionDetails"])
        return r["result"].get("value")


@pytest.fixture
def setup():
    robot = FakeRobot().start()
    profile = tempfile.mkdtemp()
    chrome = subprocess.Popen(
        [CHROME, "--headless=new", f"--remote-debugging-port={DEBUG_PORT}",
         "--remote-allow-origins=*", f"--user-data-dir={profile}", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            try:
                pages = requests.get(f"http://127.0.0.1:{DEBUG_PORT}/json", timeout=1).json()
                target = next(p for p in pages if p.get("type") == "page")
                break
            except Exception:
                time.sleep(0.1)
        else:
            pytest.fail("Chrome DevTools did not come up")
        page = Page(target["webSocketDebuggerUrl"])
        page.call("Page.enable")
        page.call("Page.navigate", url=robot.url + "/")
        yield robot, page
    finally:
        chrome.terminate()
        chrome.wait(timeout=10)
        robot.stop()
        shutil.rmtree(profile, ignore_errors=True)


def wait_for(page, expr, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        v = page.js(expr)
        if v:
            return v
        time.sleep(0.1)
    return page.js(expr)


def test_phone_page_link(setup):
    robot, page = setup

    link = wait_for(page, "document.getElementById('link').textContent.startsWith('ONLINE') "
                          "&& document.getElementById('link').textContent")
    assert link and link.startswith("ONLINE"), f"link indicator: {page.js('document.getElementById(\"link\").textContent')}"

    # Hold the throttle for 1 s: a steady ~20 frames, not one per touch event.
    n0 = len(robot.drive_frames)
    page.js("throttle = 60; steering = 0;")
    time.sleep(1.0)
    page.js("throttle = 0; stickReleased();")
    time.sleep(0.5)
    frames = robot.drive_frames[n0:]
    moving = [f for f in frames if f[1] == 60]
    assert 15 <= len(moving) <= 24, f"{len(moving)} drive frames in 1 s (want ~20)"
    gaps = [b[3] - a[3] for a, b in zip(moving, moving[1:])]
    assert max(gaps) < 0.15, f"worst gap between drive frames {max(gaps) * 1000:.0f} ms"
    seqs = [f[0] for f in frames]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), "drive seq not increasing"
    stops = [f for f in frames if f[1] == 0 and f[2] == 0]
    assert len(stops) == 3, f"release sent {len(stops)} stop frames (want 3)"

    # Robot unreachable: the page must say so and release the sticks.
    robot.chaos("drop", 3.0)
    lost = wait_for(page, "document.getElementById('link').textContent === 'RECONNECTING'", 3.0)
    assert lost, "page did not notice the lost link"
    assert page.js("throttle === 0 && steering === 0")
    status = page.js("document.getElementById('status').textContent")
    assert "LINK LOST" in status

    # Robot back: reconnects without a reload, and tells the robot it did.
    back = wait_for(page, "document.getElementById('link').textContent.startsWith('ONLINE')", 6.0)
    assert back, "page did not reconnect after the outage"
    time.sleep(0.5)
    assert any(m.startswith(("reconnected", "restored")) for m in robot.phone_logs), robot.phone_logs

    # Robot frozen: the socket stays open but goes silent (like a half-open
    # TCP link after a Wi-Fi drop). The page must drop it and reconnect.
    robot.phone_logs.clear()
    robot.chaos("stall", 4.0)
    lost = wait_for(page, "document.getElementById('link').textContent === 'RECONNECTING'", 3.0)
    assert lost, "page did not notice the silent link"
    back = wait_for(page, "document.getElementById('link').textContent.startsWith('ONLINE')", 8.0)
    assert back, "page did not recover from a silent (half-open) socket"
    time.sleep(0.5)
    assert any(m.startswith("reconnected") for m in robot.phone_logs), \
        f"no forced reconnect after the silent link: {robot.phone_logs}"

    # E-STOP reaches the robot.
    page.js("emergencyStop('button')")
    time.sleep(0.3)
    assert robot.stops[-1] == "button"
