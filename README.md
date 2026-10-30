# optimus — AUTOBOT robot + live 2D room mapping

An ESP32 differential-drive robot. You drive it with the phone controller, and a Mac builds an
**approximate** 2D occupancy grid from the HC-SR04 ultrasonic range and the MPU6050 gyro heading.

| File | What it is |
|---|---|
| `autobot_mapping/autobot_mapping.ino` | ESP32 firmware: the phone controller (unchanged UI and behaviour), gyro yaw, background ultrasonic, `/api/telemetry` |
| `mac_mapper.py` | Mac live mapper (read-only: it never drives the motors) |
| `requirements.txt` | Python dependencies (numpy, matplotlib, requests) |
| `sketch_oct27b/`, `sketch_oct27a/` | Earlier controller-only sketches, kept for reference |

> **What this is and is not.** Heading comes from integrating the gyro: it is relative to the start
> pose and it drifts, because there is no magnetometer. Position is dead reckoning from the
> **motor commands**, not from wheel rotation, because the robot has no encoders. Wheel slip,
> stalls, battery sag and floor type are all invisible to it. The mapper does **no** scan
> matching and **no** loop closure. Expect a map that is useful for the shape of a room over
> 1–3 minutes of slow driving, with errors of tens of centimetres that grow over time. It is not
> centimetre-accurate SLAM.

---

## 1. Wiring (unchanged)

| Component | ESP32 (Wemos D1 R32) |
|---|---|
| L298N ENA / IN1 / IN2 (left motor, OUT1/OUT2) | GPIO 25 / 26 / 27 |
| L298N IN3 / IN4 / ENB (right motor, OUT3/OUT4) | GPIO 17 / 16 / 23 |
| HC-SR04 TRIG | GPIO 18 |
| HC-SR04 ECHO | GPIO 19 **through a voltage divider** |
| MPU6050 SDA / SCL | GPIO 21 / GPIO 22 |
| MPU6050 VCC / GND | 3V3 / GND |

* **Use a divider on ECHO.** The HC-SR04 runs on 5 V and its ECHO pin outputs 5 V. ESP32 pins
  are 3.3 V only. Use, for example, **1 kΩ from ECHO to GPIO19 and 2 kΩ from GPIO19 to GND**
  (5 V × 2/3 ≈ 3.3 V), or a level shifter. TRIG can be driven straight from GPIO18.
* The MPU6050's AD0 pin must be low or unconnected (I²C address 0x68).
* **Mount the MPU6050 flat, chip side up.** If it is mounted upside down, see §8 (yaw sign).
* All grounds must be shared: ESP32, L298N, HC-SR04 and the battery negative.
* Keep the motor supply within the motors' rated voltage. The L298N drops roughly 2 V.

## 2. Arduino IDE setup

1. Arduino IDE 2.x → **Boards Manager** → install **"esp32 by Espressif Systems" version 3.x**.
   This sketch was compiled against **3.3.7**. It uses the 3.x `ledcAttach()`/`ledcWrite(pin, …)`
   API and will not build on core 2.x.
2. **Tools → Board → esp32 → "WEMOS D1 R32"**. "ESP32 Dev Module" also works.
3. No extra libraries are needed. The MPU6050 is read through direct register access with `Wire`.

**Compile from the command line** (uses the `arduino-cli` that ships inside Arduino IDE):

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn esp32:esp32:d1_uno32 autobot_mapping
```

## 3. Upload and serial output

1. Open `autobot_mapping/autobot_mapping.ino`, plug in USB, choose the port, then click Upload.
   From the command line, find the port with `"$CLI" board list` and upload with:
   ```bash
   "$CLI" upload --fqbn esp32:esp32:d1_uno32 -p /dev/cu.usbserial-XXXX autobot_mapping
   ```
2. Open the Serial Monitor at **115200 baud**. **Keep the robot still for the first ~3 s**
   while the gyro calibrates.

Expected output (the numbers will differ):

```
=========================================
AUTOBOT autobot_mapping-1.5 booting (reset reason: power_on)
Keep the robot STILL for gyro calibration.
=========================================
[IMU] MPU6050 found (WHO_AM_I=0x68)
[IMU] Startup calibration 1/3
[IMU] Gyro bias calibration started - keep the robot still.
[IMU] Gyro Z bias = -1.2345 dps (noise 0.041 dps, 400 samples)

AUTOBOT MOBILE UNIT ONLINE
Wi-Fi: BotController
Controller: http://192.168.4.1
Telemetry:  http://192.168.4.1/api/telemetry
[STAT] t=    4.0s MANUAL yaw=    0.00 rate=   0.00 bias=-1.234 imu=OK cal=ok rst=0 | us= 123.4cm ok           | L=   0 R=   0 | loop=40000Hz clients=0 heap=230000
```

One `[STAT]` line prints per second. Other messages you may see:
`[SAFETY] Drive heartbeat lost - motors stopped.` (the watchdog),
`[IMU] ERROR: MPU6050 not responding ...` (wiring), `[MODE] AUTO`.

## 4. Connect the phone and the Mac

Both devices join the robot's own Wi-Fi network:

* SSID **`BotController`**, password **`robot1234`**
* Phone: open **http://192.168.4.1** to get the joysticks, as before.
* Mac: Wi-Fi menu → BotController. The Mac loses its internet connection while joined, so
  install the Python packages (§6) **before** you switch networks.

### 4b. Home Wi-Fi (since v1.4, replaces BotController)

If `autobot_mapping/secrets.h` holds your home Wi-Fi name and password, the robot **only**
joins your router, at the fixed address **http://192.168.1.200** (router 192.168.1.1).
The Mac and the phone stay on home Wi-Fi with internet. After a robot reset both sides
reconnect by themselves.

1. `autobot_mapping/secrets.h` is git-ignored; `secrets.example.h` is the template.
2. Upload, then run `python mac_mapper.py`. It tries 192.168.1.200 first.
3. Phone: open `http://192.168.1.200`.

* The ESP32 only supports **2.4 GHz** Wi-Fi.
* If another device ever takes .200, change `ROBOT_IP` in the sketch and `ROBOT_HOSTS` in
  `mac_mapper.py`. Reserving .200 in the router's DHCP settings prevents that.
* If the robot can't join (wrong password, out of range), it can't be reached at all. Check
  the Serial Monitor. The drive watchdog still stops the motors when the link drops.
* To go back to the BotController network, empty `HOME_WIFI_SSID` in `secrets.h`. Then use
  `http://192.168.4.1` again.

## 5. Check telemetry in a browser

On the Mac, open **http://192.168.4.1/api/telemetry**, or run:

```bash
curl -s http://192.168.4.1/api/telemetry | python3 -m json.tool
```

You should see something like this (abridged):

```json
{ "millis": 51234, "mode": "manual", "distance_cm": 87.3, "distance_valid": true,
  "us_status": "ok", "imu_ok": true, "yaw_deg": 12.345, "gyro_z_dps": 0.0,
  "gyro_z_raw_dps": -1.231, "gyro_bias_dps": -1.2345, "gyro_cal": "ok",
  "left_cmd": 0, "right_cmd": 0, "left_cmd_int": 1234.5, "right_cmd_int": 1301.2, ... }
```

* `distance_cm` is `null` when there is no valid echo. `us_status` gives the reason: `no_echo`
  means nothing was reflected (open space, or a soft or angled surface); `no_pulse` means a
  sensor or wiring fault; `out_of_range`, `too_close` and `echo_stuck` are also possible.
* `left_cmd`/`right_cmd` are the PWM values actually applied (after trim), from −255 to 255.
  `*_cmd_int` is their time integral in PWM·s.
* `yaw_deg` is wrapped to ±180°. `yaw_unwrapped_deg` is continuous.

To re-zero the gyro bias at any time, stop the robot, then:
`curl http://192.168.4.1/api/calibrate_gyro`

## 6. Python setup (macOS) — do this while still on your normal Wi-Fi

```bash
cd ~/elco/optimus
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python mac_mapper.py --selftest      # headless checks, no robot needed
python mac_mapper.py --simulate      # try the map window with a simulated robot
```

## 7. Run the mapper

```bash
cd ~/elco/optimus
source .venv/bin/activate
# join the BotController Wi-Fi, then:
python mac_mapper.py
# options: --rate 8 (5–10 Hz)  --url http://192.168.4.1/api/telemetry  --no-record
# replay a recorded session (no robot needed):
python mac_mapper.py --replay maps/logs/telemetry_YYYYmmdd_HHMMSS.jsonl --speed 4
```

**Keys** (click the map window first so it has focus):

| Key | Action |
|---|---|
| `space` or `p` | Pause/resume **map updates**. The pose keeps being tracked, so the robot does not jump when you resume. |
| `r` | Reset the map and the pose. The robot's current position becomes (0, 0) and its heading 0°. |
| `s` | Save a snapshot to `maps/YYYYmmdd_HHMMSS/`: `map.png`, `occupancy_grid.npy`, `occupancy_grid.csv`, `trajectory.csv`, `metadata.json` |
| `q` or `Esc` | Quit cleanly. Closing the window does the same. Ctrl-C in the terminal also works. |

**What you will see:** a grey map (unknown) centred on a green `+` (the start point). As you
drive, white (free) cones and black (occupied) arcs appear. Also shown: the blue estimated
trajectory, the red robot triangle with its dashed **2-sigma position circle**, which grows
over time, the yellow current beam, and orange raw obstacle returns. The right-hand panel
shows the link state, the **MEASURED** values (range, gyro, commands) and the **INFERRED**
values (x, y, heading, sigma, odometer), plus the mapping status, such as
`SKIP: turning too fast` or `no valid echo (no_echo) - not used`.

**Map frame:** the origin is the start pose, +x is the initial forward direction, +y is to the
initial left, and heading is counter-clockwise positive. The grid array is `grid[iy, ix]`,
with row 0 at the **minimum y**.

## 8. Calibration (edit the marked section at the top of `mac_mapper.py`)

**a. Yaw sign (do this first).** With the mapper running, lift the robot and turn it **left**
(counter-clockwise seen from above) by about 90°. The panel's `robot yaw` should go **up** by
about 90. If it goes down, set `YAW_SIGN = -1.0`. If it reads about 80 or 100 instead of 90,
that is gyro scale error. The model allows for 2%.

**b. Speed (`MAX_SPEED_MPS`).** Both methods work best at the slider speed you normally drive at.

* *Ultrasonic method (built in):* place the robot about 2.5 m from a flat wall, **pointing
  straight at it**. Press `r`. Drive straight forward, without steering, until about 40 cm from
  the wall. The panel shows `speed cal range X m vs est Y m -> MAX_SPEED_MPS ~ Z`. Copy Z into
  the file. The hint resets automatically if the heading changes by more than 5°. It is only
  meaningful for a square-on approach to a flat wall: at an angle, the range shrinks more
  slowly than the distance driven.
* *Tape measure method:* press `r`, drive straight for 1–2 m, then measure the real distance.
  New value = `MAX_SPEED_MPS × real ÷ odometer`.

**c. Deadband (`PWM_DEADBAND`).** Push the throttle stick very gently. Note the `cmd L/R`
values in the panel at which the wheels just start turning, and use that number. Default: 70.

**d. Left/right asymmetry.** The firmware already applies `RIGHT_TRIM = 1.08`. If the robot
still curves when driving straight, the gyro captures that curve in the heading anyway.
`LEFT_SPEED_SCALE`/`RIGHT_SPEED_SCALE` only matter if one side is consistently faster in a way
that affects the *distance* estimate. Leave them at 1.0 unless you have measured a difference.

## 9. Driving for a good map

1. Start with the robot still. Power it on and wait for `cal=ok`. Start the mapper, check that
   `LINK CONNECTED` and `IMU OK` are showing, then press `r`.
2. Keep the speed slider low. The default is now 120; go up to about 140 only if needed. Drive **slowly** along walls, 30–100 cm away.
3. **Rotate slowly in place** every metre or so (about 30°/s). The sensor only sees what it
   points at, so slow sweeps fill in walls. Fast spins are ignored above 45°/s.
4. Look at walls from several angles. Each echo is a broad arc, and only repeated views from
   different directions narrow it down to the real wall.
5. Keep runs short (1–3 minutes). Save with `s`, then reset with `r` and place the robot back
   at the same start pose to begin a fresh run. When the 2-sigma circle grows past about
   50 cm, the map is getting unreliable and updates are automatically weighted down.

## 10. Troubleshooting

| Symptom | What to check |
|---|---|
| Mapper says `CONNECTING`/`DISCONNECTED` | Is the Mac on the **BotController** Wi-Fi? Run `curl -m 2 http://192.168.4.1/api/telemetry`. macOS can drop a no-internet Wi-Fi network: turn off "Auto-Join" for your home network while mapping. |
| `STALE` | The robot was reset or is out of range. Look at the `[STAT]` lines in the Serial Monitor. Also check whether the `millis` value in the browser is advancing. |
| `IMU FAIL` / `imu_ok:false` | `[IMU] ERROR` on serial: check SDA = 21 and SCL = 22, **VCC on 3V3**, GND, and that AD0 is low. Run a quick I²C check by watching for the `WHO_AM_I` line at boot. A WHO_AM_I other than 0x68 just means a clone and is fine. |
| Yaw drifts while still | The bias calibration happened while the robot was moving (`cal=unverified`/`failed_motion`). Stop the robot and run `curl http://192.168.4.1/api/calibrate_gyro`. A drift of a few degrees per minute while driving is normal for an MPU6050. |
| Map rotates the wrong way when turning | `YAW_SIGN` (§8a). |
| Map is too big or too small | `MAX_SPEED_MPS` (§8b). |
| `distance_cm` always `null`, `us_status: no_pulse` | ECHO is not reaching GPIO19. Check the divider, the 5 V supply to the HC-SR04, and the TRIG wire on GPIO18. |
| `us_status: no_echo` most of the time | The sensor is pointing at open space, a soft surface or a steep angle. That is expected. These readings are deliberately **not** used as free space. |
| `distance_cm` jumps randomly | Motor electrical noise or a loose wire. Keep the sensor wires away from motor leads, and add a 100 µF capacitor across the HC-SR04's 5 V and GND. |
| **Robot drives into something and stays there** | The HC-SR04 cannot see low, thin or steeply angled objects, such as chair legs, cables or a box edge. The firmware now (a) drives slower, (b) in AUTO, backs off and turns when the range stops shrinking for 2 s (`[AUTO] stalled`) or after 4 blind readings in a row (`[AUTO] sensor blind`), and (c) in MANUAL, blocks forward drive under 20 cm (the phone shows `OBSTACLE AHEAD`). Watch the `[AUTO]`/`[GUARD]` serial lines. |
| **Mapping stops after a few minutes** | Read the red banner at the bottom of the map. It names the cause: `NO TELEMETRY` (the Mac left BotController Wi-Fi: turn off auto-join for other networks), `IMU FAIL` or an `IMU resets` count above 0 (motor-stall power dips are resetting the MPU6050: see the next row), `SENSOR BLIND` (no valid echo, often because the robot is pressed against something), or `ROBOT STALLED`. Each session's raw telemetry is saved in `maps/logs/`. Replay it with `python mac_mapper.py --replay maps/logs/telemetry_....jsonl --speed 4`. |
| `[IMU] MPU6050 lost its config` / `IMU resets` rising | The MPU6050 is browning out when the motors stall. The firmware reconfigures it automatically, but heading during the dip is lost. Fixes: give the ESP32 and the motors separate supplies (shared ground), put a 100–470 µF capacitor across the L298N motor supply, keep the I²C wires short and away from the motor leads, and avoid stalling the motors. |
| **Only reverse works (forward blocked)** | The forward guard is latched. The phone shows `OBSTACLE xx CM - TURN OR REVERSE` with the distance the robot thinks it sees. If nothing is really there, the ECHO wire is picking up motor noise: route it away from the motor wires, twist it with a GND wire, and add a 100 µF capacitor across the HC-SR04's 5 V/GND. Since v1.2, 2 of the last 3 readings must agree, short noise pulses are ignored, and the guard releases by itself after 2 s without confirmation. To switch it off: `MANUAL_OBSTACLE_GUARD = false`. |
| **Wi-Fi drops and then the sensor and everything else stop working** | Before v1.5, a network reply stuck on a bad Wi-Fi link could freeze the robot's program for minutes (each network write can wait up to 10 s), with the motors still running. v1.5 fixes this: log replies fit in one non-blocking write, a separate **dead-man timer cuts the motors if the program freezes for 0.3 s**, and the chip reboots after a 5 s hang. Every freeze is logged as `ROBOT PROGRAM FROZE <ms> in <where>`, and the report lists them under `PROGRAM FREEZES`. |
| Mapper says `STALE` and the robot restarts | The serial boot line and the mapper's event log show `reset reason`. `brownout` means the motor current pulled the supply down and the ESP32 rebooted. Fix the power: a separate battery or regulator for the ESP32 (common GND), a large capacitor on the motor supply, and no instant forward/reverse reversals at full power. |
| Joysticks feel laggy while mapping | Lower the poll rate: `python mac_mapper.py --rate 5`. Check that the `loop=` value on serial stays above about 1000 Hz. |
| Robot stops while you hold the stick | That is the 700 ms drive watchdog: the phone's requests are not arriving. Move closer to the robot. |
| AUTO mode stops by itself | The phone stopped sending heartbeats for 2 s (screen locked, or app switched). This is intentional. |
| Keys do nothing | Click inside the map window first. |

## 11. Firmware behaviour changes from `sketch_oct27b`

**Speed and obstacle handling (firmware v1.1):**
* The default max power is **120/255** (it was 180). The slider still covers 80–255.
* AUTO cruises at the **phone slider's speed** and slows to 85% of it within 70 cm of an
  obstacle. While in AUTO, the phone status line shows what it is doing and why, e.g.
  `TURNING (STALLED) | 85 CM` or `STOPPED - SENSOR SEES NOTHING`. If it only spins and never
  drives forward, the slider is too low for your motors to move the robot: raise it.
* AUTO no longer parks forever when the sensor goes blind. It backs off (500 ms) and turns
  about 70° by gyro when:
  - an obstacle is ≤ 35 cm away;
  - the sensor reports `too_close`;
  - there are 4 invalid readings in a row;
  - or it is commanded forward but the range changes less than 4 cm in 2 s (it is pushing
    something the beam misses).
* MANUAL has a **forward obstacle guard**: closer than 20 cm, forward drive is blocked, while
  turning in place and reversing still work. It releases after a reading of 30 cm or more, a
  25° turn, or reversing. Set `MANUAL_OBSTACLE_GUARD = false` to disable it.
* MPU6050 recovery: I²C now runs at 100 kHz, the firmware checks every 0.5 s for a
  brownout reset (the chip waking up asleep) and reconfigures it, detects frozen gyro
  output, and runs an I²C bus-clear before re-initialising. New telemetry fields:
  `imu_resets`, `fwd_blocked`, `auto_backoffs`, `auto_reason`.

**Mapper (same session):** telemetry is processed in a background thread, so the map keeps
updating even when the window is hidden or throttled. The Mac is kept awake with
`caffeinate`. A red banner explains why the map is not updating. **Stall detection** freezes
the estimated position when wheels are commanded forward but the range ahead does not shrink.
Raw telemetry is recorded to `maps/logs/` (use `--no-record` to disable) and can be replayed
with `--replay`.

**Earlier changes:**

The phone UI, the endpoints (`/`, `/drive`, `/stop`, `/mode`, `/speed`), the joystick mixing,
the motor directions, the trims, the speed range and the 700 ms drive watchdog are all
unchanged. The following were added or changed:

* **Emergency stop now also exits AUTO.** Before, `/stop` in AUTO stopped the motors for one
  loop pass, and then the autonomous logic drove again. The phone UI switches to MANUAL to match.
* **AUTO link heartbeat.** The page sends `/hb` every 400 ms while in AUTO. If the robot hears
  nothing for 2 s, it stops and goes to MANUAL. Before, a phone that disconnected in AUTO left
  the robot driving indefinitely.
* AUTO obstacle avoidance now uses the background ultrasonic sample instead of a blocking
  `pulseIn()`. It stops if the latest range is invalid or older than 300 ms. The thresholds and
  timings are unchanged.
* AUTO no longer prints every distance reading. One `[STAT]` line per second replaces it.

## 11b. Debug logging: what happened, and why it stopped

Since firmware v1.3 the robot keeps an **event log** in RAM. It holds about the last 100 s,
and you can read it at `http://192.168.4.1/api/log`. It records:

* every ultrasonic reading, including the invalid ones and the reason (`no_echo`, `too_close`, …);
* yaw and gyro rate at 10 Hz;
* every motor change, tagged with **who caused it**: `phone`, `guard`, `watchdog_manual`,
  `estop_page_hidden`, `auto_backoff`, …;
* joystick changes;
* phone command gaps longer than 300 ms (`LINKGAP`);
* watchdog trips;
* emergency stops, with whether it was the button or the phone page losing focus;
* guard latches and releases, and AUTO decisions;
* IMU faults and recoveries;
* Wi-Fi devices joining or leaving;
* loop rate, free memory and HTTP requests per second.

The last ~3 s are also kept in RTC memory, which usually survives a crash or brownout
reset. After a reboot those events appear as `[BEFORE RESET]`.

**Record everything without the map window:**
```bash
python mac_mapper.py --log-only
# drive and reproduce the problem, then press Ctrl-C -> a report and a timeline chart
```
Important events print live, for example `SAFETY STOP: manual heartbeat lost (760 ms …)`,
`FORWARD BLOCKED by guard at 6.0 cm`, or `ROBOT BOOTED, reset reason: brownout`.

The normal map mode (`python mac_mapper.py`) records the same log, shows the important
events in the panel, and prints the report when you quit. Files go in `maps/logs/`:
`robotlog_*.csv` (every event, with a readable `description` column),
`robotlog_*.timeline.png`, and `telemetry_*.jsonl`.

**Analyse a saved log again:** `python mac_mapper.py --analyze maps/logs/robotlog_<time>.csv`

The report shows:
* **Resets** and their reason. `brownout` means the motors pulled the supply down.
* **Ultrasonic** quality, split into motors ON and motors OFF. If the readings are much worse
  with the motors on, the report says `ELECTRICAL NOISE` and you know the cause is wiring.
* **Why the motors stopped**, counted by cause. Every unexpected stop is listed with the
  ultrasonic readings just before it.
* **Safety/link**: watchdog trips, phone command gaps, guard and AUTO events, IMU problems.

The timeline chart puts range, motor PWM, yaw and all the events on one time axis.

## 12. Staged hardware test plan

1. **Bench, wheels off the ground.** Upload, watch serial, check `cal=ok` and `imu=OK`. Use the
   phone to check that forward, reverse, left, right and E-STOP behave exactly as before. Close
   the phone browser while holding a stick: the motors must stop within about 0.7 s.
2. **Telemetry.** Open `/api/telemetry` on the Mac while driving the wheels from the phone.
   `left_cmd`/`right_cmd` should follow the sticks, and the joysticks must stay responsive.
3. **Sensors.** Move a hand in front of the HC-SR04 and watch `distance_cm`. Rotate the robot
   by hand and confirm the yaw sign (§8a).
4. **Mapper, robot still.** Run `python mac_mapper.py` with the robot facing a wall. One
   occupied arc should appear at the right distance, and it should **not** keep darkening
   rapidly: repeated identical readings are weighted down.
5. **Speed calibration** on the floor (§8b).
6. **Small run.** Map one corner of the room slowly. Save. Inspect `map.png`.
7. **AUTO safety.** Enter AUTO with the robot on the floor, then lock the phone. The robot must
   stop within about 2 s.

## 13. How it works (short)

* **Firmware:** `loop()` never blocks. Each pass it services HTTP, polls the gyro every 5 ms
  (trapezoidal integration with the real elapsed time from `esp_timer`), and pings the HC-SR04
  every 60 ms. The echo edges are timestamped in an interrupt, so no `pulseIn()` wait is needed.
  It also accumulates the PWM·time integrals and runs the watchdogs. `/api/telemetry` only
  formats cached values. When the motors have been off for 0.8 s and the rate is under 1.5°/s,
  the gyro rate is treated as zero and the bias slowly tracks it (a zero-velocity update).
* **Mapper:** the code is split into telemetry acquisition (a background thread), validation,
  a median-of-3 range filter (only across readings taken from the same pose), gyro heading
  (based on yaw differences, so it is safe across wraps and reboots), a command-based motion
  model with an EKF-style covariance, the log-odds grid, and the GUI/export. The HC-SR04 is
  modelled as a ±15° cone: the free space inside an inner ±10° cone is cleared, and a
  Gaussian-weighted occupied arc is placed at the measured range. Log-odds are clamped to
  ±4 (p 0.018–0.982), so no cell becomes permanently certain. Updates are weighted down as
  the pose uncertainty grows.

## 14. Known limitations

* Position comes from motor commands, not measured wheel motion. The stall detector catches
  a robot that is pushing against something only while the sensor has a valid echo ahead;
  when the sensor is blind as well, the position can still drift forward.
* Gyro heading drifts, typically by a few degrees per minute. There is nothing to correct it,
  no magnetometer and no loop closure.
* The HC-SR04 cannot see soft, thin or steeply angled surfaces, and its cone blurs corners
  and doorways.
* Each range is used with the pose at the time of the poll. A range sample can be up to
  about 60 ms older than that, which is about 2 cm of error at 0.4 m/s.

**Next software-only accuracy improvement:** use the ultrasonic itself as a speed sensor.
While driving straight at a wall (the same geometry the calibration helper already detects),
the rate of range change is a *measured* forward speed. Fusing it into the pose estimate as an
EKF measurement would correct command-model errors and detect stalls (commands non-zero but
range constant). After that, the next steps are local scan matching of recent beams against the
grid, and then pose hypotheses/loop closure. The `Mapper`/`PoseEstimator` interfaces are
separated so these can be added without changing the GUI or the telemetry code.
