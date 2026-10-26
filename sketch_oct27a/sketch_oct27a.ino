
#include <WiFi.h>
#include <WebServer.h>

// ==================================================
// BATMAN MOBILE CONTROLLER
// Wemos D1 R32 / ESP32 Arduino Core 3.x
// ==================================================

const char* ssid = "BotController";
const char* password = "robot1234";

WebServer server(80);

// L298N
const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

const int IN3 = 17;
const int IN4 = 16;
const int ENB = 23;

// HC-SR04
const int TRIG = 18;
const int ECHO = 19;

// ==================================================
// SETTINGS
// ==================================================

int motorSpeed = 180;

// OUT3/OUT4 is assumed to be the heavier motor side.
const float LEFT_TRIM = 1.00;
const float RIGHT_TRIM = 1.08;

const int OBSTACLE_CM = 35;
const int REVERSE_SPEED = 120;
const int TURN_SPEED = 125;

const unsigned long SENSOR_INTERVAL_MS = 75;
const unsigned long REVERSE_TIME_MS = 350;
const unsigned long TURN_TIME_MS = 500;

// Stop only if phone heartbeat is lost.
const unsigned long DRIVE_TIMEOUT_MS = 700;

// ==================================================
// STATE
// ==================================================

bool autoMode = false;

int throttleValue = 0;
int steeringValue = 0;

unsigned long lastDriveCommand = 0;
unsigned long lastSensorTime = 0;
unsigned long stateStarted = 0;

enum AutoState {
  DRIVING,
  REVERSING,
  TURNING
};

AutoState autoState = DRIVING;
bool turnRightNext = true;

// ==================================================
// MOTORS
// ==================================================

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  left = (int)(left * LEFT_TRIM);
  right = (int)(right * RIGHT_TRIM);

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // OUT1 / OUT2: left motor
  digitalWrite(IN1, left > 0 ? HIGH : LOW);
  digitalWrite(IN2, left < 0 ? HIGH : LOW);

  // OUT3 / OUT4: right motor
  digitalWrite(IN3, right > 0 ? HIGH : LOW);
  digitalWrite(IN4, right < 0 ? HIGH : LOW);

  ledcWrite(ENA, abs(left));
  ledcWrite(ENB, abs(right));
}

void stopBot() {
  drive(0, 0);
}

void driveJoystick(int throttle, int steering) {
  throttle = constrain(throttle, -100, 100);
  steering = constrain(steering, -100, 100);

  // Forward and reverse work on both motors.
  int leftMix = throttle + steering;
  int rightMix = throttle - steering;

  // Preserve the steering ratio without clipping one wheel.
  int maxMix = max(abs(leftMix), abs(rightMix));

  if (maxMix > 100) {
    leftMix = leftMix * 100 / maxMix;
    rightMix = rightMix * 100 / maxMix;
  }

  int leftPWM = leftMix * motorSpeed / 100;
  int rightPWM = rightMix * motorSpeed / 100;

  drive(leftPWM, rightPWM);
}

// ==================================================
// ULTRASONIC SENSOR
// ==================================================

long getDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 18000);

  if (duration == 0) return -1;

  return duration / 58;
}

// ==================================================
// PHONE UI - MINIMAL BATMAN THEME
// ==================================================

const char webpage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport"
 content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#090a0b">
<title>BATMAN</title>

<style>
:root {
  --bg:#090a0b;
  --panel:#121315;
  --line:#292a2d;
  --gold:#c6a65c;
  --text:#f3f0e8;
  --muted:#85858a;
  --red:#d46b60;
}

* {
  box-sizing:border-box;
  -webkit-tap-highlight-color:transparent;
}

html,body {
  margin:0;
  width:100%;
  min-height:100%;
  background:var(--bg);
  color:var(--text);
  font-family:system-ui,-apple-system,sans-serif;
  user-select:none;
  -webkit-user-select:none;
  overscroll-behavior:none;
}

body {
  min-height:100dvh;
  min-height:100svh;
}

button,input {
  font:inherit;
}

button {
  touch-action:manipulation;
}

.app {
  width:100%;
  max-width:540px;
  margin:auto;
  min-height:100dvh;
  padding:16px 16px 22px;
  display:flex;
  flex-direction:column;
}

header {
  display:flex;
  align-items:center;
  justify-content:space-between;
  padding:4px 2px 15px;
  border-bottom:1px solid var(--line);
}

.brand {
  display:flex;
  align-items:center;
  gap:11px;
}

.bat {
  width:40px;
  height:28px;
  color:var(--gold);
}

.bat svg {
  width:100%;
  height:100%;
  fill:currentColor;
}

.name {
  font-size:15px;
  font-weight:800;
  letter-spacing:4px;
}

.sub {
  font-size:9px;
  color:var(--muted);
  letter-spacing:1.7px;
  margin-top:3px;
}

.connection {
  font-size:9px;
  color:var(--gold);
  letter-spacing:1px;
}

.connection span {
  display:inline-block;
  width:6px;
  height:6px;
  border-radius:50%;
  background:var(--gold);
  margin-right:5px;
}

.modebar {
  display:flex;
  gap:8px;
  margin:15px 0 4px;
}

.mode {
  flex:1;
  height:43px;
  border:1px solid var(--line);
  border-radius:8px;
  background:var(--panel);
  color:var(--muted);
  font-size:10px;
  font-weight:800;
  letter-spacing:1.3px;
}

.mode.active {
  background:var(--gold);
  color:#12110d;
  border-color:var(--gold);
}

.mode:active,.stop:active {
  transform:scale(.985);
}

.status {
  min-height:25px;
  text-align:center;
  color:var(--muted);
  font-size:10px;
  letter-spacing:1px;
  padding:7px 0 3px;
}

.section {
  color:var(--muted);
  font-size:9px;
  font-weight:700;
  letter-spacing:2px;
  margin:15px 2px 12px;
}

.joysticks {
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:12px;
}

.stick-card {
  min-width:0;
  background:var(--panel);
  border:1px solid var(--line);
  border-radius:12px;
  padding:14px 5px 12px;
}

.stick-title {
  font-size:10px;
  font-weight:800;
  letter-spacing:1.5px;
  text-align:center;
}

.stick-desc {
  text-align:center;
  color:var(--muted);
  font-size:9px;
  margin-top:4px;
}

.stick {
  position:relative;
  width:min(42vw,190px);
  height:min(42vw,190px);
  max-width:190px;
  max-height:190px;
  aspect-ratio:1;
  margin:14px auto 10px;
  border-radius:50%;
  touch-action:none;
  background:
    radial-gradient(circle,#202124 0%,#17181a 60%,#101113 61%);
  border:1px solid #3b3c3e;
  box-shadow:inset 0 0 0 7px #101113,
             inset 0 0 0 8px #292a2c;
}

.stick::before,.stick::after {
  content:"";
  position:absolute;
  background:#36373a;
  pointer-events:none;
}

.stick::before {
  width:1px;
  height:72%;
  left:50%;
  top:14%;
}

.stick::after {
  height:1px;
  width:72%;
  top:50%;
  left:14%;
}

.knob {
  position:absolute;
  width:40%;
  height:40%;
  left:30%;
  top:30%;
  border-radius:50%;
  pointer-events:none;
  z-index:2;
  border:1px solid var(--gold);
  background:linear-gradient(145deg,#303134,#17181a);
  box-shadow:0 4px 12px #0009,
             inset 0 0 0 4px #242528;
}

.knob::after {
  content:"";
  position:absolute;
  width:6px;
  height:6px;
  border-radius:50%;
  left:calc(50% - 3px);
  top:calc(50% - 3px);
  background:var(--gold);
}

.reading {
  border-top:1px solid var(--line);
  padding-top:10px;
  text-align:center;
  color:var(--muted);
  font-size:9px;
  letter-spacing:1px;
}

.reading strong {
  display:block;
  color:var(--text);
  font-size:13px;
  letter-spacing:0;
  margin-top:4px;
}

.speed {
  background:var(--panel);
  border:1px solid var(--line);
  border-radius:10px;
  padding:14px;
}

.speedrow {
  display:flex;
  justify-content:space-between;
  gap:10px;
  font-size:10px;
  font-weight:700;
  letter-spacing:1px;
}

#speedValue {
  color:var(--gold);
  white-space:nowrap;
}

input[type=range] {
  width:100%;
  margin:17px 0 8px;
  accent-color:var(--gold);
  touch-action:pan-y;
}

.speedlabels {
  display:flex;
  justify-content:space-between;
  color:var(--muted);
  font-size:9px;
}

.stop {
  width:100%;
  min-height:53px;
  margin-top:13px;
  border-radius:9px;
  border:1px solid #70403a;
  background:#241513;
  color:#e99a90;
  font-size:11px;
  font-weight:850;
  letter-spacing:2px;
}

.footer {
  text-align:center;
  margin-top:auto;
  padding-top:18px;
  font-size:9px;
  line-height:1.8;
  color:#66676b;
  letter-spacing:1px;
}

body.auto .joysticks {
  opacity:.32;
}

body.auto .stick {
  pointer-events:none;
}

@media (max-width:360px) {
  .app {padding:12px 10px 16px;}
  .stick-card {padding-left:3px;padding-right:3px;}
  .stick {width:42vw;height:42vw;}
  .mode {height:40px;}
}

@media (min-height:800px) {
  .app {padding-top:22px;}
  .stick {width:min(43vw,205px);height:min(43vw,205px);}
  .stick-card {padding-top:17px;}
}

@media (orientation:landscape) and (max-height:520px) {
  .app {max-width:760px;padding:8px 14px;}
  header {padding-bottom:8px;}
  .modebar {margin:8px 0 0;}
  .section {margin:8px 2px;}
  .joysticks {max-width:560px;margin:auto;width:100%;}
  .stick {width:min(29vh,145px);height:min(29vh,145px);margin:8px auto;}
  .stick-card {padding:8px 5px;}
  .footer {padding-top:8px;}
}
</style>
</head>

<body>
<div class="app">

<header>
  <div class="brand">
    <div class="bat">
      <svg viewBox="0 0 100 55">
        <path d="M4 12 L23 22 L29 5 L43 17 L50 12
        L57 17 L71 5 L77 22 L96 12 L88 43
        L72 49 L50 42 L28 49 L12 43 Z"/>
      </svg>
    </div>
    <div>
      <div class="name">BATMAN</div>
      <div class="sub">MOBILE UNIT 01</div>
    </div>
  </div>
  <div class="connection"><span></span>ONLINE</div>
</header>

<div class="modebar">
  <button id="manualBtn" class="mode active"
    onclick="setMode('M')">MANUAL</button>
  <button id="autoBtn" class="mode"
    onclick="setMode('A')">AUTONOMOUS</button>
</div>

<div class="status" id="status">
  HOLD THE JOYSTICK TO DRIVE
</div>

<div class="section">DRIVE CONTROLS</div>

<div class="joysticks">

  <div class="stick-card">
    <div class="stick-title">THROTTLE</div>
    <div class="stick-desc">FORWARD / REVERSE</div>

    <div class="stick" id="throttleStick">
      <div class="knob" id="throttleKnob"></div>
    </div>

    <div class="reading">
      DIRECTION
      <strong id="directionValue">IDLE</strong>
    </div>
  </div>

  <div class="stick-card">
    <div class="stick-title">STEERING</div>
    <div class="stick-desc">LEFT / RIGHT</div>

    <div class="stick" id="steerStick">
      <div class="knob" id="steerKnob"></div>
    </div>

    <div class="reading">
      STEERING
      <strong id="steerValue">CENTER</strong>
    </div>
  </div>

</div>

<div class="section">SPEED LIMIT</div>

<div class="speed">
  <div class="speedrow">
    <span>MOTOR POWER</span>
    <span id="speedValue">180 / 255</span>
  </div>
  <input id="speedSlider" type="range"
    min="80" max="255" value="180">
  <div class="speedlabels">
    <span>SLOW</span>
    <span>MAXIMUM</span>
  </div>
</div>

<button class="stop" onclick="emergencyStop()">
  ■ &nbsp; STOP
</button>

<div class="footer">
  BATMAN MOBILE UNIT<br>
  RELEASE THROTTLE TO STOP
</div>

</div>

<script>
let throttle = 0;
let steering = 0;
let mode = 'M';
let speed = 180;

const activePointers = {
  throttle: null,
  steer: null
};

function updateReadings() {
  document.getElementById('directionValue').textContent =
    throttle > 3 ? 'FORWARD' :
    throttle < -3 ? 'REVERSE' : 'IDLE';

  document.getElementById('steerValue').textContent =
    steering < -3 ? 'LEFT' :
    steering > 3 ? 'RIGHT' : 'CENTER';
}

// IMPORTANT FIX:
// Always send the current values, even if they haven't changed.
// This heartbeat prevents the ESP32 safety timeout from stopping
// a continuously held joystick.
function sendDrive() {
  if (mode !== 'M') return;

  fetch('/drive?t=' + throttle + '&s=' + steering +
        '&ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

function resetStick(which) {
  const knob = document.getElementById(
    which === 'throttle' ? 'throttleKnob' : 'steerKnob'
  );

  knob.style.left = '30%';
  knob.style.top = '30%';

  if (which === 'throttle') throttle = 0;
  else steering = 0;

  updateReadings();
}

function centerControls() {
  throttle = 0;
  steering = 0;

  resetStick('throttle');
  resetStick('steer');
  sendDrive();
}

function emergencyStop() {
  centerControls();

  // Dedicated stop endpoint is also used for safety.
  fetch('/stop?ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

function setMode(nextMode) {
  centerControls();
  mode = nextMode;

  document.body.classList.toggle('auto', mode === 'A');

  document.getElementById('manualBtn')
    .classList.toggle('active', mode === 'M');

  document.getElementById('autoBtn')
    .classList.toggle('active', mode === 'A');

  document.getElementById('status').textContent =
    mode === 'A' ? 'OBSTACLE AVOIDANCE ACTIVE'
                 : 'HOLD THE JOYSTICK TO DRIVE';

  fetch('/mode?m=' + mode + '&ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

document.getElementById('speedSlider').addEventListener(
  'input', function() {
    speed = Number(this.value);

    document.getElementById('speedValue').textContent =
      speed + ' / 255';

    fetch('/speed?v=' + speed + '&ts=' + Date.now(), {
      cache: 'no-store'
    }).catch(() => {});
  }
);

function setupStick(which) {
  const isThrottle = which === 'throttle';

  const stick = document.getElementById(
    isThrottle ? 'throttleStick' : 'steerStick'
  );

  const knob = document.getElementById(
    isThrottle ? 'throttleKnob' : 'steerKnob'
  );

  function move(e) {
    if (activePointers[which] !== e.pointerId) return;

    const rect = stick.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;

    const radius = rect.width * 0.30;

    let dx = (e.clientX - cx) / radius;
    let dy = (e.clientY - cy) / radius;

    const length = Math.sqrt(dx * dx + dy * dy);

    if (length > 1) {
      dx /= length;
      dy /= length;
    }

    knob.style.left = (30 + dx * 30) + '%';
    knob.style.top = (30 + dy * 30) + '%';

    if (isThrottle) {
      // Top = forward; bottom = reverse.
      throttle = Math.round(-dy * 100);
      if (Math.abs(throttle) < 4) throttle = 0;
    } else {
      steering = Math.round(dx * 100);
      if (Math.abs(steering) < 4) steering = 0;
    }

    updateReadings();
    sendDrive();
  }

  stick.addEventListener('pointerdown', e => {
    if (mode !== 'M') return;

    e.preventDefault();
    activePointers[which] = e.pointerId;
    stick.setPointerCapture(e.pointerId);
    move(e);
  });

  stick.addEventListener('pointermove', move);

  function release(e) {
    if (activePointers[which] !== e.pointerId) return;

    activePointers[which] = null;
    resetStick(which);
    sendDrive();
  }

  stick.addEventListener('pointerup', release);
  stick.addEventListener('pointercancel', release);
  stick.addEventListener('lostpointercapture', release);
}

setupStick('throttle');
setupStick('steer');

// Send a fresh drive command every 150 ms while the throttle
// or steering is active. This is the continuous-driving fix.
setInterval(() => {
  if (mode === 'M' && (throttle !== 0 || steering !== 0)) {
    sendDrive();
  }
}, 150);

window.addEventListener('blur', emergencyStop);

document.addEventListener('visibilitychange', () => {
  if (document.hidden) emergencyStop();
});
</script>
</body>
</html>
)HTML";

// ==================================================
// HTTP HANDLERS
// ==================================================

void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleDrive() {
  if (!autoMode && server.hasArg("t") && server.hasArg("s")) {
    throttleValue = constrain(server.arg("t").toInt(), -100, 100);
    steeringValue = constrain(server.arg("s").toInt(), -100, 100);

    // Refresh on EVERY heartbeat, even if values are identical.
    lastDriveCommand = millis();

    driveJoystick(throttleValue, steeringValue);
  }

  server.send(200, "text/plain", "OK");
}

void handleStop() {
  throttleValue = 0;
  steeringValue = 0;
  stopBot();

  server.send(200, "text/plain", "STOPPED");
}

void handleMode() {
  autoMode = server.hasArg("m") && server.arg("m") == "A";

  throttleValue = 0;
  steeringValue = 0;
  stopBot();

  autoState = DRIVING;
  stateStarted = millis();
  lastSensorTime = 0;

  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("v")) {
    motorSpeed = constrain(server.arg("v").toInt(), 80, 255);

    if (!autoMode) {
      driveJoystick(throttleValue, steeringValue);
    }
  }

  server.send(200, "text/plain", "OK");
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  if (!ledcAttach(ENA, 1000, 8) ||
      !ledcAttach(ENB, 1000, 8)) {
    Serial.println("PWM initialization failed");
    while (true) delay(1000);
  }

  stopBot();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.println("BATMAN MOBILE UNIT ONLINE");
  Serial.print("Wi-Fi: ");
  Serial.println(ssid);
  Serial.print("Controller: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/mode", handleMode);
  server.on("/speed", handleSpeed);

  server.begin();
}

// ==================================================
// AUTONOMOUS MODE
// ==================================================

void loop() {
  server.handleClient();

  // Manual safety timeout: stop only when heartbeat is lost.
  if (!autoMode) {
    if ((throttleValue != 0 || steeringValue != 0) &&
        millis() - lastDriveCommand > DRIVE_TIMEOUT_MS) {
      throttleValue = 0;
      steeringValue = 0;
      stopBot();
    }
    return;
  }

  unsigned long now = millis();

  // Reverse away from the obstacle.
  if (autoState == REVERSING) {
    if (now - stateStarted >= REVERSE_TIME_MS) {
      stopBot();

      autoState = TURNING;
      stateStarted = now;

      if (turnRightNext) {
        drive(TURN_SPEED, -TURN_SPEED);
      } else {
        drive(-TURN_SPEED, TURN_SPEED);
      }

      turnRightNext = !turnRightNext;
    }
    return;
  }

  // Turn, then check the path again.
  if (autoState == TURNING) {
    if (now - stateStarted >= TURN_TIME_MS) {
      stopBot();

      autoState = DRIVING;
      stateStarted = now;
      lastSensorTime = 0;
    }
    return;
  }

  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorTime = now;

  long distance = getDistanceCM();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // No valid echo: stop safely.
  if (distance < 0) {
    stopBot();
    return;
  }

  if (distance <= OBSTACLE_CM) {
    stopBot();

    autoState = REVERSING;
    stateStarted = now;

    drive(-REVERSE_SPEED, -REVERSE_SPEED);
  } else {
    drive(motorSpeed, motorSpeed);
  }
}
