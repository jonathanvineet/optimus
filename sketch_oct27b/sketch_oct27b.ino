
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// AUTOBOT MOBILE UNIT
// Wemos D1 R32 / ESP32 Arduino Core 3.x
// =====================================================

const char* ssid = "BotController";
const char* password = "robot1234";

WebServer server(80);

// L298N MOTOR DRIVER
// OUT1/OUT2 = left motor
// OUT3/OUT4 = right motor
const int ENA = 25;
const int IN1 = 26;
const int IN2 = 27;

const int IN3 = 17;
const int IN4 = 16;
const int ENB = 23;

// HC-SR04
const int TRIG = 18;
const int ECHO = 19;

// =====================================================
// SETTINGS
// =====================================================

int motorSpeed = 180;

// Assumes OUT3/OUT4 is the heavier motor side.
const float LEFT_TRIM = 1.00;
const float RIGHT_TRIM = 1.08;

const int OBSTACLE_CM = 35;
const int REVERSE_SPEED = 120;
const int TURN_SPEED = 125;

const unsigned long SENSOR_INTERVAL_MS = 75;
const unsigned long REVERSE_TIME_MS = 350;
const unsigned long TURN_TIME_MS = 500;
const unsigned long DRIVE_TIMEOUT_MS = 700;

// =====================================================
// STATE
// =====================================================

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

// =====================================================
// MOTOR CONTROL
// =====================================================

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  left = (int)(left * LEFT_TRIM);
  right = (int)(right * RIGHT_TRIM);

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // LEFT MOTOR
  digitalWrite(IN1, left > 0 ? HIGH : LOW);
  digitalWrite(IN2, left < 0 ? HIGH : LOW);

  // RIGHT MOTOR
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

  // Positive throttle = forward.
  // Negative throttle = reverse.
  // Positive steering = right.
  int leftMix = throttle + steering;
  int rightMix = throttle - steering;

  // Normalize both sides to preserve the steering ratio.
  int maxMix = max(abs(leftMix), abs(rightMix));

  if (maxMix > 100) {
    leftMix = leftMix * 100 / maxMix;
    rightMix = rightMix * 100 / maxMix;
  }

  int leftPWM = leftMix * motorSpeed / 100;
  int rightPWM = rightMix * motorSpeed / 100;

  drive(leftPWM, rightPWM);
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

long getDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 18000);

  if (duration == 0) {
    return -1;
  }

  return duration / 58;
}

// =====================================================
// PHONE CONTROLLER
// Minimal Autobot-inspired red / blue / silver UI
// =====================================================

const char webpage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport"
 content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#080b10">
<title>AUTOBOT</title>

<style>
:root {
  --bg: #080b10;
  --panel: #111720;
  --line: #28313d;
  --silver: #dce5ed;
  --muted: #82909f;
  --red: #e43d46;
  --blue: #3296ff;
}

* {
  box-sizing: border-box;
  -webkit-tap-highlight-color: transparent;
}

html, body {
  margin: 0;
  width: 100%;
  min-height: 100%;
  background: var(--bg);
  color: var(--silver);
  font-family: system-ui, -apple-system, sans-serif;
  user-select: none;
  -webkit-user-select: none;
  overscroll-behavior: none;
}

body {
  min-height: 100dvh;
  min-height: 100svh;
}

button, input {
  font: inherit;
}

button {
  touch-action: manipulation;
}

.app {
  width: 100%;
  max-width: 600px;
  min-height: 100dvh;
  margin: auto;
  padding: 15px 14px 20px;
  display: flex;
  flex-direction: column;
}

header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 4px 2px 14px;
  border-bottom: 1px solid var(--line);
}

.brand {
  display: flex;
  align-items: center;
  gap: 10px;
}

.emblem {
  width: 35px;
  height: 35px;
  border: 1px solid #435368;
  border-radius: 9px;
  display: grid;
  place-items: center;
  color: var(--red);
  font-size: 19px;
  font-weight: 900;
}

.name {
  font-size: 15px;
  font-weight: 850;
  letter-spacing: 3px;
}

.subtitle {
  margin-top: 3px;
  color: var(--muted);
  font-size: 9px;
  letter-spacing: 2px;
}

.connection {
  font-size: 9px;
  letter-spacing: 1px;
  color: #82d7ad;
}

.connection::before {
  content: "";
  display: inline-block;
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #65d69c;
  margin-right: 6px;
}

.modebar {
  display: flex;
  gap: 8px;
  margin-top: 13px;
}

.mode {
  flex: 1;
  height: 43px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--panel);
  color: var(--muted);
  font-size: 10px;
  font-weight: 800;
  letter-spacing: 1.4px;
}

#manualBtn.active {
  color: white;
  background: #182b40;
  border-color: var(--blue);
}

#autoBtn.active {
  color: white;
  background: #35181e;
  border-color: var(--red);
}

.status {
  height: 29px;
  display: grid;
  place-items: center;
  color: var(--muted);
  font-size: 9px;
  letter-spacing: 1.5px;
}

.section {
  color: var(--muted);
  font-size: 9px;
  letter-spacing: 2px;
  font-weight: 750;
  margin: 9px 2px 12px;
}

.joysticks {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}

.stick-card {
  min-width: 0;
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 13px;
  padding: 14px 5px 12px;
}

.stick-title {
  text-align: center;
  font-size: 11px;
  font-weight: 800;
  letter-spacing: 1.5px;
}

.stick-desc {
  text-align: center;
  margin-top: 4px;
  color: var(--muted);
  font-size: 9px;
}

.stick {
  position: relative;
  width: min(44vw, 238px);
  height: min(44vw, 238px);
  max-width: 238px;
  max-height: 238px;
  aspect-ratio: 1;
  margin: 14px auto 12px;
  border-radius: 50%;
  touch-action: none;
  background:
    radial-gradient(circle,
      #1b2531 0%,
      #131b25 62%,
      #0c1118 63%);
  border: 1px solid #465568;
  box-shadow:
    inset 0 0 0 7px #0d1219,
    inset 0 0 0 8px #303d4d;
}

.stick::before,
.stick::after {
  content: "";
  position: absolute;
  background: #354354;
  pointer-events: none;
}

.stick::before {
  width: 1px;
  height: 72%;
  left: 50%;
  top: 14%;
}

.stick::after {
  height: 1px;
  width: 72%;
  top: 50%;
  left: 14%;
}

.knob {
  position: absolute;
  width: 43%;
  height: 43%;
  left: 28.5%;
  top: 28.5%;
  border-radius: 50%;
  pointer-events: none;
  z-index: 2;
  background: linear-gradient(145deg, #3b4654, #151c25);
  border: 2px solid var(--blue);
  box-shadow:
    0 5px 16px #0009,
    inset 0 0 0 5px #202b38;
}

#steerKnob {
  border-color: var(--red);
}

.knob::after {
  content: "";
  position: absolute;
  width: 7px;
  height: 7px;
  border-radius: 50%;
  top: calc(50% - 3.5px);
  left: calc(50% - 3.5px);
  background: var(--silver);
}

.reading {
  border-top: 1px solid var(--line);
  padding-top: 10px;
  text-align: center;
  color: var(--muted);
  font-size: 9px;
  letter-spacing: 1.5px;
}

.reading strong {
  display: block;
  color: var(--silver);
  font-size: 13px;
  letter-spacing: .5px;
  margin-top: 4px;
}

.speed {
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 10px;
  padding: 14px;
}

.speedrow {
  display: flex;
  justify-content: space-between;
  gap: 8px;
  font-size: 10px;
  font-weight: 750;
  letter-spacing: 1px;
}

#speedValue {
  color: var(--blue);
  white-space: nowrap;
}

input[type=range] {
  width: 100%;
  margin: 16px 0 8px;
  accent-color: var(--blue);
  touch-action: pan-y;
}

.scale {
  display: flex;
  justify-content: space-between;
  color: var(--muted);
  font-size: 9px;
}

.stop {
  width: 100%;
  min-height: 52px;
  margin-top: 12px;
  border-radius: 9px;
  border: 1px solid #a3333b;
  background: #35171c;
  color: #ffb8bc;
  font-size: 11px;
  font-weight: 850;
  letter-spacing: 2px;
}

.stop:active {
  background: #552129;
}

.footer {
  text-align: center;
  color: #697583;
  font-size: 9px;
  letter-spacing: 1.5px;
  margin-top: auto;
  padding-top: 17px;
}

body.auto .joysticks {
  opacity: .35;
}

body.auto .stick {
  pointer-events: none;
}

@media (min-width: 520px) {
  .app {
    padding-top: 22px;
  }

  .stick {
    width: min(40vw, 238px);
    height: min(40vw, 238px);
  }
}

@media (max-width: 370px) {
  .app {
    padding: 10px 8px 14px;
  }

  .stick-card {
    padding-left: 3px;
    padding-right: 3px;
  }

  .stick {
    width: 45vw;
    height: 45vw;
  }
}

@media (orientation: landscape) and (max-height: 520px) {
  .app {
    max-width: 850px;
    padding: 8px 14px;
  }

  header {
    padding-bottom: 8px;
  }

  .modebar {
    margin-top: 8px;
  }

  .status {
    height: 22px;
  }

  .section {
    margin: 5px 2px 8px;
  }

  .joysticks {
    max-width: 590px;
    width: 100%;
    margin: auto;
  }

  .stick {
    width: min(32vh, 165px);
    height: min(32vh, 165px);
    margin: 8px auto;
  }

  .stick-card {
    padding: 8px 5px;
  }

  .footer {
    padding-top: 8px;
  }
}
</style>
</head>

<body>
<div class="app">

<header>
  <div class="brand">
    <div class="emblem">A</div>
    <div>
      <div class="name">AUTOBOT</div>
      <div class="subtitle">MOBILE UNIT 01</div>
    </div>
  </div>
  <div class="connection">ONLINE</div>
</header>

<div class="modebar">
  <button id="manualBtn" class="mode active"
    onclick="setMode('M')">MANUAL</button>
  <button id="autoBtn" class="mode"
    onclick="setMode('A')">AUTO</button>
</div>

<div class="status" id="status">
  HOLD TO DRIVE
</div>

<div class="section">CONTROLS</div>

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
      DIRECTION
      <strong id="steerValue">CENTER</strong>
    </div>
  </div>

</div>

<div class="section">SPEED</div>

<div class="speed">
  <div class="speedrow">
    <span>MAX MOTOR POWER</span>
    <span id="speedValue">180 / 255</span>
  </div>

  <input id="speedSlider" type="range"
    min="80" max="255" value="180">

  <div class="scale">
    <span>LOW</span>
    <span>MAX</span>
  </div>
</div>

<button class="stop" onclick="emergencyStop()">
  ■ &nbsp; EMERGENCY STOP
</button>

<div class="footer">
  AUTOBOT MOBILE UNIT
</div>

</div>

<script>
let throttle = 0;
let steering = 0;
let mode = 'M';

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

// Send current joystick position, including unchanged values.
// The heartbeat keeps the motors running while a joystick is held.
function sendDrive() {
  if (mode !== 'M') return;

  fetch('/drive?t=' + throttle +
        '&s=' + steering +
        '&ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

function resetStick(which) {
  const knob = document.getElementById(
    which === 'throttle' ? 'throttleKnob' : 'steerKnob'
  );

  knob.style.left = '28.5%';
  knob.style.top = '28.5%';

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
  throttle = 0;
  steering = 0;

  resetStick('throttle');
  resetStick('steer');

  // Dedicated stop request.
  fetch('/stop?ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

function setMode(nextMode) {
  throttle = 0;
  steering = 0;

  resetStick('throttle');
  resetStick('steer');

  mode = nextMode;

  document.body.classList.toggle('auto', mode === 'A');

  document.getElementById('manualBtn')
    .classList.toggle('active', mode === 'M');

  document.getElementById('autoBtn')
    .classList.toggle('active', mode === 'A');

  document.getElementById('status').textContent =
    mode === 'A' ? 'AUTONOMOUS MODE ACTIVE' : 'HOLD TO DRIVE';

  fetch('/mode?m=' + mode + '&ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

document.getElementById('speedSlider').addEventListener(
  'input', function() {
    const speed = Number(this.value);

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
    const radius = rect.width * 0.285;

    let dx = (e.clientX - cx) / radius;
    let dy = (e.clientY - cy) / radius;

    // Keep the joystick movement inside a circle.
    const length = Math.sqrt(dx * dx + dy * dy);

    if (length > 1) {
      dx /= length;
      dy /= length;
    }

    knob.style.left = (28.5 + dx * 28.5) + '%';
    knob.style.top = (28.5 + dy * 28.5) + '%';

    if (isThrottle) {
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

// Continuous heartbeat while either joystick is held.
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

// =====================================================
// HTTP HANDLERS
// =====================================================

void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleDrive() {
  if (!autoMode && server.hasArg("t") && server.hasArg("s")) {
    throttleValue = constrain(server.arg("t").toInt(), -100, 100);
    steeringValue = constrain(server.arg("s").toInt(), -100, 100);

    // Refresh timeout on every received heartbeat.
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

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);

  // ESP32 Arduino core 3.x PWM API.
  if (!ledcAttach(ENA, 1000, 8) ||
      !ledcAttach(ENB, 1000, 8)) {
    Serial.println("PWM initialization failed!");
    while (true) {
      delay(1000);
    }
  }

  stopBot();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.println("AUTOBOT MOBILE UNIT ONLINE");
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

// =====================================================
// AUTONOMOUS OBSTACLE AVOIDANCE
// =====================================================

void loop() {
  server.handleClient();

  // Stop if the manual controller heartbeat is lost.
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

  // Reverse briefly after detecting an obstacle.
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

  // Turn, then resume distance checking.
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

  // Fail safe if no valid echo is received.
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
