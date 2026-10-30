
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"

// =====================================================
// AUTOBOT MOBILE UNIT + MAPPING TELEMETRY
// Wemos D1 R32 / ESP32 Arduino Core 3.x
//
// Built on sketch_oct27b. The phone controller, joystick
// mixing, motor trims, speed control, AUTO mode and the
// drive watchdog are unchanged in behaviour. Added:
//   - MPU6050 gyro Z bias calibration + yaw integration
//   - background (interrupt-timed) HC-SR04 sampling
//   - GET /api/telemetry for the Mac mapper (read-only)
//   - GET /api/calibrate_gyro (re-zero gyro bias, robot still)
//   - AUTO-mode link heartbeat (/hb) and E-STOP that also
//     exits AUTO
// =====================================================

const char* FW_VERSION = "autobot_mapping-1.5";

const char* ssid = "BotController";
const char* password = "robot1234";

// Home Wi-Fi (station mode) in secrets.h, see secrets.example.h.
// With it set, the robot ONLY joins the home network, at the fixed
// address ROBOT_IP below. Without it, the robot hosts its own
// BotController access point (192.168.4.1) as before.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef HOME_WIFI_SSID
#define HOME_WIFI_SSID ""
#define HOME_WIFI_PASSWORD ""
#endif

// Fixed address on the home network (router 192.168.1.1, /24).
// Pick an address no other device uses; .200 was free when set up.
const IPAddress ROBOT_IP(192, 168, 1, 200);
const IPAddress ROUTER_IP(192, 168, 1, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const unsigned long HOME_WIFI_RETRY_MS = 10000;
bool homeWifiEnabled = false;
bool homeWifiConnected = false;
unsigned long homeWifiLastTryMs = 0;

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

// HC-SR04 (ECHO through a 5V -> 3.3V voltage divider)
const int TRIG = 18;
const int ECHO = 19;

// MPU6050 (VCC = 3V3, AD0 low -> address 0x68)
const int IMU_SDA = 21;
const int IMU_SCL = 22;

// =====================================================
// SETTINGS
// =====================================================

// Default max power. Lower = slower = the ultrasonic has time to
// see obstacles. The phone slider still ranges 80..255.
int motorSpeed = 120;

// Assumes OUT3/OUT4 is the heavier motor side.
const float LEFT_TRIM = 1.00;
const float RIGHT_TRIM = 1.08;

const int OBSTACLE_CM = 35;
const int REVERSE_SPEED = 120;
const int TURN_SPEED = 125;

const unsigned long REVERSE_TIME_MS = 500;
const unsigned long TURN_TIME_MS = 500;       // fallback if no gyro
const unsigned long DRIVE_TIMEOUT_MS = 700;

// AUTO cruises at the phone slider's speed and slows to this
// percentage of it near obstacles. If the robot spins but never
// drives forward in AUTO, the slider is too low for your motors.
const int AUTO_APPROACH_PERCENT = 85;
const int SLOWDOWN_CM = 70;

// After backing off, turn this far using the gyro (time-capped).
const float AUTO_TURN_DEG = 70.0f;
const unsigned long TURN_MAX_MS = 2500;

// Back off instead of parking when the sensor goes blind (touching
// an object gives no / junk echo) or when the range stops changing
// while driving forward (object below or beside the beam).
const int AUTO_MAX_INVALID = 4;
const unsigned long STALL_WINDOW_MS = 2000;
const float STALL_MIN_CHANGE_CM = 4.0f;

// MANUAL: refuse forward drive closer than this. Turning in place
// and reversing still work. Set false to disable.
const bool MANUAL_OBSTACLE_GUARD = true;
const int MANUAL_GUARD_CM = 20;
const int MANUAL_GUARD_RELEASE_CM = 30;
const float MANUAL_GUARD_RELEASE_DEG = 25.0f;
// Release the guard if nothing has confirmed the obstacle for this long
// (e.g. sensor blind). In MANUAL you are in control after that.
const unsigned long MANUAL_GUARD_STALE_MS = 2000;
// AUTO: after this many back-offs in a row caused by a blind sensor,
// stop and wait for a valid reading instead of reversing again.
const int AUTO_MAX_BLIND_BACKOFFS = 3;

// AUTO mode stops if the phone stops sending /hb for this long.
const unsigned long AUTO_LINK_TIMEOUT_MS = 2000;

// Ultrasonic sampling. 60 ms between pings lets echoes die out.
const unsigned long US_INTERVAL_MS = 60;
const int64_t US_ECHO_TIMEOUT_US = 30000;
const float US_CM_PER_US = 0.01715f;     // 343 m/s, round trip
const float US_MIN_VALID_CM = 2.0f;
const float US_MAX_VALID_CM = 400.0f;
// Echo pulses shorter than this are electrical noise, not echoes.
const int64_t US_MIN_PULSE_US = 100;
// Obstacle decisions need this many of the last 3 readings to agree,
// so one noisy reading cannot block or reverse the robot.
const int US_CONFIRM_VOTES = 2;
// AUTO mode only trusts a range sample this recent.
const unsigned long US_FRESH_MS = 300;

// MPU6050 registers (direct register access, no library needed)
const uint8_t MPU_ADDR = 0x68;
const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
const uint8_t MPU_REG_CONFIG = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_GYRO_ZOUT_H = 0x47;
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_REG_WHO_AM_I = 0x75;
// +/-1000 dps: spins reached the old +/-250 dps limit and clipped.
const float GYRO_LSB_PER_DPS = 32.8f;

const int64_t IMU_INTERVAL_US = 5000;     // poll gyro at ~200 Hz
const float IMU_MAX_DT_S = 0.05f;         // longer gaps are counted
const unsigned long IMU_RETRY_MS = 1000;
const int IMU_MAX_CONSECUTIVE_ERRORS = 5;
// 100 kHz is more tolerant of motor noise than 400 kHz.
const uint32_t IMU_I2C_HZ = 100000;
// Motor stall current can brown the MPU6050 out; it then wakes in
// SLEEP mode with its config lost. Check for that regularly.
const unsigned long IMU_HEALTH_MS = 500;
// A live gyro always has a few LSB of noise; 2 s of identical raw
// values means the sensor output is frozen.
const unsigned long GYRO_FROZEN_SAMPLES = 400;

// Startup / on-demand bias calibration (robot must be still).
const int GYRO_CAL_SAMPLES = 400;         // ~2 s at 200 Hz
const float GYRO_CAL_MAX_STDDEV_DPS = 0.6f;
const int GYRO_CAL_STARTUP_ATTEMPTS = 3;

// Zero-velocity update: with motors off and settled, a small
// rate is treated as bias. Slowly tracks thermal bias drift.
const unsigned long ZUPT_SETTLE_MS = 800;
const float ZUPT_MAX_RATE_DPS = 1.5f;
const float ZUPT_BIAS_ALPHA = 0.002f;

const unsigned long DIAG_INTERVAL_MS = 1000;

// =====================================================
// STATE
// =====================================================

bool autoMode = false;

int throttleValue = 0;
int steeringValue = 0;

unsigned long lastDriveCommand = 0;
unsigned long lastAutoHeartbeat = 0;
unsigned long stateStarted = 0;
unsigned long watchdogStops = 0;

enum AutoState {
  DRIVING,
  REVERSING,
  TURNING
};

AutoState autoState = DRIVING;
bool turnRightNext = true;
uint32_t autoLastSeq = 0;
int autoInvalidCount = 0;
float stallRefCm = -1000;
unsigned long stallRefMs = 0;
double turnStartYaw = 0;
unsigned long autoBackoffs = 0;
const char* autoReason = "none";

// MANUAL forward guard (latched until the robot turns or backs away)
bool guardLatched = false;
double guardYaw = 0;
uint32_t guardSeenSeq = 0;
unsigned long guardLastCloseMs = 0;
int autoBlindStreak = 0;

// Applied motor PWM after trim (signed, -255..255).
int leftApplied = 0;
int rightApplied = 0;
unsigned long lastMotorActiveMs = 0;

// Time integral of applied PWM (PWM * s). The mapper differences
// these, so it gets the true average command between its polls.
double leftCmdIntegral = 0;
double rightCmdIntegral = 0;
int64_t cmdIntegralUs = 0;

// IMU
bool imuOk = false;
uint8_t imuWhoAmI = 0;
unsigned long imuErrors = 0;
int imuConsecutiveErrors = 0;
unsigned long imuLastRetryMs = 0;
int64_t imuLastPollUs = 0;
int64_t imuLastSampleUs = 0;
unsigned long imuGapCount = 0;
unsigned long gyroSampleCount = 0;
unsigned long imuResets = 0;
unsigned long imuLastHealthMs = 0;
int16_t gyroLastRaw = 0;
unsigned long gyroSameRawCount = 0;

float gyroRawDps = 0;       // sensor output, bias NOT removed
float gyroBiasDps = 0;
float gyroRateDps = 0;      // bias removed (0 while stationary)
float gyroPrevRateDps = 0;
double yawUnwrappedDeg = 0; // + = counter-clockwise, chip face up
bool gyroStationary = false;

enum GyroCalState {
  CAL_NONE,
  CAL_RUNNING,
  CAL_OK,
  CAL_FAILED_MOTION,
  CAL_FAILED_IMU,
  CAL_UNVERIFIED
};

GyroCalState gyroCalState = CAL_NONE;
int calCount = 0;
double calMean = 0;
double calM2 = 0;
float calStddevDps = 0;

// Ultrasonic: echo edges timed in an ISR, processed in loop().
volatile int64_t echoRiseUs = 0;
volatile int64_t echoFallUs = 0;
volatile bool echoHigh = false;
volatile bool echoComplete = false;
volatile bool echoArmed = false;
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;

bool usWaiting = false;
int64_t usTriggerUs = 0;
unsigned long usLastTriggerMs = 0;

float usDistanceCm = -1;
bool usValid = false;
const char* usStatus = "none";
unsigned long usSampleMs = 0;
uint32_t usSeq = 0;
unsigned long usValidCount = 0;
unsigned long usInvalidCount = 0;

// Last 3 readings for obstacle votes (-1 = invalid, 0 = too close).
float usRecentCm[3] = {-1, -1, -1};
int usRecentIdx = 0;

// Diagnostics
uint32_t bootId = 0;
const char* resetReason = "unknown";
unsigned long loopCount = 0;
unsigned long loopHz = 0;
unsigned long lastDiagMs = 0;

// =====================================================
// EVENT LOG
// Everything useful for debugging goes into a RAM ring buffer
// that the Mac downloads with GET /api/log?since=N. The newest
// events are mirrored into RTC memory, which usually survives a
// crash / watchdog / brownout reset, so the moments before a
// reboot can be read after it.
// Tags MUST be string literals: they are stored as pointers.
// =====================================================

enum EvType : uint8_t {
  EV_BOOT, EV_RANGE, EV_GYRO, EV_MOTOR, EV_JOY, EV_LINKGAP,
  EV_WATCHDOG, EV_ESTOP, EV_MODE, EV_SPEED, EV_GUARD, EV_AUTO,
  EV_IMU, EV_WIFI, EV_STATS, EV_REQS, EV_PREV_BOOT, EV_STALL,
  EV_COUNT
};

const char* const EV_NAMES[EV_COUNT] = {
  "BOOT", "RANGE", "GYRO", "MOTOR", "JOY", "LINKGAP",
  "WATCHDOG", "ESTOP", "MODE", "SPEED", "GUARD", "AUTO",
  "IMU", "WIFI", "STATS", "REQS", "PREV_BOOT", "STALL"
};

const uint8_t EV_FROM_PREV_BOOT = 0x80;

struct LogEvent {
  uint32_t ms;
  const char* tag;
  float f;
  int16_t a;
  int16_t b;
  uint8_t type;
};

const uint32_t LOG_SIZE = 3000;        // ~100 s at ~30 events/s
const uint32_t RTC_LOG_SIZE = 96;      // ~3 s before a reset
const unsigned long GYRO_LOG_MS = 100; // yaw logged at 10 Hz
const unsigned long LINK_GAP_LOG_MS = 300;

LogEvent logBuf[LOG_SIZE];
uint32_t logSeq = 0;

struct RtcLog {
  uint32_t magic;
  uint32_t seq;
  LogEvent ev[RTC_LOG_SIZE];
};

RTC_NOINIT_ATTR RtcLog rtcLog;

unsigned long lastGyroLogMs = 0;
unsigned long lastWifiCheckMs = 0;
int lastStationCount = 0;
unsigned long driveReqs = 0;
unsigned long telemetryReqs = 0;
unsigned long otherReqs = 0;
int lastLoggedThrottle = 0;
int lastLoggedSteering = 0;

// Changes with every build, so a fresh upload ignores RTC data from
// the previous firmware (whose tag pointers would be meaningless).
uint32_t firmwareMagic() {
  const char* p = __DATE__ " " __TIME__;
  uint32_t h = 2166136261u;

  while (*p) {
    h ^= (uint8_t)*p++;
    h *= 16777619u;
  }

  return h;
}

int16_t clamp16(long v) {
  return (int16_t)constrain(v, -32768L, 32767L);
}

void logRaw(const LogEvent& e) {
  logBuf[logSeq % LOG_SIZE] = e;
  logSeq++;
}

void logEvent(uint8_t type, const char* tag, long a, long b, float f) {
  LogEvent e;
  e.ms = millis();
  e.tag = tag;
  e.f = f;
  e.a = clamp16(a);
  e.b = clamp16(b);
  e.type = type;

  logRaw(e);

  rtcLog.ev[rtcLog.seq % RTC_LOG_SIZE] = e;
  rtcLog.seq++;
}

// Called once in setup(): copy the pre-reset events into the log.
void logInit(const char* reason) {
  uint32_t magic = firmwareMagic();

  if (rtcLog.magic == magic && rtcLog.seq > 0 &&
      strcmp(reason, "power_on") != 0) {
    uint32_t n = min(rtcLog.seq, RTC_LOG_SIZE);
    uint32_t first = rtcLog.seq - n;

    LogEvent marker = {0, "begin", 0, clamp16(n), 0, EV_PREV_BOOT};
    logRaw(marker);

    for (uint32_t i = first; i < rtcLog.seq; i++) {
      LogEvent e = rtcLog.ev[i % RTC_LOG_SIZE];
      e.type |= EV_FROM_PREV_BOOT;
      logRaw(e);
    }

    marker.tag = "end";
    logRaw(marker);

    Serial.printf("[LOG] Recovered %lu events from before the reset.\n",
                  (unsigned long)n);
  }

  rtcLog.magic = magic;
  rtcLog.seq = 0;

  logEvent(EV_BOOT, reason, 0, 0, 0);
}

// =====================================================
// LOOP DEAD-MAN
// Everything (sensors, watchdogs, obstacle checks) runs in loop().
// If loop() freezes - e.g. a network write stuck on a bad Wi-Fi
// link - the motors would keep their last command. A separate
// esp_timer task cuts the motors at the hardware when loop() has
// not checked in for DEADMAN_MS, and records where it was stuck.
// If loop() hangs for 5 s the task watchdog reboots the chip.
// =====================================================

const unsigned long DEADMAN_MS = 300;
const unsigned long STALL_LOG_MS = 150;
const unsigned long STALL_RECORD_NOW_MS = 1000;

volatile unsigned long loopAliveMs = 0;
volatile const char* currentActivity = "boot";
volatile const char* stallActivity = "unknown";
volatile bool deadmanTripped = false;
volatile bool stallRecorded = false;
unsigned long deadmanTrips = 0;
esp_timer_handle_t deadmanTimer = nullptr;

void deadmanCheck(void*) {
  unsigned long stuck = millis() - loopAliveMs;

  if (stuck < STALL_LOG_MS) return;

  if (!stallRecorded) {
    stallActivity = (const char*)currentActivity;
  }

  if (stuck >= DEADMAN_MS && !deadmanTripped &&
      (leftApplied != 0 || rightApplied != 0)) {
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    deadmanTripped = true;
  }

  // Long hang: write it down now (also into RTC memory), so it is
  // readable even if the task watchdog has to reboot the chip.
  if (stuck >= STALL_RECORD_NOW_MS && !stallRecorded) {
    stallRecorded = true;
    logEvent(EV_STALL, (const char*)currentActivity, stuck,
             deadmanTripped ? 1 : 0, 0);
  }
}

void startDeadman() {
  esp_timer_create_args_t args = {};
  args.callback = deadmanCheck;
  args.name = "deadman";

  if (esp_timer_create(&args, &deadmanTimer) == ESP_OK) {
    esp_timer_start_periodic(deadmanTimer, 50000);  // 50 ms
  } else {
    Serial.println("[SAFETY] WARNING: dead-man timer could not start.");
  }
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void accumulateCommands() {
  int64_t nowUs = esp_timer_get_time();
  double dt = (nowUs - cmdIntegralUs) * 1e-6;
  cmdIntegralUs = nowUs;

  leftCmdIntegral += leftApplied * dt;
  rightCmdIntegral += rightApplied * dt;
}

// src says who commanded the motors; it is logged on every change.
void drive(int left, int right, const char* src) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  left = (int)(left * LEFT_TRIM);
  right = (int)(right * RIGHT_TRIM);

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Close the command integral before the command changes.
  accumulateCommands();

  if (left != 0 || right != 0 ||
      leftApplied != 0 || rightApplied != 0) {
    lastMotorActiveMs = millis();
  }

  if (left != leftApplied || right != rightApplied) {
    logEvent(EV_MOTOR, src, left, right, 0);
  }

  leftApplied = left;
  rightApplied = right;

  // LEFT MOTOR
  digitalWrite(IN1, left > 0 ? HIGH : LOW);
  digitalWrite(IN2, left < 0 ? HIGH : LOW);

  // RIGHT MOTOR
  digitalWrite(IN3, right > 0 ? HIGH : LOW);
  digitalWrite(IN4, right < 0 ? HIGH : LOW);

  ledcWrite(ENA, abs(left));
  ledcWrite(ENB, abs(right));
}

void stopBot(const char* src) {
  drive(0, 0, src);
}

void driveJoystick(int throttle, int steering, const char* src) {
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

  // Obstacle guard: drop the forward part, keep turning / reverse.
  if (guardLatched && throttle > 0) {
    throttle = 0;
    leftMix = steering;
    rightMix = -steering;
  }

  int leftPWM = leftMix * motorSpeed / 100;
  int rightPWM = rightMix * motorSpeed / 100;

  drive(leftPWM, rightPWM, src);
}

// =====================================================
// MPU6050 GYRO (direct register access)
// =====================================================

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, len) != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }

  return true;
}

void i2cBegin() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(IMU_I2C_HZ);
  Wire.setTimeOut(10);
}

// Free a bus left stuck by a half-finished transfer (SDA held low),
// e.g. after a power dip, by clocking SCL until SDA is released.
void i2cBusRecover() {
  Wire.end();

  pinMode(IMU_SDA, INPUT_PULLUP);
  pinMode(IMU_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(IMU_SCL, HIGH);

  for (int i = 0; i < 9 && digitalRead(IMU_SDA) == LOW; i++) {
    digitalWrite(IMU_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(IMU_SCL, HIGH);
    delayMicroseconds(10);
  }

  i2cBegin();
}

void imuFault(const char* reason) {
  imuOk = false;
  imuLastRetryMs = millis();
  gyroRateDps = 0;
  gyroSameRawCount = 0;

  Serial.printf("[IMU] ERROR: %s. Yaw frozen, recovering.\n", reason);
  logEvent(EV_IMU, reason, imuErrors, imuResets, gyroRawDps);

  if (gyroCalState == CAL_RUNNING) {
    gyroCalState = CAL_FAILED_IMU;
  }
}

bool mpuInit() {
  uint8_t who = 0;

  if (!mpuRead(MPU_REG_WHO_AM_I, &who, 1)) {
    Serial.println("[IMU] ERROR: MPU6050 not responding at 0x68.");
    Serial.println("[IMU]        Check SDA=GPIO21, SCL=GPIO22, VCC=3V3, GND, AD0 low.");
    imuOk = false;
    return false;
  }

  imuWhoAmI = who;

  if (who != 0x68) {
    Serial.printf("[IMU] WARNING: WHO_AM_I=0x%02X (expected 0x68). "
                  "Compatible clone? Continuing.\n", who);
  }

  // Wake, PLL clock from gyro X, 200 Hz, DLPF ~44 Hz, +/-1000 dps.
  bool ok = mpuWrite(MPU_REG_PWR_MGMT_1, 0x01) &&
            mpuWrite(MPU_REG_SMPLRT_DIV, 0x04) &&
            mpuWrite(MPU_REG_CONFIG, 0x03) &&
            mpuWrite(MPU_REG_GYRO_CONFIG, 0x10);

  if (!ok) {
    Serial.println("[IMU] ERROR: MPU6050 configuration write failed.");
    imuOk = false;
    return false;
  }

  imuOk = true;
  imuConsecutiveErrors = 0;
  return true;
}

bool mpuReadGyroZ(int16_t& raw) {
  uint8_t buf[2];

  if (!mpuRead(MPU_REG_GYRO_ZOUT_H, buf, 2)) {
    return false;
  }

  raw = (int16_t)((buf[0] << 8) | buf[1]);
  return true;
}

double wrap180(double deg) {
  double w = fmod(deg + 180.0, 360.0);
  if (w < 0) w += 360.0;
  return w - 180.0;
}

const char* gyroCalName() {
  switch (gyroCalState) {
    case CAL_RUNNING: return "running";
    case CAL_OK: return "ok";
    case CAL_FAILED_MOTION: return "failed_motion";
    case CAL_FAILED_IMU: return "failed_imu";
    case CAL_UNVERIFIED: return "unverified";
    default: return "none";
  }
}

bool startGyroCalibration() {
  if (!imuOk) {
    return false;
  }

  calCount = 0;
  calMean = 0;
  calM2 = 0;
  gyroCalState = CAL_RUNNING;

  Serial.println("[IMU] Gyro bias calibration started - keep the robot still.");
  return true;
}

void gyroCalibrationStep() {
  if (leftApplied != 0 || rightApplied != 0) {
    gyroCalState = CAL_FAILED_MOTION;
    Serial.println("[IMU] Calibration aborted: motors running. Bias unchanged.");
    logEvent(EV_IMU, "cal_aborted_motors", 0, 0, 0);
    return;
  }

  // Welford running mean / variance.
  calCount++;
  double delta = gyroRawDps - calMean;
  calMean += delta / calCount;
  calM2 += delta * (gyroRawDps - calMean);

  if (calCount < GYRO_CAL_SAMPLES) {
    return;
  }

  calStddevDps = sqrt(calM2 / (calCount - 1));

  if (calStddevDps > GYRO_CAL_MAX_STDDEV_DPS) {
    gyroCalState = CAL_FAILED_MOTION;
    logEvent(EV_IMU, "cal_rejected_moving", calCount, 0, calStddevDps);
    Serial.printf("[IMU] Calibration rejected: noise %.3f dps > %.2f "
                  "(robot moved?). Bias unchanged.\n",
                  calStddevDps, GYRO_CAL_MAX_STDDEV_DPS);
    return;
  }

  gyroBiasDps = calMean;
  gyroCalState = CAL_OK;
  logEvent(EV_IMU, "cal_ok", calCount, 0, gyroBiasDps);
  Serial.printf("[IMU] Gyro Z bias = %.4f dps (noise %.3f dps, %d samples)\n",
                gyroBiasDps, calStddevDps, calCount);
}

void imuUpdate() {
  if (!imuOk) {
    if (millis() - imuLastRetryMs >= IMU_RETRY_MS) {
      imuLastRetryMs = millis();
      i2cBusRecover();

      if (mpuInit()) {
        imuResets++;
        logEvent(EV_IMU, "recovered", imuErrors, imuResets, 0);
        Serial.println("[IMU] MPU6050 recovered. Yaw resumes from its last "
                       "value; rotation during the outage was missed.");
        imuLastSampleUs = esp_timer_get_time();
        imuLastPollUs = imuLastSampleUs;
        imuLastHealthMs = millis();
        gyroPrevRateDps = 0;
      }
    }
    return;
  }

  // Detect a brown-out reset: the chip wakes asleep, config lost,
  // but still answers on I2C.
  if (millis() - imuLastHealthMs >= IMU_HEALTH_MS) {
    imuLastHealthMs = millis();
    uint8_t pm = 0;

    if (mpuRead(MPU_REG_PWR_MGMT_1, &pm, 1) && pm != 0x01) {
      imuResets++;
      logEvent(EV_IMU, "config_lost_power_dip", pm, imuResets, 0);
      Serial.printf("[IMU] MPU6050 lost its config (PWR_MGMT_1=0x%02X), "
                    "likely a power dip from motor stall. Reconfiguring.\n", pm);
      mpuInit();
      gyroSameRawCount = 0;
    }
  }

  int64_t nowUs = esp_timer_get_time();

  if (nowUs - imuLastPollUs < IMU_INTERVAL_US) {
    return;
  }

  imuLastPollUs = nowUs;

  int16_t raw;

  if (!mpuReadGyroZ(raw)) {
    imuErrors++;

    if (++imuConsecutiveErrors >= IMU_MAX_CONSECUTIVE_ERRORS) {
      imuFault("MPU6050 stopped responding");
    }
    return;
  }

  imuConsecutiveErrors = 0;

  if (raw == gyroLastRaw) {
    if (++gyroSameRawCount >= GYRO_FROZEN_SAMPLES) {
      imuFault("gyro output frozen");
      return;
    }
  } else {
    gyroSameRawCount = 0;
    gyroLastRaw = raw;
  }

  int64_t sampleUs = esp_timer_get_time();
  float dt = (sampleUs - imuLastSampleUs) * 1e-6f;
  imuLastSampleUs = sampleUs;

  gyroSampleCount++;
  gyroRawDps = raw / GYRO_LSB_PER_DPS;

  if (gyroCalState == CAL_RUNNING) {
    gyroCalibrationStep();
    gyroRateDps = 0;
    gyroPrevRateDps = 0;
    gyroStationary = true;
    return;
  }

  float rate = gyroRawDps - gyroBiasDps;

  bool motorsSettled = leftApplied == 0 && rightApplied == 0 &&
                       millis() - lastMotorActiveMs >= ZUPT_SETTLE_MS;

  gyroStationary = motorsSettled && fabsf(rate) < ZUPT_MAX_RATE_DPS;

  if (gyroStationary) {
    gyroBiasDps += ZUPT_BIAS_ALPHA * rate;
    rate = 0;
  }

  gyroRateDps = rate;

  // Integrate with the real elapsed time. Trapezoidal normally;
  // after a long gap only the current rate is trustworthy.
  if (dt > IMU_MAX_DT_S) {
    imuGapCount++;
    yawUnwrappedDeg += rate * dt;
  } else {
    yawUnwrappedDeg += 0.5f * (rate + gyroPrevRateDps) * dt;
  }

  gyroPrevRateDps = rate;
}

// =====================================================
// ULTRASONIC SENSOR (non-blocking)
// =====================================================

// Runs on both echo edges. Reads the GPIO register directly so
// the ISR stays IRAM-safe.
void IRAM_ATTR echoISR() {
  int64_t t = esp_timer_get_time();
  bool level = (REG_READ(GPIO_IN_REG) >> ECHO) & 0x1;

  portENTER_CRITICAL_ISR(&echoMux);

  // Only time the first full pulse after a trigger. Motor noise can
  // put short spikes on the ECHO wire; those are ignored.
  if (echoArmed) {
    if (level) {
      if (!echoHigh) {
        echoRiseUs = t;
        echoHigh = true;
      }
    } else if (echoHigh) {
      if (t - echoRiseUs >= US_MIN_PULSE_US) {
        echoFallUs = t;
        echoComplete = true;
        echoArmed = false;
      }
      echoHigh = false;
    }
  }

  portEXIT_CRITICAL_ISR(&echoMux);
}

void publishRange(float cm, bool valid, const char* status) {
  usDistanceCm = valid ? cm : -1;
  usValid = valid;
  usStatus = status;
  usSampleMs = millis();
  usSeq++;

  usRecentCm[usRecentIdx] = valid ? cm :
    (strcmp(status, "too_close") == 0 ? 0.0f : -1.0f);
  usRecentIdx = (usRecentIdx + 1) % 3;

  if (valid) usValidCount++;
  else usInvalidCount++;

  logEvent(EV_RANGE, status, usSeq & 0x7FFF, 0, cm);
}

// How many of the last 3 readings say "closer than limitCm".
int closeVotes(float limitCm) {
  int votes = 0;

  for (int i = 0; i < 3; i++) {
    if (usRecentCm[i] >= 0 && usRecentCm[i] <= limitCm) votes++;
  }

  return votes;
}

// The last two readings are both valid and at least minCm.
bool lastTwoClear(float minCm) {
  int a = (usRecentIdx + 2) % 3;
  int b = (usRecentIdx + 1) % 3;
  return usRecentCm[a] >= minCm && usRecentCm[b] >= minCm;
}

void ultrasonicUpdate() {
  if (usWaiting) {
    bool complete;
    bool high;
    int64_t rise;
    int64_t fall;

    portENTER_CRITICAL(&echoMux);
    complete = echoComplete;
    high = echoHigh;
    rise = echoRiseUs;
    fall = echoFallUs;
    portEXIT_CRITICAL(&echoMux);

    if (complete && rise >= usTriggerUs) {
      float cm = (fall - rise) * US_CM_PER_US;

      if (cm < US_MIN_VALID_CM) {
        publishRange(cm, false, "too_close");
      } else if (cm > US_MAX_VALID_CM) {
        publishRange(cm, false, "out_of_range");
      } else {
        publishRange(cm, true, "ok");
      }

      usWaiting = false;
    } else if (esp_timer_get_time() - usTriggerUs > US_ECHO_TIMEOUT_US) {
      // Pulse started but never ended: nothing reflected in range.
      // Pulse never started: sensor or wiring fault.
      portENTER_CRITICAL(&echoMux);
      echoArmed = false;
      portEXIT_CRITICAL(&echoMux);

      publishRange(-1, false, high ? "no_echo" : "no_pulse");
      usWaiting = false;
    }

    return;
  }

  if (millis() - usLastTriggerMs < US_INTERVAL_MS) {
    return;
  }

  usLastTriggerMs = millis();

  if (digitalRead(ECHO) == HIGH) {
    publishRange(-1, false, "echo_stuck");
    return;
  }

  portENTER_CRITICAL(&echoMux);
  echoComplete = false;
  echoHigh = false;
  echoArmed = false;
  portEXIT_CRITICAL(&echoMux);

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  usTriggerUs = esp_timer_get_time();
  usWaiting = true;

  portENTER_CRITICAL(&echoMux);
  echoArmed = true;
  portEXIT_CRITICAL(&echoMux);
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
    <span id="speedValue">120 / 255</span>
  </div>

  <input id="speedSlider" type="range"
    min="80" max="255" value="120">

  <div class="scale">
    <span>LOW</span>
    <span>MAX</span>
  </div>
</div>

<button class="stop" onclick="emergencyStop('button')">
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
let modeChangedAt = 0;

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
  })
    .then(r => r.text())
    .then(t => {
      if (mode !== 'M') return;
      const p = t.split(' ');
      document.getElementById('status').textContent =
        p[0] !== 'BLOCKED' ? 'HOLD TO DRIVE' :
        'OBSTACLE ' + (p[1] ? p[1] + ' CM ' : '') + '- TURN OR REVERSE';
    })
    .catch(() => {});
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

function emergencyStop(why) {
  throttle = 0;
  steering = 0;

  resetStick('throttle');
  resetStick('steer');

  // The robot also leaves AUTO on /stop.
  if (mode === 'A') applyModeUi('M');

  // Dedicated stop request.
  fetch('/stop?why=' + (why || 'other') + '&ts=' + Date.now(), {
    cache: 'no-store'
  }).catch(() => {});
}

function applyModeUi(nextMode) {
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
}

function setMode(nextMode) {
  applyModeUi(nextMode);
  modeChangedAt = Date.now();

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

// AUTO link heartbeat. The robot leaves AUTO if these stop,
// and replies 'M' if it already has.
setInterval(() => {
  if (mode !== 'A') return;

  fetch('/hb?ts=' + Date.now(), {
    cache: 'no-store'
  })
    .then(r => r.text())
    .then(m => {
      m = m.trim();
      if (mode === 'A' && m === 'M' &&
          Date.now() - modeChangedAt > 1500) {
        applyModeUi('M');
      } else if (mode === 'A' && m.startsWith('A ')) {
        // Show what AUTO is doing and why.
        document.getElementById('status').textContent =
          m.slice(2).toUpperCase();
      }
    })
    .catch(() => {});
}, 400);

window.addEventListener('blur', () => emergencyStop('blur'));

document.addEventListener('visibilitychange', () => {
  if (document.hidden) emergencyStop('hidden');
});
</script>
</body>
</html>
)HTML";

// =====================================================
// HTTP HANDLERS
// =====================================================

void handleRoot() {
  currentActivity = "http_page";
  otherReqs++;
  server.send_P(200, "text/html", webpage);
}

void handleDrive() {
  currentActivity = "http_drive";
  driveReqs++;
  server.client().setTimeout(200);

  if (!autoMode && server.hasArg("t") && server.hasArg("s")) {
    unsigned long gap = millis() - lastDriveCommand;

    // While driving the phone repeats every 150 ms; a longer gap
    // means Wi-Fi lag and explains jerky or watchdog stops.
    if ((throttleValue != 0 || steeringValue != 0) &&
        gap > LINK_GAP_LOG_MS) {
      logEvent(EV_LINKGAP, "drive", gap, 0, 0);
    }

    throttleValue = constrain(server.arg("t").toInt(), -100, 100);
    steeringValue = constrain(server.arg("s").toInt(), -100, 100);

    if (throttleValue != lastLoggedThrottle ||
        steeringValue != lastLoggedSteering) {
      logEvent(EV_JOY, "phone", throttleValue, steeringValue, 0);
      lastLoggedThrottle = throttleValue;
      lastLoggedSteering = steeringValue;
    }

    // Refresh timeout on every received heartbeat.
    lastDriveCommand = millis();

    driveJoystick(throttleValue, steeringValue, "phone");
  }

  if (!autoMode && guardLatched && throttleValue > 0) {
    String reply = "BLOCKED";
    if (usValid) reply += " " + String((int)usDistanceCm);
    server.send(200, "text/plain", reply);
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleStop() {
  currentActivity = "http_stop";
  otherReqs++;
  server.client().setTimeout(200);

  // Why the phone sent it: the button, or the page losing focus
  // (notification, app switch, screen lock).
  String w = server.arg("why");
  const char* why = w == "button" ? "estop_button" :
                    w == "blur" ? "estop_page_blur" :
                    w == "hidden" ? "estop_page_hidden" : "estop_other";

  logEvent(EV_ESTOP, why, autoMode ? 1 : 0, 0, 0);

  throttleValue = 0;
  steeringValue = 0;
  stopBot(why);

  // Emergency stop must also end autonomous driving.
  if (autoMode) {
    autoMode = false;
    autoState = DRIVING;
    Serial.println("[SAFETY] Emergency stop - AUTO mode ended.");
  }

  server.send(200, "text/plain", "STOPPED");
}

void handleMode() {
  currentActivity = "http_mode";
  otherReqs++;
  server.client().setTimeout(200);
  autoMode = server.hasArg("m") && server.arg("m") == "A";
  logEvent(EV_MODE, autoMode ? "auto" : "manual", 0, 0, 0);

  throttleValue = 0;
  steeringValue = 0;
  stopBot("mode_change");

  autoState = DRIVING;
  stateStarted = millis();
  lastAutoHeartbeat = millis();
  autoLastSeq = usSeq;
  autoInvalidCount = 0;
  autoBlindStreak = 0;
  stallRefCm = -1000;
  guardLatched = false;

  Serial.printf("[MODE] %s\n", autoMode ? "AUTO" : "MANUAL");

  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("v")) {
    int v = constrain(server.arg("v").toInt(), 80, 255);

    if (v != motorSpeed) {
      logEvent(EV_SPEED, "slider", v, motorSpeed, 0);
    }

    motorSpeed = v;

    if (!autoMode) {
      driveJoystick(throttleValue, steeringValue, "speed_slider");
    }
  }

  server.send(200, "text/plain", "OK");
}

// Reply "M", or "A <what AUTO is doing> | <range>" for the phone.
void handleHeartbeat() {
  currentActivity = "http_hb";
  otherReqs++;
  server.client().setTimeout(200);

  if (!autoMode) {
    server.send(200, "text/plain", "M");
    return;
  }

  lastAutoHeartbeat = millis();

  bool stopped = leftApplied == 0 && rightApplied == 0;
  char state[48];

  if (autoState == REVERSING) {
    snprintf(state, sizeof(state), "BACKING OFF (%s)", autoReason);
  } else if (autoState == TURNING) {
    snprintf(state, sizeof(state), "TURNING (%s)", autoReason);
  } else if (stopped && strcmp(autoReason, "blind_waiting") == 0) {
    snprintf(state, sizeof(state), "STOPPED - SENSOR SEES NOTHING");
  } else if (stopped) {
    snprintf(state, sizeof(state), "WAITING FOR CLEAR READING");
  } else {
    snprintf(state, sizeof(state), "DRIVING PWM %d", leftApplied);
  }

  char msg[96];
  if (usValid) {
    snprintf(msg, sizeof(msg), "A %s | %d CM", state, (int)usDistanceCm);
  } else {
    snprintf(msg, sizeof(msg), "A %s | %s", state, usStatus);
  }

  server.send(200, "text/plain", msg);
}

// Read-only snapshot for the Mac mapper. Never touches the motors
// and never triggers a sensor read; it only formats cached state.
void handleTelemetry() {
  currentActivity = "http_telemetry";
  telemetryReqs++;
  server.client().setTimeout(200);
  accumulateCommands();

  unsigned long now = millis();

  char dist[16];
  if (usValid) {
    snprintf(dist, sizeof(dist), "%.1f", usDistanceCm);
  } else {
    strcpy(dist, "null");
  }

  const char* stateName =
    autoState == REVERSING ? "reversing" :
    autoState == TURNING ? "turning" : "driving";

  char json[1280];
  int n = snprintf(json, sizeof(json),
    "{"
    "\"fw\":\"%s\","
    "\"boot_id\":%lu,"
    "\"millis\":%lu,"
    "\"mode\":\"%s\","
    "\"auto_state\":\"%s\","
    "\"distance_cm\":%s,"
    "\"distance_valid\":%s,"
    "\"us_status\":\"%s\","
    "\"us_seq\":%lu,"
    "\"us_age_ms\":%lu,"
    "\"us_valid_count\":%lu,"
    "\"us_invalid_count\":%lu,"
    "\"imu_ok\":%s,"
    "\"imu_who_am_i\":%u,"
    "\"imu_errors\":%lu,"
    "\"imu_gaps\":%lu,"
    "\"yaw_deg\":%.3f,"
    "\"yaw_unwrapped_deg\":%.3f,"
    "\"gyro_z_dps\":%.3f,"
    "\"gyro_z_raw_dps\":%.3f,"
    "\"gyro_bias_dps\":%.4f,"
    "\"gyro_stationary\":%s,"
    "\"gyro_cal\":\"%s\","
    "\"gyro_cal_stddev_dps\":%.4f,"
    "\"gyro_samples\":%lu,"
    "\"left_cmd\":%d,"
    "\"right_cmd\":%d,"
    "\"left_cmd_int\":%.3f,"
    "\"right_cmd_int\":%.3f,"
    "\"throttle\":%d,"
    "\"steering\":%d,"
    "\"motor_speed\":%d,"
    "\"watchdog_stops\":%lu,"
    "\"fwd_blocked\":%s,"
    "\"auto_backoffs\":%lu,"
    "\"auto_reason\":\"%s\","
    "\"imu_resets\":%lu,"
    "\"reset_reason\":\"%s\","
    "\"loop_hz\":%lu,"
    "\"home_ip\":\"%s\","
    "\"home_rssi\":%d,"
    "\"free_heap\":%lu"
    "}",
    FW_VERSION,
    (unsigned long)bootId,
    now,
    autoMode ? "auto" : "manual",
    stateName,
    dist,
    usValid ? "true" : "false",
    usStatus,
    (unsigned long)usSeq,
    usSeq == 0 ? 999999UL : now - usSampleMs,
    usValidCount,
    usInvalidCount,
    imuOk ? "true" : "false",
    (unsigned)imuWhoAmI,
    imuErrors,
    imuGapCount,
    wrap180(yawUnwrappedDeg),
    yawUnwrappedDeg,
    gyroRateDps,
    gyroRawDps,
    gyroBiasDps,
    gyroStationary ? "true" : "false",
    gyroCalName(),
    calStddevDps,
    gyroSampleCount,
    leftApplied,
    rightApplied,
    leftCmdIntegral,
    rightCmdIntegral,
    throttleValue,
    steeringValue,
    motorSpeed,
    watchdogStops,
    guardLatched ? "true" : "false",
    autoBackoffs,
    autoReason,
    imuResets,
    resetReason,
    loopHz,
    homeWifiConnected ? WiFi.localIP().toString().c_str() : "",
    homeWifiConnected ? (int)WiFi.RSSI() : 0,
    (unsigned long)ESP.getFreeHeap());

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (n < 0 || n >= (int)sizeof(json)) {
    server.send(500, "application/json", "{\"error\":\"telemetry overflow\"}");
    return;
  }

  server.send(200, "application/json", json);
}

// GET /api/log?since=N -> CSV lines
//   #boot=<id>,next=<seq>,oldest=<seq>,more=<0|1>,now=<ms>,fw=<ver>
//   seq,ms,TYPE,tag,a,b,f
// Ask again with since=<next>. If since < oldest, events were lost.
// The whole reply is kept under LOG_REPLY_BYTES so it fits the TCP
// send buffer (5744 B) and goes out in ONE non-blocking write. Big
// multi-write replies froze loop() for minutes on a bad Wi-Fi link.
const size_t LOG_REPLY_BYTES = 3500;
char logReply[LOG_REPLY_BYTES + 128];

void handleLog() {
  currentActivity = "http_log";
  otherReqs++;
  server.client().setTimeout(200);

  uint32_t since = server.hasArg("since")
    ? strtoul(server.arg("since").c_str(), nullptr, 10) : 0;

  uint32_t oldest = logSeq > LOG_SIZE ? logSeq - LOG_SIZE : 0;
  uint32_t start = since < oldest ? oldest : since;
  if (start > logSeq) start = logSeq;

  // Lines first (header is written in front afterwards).
  char lines[LOG_REPLY_BYTES];
  size_t len = 0;
  uint32_t i = start;

  for (; i < logSeq; i++) {
    const LogEvent& e = logBuf[i % LOG_SIZE];
    uint8_t t = e.type & 0x7F;
    char line[112];

    int n = snprintf(line, sizeof(line), "%lu,%lu,%s%s,%s,%d,%d,%.2f\n",
                     (unsigned long)i, (unsigned long)e.ms,
                     (e.type & EV_FROM_PREV_BOOT) ? "PREV:" : "",
                     t < EV_COUNT ? EV_NAMES[t] : "?",
                     e.tag ? e.tag : "",
                     e.a, e.b, e.f);

    if (n <= 0) continue;
    if (len + n >= sizeof(lines)) break;

    memcpy(lines + len, line, n);
    len += n;
  }

  lines[len] = 0;

  snprintf(logReply, sizeof(logReply),
           "#boot=%lu,next=%lu,oldest=%lu,more=%d,now=%lu,fw=%s\n%s",
           (unsigned long)bootId, (unsigned long)i, (unsigned long)oldest,
           i < logSeq ? 1 : 0, millis(), FW_VERSION, lines);

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", logReply);
}

void handleCalibrateGyro() {
  server.sendHeader("Cache-Control", "no-store");

  if (leftApplied != 0 || rightApplied != 0) {
    server.send(409, "application/json",
                "{\"ok\":false,\"error\":\"stop the motors first\"}");
    return;
  }

  if (!startGyroCalibration()) {
    server.send(503, "application/json",
                "{\"ok\":false,\"error\":\"imu not ok\"}");
    return;
  }

  server.send(200, "application/json",
              "{\"ok\":true,\"info\":\"calibrating ~2 s, keep still\"}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// =====================================================
// SETUP
// =====================================================

void setupImu() {
  i2cBegin();

  if (!mpuInit()) {
    Serial.println("[IMU] Continuing WITHOUT gyro: imu_ok=false, yaw frozen.");
    Serial.println("[IMU] The controller still works. Retrying every 1 s.");
    imuLastRetryMs = millis();
    return;
  }

  Serial.printf("[IMU] MPU6050 found (WHO_AM_I=0x%02X)\n", imuWhoAmI);

  // Let the gyro and its filter settle after wake-up.
  delay(150);

  imuLastSampleUs = esp_timer_get_time();
  imuLastPollUs = imuLastSampleUs;

  for (int attempt = 1; attempt <= GYRO_CAL_STARTUP_ATTEMPTS; attempt++) {
    Serial.printf("[IMU] Startup calibration %d/%d\n",
                  attempt, GYRO_CAL_STARTUP_ATTEMPTS);

    if (!startGyroCalibration()) break;

    while (gyroCalState == CAL_RUNNING) {
      imuUpdate();
      delay(1);
    }

    if (gyroCalState == CAL_OK || !imuOk) break;
  }

  if (gyroCalState == CAL_FAILED_MOTION && calCount > 50) {
    gyroBiasDps = calMean;
    gyroCalState = CAL_UNVERIFIED;
    logEvent(EV_IMU, "cal_unverified", calCount, 0, gyroBiasDps);
    Serial.printf("[IMU] WARNING: robot never held still. Using bias %.4f dps "
                  "anyway. Re-run: curl http://192.168.4.1/api/calibrate_gyro\n",
                  gyroBiasDps);
  }

  yawUnwrappedDeg = 0;
  gyroPrevRateDps = 0;
  imuLastHealthMs = millis();
}

void setup() {
  Serial.begin(115200);
  delay(50);

  bootId = esp_random();

  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: resetReason = "power_on"; break;
    case ESP_RST_BROWNOUT: resetReason = "brownout"; break;
    case ESP_RST_PANIC: resetReason = "crash"; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: resetReason = "watchdog"; break;
    case ESP_RST_SW: resetReason = "software"; break;
    case ESP_RST_EXT: resetReason = "reset_button"; break;
    default: resetReason = "other"; break;
  }

  logInit(resetReason);

  Serial.println();
  Serial.println("=========================================");
  Serial.printf("AUTOBOT %s booting (reset reason: %s)\n",
                FW_VERSION, resetReason);

  if (strcmp(resetReason, "brownout") == 0) {
    Serial.println("WARNING: last reset was a BROWNOUT - the supply dipped, "
                   "usually from motor current. See README troubleshooting.");
  }
  Serial.println("Keep the robot STILL for gyro calibration.");
  Serial.println("=========================================");

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

  cmdIntegralUs = esp_timer_get_time();
  stopBot("boot");

  setupImu();

  attachInterrupt(digitalPinToInterrupt(ECHO), echoISR, CHANGE);

  homeWifiEnabled = strlen(HOME_WIFI_SSID) > 0 &&
                    strcmp(HOME_WIFI_PASSWORD, "PUT_YOUR_WIFI_PASSWORD_HERE") != 0;

  WiFi.mode(homeWifiEnabled ? WIFI_STA : WIFI_AP);

  if (!homeWifiEnabled) {
    WiFi.softAP(ssid, password);
  }

  // Power save adds 100+ ms latency spikes in station mode, enough
  // to trip the 700 ms drive watchdog. Keep the radio fully awake.
  WiFi.setSleep(false);

  if (homeWifiEnabled) {
    WiFi.setAutoReconnect(true);

    if (!WiFi.config(ROBOT_IP, ROUTER_IP, SUBNET_MASK, ROUTER_IP)) {
      Serial.println("[WIFI] Fixed IP setup failed - router will assign one.");
    }

    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
    homeWifiLastTryMs = millis();
    Serial.printf("[WIFI] Joining home network \"%s\" (non-blocking)...\n",
                  HOME_WIFI_SSID);
  } else {
    Serial.println("[WIFI] No home Wi-Fi in secrets.h - BotController AP.");
  }

  Serial.println();
  Serial.println("AUTOBOT MOBILE UNIT ONLINE");

  if (homeWifiEnabled) {
    Serial.printf("Wi-Fi: %s\n", HOME_WIFI_SSID);
    Serial.printf("Controller: http://%s\n", ROBOT_IP.toString().c_str());
  } else {
    Serial.print("Wi-Fi: ");
    Serial.println(ssid);
    Serial.print("Controller: http://");
    Serial.println(WiFi.softAPIP());
  }

  Serial.println("Telemetry:  /api/telemetry   Event log: /api/log");

  server.on("/", handleRoot);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/mode", handleMode);
  server.on("/speed", handleSpeed);
  server.on("/hb", handleHeartbeat);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/calibrate_gyro", HTTP_GET, handleCalibrateGyro);
  server.on("/api/log", HTTP_GET, handleLog);
  server.onNotFound(handleNotFound);

  server.begin();

  lastDiagMs = millis();
  loopAliveMs = millis();
  startDeadman();

  // Reboot if loop() ever hangs for 5 s (task watchdog).
  enableLoopWDT();
  Serial.println("[SAFETY] Motor dead-man 300 ms + 5 s hang reboot active.");
}

// =====================================================
// MANUAL WATCHDOG + FORWARD OBSTACLE GUARD
// =====================================================

void updateManualWatchdog() {
  // Stop if the manual controller heartbeat is lost.
  if ((throttleValue != 0 || steeringValue != 0) &&
      millis() - lastDriveCommand > DRIVE_TIMEOUT_MS) {
    logEvent(EV_WATCHDOG, "manual", millis() - lastDriveCommand,
             throttleValue, 0);
    throttleValue = 0;
    steeringValue = 0;
    stopBot("watchdog_manual");

    watchdogStops++;
    Serial.println("[SAFETY] Drive heartbeat lost - motors stopped.");
  }
}

// Runs once per new range sample. Latches when something is close,
// and stays latched while the sensor is blind (touching an object
// gives no echo) until a clear reading, a turn, or reversing.
void updateForwardGuard() {
  if (usSeq == guardSeenSeq) return;
  guardSeenSeq = usSeq;

  if (!MANUAL_OBSTACLE_GUARD || autoMode) {
    guardLatched = false;
    return;
  }

  bool closeNow = (usValid && usDistanceCm <= MANUAL_GUARD_CM) ||
                  strcmp(usStatus, "too_close") == 0;

  if (closeNow) {
    guardLastCloseMs = millis();
  }

  if (closeVotes(MANUAL_GUARD_CM) >= US_CONFIRM_VOTES) {
    if (!guardLatched) {
      guardLatched = true;
      guardYaw = yawUnwrappedDeg;
      logEvent(EV_GUARD, "latch", closeVotes(MANUAL_GUARD_CM), throttleValue,
               usValid ? usDistanceCm : -1.0f);
      Serial.printf("[GUARD] Obstacle at %.1f cm - forward blocked.\n",
                    usValid ? usDistanceCm : 0.0f);

      // Re-apply the joystick so forward motion stops now.
      if (throttleValue > 0) {
        driveJoystick(throttleValue, steeringValue, "guard");
      }
    }
    return;
  }

  if (!guardLatched) return;

  bool clear = lastTwoClear(MANUAL_GUARD_RELEASE_CM);
  bool turned = imuOk &&
                fabs(yawUnwrappedDeg - guardYaw) >= MANUAL_GUARD_RELEASE_DEG;
  bool reversing = throttleValue < 0;
  bool unconfirmed = millis() - guardLastCloseMs > MANUAL_GUARD_STALE_MS;

  if (clear || turned || reversing || unconfirmed) {
    logEvent(EV_GUARD,
             clear ? "release_clear" : turned ? "release_turned" :
             reversing ? "release_reversing" : "release_unconfirmed",
             0, throttleValue, usValid ? usDistanceCm : -1.0f);
    guardLatched = false;
    Serial.println("[GUARD] Forward released.");
  }
}

// =====================================================
// AUTONOMOUS OBSTACLE AVOIDANCE
// =====================================================

void startBackoff(unsigned long now, const char* reason) {
  autoBackoffs++;
  autoReason = reason;
  autoInvalidCount = 0;

  logEvent(EV_AUTO, reason, autoBlindStreak, closeVotes(OBSTACLE_CM),
           usValid ? usDistanceCm : -1.0f);

  Serial.printf("[AUTO] %s (range %s %.1f cm) -> reverse + turn\n",
                reason, usStatus, usValid ? usDistanceCm : -1.0f);

  stopBot("auto_backoff");

  autoState = REVERSING;
  stateStarted = now;

  drive(-REVERSE_SPEED, -REVERSE_SPEED, "auto_reverse");
}

void updateAutonomous() {
  unsigned long now = millis();

  // Leave AUTO if the phone disappears.
  if (now - lastAutoHeartbeat > AUTO_LINK_TIMEOUT_MS) {
    logEvent(EV_WATCHDOG, "auto_link", now - lastAutoHeartbeat, 0, 0);
    autoMode = false;
    autoState = DRIVING;
    stopBot("watchdog_auto_link");

    watchdogStops++;
    Serial.println("[SAFETY] AUTO link heartbeat lost - stopped, MANUAL.");
    return;
  }

  // Reverse briefly after detecting an obstacle.
  if (autoState == REVERSING) {
    if (now - stateStarted >= REVERSE_TIME_MS) {
      stopBot("auto_reverse_done");
      logEvent(EV_AUTO, "reverse_done", 0, 0, 0);

      autoState = TURNING;
      stateStarted = now;
      turnStartYaw = yawUnwrappedDeg;

      if (turnRightNext) {
        drive(TURN_SPEED, -TURN_SPEED, "auto_turn");
      } else {
        drive(-TURN_SPEED, TURN_SPEED, "auto_turn");
      }

      turnRightNext = !turnRightNext;
    }

    return;
  }

  // Turn by gyro angle (or time without a gyro), then resume.
  if (autoState == TURNING) {
    bool turnedEnough = imuOk
      ? fabs(yawUnwrappedDeg - turnStartYaw) >= AUTO_TURN_DEG
      : now - stateStarted >= TURN_TIME_MS;

    if (turnedEnough || now - stateStarted >= TURN_MAX_MS) {
      logEvent(EV_AUTO, turnedEnough ? "turn_done" : "turn_timeout",
               (long)fabs(yawUnwrappedDeg - turnStartYaw),
               now - stateStarted, 0);
      stopBot("auto_turn_done");

      autoState = DRIVING;
      stateStarted = now;
      autoLastSeq = usSeq;
      autoInvalidCount = 0;
      stallRefCm = -1000;
    }

    return;
  }

  // DRIVING: act once per new background range sample.
  if (usSeq == autoLastSeq) {
    // Fail safe if the sensor stops producing samples.
    if (now - usSampleMs > US_FRESH_MS) {
      stopBot("auto_sensor_stale");
    }
    return;
  }

  autoLastSeq = usSeq;

  // No valid echo: stop. Repeated blindness usually means the sensor
  // is pressed against something, so back off instead of parking.
  if (!usValid) {
    stopBot("auto_invalid_range");

    if (closeVotes(OBSTACLE_CM) >= US_CONFIRM_VOTES) {
      startBackoff(now, "too close");
    } else if (++autoInvalidCount >= AUTO_MAX_INVALID) {
      if (autoBlindStreak >= AUTO_MAX_BLIND_BACKOFFS) {
        // Keep reversing blind forever is worse than waiting.
        autoReason = "blind_waiting";
        logEvent(EV_AUTO, "blind_waiting", autoBlindStreak, 0, -1);
        autoInvalidCount = 0;
        return;
      }
      autoBlindStreak++;
      startBackoff(now, "sensor blind");
    }
    return;
  }

  autoInvalidCount = 0;

  if (closeVotes(OBSTACLE_CM) >= US_CONFIRM_VOTES) {
    autoBlindStreak = 0;
    startBackoff(now, "obstacle");
    return;
  }

  if (usDistanceCm <= OBSTACLE_CM) {
    // Single close reading: stop and wait for confirmation.
    stopBot("auto_close_unconfirmed");
    return;
  }

  autoBlindStreak = 0;

  // Stall: driving forward but the range is not shrinking, e.g. the
  // robot is pushing a low object the beam passes over.
  bool drivingForward = leftApplied > 0 && rightApplied > 0;

  if (!drivingForward ||
      fabsf(usDistanceCm - stallRefCm) >= STALL_MIN_CHANGE_CM) {
    stallRefCm = usDistanceCm;
    stallRefMs = now;
  } else if (now - stallRefMs >= STALL_WINDOW_MS) {
    startBackoff(now, "stalled");
    return;
  }

  int cruise = motorSpeed;
  int speed = usDistanceCm <= SLOWDOWN_CM
    ? max(80, cruise * AUTO_APPROACH_PERCENT / 100) : cruise;

  drive(speed, speed, "auto_cruise");
}

// =====================================================
// SERIAL DIAGNOSTICS
// =====================================================

void serialDiagnostics() {
  loopCount++;

  unsigned long now = millis();

  if (now - lastDiagMs < DIAG_INTERVAL_MS) {
    return;
  }

  loopHz = loopCount * 1000UL / (now - lastDiagMs);
  loopCount = 0;
  lastDiagMs = now;

  logEvent(EV_STATS, autoMode ? "auto" : "manual", loopHz / 10,
           WiFi.softAPgetStationNum(), (float)ESP.getFreeHeap());
  logEvent(EV_REQS, "per_s", driveReqs, telemetryReqs, (float)otherReqs);
  driveReqs = 0;
  telemetryReqs = 0;
  otherReqs = 0;

  char dist[12];
  if (usValid) snprintf(dist, sizeof(dist), "%6.1fcm", usDistanceCm);
  else snprintf(dist, sizeof(dist), "  ---   ");

  Serial.printf("[STAT] t=%7.1fs %-6s yaw=%8.2f rate=%7.2f bias=%7.3f "
                "imu=%s cal=%s rst=%lu | us=%s %-12s%s | L=%4d R=%4d | "
                "loop=%luHz clients=%d heap=%lu\n",
                now / 1000.0f,
                autoMode ? "AUTO" : "MANUAL",
                wrap180(yawUnwrappedDeg),
                gyroRateDps,
                gyroBiasDps,
                imuOk ? "OK" : "FAIL",
                gyroCalName(),
                imuResets,
                dist,
                usStatus,
                guardLatched ? " BLOCKED" : "",
                leftApplied,
                rightApplied,
                loopHz,
                WiFi.softAPgetStationNum(),
                (unsigned long)ESP.getFreeHeap());
}

// Track the home Wi-Fi link: log changes, start mDNS, retry.
void updateHomeWifi() {
  if (!homeWifiEnabled) return;

  bool up = WiFi.status() == WL_CONNECTED;

  if (up && !homeWifiConnected) {
    homeWifiConnected = true;
    IPAddress ip = WiFi.localIP();
    logEvent(EV_WIFI, "home_connected", ip[3], WiFi.RSSI(), 0);
    Serial.printf("[WIFI] Home network connected: Controller http://%s  "
                  "RSSI %d dBm\n", ip.toString().c_str(), WiFi.RSSI());
  } else if (!up && homeWifiConnected) {
    homeWifiConnected = false;
    homeWifiLastTryMs = millis();
    logEvent(EV_WIFI, "home_lost", 0, 0, 0);
    Serial.println("[WIFI] Home network lost - reconnecting. "
                   "Drive watchdog stops the motors meanwhile.");
  } else if (!up && millis() - homeWifiLastTryMs >= HOME_WIFI_RETRY_MS) {
    homeWifiLastTryMs = millis();
    WiFi.reconnect();
  }
}

void logPeriodic() {
  unsigned long now = millis();

  if (now - lastGyroLogMs >= GYRO_LOG_MS) {
    lastGyroLogMs = now;
    logEvent(EV_GYRO, !imuOk ? "imu_fail" : gyroStationary ? "still" : "moving",
             lround(gyroRateDps * 10), lround(gyroRawDps * 10),
             yawUnwrappedDeg);
  }

  if (now - lastWifiCheckMs >= 500) {
    lastWifiCheckMs = now;
    updateHomeWifi();
    int n = WiFi.softAPgetStationNum();

    if (n != lastStationCount) {
      logEvent(EV_WIFI, n > lastStationCount ? "join" : "leave", n, 0, 0);
      Serial.printf("[WIFI] %d device(s) connected\n", n);
      lastStationCount = n;
    }
  }
}

// =====================================================
// MAIN LOOP (non-blocking; HTTP serviced every pass)
// =====================================================

// Called at the top of every loop() pass: tell the dead-man we are
// alive, and if the previous pass froze, log where and for how long.
void loopCheckIn() {
  unsigned long now = millis();
  unsigned long stuck = now - loopAliveMs;
  loopAliveMs = now;

  if (stuck >= STALL_LOG_MS) {
    const char* where = (const char*)stallActivity;

    if (!stallRecorded) {
      logEvent(EV_STALL, where, stuck, deadmanTripped ? 1 : 0, 0);
    }

    Serial.printf("[SAFETY] loop() froze %lu ms in %s%s\n", stuck, where,
                  deadmanTripped ? " - dead-man cut the motors" : "");
  }

  if (deadmanTripped) {
    deadmanTrips++;
    deadmanTripped = false;
    // Make the software state match the hardware (motors are off).
    throttleValue = 0;
    steeringValue = 0;
    stopBot("deadman");
  }

  stallRecorded = false;
  stallActivity = "unknown";
}

void loop() {
  loopCheckIn();

  currentActivity = "http";
  server.handleClient();

  currentActivity = "imu";
  imuUpdate();

  currentActivity = "ultrasonic";
  ultrasonicUpdate();

  currentActivity = "control";
  accumulateCommands();

  updateForwardGuard();

  if (autoMode) {
    updateAutonomous();
  } else {
    updateManualWatchdog();
  }

  currentActivity = "wifi_log";
  logPeriodic();
  serialDiagnostics();
}
