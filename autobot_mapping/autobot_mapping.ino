
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "freertos/queue.h"
#include "link_types.h"
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
//   - full 3D attitude (gyro XYZ + accel) and a 100 Hz USB serial
//     stream for viewer/index.html
//   - v2.0: network moved to esp_http_server (its own task) with a
//     WebSocket for the phone, a lossless /api/samples feed for the
//     mapper, persistent boot history (/api/diag) and link logging
// =====================================================

const char* FW_VERSION = "autobot_mapping-2.0";

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
// Only a backstop: setAutoReconnect() normally reconnects first, and a
// manual reconnect() would interrupt an attempt already in progress.
const unsigned long HOME_WIFI_RETRY_MS = 20000;
bool homeWifiEnabled = false;
bool homeWifiConnected = false;
unsigned long homeWifiLastTryMs = 0;

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
// The phone resends the stick 20x/s; 10 missed frames = stop.
const unsigned long DRIVE_TIMEOUT_MS = 500;

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

// AUTO mode stops if no WebSocket frame (the phone pings 4x/s)
// arrives for this long.
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
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;  // accel, temp, gyro: 14 B
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_REG_WHO_AM_I = 0x75;
// +/-1000 dps: spins reached the old +/-250 dps limit and clipped.
const float GYRO_LSB_PER_DPS = 32.8f;
// +/-4 g: bumps while driving clipped at +/-2 g.
const float ACCEL_LSB_PER_G = 8192.0f;

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

// 3D attitude (Mahony filter). Gyro gives the motion; gravity from
// the accelerometer slowly pulls roll / pitch back to the truth. The
// accel is ignored while the robot accelerates hard (|a| far from
// 1 g), when it does not point along gravity. Yaw has no absolute
// reference (no magnetometer), so it drifts like yaw_deg does.
const float ATT_KP = 2.0f;                // accel correction gain, 1/s
const float ATT_ACC_MIN_G = 0.85f;
const float ATT_ACC_MAX_G = 1.15f;

// USB serial stream for viewer/index.html. Off until the viewer
// sends "@stream 1", so the Serial Monitor stays readable. The viewer
// repeats that every second; the stream stops 3 s after it goes away.
const uint32_t SERIAL_BAUD = 921600;
const int64_t STREAM_INTERVAL_US = 10000; // 100 Hz
const unsigned long STREAM_KEEPALIVE_MS = 3000;

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

// Gyro X / Y (bias removed) and accel, sensor axes.
float gyroXDps = 0;
float gyroYDps = 0;
float gyroXBiasDps = 0;
float gyroYBiasDps = 0;
float accelG[3] = {0, 0, 1};
// Sensor -> world rotation (w, x, y, z), world Z up.
float attQ[4] = {1, 0, 0, 0};
bool attInit = false;

// Viewer serial stream
bool streamOn = false;
unsigned long streamKeepaliveMs = 0;
int64_t streamLastUs = 0;
unsigned long streamFrames = 0;
unsigned long streamDropped = 0;
char serialCmd[40];
int serialCmdLen = 0;

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
double calSumX = 0;
double calSumY = 0;

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
  EV_LINK, EV_HEALTH,
  EV_COUNT
};

const char* const EV_NAMES[EV_COUNT] = {
  "BOOT", "RANGE", "GYRO", "MOTOR", "JOY", "LINKGAP",
  "WATCHDOG", "ESTOP", "MODE", "SPEED", "GUARD", "AUTO",
  "IMU", "WIFI", "STATS", "REQS", "PREV_BOOT", "STALL",
  "LINK", "HEALTH"
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
const unsigned long LINK_GAP_LOG_MS = 200;

LogEvent logBuf[LOG_SIZE];
uint32_t logSeq = 0;
// logEvent() is called from loop(), the HTTP server task, the Wi-Fi
// event task and the dead-man timer. Never call logEvent() (or take
// shareMux) while holding this lock.
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

struct RtcLog {
  uint32_t magic;
  uint32_t seq;
  LogEvent ev[RTC_LOG_SIZE];
};

RTC_NOINIT_ATTR RtcLog rtcLog;

unsigned long lastGyroLogMs = 0;
unsigned long lastWifiCheckMs = 0;
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
  portENTER_CRITICAL(&logMux);
  logBuf[logSeq % LOG_SIZE] = e;
  logSeq++;
  portEXIT_CRITICAL(&logMux);
}

void logEvent(uint8_t type, const char* tag, long a, long b, float f) {
  LogEvent e;
  e.ms = millis();
  e.tag = tag;
  e.f = f;
  e.a = clamp16(a);
  e.b = clamp16(b);
  e.type = type;

  portENTER_CRITICAL(&logMux);
  logBuf[logSeq % LOG_SIZE] = e;
  logSeq++;
  rtcLog.ev[rtcLog.seq % RTC_LOG_SIZE] = e;
  rtcLog.seq++;
  portEXIT_CRITICAL(&logMux);
}

uint32_t logSeqNow() {
  portENTER_CRITICAL(&logMux);
  uint32_t n = logSeq;
  portEXIT_CRITICAL(&logMux);
  return n;
}

LogEvent logAt(uint32_t i) {
  portENTER_CRITICAL(&logMux);
  LogEvent e = logBuf[i % LOG_SIZE];
  portEXIT_CRITICAL(&logMux);
  return e;
}

// =====================================================
// BOOT HISTORY (flash, survives power-off)
// Written ONCE per boot: the reset reason, how long the previous boot
// ran, and its last events from RTC memory. Read with GET /api/diag
// or on the serial monitor at boot, so a reboot in the middle of a
// drive always leaves a reason behind.
// =====================================================

const int BOOT_HISTORY = 4;
const int BOOT_EVENTS = 24;
const int BOOT_EVENT_LEN = 40;

struct BootRecord {
  uint32_t bootNo;
  uint32_t prevUptimeMs;   // time of the last event before the reset
  char reason[14];
  char fw[24];
  uint8_t nEvents;
  char events[BOOT_EVENTS][BOOT_EVENT_LEN];
};

BootRecord bootHistory[BOOT_HISTORY];   // [0] = this boot
int bootHistoryCount = 0;

void formatEvent(const LogEvent& e, char* out, size_t len) {
  uint8_t t = e.type & 0x7F;
  snprintf(out, len, "%lu %s %s %d %d %.1f",
           (unsigned long)e.ms, t < EV_COUNT ? EV_NAMES[t] : "?",
           e.tag ? e.tag : "", e.a, e.b, e.f);
}

// rtcValid: the RTC ring holds events from the boot that just ended.
void recordBootHistory(const char* reason, bool rtcValid) {
  Preferences prefs;

  if (!prefs.begin("autobot", false)) {
    Serial.println("[BOOT] WARNING: could not open flash for boot history.");
    return;
  }

  memset(bootHistory, 0, sizeof(bootHistory));

  if (prefs.getBytesLength("hist") == sizeof(bootHistory)) {
    prefs.getBytes("hist", bootHistory, sizeof(bootHistory));
  }

  memmove(&bootHistory[1], &bootHistory[0],
          sizeof(BootRecord) * (BOOT_HISTORY - 1));

  BootRecord& r = bootHistory[0];
  memset(&r, 0, sizeof(r));
  r.bootNo = prefs.getUInt("boots", 0) + 1;
  strlcpy(r.reason, reason, sizeof(r.reason));
  strlcpy(r.fw, FW_VERSION, sizeof(r.fw));

  if (rtcValid) {
    uint32_t n = min(rtcLog.seq, (uint32_t)BOOT_EVENTS);
    uint32_t first = rtcLog.seq - n;

    for (uint32_t i = first; i < rtcLog.seq; i++) {
      formatEvent(rtcLog.ev[i % RTC_LOG_SIZE], r.events[r.nEvents++],
                  BOOT_EVENT_LEN);
    }

    r.prevUptimeMs = rtcLog.ev[(rtcLog.seq - 1) % RTC_LOG_SIZE].ms;
  }

  prefs.putUInt("boots", r.bootNo);
  prefs.putBytes("hist", bootHistory, sizeof(bootHistory));
  prefs.end();

  bootHistoryCount = 0;
  for (int i = 0; i < BOOT_HISTORY; i++) {
    if (bootHistory[i].bootNo != 0) bootHistoryCount++;
  }

  Serial.printf("[BOOT] Boot #%lu, reset reason: %s\n",
                (unsigned long)r.bootNo, r.reason);

  if (bootHistoryCount > 1) {
    const BootRecord& p = bootHistory[1];
    Serial.printf("[BOOT] Previous boot #%lu (%s) ended after %.1f s.\n",
                  (unsigned long)p.bootNo, p.fw, r.prevUptimeMs / 1000.0f);
  }

  if (r.nEvents > 0) {
    Serial.println("[BOOT] Last events before the reset (ms TYPE tag a b f):");
    for (int i = max(0, (int)r.nEvents - 8); i < r.nEvents; i++) {
      Serial.printf("[BOOT]   %s\n", r.events[i]);
    }
  }
}

// Called once in setup(): copy the pre-reset events into the log.
void logInit(const char* reason) {
  uint32_t magic = firmwareMagic();
  bool rtcValid = rtcLog.magic == magic && rtcLog.seq > 0 &&
                  strcmp(reason, "power_on") != 0;

  recordBootHistory(reason, rtcValid);

  if (rtcValid) {
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
// The POINTER is volatile (written on core 1, read by the dead-man
// timer on core 0), not the characters.
const char* volatile currentActivity = "boot";
const char* volatile stallActivity = "unknown";
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

  // Wake, PLL clock from gyro X, 200 Hz, DLPF ~44 Hz, +/-1000 dps,
  // +/-4 g.
  bool ok = mpuWrite(MPU_REG_PWR_MGMT_1, 0x01) &&
            mpuWrite(MPU_REG_SMPLRT_DIV, 0x04) &&
            mpuWrite(MPU_REG_CONFIG, 0x03) &&
            mpuWrite(MPU_REG_GYRO_CONFIG, 0x10) &&
            mpuWrite(MPU_REG_ACCEL_CONFIG, 0x08);

  if (!ok) {
    Serial.println("[IMU] ERROR: MPU6050 configuration write failed.");
    imuOk = false;
    return false;
  }

  imuOk = true;
  imuConsecutiveErrors = 0;
  return true;
}

// One burst read: accel XYZ, temperature, gyro XYZ (raw counts).
bool mpuReadAll(int16_t accel[3], int16_t gyro[3]) {
  uint8_t buf[14];

  if (!mpuRead(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    accel[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);
    gyro[i] = (int16_t)((buf[8 + 2 * i] << 8) | buf[9 + 2 * i]);
  }

  return true;
}

void quatNormalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-6f) {
    q[0] = 1; q[1] = q[2] = q[3] = 0;
    return;
  }
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// Level the attitude from gravity alone, heading 0. Used at start-up
// and after a calibration, which also zeroes yaw_deg.
void attitudeFromAccel() {
  float ax = accelG[0], ay = accelG[1], az = accelG[2];
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll / 2), sr = sinf(roll / 2);
  float cp = cosf(pitch / 2), sp = sinf(pitch / 2);

  attQ[0] = cr * cp;
  attQ[1] = sr * cp;
  attQ[2] = cr * sp;
  attQ[3] = -sr * sp;
  quatNormalize(attQ);
  attInit = true;
}

// Mahony update. g = body rates in rad/s, accelG in g.
void attitudeUpdate(float gx, float gy, float gz, float dt) {
  if (!attInit) {
    attitudeFromAccel();
    return;
  }

  float* q = attQ;
  float ax = accelG[0], ay = accelG[1], az = accelG[2];
  float an = sqrtf(ax * ax + ay * ay + az * az);

  if (an > ATT_ACC_MIN_G && an < ATT_ACC_MAX_G) {
    ax /= an; ay /= an; az /= an;

    // "Up" as the current estimate sees it, in sensor axes.
    float vx = 2 * (q[1] * q[3] - q[0] * q[2]);
    float vy = 2 * (q[0] * q[1] + q[2] * q[3]);
    float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    // Rotate the estimate towards the measured up direction.
    gx += ATT_KP * (ay * vz - az * vy);
    gy += ATT_KP * (az * vx - ax * vz);
    gz += ATT_KP * (ax * vy - ay * vx);
  }

  float h = 0.5f * dt;
  float w = q[0], x = q[1], y = q[2], z = q[3];
  q[0] += h * (-x * gx - y * gy - z * gz);
  q[1] += h * ( w * gx + y * gz - z * gy);
  q[2] += h * ( w * gy - x * gz + z * gx);
  q[3] += h * ( w * gz + x * gy - y * gx);
  quatNormalize(q);
}

float attRollDeg() {
  const float* q = attQ;
  return atan2f(2 * (q[0] * q[1] + q[2] * q[3]),
                1 - 2 * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
}

float attPitchDeg() {
  const float* q = attQ;
  float s = 2 * (q[0] * q[2] - q[3] * q[1]);
  s = s > 1 ? 1 : (s < -1 ? -1 : s);
  return asinf(s) * RAD_TO_DEG;
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
  calSumX = 0;
  calSumY = 0;
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
  calSumX += gyroXDps + gyroXBiasDps;
  calSumY += gyroYDps + gyroYBiasDps;

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
  gyroXBiasDps = calSumX / calCount;
  gyroYBiasDps = calSumY / calCount;
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

  int16_t accelRaw[3];
  int16_t gyroRaw[3];

  if (!mpuReadAll(accelRaw, gyroRaw)) {
    imuErrors++;

    if (++imuConsecutiveErrors >= IMU_MAX_CONSECUTIVE_ERRORS) {
      imuFault("MPU6050 stopped responding");
    }
    return;
  }

  imuConsecutiveErrors = 0;
  int16_t raw = gyroRaw[2];

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
  gyroXDps = gyroRaw[0] / GYRO_LSB_PER_DPS - gyroXBiasDps;
  gyroYDps = gyroRaw[1] / GYRO_LSB_PER_DPS - gyroYBiasDps;

  for (int i = 0; i < 3; i++) {
    accelG[i] = accelRaw[i] / ACCEL_LSB_PER_G;
  }

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

  // Z uses the same bias-corrected, zero-when-still rate as yaw_deg,
  // so the 3D heading drifts no more than yaw_deg does.
  attitudeUpdate(gyroXDps * DEG_TO_RAD, gyroYDps * DEG_TO_RAD,
                 rate * DEG_TO_RAD, dt);
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

.connection.weak { color: #f2c46d; }
.connection.weak::before { background: #f2c46d; }
.connection.lost { color: #ff7b82; }
.connection.lost::before { background: #ff4d57; }

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
  <div class="connection lost" id="link">CONNECTING</div>
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
let speedPending = null;

const activePointers = {
  throttle: null,
  steer: null
};

// ---------------------------------------------------------------
// Link: ONE WebSocket for everything. The newest stick position is
// sent 20x/s while a stick is held (never per touch event), nothing
// is sent while the previous frame is still queued, and the robot
// pushes its status 10x/s. No status for 1 s = link lost: the sticks
// are released, and the socket reconnects by itself.
// ---------------------------------------------------------------

const SEND_MS = 50;
const PING_MS = 250;
const WEAK_STATUS_MS = 300;
const LOST_STATUS_MS = 1000;
// A dropped Wi-Fi link often leaves the socket half-open (no close event
// for minutes): after this long without status, drop it and reconnect.
const DEAD_SOCKET_MS = 2500;
const WEAK_RTT_MS = 150;

let ws = null;
let wsOpen = false;
let wsOpenedAt = 0;
let seq = 0;
let releaseRepeats = 0;
let lastStatusAt = 0;
let rttMs = -1;
let pingId = 0;
const pingSentAt = new Map();
let reconnects = 0;
let retryMs = 250;
let linkState = 'lost';
let robotFlags = '';
let robotText = '';
let robotDist = '-';
let robotBlocked = false;

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  wsOpenedAt = Date.now();

  ws.onopen = () => {
    wsOpen = true;
    wsOpenedAt = Date.now();
    retryMs = 250;
    if (reconnects > 0) ws.send('log,reconnected,' + reconnects);
  };

  ws.onmessage = e => onMessage(e.data);

  ws.onclose = () => {
    wsOpen = false;
    reconnects++;
    setTimeout(connect, retryMs);
    retryMs = Math.min(2000, retryMs * 2);
  };

  ws.onerror = () => {};   // onclose follows and reconnects
}

// Send unless the previous frame is still waiting in the socket.
function linkSend(text) {
  if (!wsOpen || ws.bufferedAmount > 0) return false;
  ws.send(text);
  return true;
}

// Safety messages go out even behind queued frames.
function linkSendUrgent(text) {
  if (wsOpen) ws.send(text);
}

function onMessage(t) {
  if (t.startsWith('pong,')) {
    const id = Number(t.slice(5));
    const sent = pingSentAt.get(id);
    if (sent !== undefined) {
      rttMs = Date.now() - sent;
      pingSentAt.delete(id);
      if (rttMs > 250) linkSend('log,rtt,' + rttMs);
    }
    return;
  }

  if (!t.startsWith('s,')) return;

  // s,<drive seq>,<dist|->,<blocked>,<M|A>,<flags>,<text>
  const p = t.split(',');
  lastStatusAt = Date.now();
  robotDist = p[2];
  robotBlocked = p[3] === '1';
  robotFlags = p[5] || '';
  robotText = p.slice(6).join(',');

  // The robot is the truth (it leaves AUTO by itself on E-STOP or a
  // lost link); give a just-pressed mode button time to arrive.
  if (p[4] !== mode && Date.now() - modeChangedAt > 1500) {
    applyModeUi(p[4]);
  }

  showStatus();
}

function showStatus() {
  const el = document.getElementById('status');

  if (linkState === 'lost') {
    el.textContent = 'LINK LOST - RECONNECTING';
  } else if (robotFlags.includes('C')) {
    el.textContent = 'CALIBRATING GYRO - KEEP STILL';
  } else if (mode === 'A') {
    el.textContent = (robotText || 'AUTONOMOUS MODE ACTIVE') +
      (robotDist !== '-' ? ' | ' + robotDist + ' CM' : '');
  } else if (robotBlocked && throttle > 0) {
    el.textContent = 'OBSTACLE ' +
      (robotDist !== '-' ? robotDist + ' CM ' : '') + '- TURN OR REVERSE';
  } else {
    el.textContent = robotFlags.includes('I') ? 'HOLD TO DRIVE (IMU FAULT)' : 'HOLD TO DRIVE';
  }
}

function updateLink() {
  const age = Date.now() - lastStatusAt;

  // Silent socket (half-open after a Wi-Fi drop, or a connect attempt
  // that hangs): close it; onclose reconnects.
  if (ws && ws.readyState <= 1 &&
      Date.now() - Math.max(lastStatusAt, wsOpenedAt) > DEAD_SOCKET_MS) {
    wsOpen = false;
    wsOpenedAt = Date.now();
    ws.close();
  }

  const next = !wsOpen || age > LOST_STATUS_MS ? 'lost' :
               age > WEAK_STATUS_MS || rttMs > WEAK_RTT_MS ? 'weak' : 'live';

  if (next === 'lost' && linkState !== 'lost') {
    // Release everything; the finger must touch again after reconnect.
    activePointers.throttle = null;
    activePointers.steer = null;
    resetStick('throttle');
    resetStick('steer');
    releaseRepeats = 0;
  }

  if (next !== 'lost' && linkState === 'lost' && lastStatusAt > 0) {
    linkSend('log,restored,' + reconnects);
  }

  linkState = next;

  const el = document.getElementById('link');
  el.className = 'connection ' + (next === 'live' ? '' : next);
  el.textContent =
    next === 'lost' ? 'RECONNECTING' :
    (next === 'weak' ? 'WEAK ' : 'ONLINE ') + (rttMs >= 0 ? rttMs + ' MS' : '');

  showStatus();
}

function updateReadings() {
  document.getElementById('directionValue').textContent =
    throttle > 3 ? 'FORWARD' :
    throttle < -3 ? 'REVERSE' : 'IDLE';

  document.getElementById('steerValue').textContent =
    steering < -3 ? 'LEFT' :
    steering > 3 ? 'RIGHT' : 'CENTER';
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

// Stick released: send the stop a few times so it cannot be missed.
function stickReleased() {
  if (throttle === 0 && steering === 0) releaseRepeats = 3;
}

function emergencyStop(why) {
  throttle = 0;
  steering = 0;
  releaseRepeats = 0;

  resetStick('throttle');
  resetStick('steer');

  // The robot also leaves AUTO on stop.
  if (mode === 'A') applyModeUi('M');

  linkSendUrgent('stop,' + (why || 'other'));
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

  showStatus();
}

function setMode(nextMode) {
  applyModeUi(nextMode);
  modeChangedAt = Date.now();
  linkSendUrgent('mode,' + mode);
}

document.getElementById('speedSlider').addEventListener(
  'input', function() {
    const speed = Number(this.value);

    document.getElementById('speedValue').textContent =
      speed + ' / 255';

    speedPending = speed;   // sent by the 20 Hz loop, newest value only
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

  // Only updates the position; the send loop transmits it.
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
  }

  stick.addEventListener('pointerdown', e => {
    if (mode !== 'M' || linkState === 'lost') return;

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
    stickReleased();
  }

  stick.addEventListener('pointerup', release);
  stick.addEventListener('pointercancel', release);
  stick.addEventListener('lostpointercapture', release);
}

setupStick('throttle');
setupStick('steer');

// 20 Hz send loop: newest stick position (also the drive heartbeat).
setInterval(() => {
  if (speedPending !== null && linkSend('speed,' + speedPending)) {
    speedPending = null;
  }

  if (mode !== 'M') return;

  const moving = throttle !== 0 || steering !== 0;

  if (moving || releaseRepeats > 0) {
    if (linkSend('d,' + (seq + 1) + ',' + throttle + ',' + steering)) {
      seq++;
      if (!moving) releaseRepeats--;
    }
  }
}, SEND_MS);

// Pings: round-trip time, and the AUTO-mode heartbeat.
setInterval(() => {
  const id = ++pingId;
  if (linkSend('ping,' + id)) pingSentAt.set(id, Date.now());
  for (const k of pingSentAt.keys()) {
    if (id - k > 20) pingSentAt.delete(k);
  }
}, PING_MS);

setInterval(updateLink, 200);

// Screen lock / app switch: stop. (Not on 'blur': notification
// shades and popups blur the page and caused false stops.)
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    emergencyStop('hidden');
    linkSend('log,hidden,0');
  }
});

window.addEventListener('pagehide', () => emergencyStop('hidden'));

connect();
</script>
</body>
</html>
)HTML";

// =====================================================
// LINK: esp_http_server (own task, core 0) + WebSocket
// =====================================================
//
// loop() never touches the network and the server task never
// touches the hardware:
//   client frames  -> cmdQueue  -> loop() applies them
//   loop() state   -> snapshot  -> server task replies / pushes
// A slow or dead client can only delay its own replies, never the
// motors or another client. (The old WebServer ran inside loop(),
// served one connection at a time with a listen backlog of 1, and
// dropped connection attempts while busy - the "disconnects".)
//
// Locks: logMux (event log) and shareMux (snapshot, sample ring).
// Both are short spinlocks, never nested, and nothing that can
// block is called while holding one.
//
// /ws text frames, phone -> robot:
//   d,<seq>,<throttle>,<steering>     drive, 20x/s while held
//   stop,<button|hidden|link|other>   emergency stop, also ends AUTO
//   mode,<M|A>   speed,<80..255>   ping,<id>   log,<what>,<value>
// robot -> phone:
//   pong,<id>
//   s,<drive seq>,<dist cm|->,<blocked 0|1>,<M|A>,<flags>,<text>
//     flags: C = gyro calibrating (drive refused), I = IMU fault

// Cmd / CmdType: see link_types.h.

const int CMD_QUEUE_LEN = 32;
const int MAX_WS_CLIENTS = 6;
const size_t WS_MAX_FRAME = 128;
// Phones ping 4x/s. A client silent this long is gone (often a half-open
// socket after a Wi-Fi drop): close it before its send buffer fills and
// sends to it start blocking the server task.
const unsigned long WS_SILENT_CLOSE_MS = 3000;
// The server task runs pushStatusWork 10x/s while clients are open; if it
// stops doing so for this long, it is stuck - log it.
const unsigned long HTTPD_STALL_LOG_MS = 3000;
const unsigned long STATUS_PUSH_MS = 100;
const unsigned long TELEMETRY_RENDER_MS = 200;
const unsigned long SAMPLE_PERIOD_MS = 40;   // 25 Hz lossless feed
const uint32_t SAMPLE_RING = 512;            // ~20 s of history
const uint32_t LOG_REPLY_MAX = 400;          // events per /api/log reply
const uint32_t SAMPLE_REPLY_MAX = 512;

httpd_handle_t httpServer = nullptr;
QueueHandle_t cmdQueue = nullptr;
portMUX_TYPE shareMux = portMUX_INITIALIZER_UNLOCKED;

// Server-task only.
int wsFds[MAX_WS_CLIENTS];
uint8_t wsIps[MAX_WS_CLIENTS];
unsigned long wsLastRx[MAX_WS_CLIENTS];

// Written by the server task, read by loop().
volatile int wsClientCount = 0;
volatile unsigned long wsLastFrameMs = 0;
volatile uint32_t wsFramesIn = 0;
volatile uint32_t wsFramesOut = 0;
volatile uint32_t httpRequests = 0;
volatile uint32_t cmdDropped = 0;
volatile unsigned long httpdWorkMs = 0;   // last pushStatusWork run

// Written by loop() under shareMux, read by the server task.
char statusMsg[128] = "";
char telemetryJson[1536];
int telemetryLen = 0;

struct Sample {
  uint32_t ms;
  float yawUnwrapped;
  float roll;
  float pitch;
  float gyroRate;
  float gyroRaw;
  float gyroBias;
  float distCm;            // -1 = no valid echo
  uint32_t usSeq;
  uint32_t usAgeMs;
  const char* usStatus;    // string literal
  double leftInt;
  double rightInt;
  int16_t left;
  int16_t right;
  uint8_t calState;
  bool imuOk;
  bool autoMode;
};

// ~37 KB: allocated from the heap in setup(); static data does not fit
// the ESP32's fixed DRAM segment.
Sample* sampleBuf = nullptr;
uint32_t sampleSeq = 0;

// loop() only.
int driverFd = -1;
uint32_t driverSeq = 0;
unsigned long staleDriveCmds = 0;
unsigned long lastStatusPushMs = 0;
unsigned long lastTelemetryRenderMs = 0;
unsigned long lastSampleMs = 0;
bool startupCalPending = false;
int startupCalAttempt = 0;

// Health (loop() writes, /api/diag reads; word-sized).
volatile uint32_t loopMaxUsSecond = 0;
volatile uint32_t loopMaxUsBoot = 0;
volatile uint32_t loopStackFree = 0;
volatile uint32_t httpdStackFree = 0;
volatile uint8_t wifiLastReason = 0;
volatile uint32_t wifiDisconnects = 0;

const char* calNameOf(uint8_t state) {
  switch (state) {
    case CAL_RUNNING: return "running";
    case CAL_OK: return "ok";
    case CAL_FAILED_MOTION: return "failed_motion";
    case CAL_FAILED_IMU: return "failed_imu";
    case CAL_UNVERIFIED: return "unverified";
    default: return "none";
  }
}

bool calibrating() {
  return startupCalPending || gyroCalState == CAL_RUNNING;
}

// ---------- server task ----------

void postCmd(const Cmd& c) {
  if (xQueueSend(cmdQueue, &c, 0) != pdTRUE) {
    cmdDropped = cmdDropped + 1;
  }
}

uint8_t clientIpOctet(int fd) {
  struct sockaddr_storage addr;
  socklen_t len = sizeof(addr);

  if (getpeername(fd, (struct sockaddr*)&addr, &len) != 0) return 0;

  if (addr.ss_family == AF_INET) {
    return ((struct sockaddr_in*)&addr)->sin_addr.s_addr >> 24;
  }

  // IPv4-mapped IPv6 address: the last byte is the last octet.
  return ((struct sockaddr_in6*)&addr)->sin6_addr.s6_addr[15];
}

void recountWsClients() {
  int n = 0;
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsFds[i] >= 0) n++;
  }
  wsClientCount = n;
}

uint8_t wsIpOf(int fd) {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsFds[i] == fd) return wsIps[i];
  }
  return 0;
}

void noStore(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

uint32_t queryUint(httpd_req_t* req, const char* key, uint32_t def) {
  char q[64];
  char v[16];

  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return def;
  if (httpd_query_key_value(q, key, v, sizeof(v)) != ESP_OK) return def;

  return strtoul(v, nullptr, 10);
}

// Called by the server whenever a socket closes (client gone, error,
// keep-alive probe failed, LRU purge). We must close it ourselves.
void onSocketClose(httpd_handle_t hd, int fd) {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsFds[i] == fd) {
      Cmd c = {};
      c.type = CMD_WS_CLOSE;
      c.fd = fd;
      c.ip = wsIps[i];
      wsFds[i] = -1;
      recountWsClients();
      postCmd(c);
    }
  }

  close(fd);
}

void wsReply(httpd_req_t* req, const char* text) {
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t*)text;
  f.len = strlen(text);

  if (httpd_ws_send_frame(req, &f) == ESP_OK) wsFramesOut = wsFramesOut + 1;
}

void handleWsText(httpd_req_t* req, int fd, char* msg) {
  Cmd c = {};
  c.fd = fd;

  if (msg[0] == 'd' && msg[1] == ',') {
    unsigned long seq;
    int t;
    int st;

    if (sscanf(msg + 2, "%lu,%d,%d", &seq, &t, &st) != 3) return;

    c.type = CMD_DRIVE;
    c.seq = seq;
    c.a = constrain(t, -100, 100);
    c.b = constrain(st, -100, 100);
    postCmd(c);
  } else if (strncmp(msg, "ping,", 5) == 0) {
    char reply[32];
    snprintf(reply, sizeof(reply), "pong,%.20s", msg + 5);
    wsReply(req, reply);
  } else if (strncmp(msg, "stop,", 5) == 0) {
    const char* why = msg + 5;
    c.type = CMD_STOP;
    c.tag = strcmp(why, "button") == 0 ? "estop_button" :
            strcmp(why, "hidden") == 0 ? "estop_page_hidden" :
            strcmp(why, "link") == 0 ? "estop_link_lost" : "estop_other";
    postCmd(c);
  } else if (strncmp(msg, "mode,", 5) == 0) {
    c.type = CMD_MODE;
    c.a = msg[5] == 'A';
    postCmd(c);
  } else if (strncmp(msg, "speed,", 6) == 0) {
    c.type = CMD_SPEED;
    c.a = constrain(atoi(msg + 6), 80, 255);
    postCmd(c);
  } else if (strncmp(msg, "log,", 4) == 0) {
    // Phone-side link events, so they end up in the robot log.
    char* what = msg + 4;
    char* comma = strchr(what, ',');
    long v = comma ? atol(comma + 1) : 0;
    if (comma) *comma = 0;

    const char* tag =
      strcmp(what, "reconnected") == 0 ? "phone_reconnected" :
      strcmp(what, "restored") == 0 ? "phone_link_restored" :
      strcmp(what, "rtt") == 0 ? "phone_rtt_spike" :
      strcmp(what, "hidden") == 0 ? "phone_page_hidden" : "phone_other";

    logEvent(EV_LINK, tag, v, wsIpOf(fd), 0);
  }
}

esp_err_t wsHandler(httpd_req_t* req) {
  int fd = httpd_req_to_sockfd(req);

  if (req->method == HTTP_GET) {
    // Handshake finished: a new phone (or test client).
    httpRequests = httpRequests + 1;
    uint8_t ip = clientIpOctet(fd);

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
      if (wsFds[i] < 0) {
        wsFds[i] = fd;
        wsIps[i] = ip;
        wsLastRx[i] = millis();
        break;
      }
    }

    recountWsClients();

    Cmd c = {};
    c.type = CMD_WS_OPEN;
    c.fd = fd;
    c.ip = ip;
    postCmd(c);
    return ESP_OK;
  }

  static uint8_t buf[WS_MAX_FRAME];
  httpd_ws_frame_t frame = {};

  esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
  if (err != ESP_OK) return err;

  // Nothing legitimate is this big; failing closes the socket.
  if (frame.len >= WS_MAX_FRAME) return ESP_FAIL;

  frame.payload = buf;

  if (frame.len > 0) {
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) return err;
  }

  buf[frame.len] = 0;

  if (frame.type != HTTPD_WS_TYPE_TEXT) return ESP_OK;

  wsFramesIn = wsFramesIn + 1;
  wsLastFrameMs = millis();

  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsFds[i] == fd) wsLastRx[i] = wsLastFrameMs;
  }

  handleWsText(req, fd, (char*)buf);
  return ESP_OK;
}

// Runs in the server task (queued by loop()): status to every phone.
void pushStatusWork(void*) {
  char msg[sizeof(statusMsg)];

  portENTER_CRITICAL(&shareMux);
  memcpy(msg, statusMsg, sizeof(msg));
  portEXIT_CRITICAL(&shareMux);

  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t*)msg;
  f.len = strlen(msg);

  unsigned long now = millis();
  httpdWorkMs = now;

  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    int fd = wsFds[i];
    if (fd < 0) continue;
    if (httpd_ws_get_fd_info(httpServer, fd) != HTTPD_WS_CLIENT_WEBSOCKET) continue;

    if (now - wsLastRx[i] > WS_SILENT_CLOSE_MS) {
      // Closing fires onSocketClose -> CMD_WS_CLOSE -> motors stop if
      // this client was driving.
      logEvent(EV_LINK, "ws_silent_closed", wsIps[i], fd, now - wsLastRx[i]);
      wsLastRx[i] = now;   // trigger once
      httpd_sess_trigger_close(httpServer, fd);
      continue;
    }

    if (httpd_ws_send_frame_async(httpServer, fd, &f) == ESP_OK) {
      wsFramesOut = wsFramesOut + 1;
    }
  }
}

// The page is sent in 1 KB chunks from the server task.
esp_err_t pageHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  size_t len = sizeof(webpage) - 1;

  for (size_t off = 0; off < len; off += 1024) {
    size_t n = min((size_t)1024, len - off);
    if (httpd_resp_send_chunk(req, webpage + off, n) != ESP_OK) {
      return ESP_FAIL;
    }
  }

  return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t faviconHandler(httpd_req_t* req) {
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, nullptr, 0);
}

// Read-only snapshot for the Mac. loop() renders it every 200 ms.
esp_err_t telemetryHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  static char buf[sizeof(telemetryJson)];
  int n;

  portENTER_CRITICAL(&shareMux);
  n = telemetryLen;
  memcpy(buf, telemetryJson, n);
  portEXIT_CRITICAL(&shareMux);

  httpd_resp_set_type(req, "application/json");
  noStore(req);

  if (n == 0) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_sendstr(req, "{\"error\":\"starting\"}");
  }

  return httpd_resp_send(req, buf, n);
}

// GET /api/log?since=N -> CSV lines
//   #boot=<id>,next=<seq>,oldest=<seq>,more=<0|1>,now=<ms>,fw=<ver>
//   seq,ms,TYPE,tag,a,b,f
// Ask again with since=<next>. If since < oldest, events were lost.
esp_err_t logHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  static char chunk[2048];

  uint32_t since = queryUint(req, "since", 0);
  uint32_t end = logSeqNow();
  uint32_t oldest = end > LOG_SIZE ? end - LOG_SIZE : 0;
  uint32_t start = since < oldest ? oldest : min(since, end);
  uint32_t stop = min(end, start + LOG_REPLY_MAX);

  httpd_resp_set_type(req, "text/plain");
  noStore(req);

  size_t len = snprintf(chunk, sizeof(chunk),
    "#boot=%lu,next=%lu,oldest=%lu,more=%d,now=%lu,fw=%s\n",
    (unsigned long)bootId, (unsigned long)stop, (unsigned long)oldest,
    stop < end ? 1 : 0, millis(), FW_VERSION);

  for (uint32_t i = start; i < stop; i++) {
    LogEvent e = logAt(i);
    uint8_t t = e.type & 0x7F;
    char line[112];

    int n = snprintf(line, sizeof(line), "%lu,%lu,%s%s,%s,%d,%d,%.2f\n",
                     (unsigned long)i, (unsigned long)e.ms,
                     (e.type & EV_FROM_PREV_BOOT) ? "PREV:" : "",
                     t < EV_COUNT ? EV_NAMES[t] : "?",
                     e.tag ? e.tag : "", e.a, e.b, e.f);

    if (n <= 0 || n >= (int)sizeof(line)) continue;

    if (len + n >= sizeof(chunk)) {
      if (httpd_resp_send_chunk(req, chunk, len) != ESP_OK) return ESP_FAIL;
      len = 0;
    }

    memcpy(chunk + len, line, n);
    len += n;
  }

  if (len > 0 && httpd_resp_send_chunk(req, chunk, len) != ESP_OK) {
    return ESP_FAIL;
  }

  return httpd_resp_send_chunk(req, nullptr, 0);
}

// GET /api/samples?since=N: every 25 Hz sample after N, no gaps.
//   #boot=<id>,next=<seq>,oldest=<seq>,lost=<n>,now=<ms>,fw=<ver>
//   #cols=<names>
//   one CSV row per sample (column names = /api/telemetry keys)
esp_err_t samplesHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  static char chunk[2048];

  uint32_t since = queryUint(req, "since", 0);

  if (!sampleBuf) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_sendstr(req, "#error=no memory for samples\n");
  }

  portENTER_CRITICAL(&shareMux);
  uint32_t end = sampleSeq;
  portEXIT_CRITICAL(&shareMux);

  uint32_t oldest = end > SAMPLE_RING ? end - SAMPLE_RING : 0;
  uint32_t start = since < oldest ? oldest : min(since, end);
  uint32_t stop = min(end, start + SAMPLE_REPLY_MAX);
  uint32_t lost = since > 0 && since < oldest ? oldest - since : 0;

  httpd_resp_set_type(req, "text/plain");
  noStore(req);

  size_t len = snprintf(chunk, sizeof(chunk),
    "#boot=%lu,next=%lu,oldest=%lu,lost=%lu,now=%lu,fw=%s\n"
    "#cols=seq,millis,mode,distance_cm,us_status,us_seq,us_age_ms,"
    "imu_ok,yaw_unwrapped_deg,roll_deg,pitch_deg,gyro_z_dps,"
    "gyro_z_raw_dps,gyro_bias_dps,gyro_cal,left_cmd,right_cmd,"
    "left_cmd_int,right_cmd_int\n",
    (unsigned long)bootId, (unsigned long)stop, (unsigned long)oldest,
    (unsigned long)lost, millis(), FW_VERSION);

  for (uint32_t i = start; i < stop; i++) {
    Sample sm;
    portENTER_CRITICAL(&shareMux);
    sm = sampleBuf[i % SAMPLE_RING];
    portEXIT_CRITICAL(&shareMux);

    char dist[12] = "";
    if (sm.distCm >= 0) snprintf(dist, sizeof(dist), "%.1f", sm.distCm);

    char line[200];
    int n = snprintf(line, sizeof(line),
      "%lu,%lu,%c,%s,%s,%lu,%lu,%d,%.3f,%.2f,%.2f,%.3f,%.3f,%.4f,%s,"
      "%d,%d,%.3f,%.3f\n",
      (unsigned long)i, (unsigned long)sm.ms, sm.autoMode ? 'A' : 'M',
      dist, sm.usStatus ? sm.usStatus : "none",
      (unsigned long)sm.usSeq, (unsigned long)sm.usAgeMs,
      sm.imuOk ? 1 : 0, sm.yawUnwrapped, sm.roll, sm.pitch,
      sm.gyroRate, sm.gyroRaw, sm.gyroBias, calNameOf(sm.calState),
      sm.left, sm.right, sm.leftInt, sm.rightInt);

    if (n <= 0 || n >= (int)sizeof(line)) continue;

    if (len + n >= sizeof(chunk)) {
      if (httpd_resp_send_chunk(req, chunk, len) != ESP_OK) return ESP_FAIL;
      len = 0;
    }

    memcpy(chunk + len, line, n);
    len += n;
  }

  if (len > 0 && httpd_resp_send_chunk(req, chunk, len) != ESP_OK) {
    return ESP_FAIL;
  }

  return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t calibrateHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  Cmd c = {};
  c.type = CMD_CAL;
  postCmd(c);

  httpd_resp_set_type(req, "application/json");
  noStore(req);
  return httpd_resp_sendstr(req,
    "{\"ok\":true,\"info\":\"requested: ~2 s, keep still; "
    "refused if the motors run (see /api/log)\"}");
}

// snprintf returns the length it WANTED; never send past the buffer.
int clampLen(int n, size_t size) {
  if (n < 0) return 0;
  return n >= (int)size ? (int)size - 1 : n;
}

// GET /api/diag: health now + boot history from flash (JSON).
esp_err_t diagHandler(httpd_req_t* req) {
  httpRequests = httpRequests + 1;
  static char buf[1024];

  httpd_resp_set_type(req, "application/json");
  noStore(req);

  int n = snprintf(buf, sizeof(buf),
    "{\"fw\":\"%s\",\"boot_id\":%lu,\"reset_reason\":\"%s\","
    "\"uptime_ms\":%lu,\"free_heap\":%lu,\"min_free_heap\":%lu,"
    "\"loop_hz\":%lu,\"loop_max_us\":%lu,\"loop_max_us_boot\":%lu,"
    "\"loop_stack_free\":%lu,\"httpd_stack_free\":%lu,"
    "\"ws_clients\":%d,\"ws_frames_in\":%lu,\"ws_frames_out\":%lu,"
    "\"http_requests\":%lu,\"cmd_dropped\":%lu,"
    "\"watchdog_stops\":%lu,\"deadman_trips\":%lu,\"imu_resets\":%lu,"
    "\"wifi_disconnects\":%lu,\"wifi_last_reason\":%u,\"rssi\":%d,"
    "\"boots\":[",
    FW_VERSION, (unsigned long)bootId, resetReason, millis(),
    (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
    loopHz, (unsigned long)loopMaxUsSecond, (unsigned long)loopMaxUsBoot,
    (unsigned long)loopStackFree, (unsigned long)httpdStackFree,
    wsClientCount, (unsigned long)wsFramesIn, (unsigned long)wsFramesOut,
    (unsigned long)httpRequests, (unsigned long)cmdDropped,
    watchdogStops, deadmanTrips, imuResets,
    (unsigned long)wifiDisconnects, (unsigned)wifiLastReason,
    WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0);

  if (httpd_resp_send_chunk(req, buf, clampLen(n, sizeof(buf))) != ESP_OK) return ESP_FAIL;

  for (int i = 0; i < bootHistoryCount; i++) {
    const BootRecord& r = bootHistory[i];
    n = snprintf(buf, sizeof(buf),
      "%s{\"boot_no\":%lu,\"reason\":\"%s\",\"fw\":\"%s\","
      "\"prev_boot_ran_ms\":%lu,\"events_before_reset\":[",
      i ? "," : "", (unsigned long)r.bootNo, r.reason, r.fw,
      (unsigned long)r.prevUptimeMs);
    if (httpd_resp_send_chunk(req, buf, clampLen(n, sizeof(buf))) != ESP_OK) return ESP_FAIL;

    for (int k = 0; k < r.nEvents && k < BOOT_EVENTS; k++) {
      char ev[BOOT_EVENT_LEN];
      strlcpy(ev, r.events[k], sizeof(ev));
      for (char* c = ev; *c; c++) {
        if (*c == '"' || *c == '\\') *c = '\'';
      }
      n = snprintf(buf, sizeof(buf), "%s\"%s\"", k ? "," : "", ev);
      if (httpd_resp_send_chunk(req, buf, clampLen(n, sizeof(buf))) != ESP_OK) return ESP_FAIL;
    }

    if (httpd_resp_send_chunk(req, "]}", 2) != ESP_OK) return ESP_FAIL;
  }

  if (httpd_resp_send_chunk(req, "]}", 2) != ESP_OK) return ESP_FAIL;
  return httpd_resp_send_chunk(req, nullptr, 0);
}

void addRoute(const char* uri, esp_err_t (*handler)(httpd_req_t*),
              bool websocket = false) {
  httpd_uri_t u = {};
  u.uri = uri;
  u.method = HTTP_GET;
  u.handler = handler;
  u.is_websocket = websocket;
  u.handle_ws_control_frames = false;

  if (httpd_register_uri_handler(httpServer, &u) != ESP_OK) {
    Serial.printf("[NET] ERROR: could not register %s\n", uri);
  }
}

bool startHttpServer() {
  for (int i = 0; i < MAX_WS_CLIENTS; i++) wsFds[i] = -1;

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.core_id = 0;                 // loop() runs on core 1
  cfg.stack_size = 10240;
  cfg.max_open_sockets = 7;
  cfg.max_uri_handlers = 12;
  cfg.backlog_conn = 5;
  cfg.lru_purge_enable = true;     // a stale socket cannot lock others out
  cfg.recv_wait_timeout = 2;
  cfg.send_wait_timeout = 2;
  cfg.close_fn = onSocketClose;
  // Detect dead phones (Wi-Fi gone without a FIN) in ~11 s.
  cfg.keep_alive_enable = true;
  cfg.keep_alive_idle = 5;
  cfg.keep_alive_interval = 2;
  cfg.keep_alive_count = 3;

  esp_err_t err = httpd_start(&httpServer, &cfg);

  if (err != ESP_OK) {
    Serial.printf("[NET] ERROR: HTTP server did not start (%s)\n",
                  esp_err_to_name(err));
    logEvent(EV_LINK, "httpd_start_failed", err, 0, 0);
    return false;
  }

  addRoute("/", pageHandler);
  addRoute("/ws", wsHandler, true);
  addRoute("/favicon.ico", faviconHandler);
  addRoute("/api/telemetry", telemetryHandler);
  addRoute("/api/samples", samplesHandler);
  addRoute("/api/log", logHandler);
  addRoute("/api/diag", diagHandler);
  addRoute("/api/calibrate_gyro", calibrateHandler);
  return true;
}

// ---------- loop() side ----------

void applyDrive(const Cmd& c) {
  if (autoMode || calibrating()) return;

  // Frames on one socket arrive in order; this only guards against a
  // client that resends old frames after reconnecting.
  if (c.fd == driverFd && c.seq <= driverSeq) {
    staleDriveCmds++;
    return;
  }

  if (c.fd != driverFd) {
    if (driverFd >= 0 && (throttleValue != 0 || steeringValue != 0)) {
      logEvent(EV_LINK, "driver_changed", driverFd, c.fd, 0);
    }
    driverFd = c.fd;
  }

  driverSeq = c.seq;

  unsigned long gap = millis() - lastDriveCommand;

  // While driving the phone repeats every 50 ms; a longer gap means
  // Wi-Fi lag and explains jerky or watchdog stops.
  if ((throttleValue != 0 || steeringValue != 0) &&
      gap > LINK_GAP_LOG_MS) {
    logEvent(EV_LINKGAP, "drive", gap, 0, 0);
  }

  throttleValue = c.a;
  steeringValue = c.b;

  if (throttleValue != lastLoggedThrottle ||
      steeringValue != lastLoggedSteering) {
    logEvent(EV_JOY, "phone", throttleValue, steeringValue, 0);
    lastLoggedThrottle = throttleValue;
    lastLoggedSteering = steeringValue;
  }

  lastDriveCommand = millis();
  driveJoystick(throttleValue, steeringValue, "phone");
}

void applyStop(const char* why) {
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
}

void applyMode(bool wantAuto) {
  if (wantAuto && calibrating()) {
    logEvent(EV_MODE, "auto_refused_calibrating", 0, 0, 0);
    return;
  }

  autoMode = wantAuto;
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
}

void applySpeed(int v) {
  if (v != motorSpeed) {
    logEvent(EV_SPEED, "slider", v, motorSpeed, 0);
  }

  motorSpeed = v;

  if (!autoMode) {
    driveJoystick(throttleValue, steeringValue, "speed_slider");
  }
}

void applyCalibrate() {
  if (leftApplied != 0 || rightApplied != 0) {
    logEvent(EV_IMU, "cal_refused_motors", 0, 0, 0);
    Serial.println("[IMU] Calibration refused: stop the motors first.");
  } else if (!startGyroCalibration()) {
    logEvent(EV_IMU, "cal_refused_imu", 0, 0, 0);
  }
}

void applyCommand(const Cmd& c) {
  switch (c.type) {
    case CMD_DRIVE: applyDrive(c); break;
    case CMD_STOP: applyStop(c.tag); break;
    case CMD_MODE: applyMode(c.a != 0); break;
    case CMD_SPEED: applySpeed(c.a); break;
    case CMD_CAL: applyCalibrate(); break;

    case CMD_WS_OPEN:
      logEvent(EV_LINK, "ws_open", c.ip, c.fd, wsClientCount);
      Serial.printf("[NET] Phone/client connected (.%u, socket %d, %d open)\n",
                    c.ip, c.fd, wsClientCount);
      break;

    case CMD_WS_CLOSE:
      logEvent(EV_LINK, "ws_close", c.ip, c.fd, wsClientCount);
      Serial.printf("[NET] Client .%u disconnected (socket %d, %d open)\n",
                    c.ip, c.fd, wsClientCount);

      // The driving phone is gone: stop now, not 500 ms later.
      if (c.fd == driverFd) {
        driverFd = -1;

        if (!autoMode && (throttleValue != 0 || steeringValue != 0)) {
          logEvent(EV_WATCHDOG, "link_closed",
                   millis() - lastDriveCommand, 0, 0);
          throttleValue = 0;
          steeringValue = 0;
          stopBot("link_closed");
        }
      }
      break;
  }
}

void drainCommands() {
  Cmd c;

  while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) {
    applyCommand(c);
  }

  // Any frame from a phone keeps AUTO alive (it pings 4x/s).
  unsigned long seen = wsLastFrameMs;
  if (autoMode && (long)(seen - lastAutoHeartbeat) > 0) {
    lastAutoHeartbeat = seen;
  }
}

// What AUTO is doing, for the phone's status line.
void autoStatusText(char* out, size_t len) {
  bool stopped = leftApplied == 0 && rightApplied == 0;

  if (autoState == REVERSING) {
    snprintf(out, len, "BACKING OFF (%s)", autoReason);
  } else if (autoState == TURNING) {
    snprintf(out, len, "TURNING (%s)", autoReason);
  } else if (stopped && strcmp(autoReason, "blind_waiting") == 0) {
    snprintf(out, len, "STOPPED - SENSOR SEES NOTHING");
  } else if (stopped) {
    snprintf(out, len, "WAITING FOR CLEAR READING");
  } else {
    snprintf(out, len, "DRIVING PWM %d", leftApplied);
  }

  for (char* p = out; *p; p++) {
    if (*p == ',') *p = ';';
  }
}

void publishStatus() {
  char text[56] = "";
  if (autoMode) autoStatusText(text, sizeof(text));

  char dist[8] = "-";
  if (usValid) snprintf(dist, sizeof(dist), "%d", (int)usDistanceCm);

  char flags[4] = "";
  int nf = 0;
  if (calibrating()) flags[nf++] = 'C';
  if (!imuOk) flags[nf++] = 'I';
  flags[nf] = 0;

  char msg[sizeof(statusMsg)];
  snprintf(msg, sizeof(msg), "s,%lu,%s,%d,%c,%s,%s",
           (unsigned long)driverSeq, dist, guardLatched ? 1 : 0,
           autoMode ? 'A' : 'M', flags, text);

  portENTER_CRITICAL(&shareMux);
  memcpy(statusMsg, msg, sizeof(statusMsg));
  portEXIT_CRITICAL(&shareMux);

  if (httpServer && wsClientCount > 0) {
    httpd_queue_work(httpServer, pushStatusWork, nullptr);
  }
}

// Same keys as before v2.0 (the Mac mapper parses these).
void renderTelemetry() {
  static char json[sizeof(telemetryJson)];
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
    "\"roll_deg\":%.2f,"
    "\"pitch_deg\":%.2f,"
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
    "\"loop_max_us\":%lu,"
    "\"ws_clients\":%d,"
    "\"sample_seq\":%lu,"
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
    attRollDeg(),
    attPitchDeg(),
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
    (unsigned long)loopMaxUsSecond,
    wsClientCount,
    (unsigned long)sampleSeq,
    homeWifiConnected ? WiFi.localIP().toString().c_str() : "",
    homeWifiConnected ? (int)WiFi.RSSI() : 0,
    (unsigned long)ESP.getFreeHeap());

  if (n < 0 || n >= (int)sizeof(json)) {
    n = snprintf(json, sizeof(json), "{\"error\":\"telemetry overflow\"}");
    logEvent(EV_LINK, "telemetry_overflow", 0, 0, 0);
  }

  portENTER_CRITICAL(&shareMux);
  memcpy(telemetryJson, json, n);
  telemetryLen = n;
  portEXIT_CRITICAL(&shareMux);
}

void pushSample() {
  if (!sampleBuf) return;

  Sample sm;
  unsigned long now = millis();

  sm.ms = now;
  sm.yawUnwrapped = yawUnwrappedDeg;
  sm.roll = attRollDeg();
  sm.pitch = attPitchDeg();
  sm.gyroRate = gyroRateDps;
  sm.gyroRaw = gyroRawDps;
  sm.gyroBias = gyroBiasDps;
  sm.distCm = usValid ? usDistanceCm : -1.0f;
  sm.usSeq = usSeq;
  sm.usAgeMs = usSeq == 0 ? 999999UL : now - usSampleMs;
  sm.usStatus = usStatus;
  sm.leftInt = leftCmdIntegral;
  sm.rightInt = rightCmdIntegral;
  sm.left = leftApplied;
  sm.right = rightApplied;
  sm.calState = gyroCalState;
  sm.imuOk = imuOk;
  sm.autoMode = autoMode;

  portENTER_CRITICAL(&shareMux);
  sampleBuf[sampleSeq % SAMPLE_RING] = sm;
  sampleSeq++;
  portEXIT_CRITICAL(&shareMux);
}

// Everything the network side reads, refreshed from loop().
void publishState() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    pushSample();
  }

  if (now - lastStatusPushMs >= STATUS_PUSH_MS) {
    lastStatusPushMs = now;
    publishStatus();
  }

  if (now - lastTelemetryRenderMs >= TELEMETRY_RENDER_MS) {
    lastTelemetryRenderMs = now;
    renderTelemetry();
  }
}

// Wi-Fi events arrive in the Wi-Fi event task: log reason codes.
void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifiLastReason = info.wifi_sta_disconnected.reason;
      wifiDisconnects = wifiDisconnects + 1;
      logEvent(EV_WIFI, "sta_disconnected", info.wifi_sta_disconnected.reason,
               wifiDisconnects, info.wifi_sta_disconnected.rssi);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      logEvent(EV_WIFI, "sta_got_ip", info.got_ip.ip_info.ip.addr >> 24,
               0, 0);
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      logEvent(EV_WIFI, "ap_join", info.wifi_ap_staconnected.mac[5],
               WiFi.softAPgetStationNum(), 0);
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      logEvent(EV_WIFI, "ap_leave", info.wifi_ap_stadisconnected.mac[5],
               WiFi.softAPgetStationNum(), 0);
      break;
    default:
      break;
  }
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
  imuLastHealthMs = millis();

  // The calibration itself runs in loop() (startupCalibrationUpdate),
  // so Wi-Fi and the phone are already up while it runs. Drive and
  // AUTO are refused until it finishes (~2 s).
  startupCalAttempt = 1;
  Serial.printf("[IMU] Startup calibration %d/%d\n",
                startupCalAttempt, GYRO_CAL_STARTUP_ATTEMPTS);
  startupCalPending = startGyroCalibration();
}

void finishStartupCalibration() {
  startupCalPending = false;
  yawUnwrappedDeg = 0;
  gyroPrevRateDps = 0;
  attitudeFromAccel();
  logEvent(EV_IMU, "startup_cal_done", startupCalAttempt, 0, gyroBiasDps);
  Serial.printf("[IMU] Ready (gyro cal: %s). Drive enabled.\n", gyroCalName());
}

void startupCalibrationUpdate() {
  if (!startupCalPending || gyroCalState == CAL_RUNNING) return;

  if (gyroCalState == CAL_OK || !imuOk) {
    finishStartupCalibration();
    return;
  }

  // CAL_FAILED_MOTION: the robot moved. Try again.
  if (startupCalAttempt < GYRO_CAL_STARTUP_ATTEMPTS) {
    startupCalAttempt++;
    Serial.printf("[IMU] Startup calibration %d/%d\n",
                  startupCalAttempt, GYRO_CAL_STARTUP_ATTEMPTS);
    if (startGyroCalibration()) return;
  }

  if (calCount > 50) {
    gyroBiasDps = calMean;
    gyroCalState = CAL_UNVERIFIED;
    logEvent(EV_IMU, "cal_unverified", calCount, 0, gyroBiasDps);
    Serial.printf("[IMU] WARNING: robot never held still. Using bias %.4f dps "
                  "anyway. Re-run: GET /api/calibrate_gyro\n", gyroBiasDps);
  }

  finishStartupCalibration();
}

void setup() {
  // Big TX buffer so a 100 Hz viewer stream never stalls loop().
  Serial.setTxBufferSize(2048);
  Serial.begin(SERIAL_BAUD);
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

  cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(Cmd));
  sampleBuf = (Sample*)calloc(SAMPLE_RING, sizeof(Sample));

  if (!sampleBuf) {
    Serial.println("[NET] WARNING: no memory for the sample ring - "
                   "/api/samples disabled.");
    logEvent(EV_LINK, "sample_ring_alloc_failed", 0, 0, 0);
  }

  attachInterrupt(digitalPinToInterrupt(ECHO), echoISR, CHANGE);

  WiFi.onEvent(onWifiEvent);

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

  Serial.println("API: /api/telemetry /api/samples /api/log /api/diag  WS: /ws");

  // Network first, so the robot is reachable within ~1 s of a reset;
  // the gyro calibrates afterwards, in loop().
  startHttpServer();

  setupImu();

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

  // Counters written by the server task: report per-second deltas.
  static uint32_t prevIn = 0, prevOut = 0, prevHttp = 0, prevDropped = 0;
  uint32_t in = wsFramesIn, out = wsFramesOut, http = httpRequests;
  uint32_t dropped = cmdDropped;

  uint32_t maxUs = loopMaxUsSecond;
  logEvent(EV_STATS, autoMode ? "auto" : "manual", loopHz / 10,
           maxUs / 1000, (float)ESP.getFreeHeap());
  logEvent(EV_REQS, "per_s", in - prevIn, http - prevHttp,
           (float)(out - prevOut));

  if (dropped != prevDropped) {
    logEvent(EV_LINK, "cmd_queue_full", dropped - prevDropped, 0, 0);
  }

  prevIn = in;
  prevOut = out;
  prevHttp = http;
  prevDropped = dropped;

  // Is the server task still alive? (It runs pushStatusWork 10x/s.)
  static bool httpdStallReported = false;
  bool httpdStalled = wsClientCount > 0 && httpdWorkMs != 0 &&
                      now - httpdWorkMs > HTTPD_STALL_LOG_MS;
  if (httpdStalled && !httpdStallReported) {
    logEvent(EV_LINK, "httpd_stalled", now - httpdWorkMs, wsClientCount, 0);
    Serial.printf("[NET] WARNING: HTTP server task has not run for %lu ms\n",
                  now - httpdWorkMs);
  } else if (!httpdStalled && httpdStallReported) {
    logEvent(EV_LINK, "httpd_recovered", 0, 0, 0);
  }
  httpdStallReported = httpdStalled;

  static unsigned long lastHealthMs = 0;
  if (now - lastHealthMs >= 10000) {
    lastHealthMs = now;
    loopStackFree = uxTaskGetStackHighWaterMark(NULL);
    TaskHandle_t httpd = xTaskGetHandle("httpd");
    httpdStackFree = httpd ? uxTaskGetStackHighWaterMark(httpd) : 0;
    logEvent(EV_HEALTH, "health", ESP.getMinFreeHeap() / 1024,
             loopStackFree, (float)httpdStackFree);
  }

  char dist[12];
  if (usValid) snprintf(dist, sizeof(dist), "%6.1fcm", usDistanceCm);
  else snprintf(dist, sizeof(dist), "  ---   ");

  Serial.printf("[STAT] t=%7.1fs %-6s yaw=%8.2f rate=%7.2f bias=%7.3f "
                "imu=%s cal=%s rst=%lu | us=%s %-12s%s | L=%4d R=%4d | "
                "loop=%luHz max=%lums clients=%d heap=%lu\n",
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
                (unsigned long)(maxUs / 1000),
                wsClientCount,
                (unsigned long)ESP.getFreeHeap());
  loopMaxUsSecond = 0;
}

// =====================================================
// VIEWER SERIAL STREAM (viewer/index.html over USB)
// =====================================================
//
// Commands, one per line:
//   @stream 1   start (or keep alive) the 100 Hz stream
//   @stream 0   stop it
//   @cal        re-zero the gyro bias (motors must be stopped)
//
// Frame, one line every 10 ms:
//   @I,ms,qw,qx,qy,qz,yaw_deg,gx,gy,gz,ax,ay,az,dist_cm,us_seq,
//      us_status,left,right,imu_ok,gyro_cal,mode,dropped
// q = sensor -> world (Z up), gyro in dps (bias removed), accel in
// g, dist_cm = -1 when there is no valid echo.

void handleSerialCommand(const char* cmd) {
  if (strcmp(cmd, "@stream 1") == 0) {
    if (!streamOn) {
      Serial.println("[VIEW] Stream on (100 Hz).");
    }
    streamOn = true;
    streamKeepaliveMs = millis();
  } else if (strcmp(cmd, "@stream 0") == 0) {
    streamOn = false;
    Serial.println("[VIEW] Stream off.");
  } else if (strcmp(cmd, "@cal") == 0) {
    if (leftApplied != 0 || rightApplied != 0) {
      Serial.println("[VIEW] Calibration refused: stop the motors first.");
    } else if (!startGyroCalibration()) {
      Serial.println("[VIEW] Calibration refused: IMU not OK.");
    }
  }
}

void serialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmd[serialCmdLen] = 0;
        handleSerialCommand(serialCmd);
        serialCmdLen = 0;
      }
    } else if (serialCmdLen < (int)sizeof(serialCmd) - 1) {
      serialCmd[serialCmdLen++] = c;
    }
  }

  if (streamOn && millis() - streamKeepaliveMs > STREAM_KEEPALIVE_MS) {
    streamOn = false;
    Serial.println("[VIEW] Viewer gone - stream off.");
  }
}

void serialStream() {
  if (!streamOn) return;

  int64_t nowUs = esp_timer_get_time();
  if (nowUs - streamLastUs < STREAM_INTERVAL_US) return;
  streamLastUs = nowUs;

  char line[200];
  int n = snprintf(line, sizeof(line),
    "@I,%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,"
    "%.1f,%lu,%s,%d,%d,%d,%s,%s,%lu\n",
    millis(),
    attQ[0], attQ[1], attQ[2], attQ[3],
    yawUnwrappedDeg,
    gyroXDps, gyroYDps, gyroRateDps,
    accelG[0], accelG[1], accelG[2],
    usValid ? usDistanceCm : -1.0f,
    (unsigned long)usSeq,
    usStatus,
    leftApplied, rightApplied,
    imuOk ? 1 : 0,
    gyroCalName(),
    autoMode ? "auto" : "manual",
    streamDropped);

  if (n <= 0 || n >= (int)sizeof(line)) return;

  // Never block loop() on a full UART: drop the frame instead.
  if (Serial.availableForWrite() < n) {
    streamDropped++;
    return;
  }

  Serial.write((const uint8_t*)line, n);
  streamFrames++;
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
  int64_t passStart = esp_timer_get_time();
  loopCheckIn();

  currentActivity = "commands";
  drainCommands();

  currentActivity = "imu";
  imuUpdate();
  startupCalibrationUpdate();

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

  currentActivity = "publish";
  publishState();

  currentActivity = "wifi_log";
  logPeriodic();
  serialDiagnostics();

  currentActivity = "serial";
  serialCommands();
  serialStream();

  uint32_t passUs = esp_timer_get_time() - passStart;
  if (passUs > loopMaxUsSecond) loopMaxUsSecond = passUs;
  if (passUs > loopMaxUsBoot) loopMaxUsBoot = passUs;

  // ~1 kHz: plenty for every task here (IMU 200 Hz, echo timing is
  // interrupt-driven), and leaves core 1 idle time.
  currentActivity = "idle";
  delay(1);
}
