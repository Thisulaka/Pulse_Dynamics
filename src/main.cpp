/*
 * Smart Posture Monitor – neck-mounted unit
 * MCU      : ESP32-C3 Super Mini
 * Sensors  : MPU6050 (0x69, AD0=VCC) + MAX30102 (0x57) on the same I2C bus
 * Outputs  : WS2812 status LED (native RMT driver), vibration motor, buzzer
 * Input    : Pushbutton on GPIO 3, active-HIGH (3.3V when pressed)
 *
 * Hold button 1 s    -> beep + calibrate (sit/stand upright and stay still)
 * Double-tap button  -> disable / re-enable the vibrator (light + sound feedback)
 * Single tap         -> silence the heart-rate alarm (5 min snooze)
 * Calibration done   -> LED green
 * Forward crouch     -> LED red throb + escalating haptic patterns
 * Sideways tilt      -> LED fast yellow blink + tick haptic pattern
 * Abnormal HR (long) -> red LED + buzzer blink together
 *
 * MPU6050 mounting: X axis arrow points UP.
 * Forward/back tilt and left/right tilt are measured relative to the
 * calibrated upright posture.
 *
 * NOTE: heart-rate readings from a neck-mounted optical sensor are for
 * wellness/testing only and are not a medical measurement.
 */

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================================
//  PIN CONFIGURATION
// ============================================================================
constexpr uint8_t PIN_BUTTON = 5;    // active-HIGH, internal pull-down
constexpr uint8_t PIN_LED    = 6;
constexpr uint8_t PIN_BUZZER = 7;
constexpr uint8_t PIN_VIBE   = 10;
constexpr uint8_t PIN_SDA    = 8;
constexpr uint8_t PIN_SCL    = 9;

// ============================================================================
//  TUNING PARAMETERS
// ============================================================================
// --- button
constexpr uint32_t LONG_PRESS_MS     = 1000;   // hold time to start calibration
constexpr uint32_t DEBOUNCE_MS       = 25;
constexpr uint32_t TAP_MAX_MS        = 350;    // a press shorter than this is a tap
constexpr uint32_t DOUBLE_TAP_GAP_MS = 400;    // max gap between the two taps
constexpr uint32_t HOLD_FEEDBACK_MS  = 400;    // LED turns white after this long

// --- LED
constexpr uint8_t  LED_BRIGHTNESS    = 80;     // 0-255 global LED brightness

// --- IMU / calibration
constexpr uint32_t IMU_PERIOD_MS     = 10;     // 100 Hz sampling
constexpr uint16_t CAL_SAMPLES       = 200;    // 2 s of stable data
constexpr float    CAL_MOTION_DPS    = 30.0f;  // gyro limit allowed during calibration
constexpr uint32_t CAL_TIMEOUT_MS    = 15000;

// --- posture detection
constexpr bool     TILT_ON_Y         = false;  // true if forward/back tilt shows up on the sensor's Y axis
constexpr float    FORWARD_SIGN      = -1.0f;  // forward lean = positive angle (flip if reversed)

constexpr float    BAD_ANGLE_DEG       = 25.0f; // forward lean that counts as a crouch
constexpr float    GOOD_ANGLE_DEG      = 15.0f; // must come back below this (hysteresis)
constexpr uint32_t BAD_HOLD_MS         = 1500;  // forward lean must persist this long

constexpr float    SIDE_BAD_ANGLE_DEG  = 20.0f; // sideways tilt (either direction) that counts as bad
constexpr float    SIDE_GOOD_ANGLE_DEG = 12.0f; // must come back below this (hysteresis)
constexpr uint32_t SIDE_HOLD_MS        = 1500;  // sideways tilt must persist this long

constexpr uint32_t GOOD_HOLD_MS      = 800;    // upright must persist this long
constexpr float    MOTION_IGNORE_DPS = 80.0f;  // ignore samples during fast movement
constexpr float    ACCEL_FILTER      = 0.08f;  // low-pass factor for accelerometer

// --- haptics
constexpr uint32_t HAPTIC_STAGE1_MS  = 10000;  // gentle -> firm
constexpr uint32_t HAPTIC_STAGE2_MS  = 25000;  // firm   -> urgent (forward crouch only)

// --- heart rate
constexpr long     HR_MIN_IR         = 20000;  // below this = no skin contact

// --- heart-rate alarm
constexpr int      HR_LOW_BPM        = 45;     // below this = abnormal
constexpr int      HR_HIGH_BPM       = 50;    // above this = abnormal
constexpr uint32_t HR_ALERT_HOLD_MS  = 3000;  // abnormal for this long -> alarm
constexpr uint32_t HR_RECOVER_MS     = 5000;   // normal/no signal this long -> reset / clear alarm
constexpr uint32_t HR_SNOOZE_MS      = 300000; // button tap silences the alarm for 5 min
constexpr uint32_t HR_BLINK_MS       = 400;    // red LED + buzzer blink half-period
constexpr uint16_t HR_ALARM_FREQ     = 2700;   // alarm tone (Hz)

constexpr uint32_t TELEMETRY_MS      = 500;

// ============================================================================
//  OUTPUT DRIVERS (PWM) – compatible with Arduino-ESP32 core 2.x and 3.x
// ============================================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static void pwmInit()               { ledcAttach(PIN_VIBE, 20000, 8); ledcAttach(PIN_BUZZER, 2700, 8); }
  static void vibeWrite(uint8_t duty) { ledcWrite(PIN_VIBE, duty); }
  static void buzzerTone(uint16_t f)  { ledcWriteTone(PIN_BUZZER, f); }
#else
  constexpr uint8_t CH_BUZZER = 0, CH_VIBE = 2;   // different LEDC timers
  static void pwmInit() {
    ledcSetup(CH_VIBE, 20000, 8);   ledcAttachPin(PIN_VIBE, CH_VIBE);
    ledcSetup(CH_BUZZER, 2700, 8);  ledcAttachPin(PIN_BUZZER, CH_BUZZER);
  }
  static void vibeWrite(uint8_t duty) { ledcWrite(CH_VIBE, duty); }
  static void buzzerTone(uint16_t f)  { ledcWriteTone(CH_BUZZER, f); }
#endif
static void buzzerOff() { buzzerTone(0); }

// ---------------------------------------------------------------------------
//  Status LED driver (WS2812 via the core's built-in RMT neopixelWrite)
// ---------------------------------------------------------------------------
static void ledShow(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(PIN_LED,
                (uint16_t)r * LED_BRIGHTNESS / 255,
                (uint16_t)g * LED_BRIGHTNESS / 255,
                (uint16_t)b * LED_BRIGHTNESS / 255);
}

// smooth sine pulse between lo..hi with the given period
static uint8_t pulse(uint32_t now, uint32_t periodMs, uint8_t lo, uint8_t hi) {
  float ph = (now % periodMs) / (float)periodMs;
  float s  = 0.5f * (1.0f - cosf(2.0f * PI * ph));
  return (uint8_t)(lo + (hi - lo) * s);
}

// ---------------------------------------------------------------------------
//  Non-blocking buzzer melody engine
// ---------------------------------------------------------------------------
struct Note { uint16_t freq; uint16_t ms; };   // freq 0 = silence

static const Note SND_BOOT[]       = {{1800, 70}, {0, 30}, {2400, 70}, {0, 30}, {3000, 90}};
static const Note SND_CAL_START[]  = {{2700, 80}};                       // small beep
static const Note SND_CAL_DONE[]   = {{2200, 90}, {0, 50}, {3000, 140}};
static const Note SND_ERROR[]      = {{600, 200}, {0, 100}, {450, 300}};
static const Note SND_HAPTIC_OFF[] = {{2600, 90}, {0, 40}, {1800, 90}, {0, 40}, {1200, 180}}; // descending
static const Note SND_HAPTIC_ON[]  = {{1200, 90}, {0, 40}, {1800, 90}, {0, 40}, {2600, 180}}; // ascending

static bool        hrAlarmActive = false;    // heart-rate alarm owns the buzzer while true
static bool        alarmBuzzOn   = false;

static const Note* tonePtr = nullptr;
static uint8_t     toneCount = 0, toneIdx = 0;
static uint32_t    toneNextMs = 0;

static void toneApply(uint32_t now) {
  buzzerTone(tonePtr[toneIdx].freq);
  toneNextMs = now + tonePtr[toneIdx].ms;
}
static void playTone(const Note* n, uint8_t count) {
  if (hrAlarmActive) return;                 // alarm has priority over all other sounds
  tonePtr = n; toneCount = count; toneIdx = 0;
  toneApply(millis());
}
static void toneUpdate(uint32_t now) {
  if (!tonePtr || (int32_t)(now - toneNextMs) < 0) return;
  if (++toneIdx >= toneCount) { buzzerOff(); tonePtr = nullptr; return; }
  toneApply(now);
}

// ---------------------------------------------------------------------------
//  Non-blocking haptic pattern engine
// ---------------------------------------------------------------------------
struct HapticStep    { uint8_t duty; uint16_t ms; };
struct HapticPattern { const HapticStep* steps; uint8_t count; uint16_t gapMs; };

// forward crouch patterns
static const HapticStep S_GENTLE[] = {{170,150},{0,120},{170,150}};
static const HapticStep S_FIRM[]   = {{210,180},{0,100},{210,180},{0,100},{210,180}};
static const HapticStep S_URGENT[] = {{140,120},{180,120},{220,120},{255,120},{255,400}};
// sideways tilt patterns (short "tick" pulses, clearly different from the above)
static const HapticStep S_SIDE[]      = {{160,60},{0,90},{160,60},{0,90},{160,60}};
static const HapticStep S_SIDE_FIRM[] = {{220,60},{0,70},{220,60},{0,70},{220,60},{0,70},{220,60},{0,70},{220,60}};

static const HapticPattern PAT_GENTLE    = {S_GENTLE,    3, 2500};
static const HapticPattern PAT_FIRM      = {S_FIRM,      5, 2000};
static const HapticPattern PAT_URGENT    = {S_URGENT,    5, 1500};
static const HapticPattern PAT_SIDE      = {S_SIDE,      5, 2500};
static const HapticPattern PAT_SIDE_FIRM = {S_SIDE_FIRM, 9, 2000};

static const HapticPattern* hapPat = nullptr;
static uint8_t  hapIdx = 0;
static bool     hapInGap = false;
static uint32_t hapNextMs = 0;
static bool     hapticEnabled = true;     // toggled by double-tap

static void hapApplyStep(uint32_t now) {
  vibeWrite(hapPat->steps[hapIdx].duty);
  hapNextMs = now + hapPat->steps[hapIdx].ms;
}
static void hapticSetPattern(const HapticPattern* p, uint32_t now) {
  if (p == hapPat) return;
  hapPat = p; hapIdx = 0; hapInGap = false;
  hapApplyStep(now);
}
static void hapticStop() { hapPat = nullptr; vibeWrite(0); }
static void hapticUpdate(uint32_t now) {
  if (!hapPat || (int32_t)(now - hapNextMs) < 0) return;
  if (hapInGap) { hapInGap = false; hapIdx = 0; hapApplyStep(now); return; }
  if (++hapIdx >= hapPat->count) {
    vibeWrite(0); hapInGap = true; hapNextMs = now + hapPat->gapMs;
  } else {
    hapApplyStep(now);
  }
}

// ============================================================================
//  MPU6050 (raw register driver – no external library)
// ============================================================================
constexpr uint8_t MPU_ADDR = 0x69;   // AD0 = VCC

struct ImuSample { float ax, ay, az, gx, gy, gz; };   // g, deg/s

static bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuReadReg(uint8_t reg, uint8_t& val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}
static bool mpuInit() {
  if (!mpuWriteReg(0x6B, 0x80)) return false;   // device reset
  delay(100);
  mpuWriteReg(0x6B, 0x01);                      // wake, PLL with gyro X
  delay(10);
  mpuWriteReg(0x1A, 0x03);                      // DLPF ~44 Hz
  mpuWriteReg(0x19, 9);                         // 1 kHz / 10 = 100 Hz
  mpuWriteReg(0x1B, 0x00);                      // gyro  +-250 dps
  mpuWriteReg(0x1C, 0x00);                      // accel +-2 g
  uint8_t who = 0;
  if (!mpuReadReg(0x75, who)) return false;
  Serial.printf("[MPU6050] WHO_AM_I = 0x%02X\n", who);
  return true;
}
static bool mpuRead(ImuSample& s) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  s.ax = v[0] / 16384.0f; s.ay = v[1] / 16384.0f; s.az = v[2] / 16384.0f;
  s.gx = v[4] / 131.0f;   s.gy = v[5] / 131.0f;   s.gz = v[6] / 131.0f;
  return true;
}

// ============================================================================
//  MAX30102 heart rate + abnormal-rate alarm
// ============================================================================
static MAX30105 hrSensor;
static bool     hrOk = false;
static long     hrLastIR = 0;
static uint32_t hrLastBeatMs = 0;
static uint8_t  hrRates[4] = {0};
static uint8_t  hrRateIdx = 0;
static int      hrBpm = 0;

//jygjhgjhg

static void hrInit() {
  if (!hrSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    Serial.println("[MAX30102] NOT FOUND");
    hrOk = false;
    return;
  }
  // brightness, sample avg, LED mode (2=red+IR), sample rate, pulse width, ADC range
  hrSensor.setup(0xFF, 4, 2, 100, 411, 4096);
  hrOk = true;
  Serial.println("[MAX30102] OK");
}
static void hrUpdate(uint32_t now) {
  if (!hrOk) return;
  hrSensor.check();
  while (hrSensor.available()) {
    long ir = hrSensor.getFIFOIR();
    hrSensor.nextSample();
    hrLastIR = ir;
    if (ir < HR_MIN_IR) { hrBpm = 0; hrRateIdx = 0; memset(hrRates, 0, sizeof(hrRates)); continue; }
    if (checkForBeat(ir)) {
      uint32_t delta = now - hrLastBeatMs;
      hrLastBeatMs = now;
      float inst = 60000.0f / (float)delta;
      if (inst > 40 && inst < 190) {
        hrRates[hrRateIdx++ % 4] = (uint8_t)inst;
        int sum = 0, n = 0;
        for (uint8_t r : hrRates) if (r) { sum += r; n++; }
        hrBpm = n ? sum / n : 0;
      }
    }
  }
  if (hrBpm && (now - hrLastBeatMs) > 5000) hrBpm = 0;   // stale
}

// --- alarm state
static bool     hrTiming = false;          // currently timing an abnormal period
static uint32_t hrAbnormalSince = 0;
static uint32_t hrLastAbnormalMs = 0;
static bool     hrSnoozed = false;
static uint32_t hrSnoozeUntilMs = 0;

static void hrAlarmUpdate(uint32_t now) {
  bool abnormal = hrOk && hrBpm > 0 && (hrBpm > HR_HIGH_BPM || hrBpm < HR_LOW_BPM);
  if (abnormal) hrLastAbnormalMs = now;

  if (hrAlarmActive) {                                   // clear after a normal / no-signal period
    if (now - hrLastAbnormalMs > HR_RECOVER_MS) {
      hrAlarmActive = false; hrTiming = false; alarmBuzzOn = false;
      buzzerOff();
      Serial.println("[HR ALARM] Cleared");
    }
    return;
  }

  if (hrSnoozed) {                                       // alarm silenced by the user
    if ((int32_t)(now - hrSnoozeUntilMs) < 0) return;
    hrSnoozed = false; hrTiming = false;
  }

  if (abnormal) {
    if (!hrTiming) { hrTiming = true; hrAbnormalSince = now; }
    else if (now - hrAbnormalSince >= HR_ALERT_HOLD_MS) {
      hrAlarmActive = true; alarmBuzzOn = false;
      tonePtr = nullptr; buzzerOff();                    // drop any melody in progress
      Serial.printf("[HR ALARM] Abnormal heart rate for %lu s (%d bpm)\n",
                    (unsigned long)(HR_ALERT_HOLD_MS / 1000), hrBpm);
    }
  } else if (hrTiming && (now - hrLastAbnormalMs) > HR_RECOVER_MS) {
    hrTiming = false;                                    // back to normal long enough: reset timer
  }
}

static void hrAlarmSnooze(uint32_t now) {
  hrAlarmActive = false; alarmBuzzOn = false;
  buzzerOff();
  hrSnoozed = true; hrSnoozeUntilMs = now + HR_SNOOZE_MS;
  hrTiming = false;
  Serial.println("[HR ALARM] Silenced for 5 min");
}

// ============================================================================
//  STATE MACHINE
// ============================================================================
// NOTE: keep the order in sync with the names[] array in the telemetry output
enum State { ST_IDLE, ST_CALIBRATING, ST_GOOD, ST_BAD_FWD, ST_BAD_SIDE, ST_ERROR };
static State state = ST_IDLE;

static bool isMonitoring() { return state == ST_GOOD || state == ST_BAD_FWD || state == ST_BAD_SIDE; }
static bool isBadPosture() { return state == ST_BAD_FWD || state == ST_BAD_SIDE; }

// --- calibration data
static float    calSumX, calSumY, calSumZ, calGSumX, calGSumY, calGSumZ;
static uint16_t calN = 0;
static uint32_t calStartMs = 0;
static float    refPitch = 0, refSide = 0;          // upright reference angles (deg)
static float    gBiasX = 0, gBiasY = 0, gBiasZ = 0; // gyro bias

// --- posture data
static float    fx = 1, fy = 0, fz = 0;             // filtered accel (X is up)
static float    lean = 0;                           // forward lean in degrees (+ = forward)
static float    sideLean = 0;                       // sideways tilt in degrees (sign = direction)
static bool     pendingActive = false;
static State    pendingState = ST_GOOD;
static uint32_t pendingSince = 0;
static uint32_t badSinceMs = 0;

// --- fault handling
static uint8_t  imuFails = 0;
static uint32_t lastErrRetryMs = 0;

// --- feedback overlay (short blinking notification on the LED)
static uint8_t  notifyR = 0, notifyG = 0, notifyB = 0;
static uint32_t notifyUntilMs = 0;

// ---------------------------------------------------------------------------
//  Tilt helpers (X axis is up; Y and Z are the two horizontal axes)
// ---------------------------------------------------------------------------
static float horizFwd(float y, float z)  { return TILT_ON_Y ? y : z; }   // forward/back axis
static float horizSide(float y, float z) { return TILT_ON_Y ? z : y; }   // left/right axis

static float fwdAngle(float x, float y, float z) {
  return atan2f(horizFwd(y, z), x) * RAD_TO_DEG;
}
static float sideAngle(float x, float y, float z) {
  float hf = horizFwd(y, z), hs = horizSide(y, z);
  return atan2f(hs, sqrtf(x * x + hf * hf)) * RAD_TO_DEG;
}
static float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void enterIdle() { state = ST_IDLE; hapticStop(); }

static void enterError() {
  state = ST_ERROR; hapticStop();
  playTone(SND_ERROR, 3);
  lastErrRetryMs = millis();
  Serial.println("[ERROR] MPU6050 communication lost");
}

static void startCalibration() {
  state = ST_CALIBRATING;
  hapticStop();
  playTone(SND_CAL_START, 1);
  calN = 0;
  calSumX = calSumY = calSumZ = calGSumX = calGSumY = calGSumZ = 0;
  calStartMs = millis();
  Serial.println("[CAL] Started – stay upright and still");
}

static void toggleHaptics(uint32_t now) {
  hapticEnabled = !hapticEnabled;
  if (!hapticEnabled) {
    hapticStop();
    playTone(SND_HAPTIC_OFF, 5);
    notifyR = 150; notifyG = 0; notifyB = 255;         // purple = vibrator disabled
    Serial.println("[HAPTICS] Vibrator DISABLED");
  } else {
    playTone(SND_HAPTIC_ON, 5);
    notifyR = 0; notifyG = 255; notifyB = 0;           // green = vibrator enabled
    Serial.println("[HAPTICS] Vibrator ENABLED");
  }
  notifyUntilMs = now + 900;
}

static void calibrationFeed(const ImuSample& s, uint32_t now) {
  float gmag = sqrtf(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);
  float amag = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
  if (gmag > CAL_MOTION_DPS || amag < 0.8f || amag > 1.2f) {   // moving -> restart averaging
    calN = 0;
    calSumX = calSumY = calSumZ = calGSumX = calGSumY = calGSumZ = 0;
  } else {
    calSumX += s.ax; calSumY += s.ay; calSumZ += s.az;
    calGSumX += s.gx; calGSumY += s.gy; calGSumZ += s.gz;
    calN++;
  }

  if (calN >= CAL_SAMPLES) {
    float mx = calSumX / calN, my = calSumY / calN, mz = calSumZ / calN;
    refPitch = fwdAngle(mx, my, mz);                 // upright reference angles
    refSide  = sideAngle(mx, my, mz);
    gBiasX = calGSumX / calN; gBiasY = calGSumY / calN; gBiasZ = calGSumZ / calN;
    fx = mx; fy = my; fz = mz;
    lean = 0; sideLean = 0;
    pendingActive = false;
    state = ST_GOOD;
    playTone(SND_CAL_DONE, 3);
    Serial.printf("[CAL] Done – monitoring posture (ref fwd %.1f, side %.1f deg)\n", refPitch, refSide);
  } else if (now - calStartMs > CAL_TIMEOUT_MS) {
    playTone(SND_ERROR, 3);
    enterIdle();
    Serial.println("[CAL] Failed – too much movement");
  }
}

static void postureFeed(const ImuSample& s, uint32_t now) {
  float gx = s.gx - gBiasX, gy = s.gy - gBiasY, gz = s.gz - gBiasZ;
  float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
  float amag = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);

  if (amag > 0.75f && amag < 1.25f) {            // trust accel only near 1 g
    fx += ACCEL_FILTER * (s.ax - fx);
    fy += ACCEL_FILTER * (s.ay - fy);
    fz += ACCEL_FILTER * (s.az - fz);
  }

  // Angles relative to the calibrated upright posture.
  lean     = wrap180(FORWARD_SIGN * (fwdAngle(fx, fy, fz) - refPitch));   // + = forward
  sideLean = wrap180(sideAngle(fx, fy, fz) - refSide);                    // left/right

  if (gmag > MOTION_IGNORE_DPS) return;          // fast movement: hold decision

  // Decide which posture the current angles correspond to (with hysteresis).
  float aSide = fabsf(sideLean);
  bool fwdBad  = lean  > ((state == ST_BAD_FWD)  ? GOOD_ANGLE_DEG      : BAD_ANGLE_DEG);
  bool sideBad = aSide > ((state == ST_BAD_SIDE) ? SIDE_GOOD_ANGLE_DEG : SIDE_BAD_ANGLE_DEG);
  State target = fwdBad ? ST_BAD_FWD : (sideBad ? ST_BAD_SIDE : ST_GOOD);   // forward has priority

  if (target == state) { pendingActive = false; return; }

  if (!pendingActive || pendingState != target) {   // start timing the change
    pendingActive = true; pendingState = target; pendingSince = now;
    return;
  }

  uint32_t holdMs = (target == ST_GOOD)    ? GOOD_HOLD_MS
                  : (target == ST_BAD_FWD) ? BAD_HOLD_MS : SIDE_HOLD_MS;
  if (now - pendingSince >= holdMs) {
    state = target;
    pendingActive = false;
    hapticStop();
    if (target != ST_GOOD) badSinceMs = now;
    Serial.println(target == ST_BAD_FWD  ? "[POSTURE] Forward crouch detected" :
                   target == ST_BAD_SIDE ? "[POSTURE] Sideways tilt detected"  :
                                           "[POSTURE] Back to good posture");
  }
}

// ============================================================================
//  BUTTON (active-HIGH, debounced) – long press, double tap, press edge
// ============================================================================
enum BtnEvent { BTN_NONE, BTN_LONG, BTN_DOUBLE };

static bool     btnStable = false, btnRaw = false, btnLongFired = false;
static bool     btnPressEdge = false;              // set on every debounced press
static uint32_t btnChangeMs = 0, btnPressMs = 0, lastTapReleaseMs = 0;
static uint8_t  tapCount = 0;

static BtnEvent buttonUpdate(uint32_t now) {
  BtnEvent ev = BTN_NONE;
  bool raw = (digitalRead(PIN_BUTTON) == HIGH);    // pressed = 3.3V
  if (raw != btnRaw) { btnRaw = raw; btnChangeMs = now; }

  if ((now - btnChangeMs) >= DEBOUNCE_MS && raw != btnStable) {
    btnStable = raw;
    if (btnStable) {                                // pressed
      btnPressMs = now; btnLongFired = false;
      btnPressEdge = true;
    } else {                                        // released
      if (!btnLongFired && (btnPressMs != 0) && (now - btnPressMs) <= TAP_MAX_MS) {
        if (tapCount == 1 && (btnPressMs - lastTapReleaseMs) <= DOUBLE_TAP_GAP_MS) {
          tapCount = 0;
          ev = BTN_DOUBLE;
        } else {
          tapCount = 1;
          lastTapReleaseMs = now;
        }
      } else {
        tapCount = 0;
      }
    }
  }

  if (btnStable && !btnLongFired && (now - btnPressMs) >= LONG_PRESS_MS) {
    btnLongFired = true;
    tapCount = 0;
    ev = BTN_LONG;
  }
  return ev;
}
static bool buttonHolding(uint32_t now) {
  return btnStable && !btnLongFired && (now - btnPressMs) > HOLD_FEEDBACK_MS;
}

// ============================================================================
//  STATUS LED RENDERING (also drives the alarm buzzer so both stay in sync)
// ============================================================================
static void renderLed(uint32_t now) {
  // 0) heart-rate alarm: red LED and buzzer blink together (highest priority)
  if (hrAlarmActive) {
    bool on = ((now / HR_BLINK_MS) & 1) == 0;
    ledShow(on ? 255 : 0, 0, 0);
    if (on != alarmBuzzOn) {
      alarmBuzzOn = on;
      buzzerTone(on ? HR_ALARM_FREQ : 0);
    }
    return;
  }

  // 1) transient notification (vibrator enabled / disabled): blinking colour
  if ((int32_t)(notifyUntilMs - now) > 0) {
    if (((now / 150) & 1) == 0) ledShow(notifyR, notifyG, notifyB);
    else                        ledShow(0, 0, 0);
    return;
  }

  // 2) button hold feedback
  if (buttonHolding(now)) { ledShow(130, 130, 130); return; }   // white = keep holding

  // 3) vibrator-disabled reminder: short purple blip every 3 s
  if (!hapticEnabled && (state == ST_IDLE || isMonitoring()) && (now % 3000) < 150) {
    ledShow(150, 0, 255);
    return;
  }

  // 4) normal state colours
  switch (state) {
    case ST_IDLE:                                   // slow blue breathing = ready
      ledShow(0, 0, pulse(now, 3000, 10, 255));
      break;
    case ST_CALIBRATING: {                          // fast cyan pulse
      uint8_t v = pulse(now, 600, 15, 255);
      ledShow(0, v, v);
      break;
    }
    case ST_GOOD:                                   // solid green
      ledShow(0, 255, 0);
      break;
    case ST_BAD_FWD:                                // red with gentle throb
      ledShow(pulse(now, 1500, 150, 255), 0, 0);
      break;
    case ST_BAD_SIDE:                               // fast yellow blink
      if (((now / 200) & 1) == 0) ledShow(255, 200, 0); else ledShow(0, 0, 0);
      break;
    case ST_ERROR:                                  // amber blink
      if ((now / 250) & 1) ledShow(255, 90, 0); else ledShow(0, 0, 0);
      break;
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
static void bootSelfTest() {
  ledShow(255, 0, 0); delay(250);    // red
  ledShow(0, 255, 0); delay(250);    // green
  ledShow(0, 0, 255); delay(250);    // blue
  ledShow(0, 0, 0);
  vibeWrite(200); delay(250); vibeWrite(0);          // vibrator test
  playTone(SND_BOOT, 5);                             // buzzer test
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }   // wait for USB serial monitor
  Serial.println("\n=== Smart Posture Monitor ===");

  pinMode(PIN_BUTTON, INPUT_PULLDOWN);   // idle = LOW, pressed = HIGH
  pwmInit();
  vibeWrite(0); buzzerOff();
  ledShow(0, 0, 0);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  bootSelfTest();

  bool mpuOk = mpuInit();
  Serial.println(mpuOk ? "[MPU6050] OK" : "[MPU6050] NOT FOUND (check wiring / address 0x69)");
  hrInit();

  if (mpuOk) {
    enterIdle();
    Serial.println("Hold button 1 s = calibrate | Double-tap = vibrator on/off | Tap = silence HR alarm");
  } else {
    enterError();
  }
}

void loop() {
  static uint32_t lastImuMs = 0, lastHrMs = 0, lastLedMs = 0, lastTelemetryMs = 0;
  uint32_t now = millis();

  // ---- Button
  BtnEvent ev = buttonUpdate(now);
  if (btnPressEdge) {                               // any press silences an active HR alarm
    btnPressEdge = false;
    if (hrAlarmActive) hrAlarmSnooze(now);
  }
  if (state == ST_IDLE || isMonitoring()) {
    if (ev == BTN_LONG)        startCalibration();
    else if (ev == BTN_DOUBLE) toggleHaptics(now);
  }

  // ---- IMU
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = now;
    if (state == ST_ERROR) {
      if (now - lastErrRetryMs >= 2000) {           // auto-recovery attempts
        lastErrRetryMs = now;
        ImuSample t;
        if (mpuInit() && mpuRead(t)) { imuFails = 0; enterIdle(); playTone(SND_CAL_DONE, 3);
          Serial.println("[MPU6050] Recovered"); }
      }
    } else {
      ImuSample s;
      if (mpuRead(s)) {
        imuFails = 0;
        if (state == ST_CALIBRATING)  calibrationFeed(s, now);
        else if (isMonitoring())      postureFeed(s, now);
      } else if (++imuFails >= 25) {
        enterError();
      }
    }
  }

  // ---- Heart rate (polled every 20 ms) + abnormal-rate alarm
  if (now - lastHrMs >= 20) {
    lastHrMs = now;
    hrUpdate(now);
    hrAlarmUpdate(now);
  }

  // ---- Haptics: escalate while posture stays bad (only if enabled)
  if (isBadPosture() && hapticEnabled) {
    uint32_t t = now - badSinceMs;
    const HapticPattern* p;
    if (state == ST_BAD_FWD) {
      p = (t < HAPTIC_STAGE1_MS) ? &PAT_GENTLE
        : (t < HAPTIC_STAGE2_MS) ? &PAT_FIRM : &PAT_URGENT;
    } else {
      p = (t < HAPTIC_STAGE1_MS) ? &PAT_SIDE : &PAT_SIDE_FIRM;
    }
    hapticSetPattern(p, now);
  }
  hapticUpdate(now);
  toneUpdate(now);

  // ---- LED + alarm buzzer (50 Hz)
  if (now - lastLedMs >= 20) { lastLedMs = now; renderLed(now); }

  // ---- Telemetry
  if (now - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = now;
    static const char* names[] = {"IDLE", "CALIBRATING", "GOOD", "FORWARD", "SIDEWAYS", "ERROR"};
    Serial.printf("state=%s fwd=%+.1f side=%+.1f deg haptics=%s | HR=%d bpm (IR=%ld) hrAlarm=%s%s\n",
                  names[state], lean, sideLean, hapticEnabled ? "ON" : "OFF",
                  hrBpm, hrLastIR, hrAlarmActive ? "ON" : "OFF",
                  hrOk ? "" : " [sensor missing]");
  }
}