#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <INA226_WE.h>
#include <math.h>

// ======================================================
// ================== INA + SERVO SECTION ===============
// ======================================================

// ===============================
// DEFINES & CONSTANTS
// ===============================
#define INA226_ADDRESS       0x40
#define PCA9548A_ADDRESS     0x70

// Servo pins (your mapping)
#define SERVO0_PIN PB13  // Pinky  (INA channel 0)
#define SERVO1_PIN PB14  // Ring   (INA channel 1)
#define SERVO2_PIN PB15  // Middle (INA channel 2)
#define SERVO3_PIN PA8   // Index  (INA channel 3)
#define SERVO4_PIN PA11  // Thumb  (INA channel 4)

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// Desired full travel time (2400 ↔ 500)
#define FULL_SWEEP_TIME_SEC 10.0f

// Choose current print frequency (5 times per 1 second)
#define CURRENT_PRINT_PERIOD_MS 200

// Choose FSR print frequency (5 times per 1 second)
#define FSR_PRINT_PERIOD_MS 200

// ===============================
// OBJECTS
// ===============================
Servo servo0, servo1, servo2, servo3, servo4;

// One INA object per channel
INA226_WE ina226_0(INA226_ADDRESS);
INA226_WE ina226_1(INA226_ADDRESS);
INA226_WE ina226_2(INA226_ADDRESS);
INA226_WE ina226_3(INA226_ADDRESS);
INA226_WE ina226_4(INA226_ADDRESS);

// ===============================
// GLOBAL VARIABLES
// ===============================

// Kept for your current print format (millis,pulse,...)
// Now pulse means "average commanded pulse across 5 servos"
int currentPulseWidth = SERVO_MAX_US;

unsigned long lastCurrentPrint = 0;
unsigned long lastFSRPrint = 0;

// Servo attach-on-demand flag + helper
bool servosEnabled = false;

// ======================================================
// ====================== FSR SECTION ====================
// ======================================================

#define NUM_SENSORS 9

uint8_t fsrPins[NUM_SENSORS] = {
  PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

// Pin names for printing
const char* fsrPinNames[NUM_SENSORS] = {
  "Little", "LittlePalm", "Ring", "RingPalm", "Middle", "Index", "IndexPalm", "Thumb", "ThumbPalm"
};

// Print-only FSR readings (no stability, no saving)
void printFSRLive() {
  Serial.print("FSR Live: ");
  for (int s = 0; s < NUM_SENSORS; s++) {
    float v = analogRead(fsrPins[s]);
    Serial.print(fsrPinNames[s]);
    Serial.print("=");
    Serial.print(v, 2);
    if (s < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();
}

// ======================================================
// ========================= FSM =========================
// ======================================================

// FSM states
enum HandState {
  STATE_IDLE,
  STATE_CLOSING_FAST,
  STATE_SETTLE,
  STATE_CLOSING_SLOW,
  STATE_HOLD,
  STATE_TIGHTEN,
  STATE_RECOVER,
  STATE_OPEN
};

HandState state = STATE_IDLE;

// Print state only when it changes (clean logs)
HandState lastPrintedState = (HandState)255;

// Control loop timing for FSM sensing/math
const uint32_t CONTROL_PERIOD_MS = 10; // 10ms control update
uint32_t lastControlMs = 0;

// Which fingers are used? (set by user pattern like 11000)
// Index mapping:
// 0 = Pinky, 1 = Ring, 2 = Middle, 3 = Index, 4 = Thumb
const int NUM_SERVOS = 5;
bool fingerEnabled[NUM_SERVOS] = { false, false, false, false, false };

// Pattern arming (NEW UI protocol):
// - Receive 5-bit pattern -> store + stay idle
// - Receive '2' -> start FSM only if a pattern exists and state is IDLE
// - Receive '3' -> reset/open anytime, then go back to IDLE and wait for pattern
bool hasPattern = false;
String lastPattern = "00000";

// ======================================================
// =============== SPEED-BASED RAMP (PER FINGER) =========
// ======================================================

// AUTO SPEED CALCULATION (base)
float fullTravelUs = (float)(SERVO_MAX_US - SERVO_MIN_US);   // 1900
float autoRampSpeedUsPerSec = fullTravelUs / FULL_SWEEP_TIME_SEC;

// Per-finger commanded pulse widths (this is what we send to each servo)
int servoPulseUs[NUM_SERVOS] = { SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US };

// Per-finger ramp state
bool  rampActive[NUM_SERVOS] = { false, false, false, false, false };
int   rampTargetUs[NUM_SERVOS] = { SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US };
float rampPosUs[NUM_SERVOS] = { (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US };

// One shared timestamp for dt (we update all fingers together)
unsigned long lastRampUpdateMs = 0;

// Current ramp speed used now (FSM changes this per state)
float rampSpeedUsPerSec = 0.0f;

// ======================================================
// ===================== FILTERING + SLOPE ===============
// ======================================================

const float ALPHA_I   = 0.20f; // current smoothing
const float ALPHA_FSR = 0.20f; // fsr smoothing

// Current arrays (mA)
float iRaw[NUM_SERVOS]   = {0};
float iFilt[NUM_SERVOS]  = {0};
float iPrev[NUM_SERVOS]  = {0};
float iSlope[NUM_SERVOS] = {0};

// FSR arrays (ADC units)
float fsrRaw[NUM_SENSORS]   = {0};
float fsrFilt[NUM_SENSORS]  = {0};
float fsrPrev[NUM_SENSORS]  = {0};
float fsrSlope[NUM_SENSORS] = {0};

// ======================================================
// ====================== THRESHOLDS =====================
// ======================================================

// Impact spike
const float I_IMPACT_T       = 300.0f;   // mA
const float I_SLOPE_IMPACT_T = 2000.0f;  // mA/s

// Contact / loaded threshold
const float I_CONTACT_T      = 50.0f;    // mA

// Secure current band + stable slope
const float I_SECURE_MIN     = 8.0f;     // mA
const float I_SECURE_MAX     = 120.0f;   // mA
const float I_SLOPE_STABLE_T = 80.0f;    // mA/s

// Load increase (pour water)
const float I_LOAD_DELTA_T   = 15.0f;    // mA above baseline
const uint32_t LOAD_CONFIRM_MS = 300;

// FSR touch detection
const float FSR_TOUCH_T       = 50.0f;
const float FSR_SLOPE_TOUCH_T = 200.0f;  // ADC/s

// ======================================================
// ===================== TIMERS/COUNTERS =================
// ======================================================

uint32_t stateEnterMs = 0;

const uint32_t SETTLE_MS = 200;

const int SECURE_STABLE_CYCLES = 12; // 12 * 10ms = 120ms
int secureCounter = 0;

// tightening budget prevents crushing
int tightenBudgetUs = 250;
const int TIGHTEN_STEP_US = 30;
const uint32_t TIGHTEN_COOLDOWN_MS = 250;
uint32_t lastTightenMs = 0;

// recover behavior
const int RECOVER_OPEN_US = 150;
const uint32_t RECOVER_MS = 200;

// Baseline current for load-change detection
float iBaseline[NUM_SERVOS] = {0};
uint32_t loadAboveMsStart = 0;

// ======================================================
// =================== SERVO ATTACH ======================
// ======================================================
void attachServosOnce() {
  if (servosEnabled) return;

  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  servosEnabled = true;

  Serial.println("Servos attached (enabled).");
}

// ======================================================
// ================== PCA9548A SELECT ====================
// ======================================================
void selectPCAChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ===============================
// INA presence check at 0x40
// ===============================
bool inaPresentOnCurrentBus() {
  Wire.beginTransmission(INA226_ADDRESS);
  return (Wire.endTransmission() == 0);
}

// ===============================
// I2C ERROR CHECK
// ===============================
void checkForI2cErrors(INA226_WE &sensor) {
  byte errorCode = sensor.getI2cErrorCode();
  if (errorCode) {
    Serial.print("I2C error: ");
    Serial.println(errorCode);
    while (1) {} // halt
  }
}

// ======================================================
// ========= READ ALL CURRENTS (channels 0..4) ===========
// ======================================================
void readINA226Currents(float out_mA[NUM_SERVOS]) {
  selectPCAChannel(0);
  out_mA[0] = ina226_0.getCurrent_mA();
  checkForI2cErrors(ina226_0);

  selectPCAChannel(1);
  out_mA[1] = ina226_1.getCurrent_mA();
  checkForI2cErrors(ina226_1);

  selectPCAChannel(2);
  out_mA[2] = ina226_2.getCurrent_mA();
  checkForI2cErrors(ina226_2);

  selectPCAChannel(3);
  out_mA[3] = ina226_3.getCurrent_mA();
  checkForI2cErrors(ina226_3);

  selectPCAChannel(4);
  out_mA[4] = ina226_4.getCurrent_mA();
  checkForI2cErrors(ina226_4);
}

// ======================================================
// ================= PRINT ALL CURRENTS ==================
// Format stays compatible with your Python CURRENT_RE
// millis,pulse,Little=.. mA, Ring=.. mA, ...
// ======================================================
void printINA226Data() {
  float c0, c1, c2, c3, c4;

  selectPCAChannel(0);
  c0 = ina226_0.getCurrent_mA();
  checkForI2cErrors(ina226_0);

  selectPCAChannel(1);
  c1 = ina226_1.getCurrent_mA();
  checkForI2cErrors(ina226_1);

  selectPCAChannel(2);
  c2 = ina226_2.getCurrent_mA();
  checkForI2cErrors(ina226_2);

  selectPCAChannel(3);
  c3 = ina226_3.getCurrent_mA();
  checkForI2cErrors(ina226_3);

  selectPCAChannel(4);
  c4 = ina226_4.getCurrent_mA();
  checkForI2cErrors(ina226_4);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(currentPulseWidth);
  Serial.print(",");
  Serial.print("Little=");
  Serial.print(c0);
  Serial.print(" mA, ");
  Serial.print("Ring=");
  Serial.print(c1);
  Serial.print(" mA, ");
  Serial.print("Middle=");
  Serial.print(c2);
  Serial.print(" mA, ");
  Serial.print("Index=");
  Serial.print(c3);
  Serial.print(" mA, ");
  Serial.print("Thumb=");
  Serial.print(c4);
  Serial.println(" mA");
}

// ======================================================
// ================= APPLY SERVO PULSES ==================
// Disabled fingers forced to open (SERVO_MAX_US).
// currentPulseWidth becomes average across 5.
// ======================================================
void applyServoPulses() {
  attachServosOnce();

  for (int f = 0; f < NUM_SERVOS; f++) {
    int pw = servoPulseUs[f];
    if (pw < SERVO_MIN_US) pw = SERVO_MIN_US;
    if (pw > SERVO_MAX_US) pw = SERVO_MAX_US;

    if (!fingerEnabled[f]) pw = SERVO_MAX_US;

    servoPulseUs[f] = pw;
  }

  long sum = 0;
  for (int f = 0; f < NUM_SERVOS; f++) sum += servoPulseUs[f];
  currentPulseWidth = (int)(sum / NUM_SERVOS);

  servo0.writeMicroseconds(servoPulseUs[0]);
  servo1.writeMicroseconds(servoPulseUs[1]);
  servo2.writeMicroseconds(servoPulseUs[2]);
  servo3.writeMicroseconds(servoPulseUs[3]);
  servo4.writeMicroseconds(servoPulseUs[4]);
}

// ======================================================
// =============== START/UPDATE PER-FINGER RAMP ===========
// ======================================================
void startRampAllTo(int targetUs, float speedUsPerSec, bool onlyEnabled) {
  attachServosOnce();

  if (targetUs < SERVO_MIN_US) targetUs = SERVO_MIN_US;
  if (targetUs > SERVO_MAX_US) targetUs = SERVO_MAX_US;
  if (speedUsPerSec < 1.0f) speedUsPerSec = 1.0f;

  rampSpeedUsPerSec = speedUsPerSec;

  for (int f = 0; f < NUM_SERVOS; f++) {
    if (onlyEnabled && !fingerEnabled[f]) {
      rampTargetUs[f] = SERVO_MAX_US;
      rampPosUs[f] = (float)servoPulseUs[f];
      rampActive[f] = true;
      continue;
    }

    rampTargetUs[f] = targetUs;
    rampPosUs[f] = (float)servoPulseUs[f];
    rampActive[f] = true;
  }

  lastRampUpdateMs = millis();
}

bool anyRampActive() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (rampActive[f]) return true;
  }
  return false;
}

void stopAllRamps() {
  for (int f = 0; f < NUM_SERVOS; f++) rampActive[f] = false;
}

void updateRamp() {
  if (!anyRampActive()) return;

  unsigned long now = millis();
  unsigned long dtMs = now - lastRampUpdateMs;
  if (dtMs == 0) return;
  lastRampUpdateMs = now;

  float dt = dtMs / 1000.0f;
  float step = rampSpeedUsPerSec * dt;

  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!rampActive[f]) continue;

    float diff = (float)rampTargetUs[f] - rampPosUs[f];

    if (fabsf(diff) <= step) {
      rampPosUs[f] = (float)rampTargetUs[f];
      rampActive[f] = false;
    } else {
      rampPosUs[f] += (diff > 0.0f) ? step : -step;
    }

    servoPulseUs[f] = (int)roundf(rampPosUs[f]);
  }

  applyServoPulses();
}

// ======================================================
// ================= SENSOR UPDATE (FSM) =================
// ======================================================
static inline float lpFilter(float prev, float x, float alpha) {
  return prev + alpha * (x - prev);
}

void updateSensorsFSM(float dtSec) {
  readINA226Currents(iRaw);

  for (int f = 0; f < NUM_SERVOS; f++) {
    iFilt[f] = lpFilter(iFilt[f], iRaw[f], ALPHA_I);
    iSlope[f] = (iFilt[f] - iPrev[f]) / dtSec;
    iPrev[f] = iFilt[f];
  }

  for (int s = 0; s < NUM_SENSORS; s++) {
    fsrRaw[s] = (float)analogRead(fsrPins[s]);
    fsrFilt[s] = lpFilter(fsrFilt[s], fsrRaw[s], ALPHA_FSR);
    fsrSlope[s] = (fsrFilt[s] - fsrPrev[s]) / dtSec;
    fsrPrev[s] = fsrFilt[s];
  }
}

// ======================================================
// ================= EVENT DETECTION =====================
// ======================================================
bool detectImpact() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (iFilt[f] > I_IMPACT_T) return true;
    if (iSlope[f] > I_SLOPE_IMPACT_T) return true;
  }
  return false;
}

bool detectContact() {
  bool fsrTouch = false;
  for (int s = 0; s < NUM_SENSORS; s++) {
    if (fsrFilt[s] > FSR_TOUCH_T) fsrTouch = true;
    if (fsrSlope[s] > FSR_SLOPE_TOUCH_T) fsrTouch = true;
  }
  if (fsrTouch) return true;

  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (iFilt[f] > I_CONTACT_T) return true;
  }
  return false;
}

bool secureByCurrent() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    bool inBand = (iFilt[f] >= I_SECURE_MIN && iFilt[f] <= I_SECURE_MAX);
    bool stable = (fabsf(iSlope[f]) <= I_SLOPE_STABLE_T);
    if (inBand && stable) return true;
  }
  return false;
}

bool detectDisturbance() {
  return detectImpact();
}

bool loadIncreased() {
  bool above = false;
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (iFilt[f] > (iBaseline[f] + I_LOAD_DELTA_T) && fabsf(iSlope[f]) < I_SLOPE_STABLE_T) {
      above = true;
      break;
    }
  }

  if (above) {
    if (loadAboveMsStart == 0) loadAboveMsStart = millis();
    if (millis() - loadAboveMsStart >= LOAD_CONFIRM_MS) return true;
  } else {
    loadAboveMsStart = 0;
  }
  return false;
}

// ======================================================
// ================= FSM UTILITIES =======================
// ======================================================
const char* stateName(HandState s) {
  switch (s) {
    case STATE_IDLE:         return "IDLE";
    case STATE_CLOSING_FAST: return "CLOSING_FAST";
    case STATE_SETTLE:       return "SETTLE";
    case STATE_CLOSING_SLOW: return "CLOSING_SLOW";
    case STATE_HOLD:         return "HOLD";
    case STATE_TIGHTEN:      return "TIGHTEN";
    case STATE_RECOVER:      return "RECOVER";
    case STATE_OPEN:         return "OPEN";
    default:                 return "UNKNOWN";
  }
}

void enterState(HandState s) {
  state = s;
  stateEnterMs = millis();
  secureCounter = 0;
  loadAboveMsStart = 0;
}

void printStateIfChanged() {
  if (state != lastPrintedState) {
    lastPrintedState = state;
    Serial.print("[STATE] ");
    Serial.println(stateName(state));
  }
}

// ======================================================
// ====================== UI HELP ========================
// ======================================================
void printUiHelp() {
  Serial.println();
  Serial.println("[UI] Allowed commands ONLY:");
  Serial.println("     00000 - 11111 (5-bit pattern S0..S4)");
  Serial.println("     2      (Start FSM, only if pattern exists)");
  Serial.println("     3      (Reset/Open, then wait for new pattern)");
  Serial.println("[UI] S0=Pinky S1=Ring S2=Middle S3=Index S4=Thumb");
  Serial.print("> ");
}

// ======================================================
// ====================== PATTERN PARSE ==================
// ======================================================
bool parseShapePattern(const String& pat) {
  if (pat.length() != 5) return false;
  for (int i = 0; i < 5; i++) {
    char c = pat[i];
    if (c != '0' && c != '1') return false;
  }
  for (int i = 0; i < 5; i++) {
    fingerEnabled[i] = (pat[i] == '1');
  }
  return true;
}

bool isPattern5(const String& s) {
  if (s.length() != 5) return false;
  for (int i = 0; i < 5; i++) {
    if (s[i] != '0' && s[i] != '1') return false;
  }
  return true;
}

// ======================================================
// ========================== SETUP ======================
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== STM32 Black Pill: 5 Servo + 5 INA226 (PCA ch0..4) + 9 FSR Live + FSM Grasp ===");

  // -------- I2C SETUP --------
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  // -------- INIT INA226 SENSORS WITH CHECKS --------
  selectPCAChannel(0);
  if (!inaPresentOnCurrentBus()) { Serial.println("INA226 NOT FOUND on PCA channel 0. Halting."); while (1) {} }
  if (!ina226_0.init())          { Serial.println("INA226 init FAILED on PCA channel 0. Halting."); while (1) {} }

  selectPCAChannel(1);
  if (!inaPresentOnCurrentBus()) { Serial.println("INA226 NOT FOUND on PCA channel 1. Halting."); while (1) {} }
  if (!ina226_1.init())          { Serial.println("INA226 init FAILED on PCA channel 1. Halting."); while (1) {} }

  selectPCAChannel(2);
  if (!inaPresentOnCurrentBus()) { Serial.println("INA226 NOT FOUND on PCA channel 2. Halting."); while (1) {} }
  if (!ina226_2.init())          { Serial.println("INA226 init FAILED on PCA channel 2. Halting."); while (1) {} }

  selectPCAChannel(3);
  if (!inaPresentOnCurrentBus()) { Serial.println("INA226 NOT FOUND on PCA channel 3. Halting."); while (1) {} }
  if (!ina226_3.init())          { Serial.println("INA226 init FAILED on PCA channel 3. Halting."); while (1) {} }

  selectPCAChannel(4);
  if (!inaPresentOnCurrentBus()) { Serial.println("INA226 NOT FOUND on PCA channel 4. Halting."); while (1) {} }
  if (!ina226_4.init())          { Serial.println("INA226 init FAILED on PCA channel 4. Halting."); while (1) {} }

  // Optional: wait for conversions once after init
  selectPCAChannel(0); ina226_0.waitUntilConversionCompleted();
  selectPCAChannel(1); ina226_1.waitUntilConversionCompleted();
  selectPCAChannel(2); ina226_2.waitUntilConversionCompleted();
  selectPCAChannel(3); ina226_3.waitUntilConversionCompleted();
  selectPCAChannel(4); ina226_4.waitUntilConversionCompleted();

  // -------- SERVO SETUP --------
  Serial.println("Servos DISABLED at boot (not attached). They will attach/move only after you press Start (2) or Reset (3).");

  // -------- FSR SETUP --------
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fsrPins[i], INPUT_ANALOG);
  }

  Serial.print("FULL_SWEEP_TIME_SEC = ");
  Serial.print(FULL_SWEEP_TIME_SEC);
  Serial.print(" s, autoRampSpeedUsPerSec = ");
  Serial.print(autoRampSpeedUsPerSec);
  Serial.println(" us/s");

  hasPattern = false;
  lastPattern = "00000";
  enterState(STATE_IDLE);

  printUiHelp();
}

// ======================================================
// ================== SERIAL COMMAND PARSER ==============
// ======================================================
String cmdLine;

void handleCommandLine(const String& cmdIn) {
  String cmd = cmdIn;
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("Received: ");
  Serial.println(cmd);

  // -------- Command 3: reset/open (anytime) --------
  if (cmd == "3") {
    attachServosOnce();
    stopAllRamps();

    // open all fingers
    for (int f = 0; f < NUM_SERVOS; f++) fingerEnabled[f] = true;

    enterState(STATE_OPEN);
    startRampAllTo(SERVO_MAX_US, autoRampSpeedUsPerSec, false);

    hasPattern = false;
    lastPattern = "00000";

    Serial.println("[UI] Reset/Open started. Waiting for a new 5-bit pattern.");
    return;
  }

  // -------- 5-bit pattern: store (do not move) --------
  if (isPattern5(cmd)) {
    if (!parseShapePattern(cmd)) {
      Serial.println("[UI] ERROR: Invalid pattern (should never happen).");
      return;
    }

    hasPattern = true;
    lastPattern = cmd;

    Serial.print("[UI] Pattern stored: ");
    Serial.println(cmd);

    if (state != STATE_IDLE) {
      Serial.println("[UI] NOTE: Pattern stored, but system is not IDLE. Use '3' to reset/open, then '2' to start.");
    } else {
      Serial.println("[UI] Ready. Send '2' to start grasp.");
    }
    return;
  }

  // -------- Command 2: start FSM (only if pattern exists and IDLE) --------
  if (cmd == "2") {
    if (!hasPattern) {
      Serial.println("[UI] ERROR: No pattern set. Send 5-bit pattern (e.g., 11000) first.");
      return;
    }
    if (state != STATE_IDLE) {
      Serial.println("[UI] BUSY: FSM already running (or not IDLE). Use '3' to reset/open first.");
      return;
    }

    attachServosOnce();

    tightenBudgetUs = 250;
    lastTightenMs = 0;

    enterState(STATE_CLOSING_FAST);
    startRampAllTo(SERVO_MIN_US, autoRampSpeedUsPerSec, true);

    Serial.print("[UI] START grasp with pattern ");
    Serial.println(lastPattern);
    return;
  }

  // -------- Block everything else --------
  Serial.println("[UI] BLOCKED: Only allowed commands are: 5-bit pattern (00000..11111) OR '2' OR '3'.");
}

// ======================================================
// =========================== LOOP ======================
// ======================================================
void loop() {
  // ---- Serial line input ----
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLine.length() > 0) {
        handleCommandLine(cmdLine);
        cmdLine = "";
      }
    } else {
    // ✅ keep only valid command characters
    if (ch == '0' || ch == '1' || ch == '2' || ch == '3') {
      cmdLine += ch;
    } else {
      // ignore junk bytes (prevents "�11111")
    }

    if (cmdLine.length() > 60) cmdLine = "";
  }
}
  // ---- Update ramp (non-blocking) ----
  updateRamp();

  // ---- FSM control update @ 10ms ----
  uint32_t nowMs = millis();
  if (nowMs - lastControlMs >= CONTROL_PERIOD_MS) {
    uint32_t dtMs = nowMs - lastControlMs;
    lastControlMs = nowMs;

    float dtSec = dtMs / 1000.0f;
    if (dtSec < 0.001f) dtSec = 0.001f;

    updateSensorsFSM(dtSec);
    printStateIfChanged();

    switch (state) {
      case STATE_IDLE:
        // Waiting for pattern and '2'
        break;

      case STATE_CLOSING_FAST:
        rampSpeedUsPerSec = autoRampSpeedUsPerSec;
        if (detectImpact() || detectContact()) {
          enterState(STATE_SETTLE);
        }
        break;

      case STATE_SETTLE:
        rampSpeedUsPerSec = autoRampSpeedUsPerSec * 0.25f;
        if (millis() - stateEnterMs >= SETTLE_MS) {
          enterState(STATE_CLOSING_SLOW);
          startRampAllTo(SERVO_MIN_US, autoRampSpeedUsPerSec * 0.35f, true);
        }
        break;

      case STATE_CLOSING_SLOW:
        rampSpeedUsPerSec = autoRampSpeedUsPerSec * 0.35f;

        if (detectImpact()) {
          enterState(STATE_RECOVER);
          break;
        }

        if (secureByCurrent()) secureCounter++;
        else secureCounter = 0;

        if (secureCounter >= SECURE_STABLE_CYCLES) {
          for (int f = 0; f < NUM_SERVOS; f++) iBaseline[f] = iFilt[f];
          enterState(STATE_HOLD);
        }
        break;

      case STATE_HOLD:
        stopAllRamps(); // hold current servo pulses

        if (detectDisturbance()) {
          enterState(STATE_RECOVER);
          break;
        }

        if (loadIncreased()) {
          if (millis() - lastTightenMs >= TIGHTEN_COOLDOWN_MS) {
            enterState(STATE_TIGHTEN);
          }
        }
        break;

      case STATE_TIGHTEN: {
        if (tightenBudgetUs <= 0) {
          enterState(STATE_HOLD);
          break;
        }

        for (int f = 0; f < NUM_SERVOS; f++) {
          if (!fingerEnabled[f]) continue;
          int newPw = servoPulseUs[f] - TIGHTEN_STEP_US;
          if (newPw < SERVO_MIN_US) newPw = SERVO_MIN_US;
          servoPulseUs[f] = newPw;
        }

        tightenBudgetUs -= TIGHTEN_STEP_US;
        if (tightenBudgetUs < 0) tightenBudgetUs = 0;

        lastTightenMs = millis();

        applyServoPulses();

        for (int f = 0; f < NUM_SERVOS; f++) {
          iBaseline[f] = 0.98f * iBaseline[f] + 0.02f * iFilt[f];
        }

        enterState(STATE_HOLD);
        break;
      }

      case STATE_RECOVER: {
        stopAllRamps();

        for (int f = 0; f < NUM_SERVOS; f++) {
          if (!fingerEnabled[f]) continue;
          int newPw = servoPulseUs[f] + RECOVER_OPEN_US;
          if (newPw > SERVO_MAX_US) newPw = SERVO_MAX_US;
          servoPulseUs[f] = newPw;
        }
        applyServoPulses();

        if (millis() - stateEnterMs >= RECOVER_MS) {
          enterState(STATE_CLOSING_SLOW);
          startRampAllTo(SERVO_MIN_US, autoRampSpeedUsPerSec * 0.30f, true);
        }
        break;
      }

      case STATE_OPEN:
        rampSpeedUsPerSec = autoRampSpeedUsPerSec;
        if (!anyRampActive()) {
          for (int f = 0; f < NUM_SERVOS; f++) servoPulseUs[f] = SERVO_MAX_US;

          enterState(STATE_IDLE);
          Serial.println("[UI] Reset complete. Waiting for 5-bit pattern.");
          Serial.print("> ");
        }
        break;
    }
  }

  // ---- Periodic current print ----
  if (millis() - lastCurrentPrint >= CURRENT_PRINT_PERIOD_MS) {
    lastCurrentPrint = millis();
    printINA226Data();
  }

  // ---- Periodic FSR live print ----
  if (millis() - lastFSRPrint >= FSR_PRINT_PERIOD_MS) {
    lastFSRPrint = millis();
    printFSRLive();
  }
}
