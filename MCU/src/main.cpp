#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <INA226_WE.h>
#include <math.h>

// ======================================================
// ================== INA + SERVO SECTION ===============
// ======================================================

#define INA226_ADDRESS       0x40
#define PCA9548A_ADDRESS     0x70

// Servo pins
#define SERVO0_PIN PB13  // Pinky  (INA channel 0)
#define SERVO1_PIN PB14  // Ring   (INA channel 1)
#define SERVO2_PIN PB15  // Middle (INA channel 2)
#define SERVO3_PIN PA8   // Index  (INA channel 3)
#define SERVO4_PIN PA11  // Thumb  (INA channel 4)

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

#define FULL_SWEEP_TIME_SEC 8.0f

#define CURRENT_PRINT_PERIOD_MS 200
#define FSR_PRINT_PERIOD_MS 200

// Targets (requested)
#define FAST_TARGET_US   1800
#define SLOW_TARGET_US    1000

Servo servo0, servo1, servo2, servo3, servo4;

INA226_WE ina226_0(INA226_ADDRESS);
INA226_WE ina226_1(INA226_ADDRESS);
INA226_WE ina226_2(INA226_ADDRESS);
INA226_WE ina226_3(INA226_ADDRESS);
INA226_WE ina226_4(INA226_ADDRESS);

// Kept for the current print format (millis,pulse,...)
int currentPulseWidth = SERVO_MAX_US;

unsigned long lastCurrentPrint = 0;
unsigned long lastFSRPrint = 0;

bool servosEnabled = false;

// ======================================================
// ====================== FSR SECTION ====================
// ======================================================

#define NUM_SENSORS 9
uint8_t fsrPins[NUM_SENSORS] = { PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0 };

const char* fsrPinNames[NUM_SENSORS] = {
  "Little", "LittlePalm", "Ring", "RingPalm", "Middle", "Index", "IndexPalm", "Thumb", "ThumbPalm"
};

void printFSRLive() {
  Serial.print(millis());
  Serial.print(",");
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

enum HandState {
  STATE_IDLE,
  STATE_CLOSING_FAST,
  STATE_CLOSING_SLOW,
  STATE_TIGHTEN,
  STATE_HOLD,
  STATE_SETTLE,
  STATE_RESETTING  
};

HandState state = STATE_IDLE;
HandState lastPrintedState = (HandState)255;

const uint32_t CONTROL_PERIOD_MS = 10;
uint32_t lastControlMs = 0;

const int NUM_SERVOS = 5;
bool fingerEnabled[NUM_SERVOS] = { false, false, false, false, false };

// Global per-finger speed multipliers (can be changed per-state)
// Order: 0=Pinky, 1=Ring, 2=Middle, 3=Index, 4=Thumb
float speedMul[NUM_SERVOS] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

bool hasPattern = false;
String lastPattern = "00000";

volatile bool resetRequested = false;

// command-once flags for motion states
bool fastCommanded    = false;
bool slowCommanded    = false;
bool tightenCommanded = false;

// ======================================================
// =============== SPEED-BASED RAMP (PER FINGER) =========
// ======================================================

float fullTravelUs = (float)(SERVO_MAX_US - SERVO_MIN_US);   // 1900
float autoRampSpeedUsPerSec = fullTravelUs / FULL_SWEEP_TIME_SEC;

int servoPulseUs[NUM_SERVOS] = { SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US };

bool  rampActive[NUM_SERVOS]   = { false, false, false, false, false };
int   rampTargetUs[NUM_SERVOS] = { SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US };
float rampPosUs[NUM_SERVOS]    = { (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US };

float rampSpeedUsPerSec_f[NUM_SERVOS] = {0, 0, 0, 0, 0};
unsigned long lastRampUpdateMs = 0;

// ======================================================
// ===================== FILTERING + SLOPE ===============
// ======================================================

const float ALPHA_I   = 0.20f;
const float ALPHA_FSR = 0.20f;

float iRaw[NUM_SERVOS]   = {0};
float iFilt[NUM_SERVOS]  = {0};
float iPrev[NUM_SERVOS]  = {0};
float iSlope[NUM_SERVOS] = {0};

float fsrRaw[NUM_SENSORS]   = {0};
float fsrFilt[NUM_SENSORS]  = {0};
float fsrPrev[NUM_SENSORS]  = {0};
float fsrSlope[NUM_SENSORS] = {0};

// ======================================================
// ====================== THRESHOLDS =====================
// ======================================================

// huge FSR change only in CLOSING_SLOW
// Per-sensor max range (based on your info)
const float fsrMaxRange[NUM_SENSORS] = {
  300, // PB0
  600, // PA7
  300, // PA6
  600, // PA5
  300, // PA4
  300, // PA3
  600, // PA2
  300, // PA1
  600  // PA0
};

// Huge change threshold as a fraction of that sensor’s range
const float FSR_SLOW_HUGE_DELTA_FRAC = 0.30f;  // start with 0.25~0.35

float fsrSlowBase[NUM_SENSORS] = {0};

// current high after closing slow => tighten
const float I_TIGHTEN_HIGH_MA = 800.0f;

// ======================================================
// ================= HOLD RELEASE RULE ===================
// ======================================================

// If we're in HOLD, and contact disappears:
// - FSR drops below 20% of range
// - Current drops below 100 mA
// then go to TIGHTEN again.
const float HOLD_RELEASE_FSR_FRAC = 0.20f;
const float HOLD_RELEASE_I_MA     = 100.0f;

// Debounce: must stay low for this long to avoid noise flipping states
const uint32_t HOLD_RELEASE_DEBOUNCE_MS = 150;
uint32_t holdReleaseLowStartMs = 0;

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

bool inaPresentOnCurrentBus() {
  Wire.beginTransmission(INA226_ADDRESS);
  return (Wire.endTransmission() == 0);
}

void checkForI2cErrors(INA226_WE &sensor) {
  byte errorCode = sensor.getI2cErrorCode();
  if (errorCode) {
    Serial.print("I2C error: ");
    Serial.println(errorCode);
    while (1) {}
  }
}

void readINA226Currents(float out_mA[NUM_SERVOS]) {
  selectPCAChannel(0); out_mA[0] = ina226_0.getCurrent_mA(); checkForI2cErrors(ina226_0);
  selectPCAChannel(1); out_mA[1] = ina226_1.getCurrent_mA(); checkForI2cErrors(ina226_1);
  selectPCAChannel(2); out_mA[2] = ina226_2.getCurrent_mA(); checkForI2cErrors(ina226_2);
  selectPCAChannel(3); out_mA[3] = ina226_3.getCurrent_mA(); checkForI2cErrors(ina226_3);
  selectPCAChannel(4); out_mA[4] = ina226_4.getCurrent_mA(); checkForI2cErrors(ina226_4);
}

void printINA226Data() {
  float c0, c1, c2, c3, c4;
  selectPCAChannel(0); c0 = ina226_0.getCurrent_mA(); checkForI2cErrors(ina226_0);
  selectPCAChannel(1); c1 = ina226_1.getCurrent_mA(); checkForI2cErrors(ina226_1);
  selectPCAChannel(2); c2 = ina226_2.getCurrent_mA(); checkForI2cErrors(ina226_2);
  selectPCAChannel(3); c3 = ina226_3.getCurrent_mA(); checkForI2cErrors(ina226_3);
  selectPCAChannel(4); c4 = ina226_4.getCurrent_mA(); checkForI2cErrors(ina226_4);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(currentPulseWidth);
  Serial.print(",");
  Serial.print("Little="); Serial.print(c0); Serial.print(" mA, ");
  Serial.print("Ring=");   Serial.print(c1); Serial.print(" mA, ");
  Serial.print("Middle="); Serial.print(c2); Serial.print(" mA, ");
  Serial.print("Index=");  Serial.print(c3); Serial.print(" mA, ");
  Serial.print("Thumb=");  Serial.print(c4); Serial.println(" mA");
}

void applyServoPulses() {
  attachServosOnce();

  for (int f = 0; f < NUM_SERVOS; f++) {
    int pw = servoPulseUs[f];
    if (pw < SERVO_MIN_US) pw = SERVO_MIN_US;
    if (pw > SERVO_MAX_US) pw = SERVO_MAX_US;

    // disabled finger always open/spread
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
void startRampAllToUsingGlobalMul(int targetUs, float baseSpeedUsPerSec, bool onlyEnabled) {
  attachServosOnce();

  if (targetUs < SERVO_MIN_US) targetUs = SERVO_MIN_US;
  if (targetUs > SERVO_MAX_US) targetUs = SERVO_MAX_US;
  if (baseSpeedUsPerSec < 1.0f) baseSpeedUsPerSec = 1.0f;

  for (int f = 0; f < NUM_SERVOS; f++) {
    int tgt = targetUs;
    if (onlyEnabled && !fingerEnabled[f]) tgt = SERVO_MAX_US;

    rampTargetUs[f] = tgt;
    rampPosUs[f]    = (float)servoPulseUs[f];
    rampActive[f]   = true;

    float scaled = baseSpeedUsPerSec * speedMul[f];
    if (scaled < 1.0f) scaled = 1.0f;
    rampSpeedUsPerSec_f[f] = scaled;
  }

  lastRampUpdateMs = millis();
}


bool anyRampActive() {
  for (int f = 0; f < NUM_SERVOS; f++) if (rampActive[f]) return true;
  return false;
}

bool anyEnabledRampActive() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
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

  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!rampActive[f]) continue;

    float step = rampSpeedUsPerSec_f[f] * dt;
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
// ================= EVENT DETECTION (NEW) ===============
// ======================================================
bool hugeFsrChangeDuringSlow() {
  for (int s = 0; s < NUM_SENSORS; s++) {
    float delta = fabsf(fsrFilt[s] - fsrSlowBase[s]);
    float frac  = delta / fsrMaxRange[s];
    if (frac > FSR_SLOW_HUGE_DELTA_FRAC) return true;
  }
  return false;
}

bool currentHighSuggestHold() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (iFilt[f] >= I_TIGHTEN_HIGH_MA) return true;
  }
  return false;
}

bool fsrLowEnoughForRelease() {
  for (int s = 0; s < NUM_SENSORS; s++) {
    float frac = fsrFilt[s] / fsrMaxRange[s];
    if (frac > HOLD_RELEASE_FSR_FRAC) return false;
  }
  return true;
}

bool currentLowEnoughForRelease() {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (iFilt[f] > HOLD_RELEASE_I_MA) return false;
  }
  return true;
}

bool enabledReachedTargetUs(int targetUs, int tolUs = 5) {
  for (int f = 0; f < NUM_SERVOS; f++) {
    if (!fingerEnabled[f]) continue;
    if (abs(servoPulseUs[f] - targetUs) > tolUs) return false;
  }
  return true;
}

// ======================================================
// ================= FSM UTILITIES =======================
// ======================================================
const char* stateName(HandState s) {
  switch (s) {
    case STATE_IDLE:         return "IDLE";
    case STATE_CLOSING_FAST: return "CLOSING_FAST";
    case STATE_CLOSING_SLOW: return "CLOSING_SLOW";
    case STATE_TIGHTEN:      return "TIGHTEN";
    case STATE_HOLD:         return "HOLD";
    case STATE_SETTLE:       return "SETTLE";
    case STATE_RESETTING:    return "RESETTING";
    default:                 return "UNKNOWN";
  }
}

void enterState(HandState s) {
  state = s;

  if (s == STATE_HOLD) {
    holdReleaseLowStartMs = 0;
  }

  // reset command flags on every state entry
  fastCommanded    = false;
  slowCommanded    = false;
  tightenCommanded = false;

  // snapshot baseline for slow-phase FSR rule
  if (s == STATE_CLOSING_SLOW) {
    for (int i = 0; i < NUM_SENSORS; i++) fsrSlowBase[i] = fsrFilt[i];
  }

  // print immediately on transition
  Serial.print("[STATE] ");
  Serial.println(stateName(state));

  // keep consistency with printStateIfChanged()
  lastPrintedState = state;
}

// ======================================================
// ====================== RESET (RAMP OPEN) ==============
// ======================================================
void startResettingNow() {
  resetRequested = false;

  attachServosOnce();
  stopAllRamps();

  // Clear pattern so user can send a new one after reset completes
  hasPattern = false;
  lastPattern = "00000";

  // Open all fingers (ignore last enabled selection during reset)
  for (int f = 0; f < NUM_SERVOS; f++) fingerEnabled[f] = true;

  enterState(STATE_RESETTING);

  // Ramp open at NORMAL speed
  // Reset: usually keep all at 1.0 (same speed)
  speedMul[0]=1.0f; speedMul[1]=1.0f; speedMul[2]=1.0f; speedMul[3]=1.0f; speedMul[4]=1.0f;
  startRampAllToUsingGlobalMul(SERVO_MAX_US, autoRampSpeedUsPerSec, false);

  Serial.println("[UI] RESET: Opening/spreading at normal speed...");
}

// ======================================================
// ====================== GRASP START ====================
// ======================================================
void startGraspNow() {
  attachServosOnce();

  enterState(STATE_CLOSING_FAST);

  Serial.print("[UI] START grasp with pattern ");
  Serial.println(lastPattern);
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
  for (int i = 0; i < 5; i++) fingerEnabled[i] = (pat[i] == '1');
  return true;
}

bool isPattern5(const String& s) {
  if (s.length() != 5) return false;
  for (int i = 0; i < 5; i++) if (s[i] != '0' && s[i] != '1') return false;
  return true;
}

// ======================================================
// ========================== SETUP ======================
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // DO NOT change I2C config
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  // INA init checks
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

  // FSR pins
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(fsrPins[i], INPUT_ANALOG);

  // Start opened/spread
  attachServosOnce();
  for (int f = 0; f < NUM_SERVOS; f++) {
    fingerEnabled[f] = true;
    servoPulseUs[f] = SERVO_MAX_US;
  }
  applyServoPulses();

  hasPattern = false;
  lastPattern = "00000";
  enterState(STATE_IDLE);

  Serial.print("FULL_SWEEP_TIME_SEC = ");
  Serial.print(FULL_SWEEP_TIME_SEC);
  Serial.print(" s, autoRampSpeedUsPerSec = ");
  Serial.print(autoRampSpeedUsPerSec);
  Serial.println(" us/s");

  Serial.println("[UI] Send pattern (00000..11111), then '2' to start. Send '3' to reset (works in ANY state).");
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

  if (cmd == "3") {
    resetRequested = true;
    Serial.println("[UI] Reset requested.");
    return;
  }

  if (isPattern5(cmd)) {
    if (!parseShapePattern(cmd)) {
      Serial.println("[UI] ERROR: Invalid pattern.");
      return;
    }
    hasPattern = true;
    lastPattern = cmd;

    Serial.print("[UI] Pattern stored: ");
    Serial.println(cmd);

    if (state == STATE_IDLE) Serial.println("[UI] Ready. Send '2' to start grasp.");
    else Serial.println("[UI] Pattern stored (hand busy). Press '3' to reset if you want to restart.");
    return;
  }

  if (cmd == "2") {
    if (!hasPattern) {
      Serial.println("[UI] ERROR: No pattern set. Send 5-bit pattern first.");
      return;
    }
    if (state != STATE_IDLE) {
      Serial.println("[UI] BUSY: Hand is not IDLE. Press '3' to reset.");
      return;
    }
    startGraspNow();
    return;
  }

  Serial.println("[UI] BLOCKED: Only allowed commands are pattern OR '2' OR '3'.");
}

// ======================================================
// =========================== LOOP ======================
// ======================================================
void loop() {
  // Serial line input
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLine.length() > 0) {
        handleCommandLine(cmdLine);
        cmdLine = "";
      }
    } else {
      if (ch == '0' || ch == '1' || ch == '2' || ch == '3') cmdLine += ch;
      if (cmdLine.length() > 60) cmdLine = "";
    }
  }

  // Reset works anywhere (but now ramps open at normal speed)
  if (resetRequested) {
    startResettingNow();
  }

  // update ramps continuously
  updateRamp();

  // FSM tick
  uint32_t nowMs = millis();
  if (nowMs - lastControlMs >= CONTROL_PERIOD_MS) {
    uint32_t dtMs = nowMs - lastControlMs;
    lastControlMs = nowMs;

    float dtSec = dtMs / 1000.0f;
    if (dtSec < 0.001f) dtSec = 0.001f;

    updateSensorsFSM(dtSec);

    switch (state) {
      case STATE_IDLE:
        // keep open/spread while idle (no ramps)
        break;

      case STATE_RESETTING: {
        // When opening ramp completes -> IDLE
        if (!anyRampActive()) {

          stopAllRamps();

          // Force exact open position (eliminates rounding/tolerance issues)
          for (int f = 0; f < NUM_SERVOS; f++) {
            servoPulseUs[f] = SERVO_MAX_US;
          }

          applyServoPulses();

          enterState(STATE_IDLE);
          Serial.println("[UI] Waiting for 5-bit pattern.");
        }
        break;
      }


      case STATE_CLOSING_FAST: {
        // Move to 1800 @ normal speed (only once)
        if (!fastCommanded) {
          // FAST multipliers
          speedMul[0] = 1.00f; // Pinky
          speedMul[1] = 1.50f; // Ring
          speedMul[2] = 0.50f; // Middle 
          speedMul[3] = 0.50f; // Index
          speedMul[4] = 1.00f; // Thumb

          startRampAllToUsingGlobalMul(FAST_TARGET_US, autoRampSpeedUsPerSec, true);
          fastCommanded = true;
        }

        // When enabled fingers finish reaching 1800 -> go to slow phase
        if (!anyEnabledRampActive() && enabledReachedTargetUs(FAST_TARGET_US, 8)) {
          enterState(STATE_CLOSING_SLOW);
        }
        break;
      }

      case STATE_CLOSING_SLOW: {
        // Move 1800 -> 1000 @ half speed (only once)
        if (!slowCommanded) {
          // SLOW multipliers 
          speedMul[0] = 1.00f; // Pinky
          speedMul[1] = 1.50f; // Ring
          speedMul[2] = 0.50f; // Middle 
          speedMul[3] = 0.50f; // Index
          speedMul[4] = 1.00f; // Thumb

          startRampAllToUsingGlobalMul(SLOW_TARGET_US, autoRampSpeedUsPerSec * 0.50f, true);
          slowCommanded = true;
        }

        // If huge FSR change happens during slow -> HOLD
        if (hugeFsrChangeDuringSlow()) {
          enterState(STATE_HOLD);
          Serial.println("[FSM] Huge FSR change detected in CLOSING_SLOW -> HOLD");
          stopAllRamps(); 
          break;
        }
        // If current goes high DURING slow ramp -> HOLD immediately
        if (anyEnabledRampActive() && currentHighSuggestHold()) {                // stop slow motion
          enterState(STATE_HOLD);     
          Serial.println("[FSM] Current high during CLOSING_SLOW -> HOLD");
          stopAllRamps(); 
          break;
        }

        // When slow ramp finishes:
        if (!anyEnabledRampActive() && enabledReachedTargetUs(SLOW_TARGET_US, 8)) {
            enterState(STATE_TIGHTEN);
            Serial.println("[FSM] CLOSING_SLOW complete -> TIGHTEN");
          }
        }
        break;

      case STATE_TIGHTEN: {
        // Tighten = ramp enabled fingers toward 500 @ 30% speed (only once)
        if (!tightenCommanded) {
          speedMul[0] = 1.00f;
          speedMul[1] = 1.00f;
          speedMul[2] = 1.00f; 
          speedMul[3] = 1.00f;
          speedMul[4] = 1.00f;

          startRampAllToUsingGlobalMul(SERVO_MIN_US, autoRampSpeedUsPerSec * 0.30f, true);
          tightenCommanded = true;
          Serial.println("[FSM] TIGHTEN started: ramp -> 500us @ 30% speed");
        }

        // If huge FSR change happens during slow -> HOLD
        if (hugeFsrChangeDuringSlow()) {
          enterState(STATE_HOLD);
          Serial.println("[FSM] Huge FSR change detected in TIGHTEN -> HOLD");
          stopAllRamps(); 
          break;
        }
        // If current goes high DURING slow ramp -> HOLD immediately
        if (anyEnabledRampActive() && currentHighSuggestHold()) {                // stop slow motion
          enterState(STATE_HOLD);     
          Serial.println("[FSM] Current high during TIGHTEN -> HOLD");
          stopAllRamps(); 
          break;
        }

        // If enabled fingers reached 500 -> SETTLE
        if (!anyEnabledRampActive() && enabledReachedTargetUs(SERVO_MIN_US, 8)) {
          stopAllRamps();
          enterState(STATE_SETTLE);
          Serial.println("[FSM] Reached 500us -> SETTLE (no movement)");
        }
        break;
      }

      case STATE_HOLD: {
        stopAllRamps(); // maintain grasp

        bool fsrLow = fsrLowEnoughForRelease();
        bool iLow   = currentLowEnoughForRelease();

        if (fsrLow && iLow) {
          if (holdReleaseLowStartMs == 0) {
            holdReleaseLowStartMs = millis();
          }

          if (millis() - holdReleaseLowStartMs >= HOLD_RELEASE_DEBOUNCE_MS) {
            enterState(STATE_TIGHTEN);
            Serial.println("[FSM] HOLD: contact low (FSR<10% + I<100mA) -> TIGHTEN");
            // tightenCommanded will reset because enterState() resets flags
          }
        } else {
          holdReleaseLowStartMs = 0; // not consistently low -> stay HOLD
        }

        break;
      }

      case STATE_SETTLE:
        stopAllRamps();
        break;
    }
  }

  // Periodic prints (unchanged format)
  if (millis() - lastCurrentPrint >= CURRENT_PRINT_PERIOD_MS) {
    lastCurrentPrint = millis();
    printINA226Data();
  }

  if (millis() - lastFSRPrint >= FSR_PRINT_PERIOD_MS) {
    lastFSRPrint = millis();
    printFSRLive();
  }
}
