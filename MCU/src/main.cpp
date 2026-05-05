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

// Servo pins (your mapping)
#define SERVO0_PIN PB13  // Pinky  (INA channel 0)
#define SERVO1_PIN PB14  // Ring   (INA channel 1)
#define SERVO2_PIN PB15  // Middle (INA channel 2)
#define SERVO3_PIN PA8   // Index  (INA channel 3)
#define SERVO4_PIN PA11  // Thumb  (INA channel 4)

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// ===============================
// PER-FINGER SWEEP TIMES (seconds)
// Change these 5 values to tune each finger speed.
// ===============================
#define SWEEP_TIME_S0_SEC 10.0f  // Pinky
#define SWEEP_TIME_S1_SEC 2.0f  // Ring
#define SWEEP_TIME_S2_SEC 10.0f  // Middle
#define SWEEP_TIME_S3_SEC 10.0f  // Index
#define SWEEP_TIME_S4_SEC 10.0f  // Thumb

// Choose print frequency
#define CURRENT_PRINT_PERIOD_MS 200
#define FSR_PRINT_PERIOD_MS     200

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
unsigned long lastCurrentPrint = 0;
unsigned long lastFSRPrint = 0;

bool servosEnabled = false;

// ===============================
// PER-FINGER RAMP STATE
// ===============================
static const int N_SERVOS = 5;

// Current pulse per finger (start at open)
int currentPulseWidth[N_SERVOS] = {
  SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US
};

// Ramp state per finger
bool  rampActive[N_SERVOS] = { false, false, false, false, false };
int   rampTargetUs[N_SERVOS] = { SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US, SERVO_MAX_US };
float rampPosUs[N_SERVOS] = {
  (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US, (float)SERVO_MAX_US
};
unsigned long lastRampUpdateMs[N_SERVOS] = { 0, 0, 0, 0, 0 };

// Auto speed per finger based on sweep time
float fullTravelUs = (float)(SERVO_MAX_US - SERVO_MIN_US); // 1900

float autoSpeedUsPerSec[N_SERVOS] = {
  fullTravelUs / SWEEP_TIME_S0_SEC,
  fullTravelUs / SWEEP_TIME_S1_SEC,
  fullTravelUs / SWEEP_TIME_S2_SEC,
  fullTravelUs / SWEEP_TIME_S3_SEC,
  fullTravelUs / SWEEP_TIME_S4_SEC
};

// ===============================
// SERVO ATTACH (no movement on boot)
// ===============================
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

// ===============================
// PCA9548A CHANNEL SELECT
// ===============================
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
    while (1) {} // halt
  }
}

// ======================================================
// APPLY PULSE TO ONE SERVO (clamped)
// ======================================================
static inline int clampPulse(int us) {
  if (us < SERVO_MIN_US) return SERVO_MIN_US;
  if (us > SERVO_MAX_US) return SERVO_MAX_US;
  return us;
}

void applyServoPulseUS(int idx, int pulseWidth) {
  pulseWidth = clampPulse(pulseWidth);
  currentPulseWidth[idx] = pulseWidth;

  // optional per-finger debug
  Serial.print("Moving S");
  Serial.print(idx);
  Serial.print(" (us): ");
  Serial.println(pulseWidth);

  switch (idx) {
    case 0: servo0.writeMicroseconds(pulseWidth); break;
    case 1: servo1.writeMicroseconds(pulseWidth); break;
    case 2: servo2.writeMicroseconds(pulseWidth); break;
    case 3: servo3.writeMicroseconds(pulseWidth); break;
    case 4: servo4.writeMicroseconds(pulseWidth); break;
  }
}

// ======================================================
// START RAMP FOR ALL FINGERS TO TARGET
// ======================================================
void startRampAllTo(int targetUs) {
  attachServosOnce();

  targetUs = clampPulse(targetUs);

  unsigned long now = millis();
  for (int i = 0; i < N_SERVOS; i++) {
    rampTargetUs[i] = targetUs;
    rampPosUs[i] = (float)currentPulseWidth[i]; // start from current pos
    rampActive[i] = true;
    lastRampUpdateMs[i] = now;
  }
}

// ======================================================
// UPDATE RAMPS (non-blocking) — each finger uses its own speed
// ======================================================
void updateRamps() {
  for (int i = 0; i < N_SERVOS; i++) {
    if (!rampActive[i]) continue;

    unsigned long now = millis();
    unsigned long dtMs = now - lastRampUpdateMs[i];
    if (dtMs == 0) continue;
    lastRampUpdateMs[i] = now;

    float dt = dtMs / 1000.0f;

    float diff = (float)rampTargetUs[i] - rampPosUs[i];
    float step = autoSpeedUsPerSec[i] * dt;
    if (step < 1.0f) step = 1.0f;

    if (fabsf(diff) <= step) {
      rampPosUs[i] = (float)rampTargetUs[i];
      applyServoPulseUS(i, (int)roundf(rampPosUs[i]));
      rampActive[i] = false;
      continue;
    }

    rampPosUs[i] += (diff > 0.0f) ? step : -step;
    applyServoPulseUS(i, (int)roundf(rampPosUs[i]));
  }
}

// ======================================================
// CURRENT PRINT (format must stay)
// We keep "pulse" as a single integer so your Python regex works.
// Here: print average pulse across 5 fingers.
// ======================================================
int getAveragePulse() {
  long sum = 0;
  for (int i = 0; i < N_SERVOS; i++) sum += currentPulseWidth[i];
  return (int)(sum / N_SERVOS);
}

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
  //Serial.print(currentPulseWidth);
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
// ====================== FSR SECTION ====================
// ======================================================

#define NUM_SENSORS 9

uint8_t fsrPins[NUM_SENSORS] = {
  PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

const char* fsrPinNames[NUM_SENSORS] = {
  "Little", "LittlePalm", "Ring", "RingPalm", "Middle", "Index", "IndexPalm", "Thumb", "ThumbPalm"
};

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
// ========================== SETUP ======================
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== STM32 Black Pill: 5 Servo + 5 INA226 (PCA ch0..4) + 9 FSR Live ===");

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

  selectPCAChannel(0); ina226_0.waitUntilConversionCompleted();
  selectPCAChannel(1); ina226_1.waitUntilConversionCompleted();
  selectPCAChannel(2); ina226_2.waitUntilConversionCompleted();
  selectPCAChannel(3); ina226_3.waitUntilConversionCompleted();
  selectPCAChannel(4); ina226_4.waitUntilConversionCompleted();

  // -------- SERVO SETUP --------
  Serial.println("Servos DISABLED at boot (not attached). They will attach/move only after you send a command.");
  Serial.println("S0 PB13 (Pinky), S1 PB14 (Ring), S2 PB15 (Middle), S3 PA8 (Index), S4 PA11 (Thumb)");

  // -------- FSR SETUP --------
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fsrPins[i], INPUT_ANALOG);
  }

  Serial.println("\n📟 Serial Commands Reference (ONLY 0 and 1)");
  Serial.println("1 → Ramp CLOSE (2400 → 500)  [each finger uses its own sweep time]");
  Serial.println("0 → Ramp OPEN  (500  → 2400) [each finger uses its own sweep time]");
  Serial.println("------------------------------");

  Serial.print("S0 sweep(s)="); Serial.print(SWEEP_TIME_S0_SEC);
  Serial.print("  S1 sweep(s)="); Serial.print(SWEEP_TIME_S1_SEC);
  Serial.print("  S2 sweep(s)="); Serial.print(SWEEP_TIME_S2_SEC);
  Serial.print("  S3 sweep(s)="); Serial.print(SWEEP_TIME_S3_SEC);
  Serial.print("  S4 sweep(s)="); Serial.println(SWEEP_TIME_S4_SEC);

  Serial.print("Auto speeds (us/s): ");
  Serial.print("S0="); Serial.print(autoSpeedUsPerSec[0]);
  Serial.print(", S1="); Serial.print(autoSpeedUsPerSec[1]);
  Serial.print(", S2="); Serial.print(autoSpeedUsPerSec[2]);
  Serial.print(", S3="); Serial.print(autoSpeedUsPerSec[3]);
  Serial.print(", S4="); Serial.println(autoSpeedUsPerSec[4]);

  Serial.println("Enter command:");
}

// ======================================================
// =========================== LOOP ======================
// ======================================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd != '\n' && cmd != '\r') {
      Serial.print("Received: ");
      Serial.println(cmd);
    }

    switch (cmd) {
      case '1':
        // CLOSE: ramp to MIN (each finger own speed)
        startRampAllTo(SERVO_MIN_US);
        break;

      case '0':
        // OPEN: ramp to MAX (each finger own speed)
        startRampAllTo(SERVO_MAX_US);
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.println("Invalid command. Use 0 or 1.");
        break;
    }

    Serial.println("Enter next command:");
  }

  // Update ramps (non-blocking)
  updateRamps();

  // Periodic current print (fixed format)
  if (millis() - lastCurrentPrint >= CURRENT_PRINT_PERIOD_MS) {
    lastCurrentPrint = millis();
    printINA226Data();
  }

  // Periodic FSR live print
  if (millis() - lastFSRPrint >= FSR_PRINT_PERIOD_MS) {
    lastFSRPrint = millis();
    printFSRLive();
  }
}

