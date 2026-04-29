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

// NEW — desired full travel time (2400 ↔ 500)
#define FULL_SWEEP_TIME_SEC 8.0f

// Command 3/4 step size (instant)
#define SERVO_STEP   10

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

// No initial servo movement: this is just a software variable
int currentPulseWidth = SERVO_MAX_US;

unsigned long lastCurrentPrint = 0;
unsigned long lastFSRPrint = 0;

// Servo attach-on-demand flag + helper
bool servosEnabled = false;

// ===============================
// SPEED-BASED RAMP STATE
// ===============================
bool rampActive = false;
int  rampTargetUs = SERVO_MAX_US;

// Current position stored as float for smooth dt integration
float rampPosUs = (float)SERVO_MAX_US;

// timing for dt
unsigned long lastRampUpdateMs = 0;

// ===============================
// AUTO SPEED CALCULATION (set once)
// ===============================
float fullTravelUs = (float)(SERVO_MAX_US - SERVO_MIN_US);   // 1900
float autoRampSpeedUsPerSec = fullTravelUs / FULL_SWEEP_TIME_SEC;

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

// ===============================
// PRINT ALL CURRENTS (channels 0..4)
// Format MUST stay:
// millis,pulse,S0=.. mA, S1=.. mA, S2=.. mA, S3=.. mA, S4=.. mA
// ===============================
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
  Serial.print("S0=");
  Serial.print(c0);
  Serial.print(" mA, ");
  Serial.print("S1=");
  Serial.print(c1);
  Serial.print(" mA, ");
  Serial.print("S2=");
  Serial.print(c2);
  Serial.print(" mA, ");
  Serial.print("S3=");
  Serial.print(c3);
  Serial.print(" mA, ");
  Serial.print("S4=");
  Serial.print(c4);
  Serial.println(" mA");
}

// ===============================
// APPLY SERVO PULSE (clamped)
// ===============================
void applyServoPulseUS(int pulseWidth) {
  if (pulseWidth < SERVO_MIN_US) pulseWidth = SERVO_MIN_US;
  if (pulseWidth > SERVO_MAX_US) pulseWidth = SERVO_MAX_US;

  currentPulseWidth = pulseWidth;

  Serial.print("Moving servos (us): ");
  Serial.println(currentPulseWidth);

  servo0.writeMicroseconds(currentPulseWidth);
  servo1.writeMicroseconds(currentPulseWidth);
  servo2.writeMicroseconds(currentPulseWidth);
  servo3.writeMicroseconds(currentPulseWidth);
  servo4.writeMicroseconds(currentPulseWidth);
}

// ===============================
// INSTANT MOVE (stop ramp)
// ===============================
void moveServoUS(int pulseWidth) {
  attachServosOnce();
  rampActive = false; // stop any ramp

  if (pulseWidth < SERVO_MIN_US) pulseWidth = SERVO_MIN_US;
  if (pulseWidth > SERVO_MAX_US) pulseWidth = SERVO_MAX_US;

  rampPosUs = (float)pulseWidth; // keep ramp position synced
  applyServoPulseUS((int)roundf(rampPosUs));
}

// ===============================
// START SPEED-BASED RAMP
// ===============================
void startRampTo(int targetUs, float speedUsPerSec) {
  attachServosOnce();

  if (targetUs < SERVO_MIN_US) targetUs = SERVO_MIN_US;
  if (targetUs > SERVO_MAX_US) targetUs = SERVO_MAX_US;

  if (speedUsPerSec < 1.0f) speedUsPerSec = 1.0f;

  rampTargetUs = targetUs;

  // start from current position
  rampPosUs = (float)currentPulseWidth;

  rampActive = true;
  lastRampUpdateMs = millis();
}

// ===============================
// UPDATE SPEED-BASED RAMP (call from loop)
// ===============================
void updateRamp() {
  if (!rampActive) return;

  unsigned long now = millis();
  unsigned long dtMs = now - lastRampUpdateMs;
  if (dtMs == 0) return;
  lastRampUpdateMs = now;

  float dt = dtMs / 1000.0f; // seconds

  float diff = (float)rampTargetUs - rampPosUs;
  float step = autoRampSpeedUsPerSec * dt; // <-- auto speed based on FULL_SWEEP_TIME_SEC

  if (fabsf(diff) <= step) {
    rampPosUs = (float)rampTargetUs;
    applyServoPulseUS((int)roundf(rampPosUs));
    rampActive = false;
    return;
  }

  rampPosUs += (diff > 0.0f) ? step : -step;
  applyServoPulseUS((int)roundf(rampPosUs));
}

// ======================================================
// ====================== FSR SECTION ====================
// ======================================================

#define NUM_SENSORS 9

uint8_t fsrPins[NUM_SENSORS] = {
  PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

// Pin names for printing
const char* fsrPinNames[NUM_SENSORS] = {
  "PB0", "PA7", "PA6", "PA5", "PA4", "PA3", "PA2", "PA1", "PA0"
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

  // Optional: wait for conversions once after init
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

  Serial.println("\n📟 Serial Commands Reference");
  Serial.println("0 → Fully bent (ramp from 2400 to 500)");
  Serial.println("1 → Fully straight (ramp from 500 to 2400)");
  Serial.println("2 → Fully straight (instant)");
  Serial.println("3 → Step −10 µs (instant)");
  Serial.println("4 → Step +10 µs (instant)");
  Serial.println("------------------------------");
  Serial.print("FULL_SWEEP_TIME_SEC = ");
  Serial.print(FULL_SWEEP_TIME_SEC);
  Serial.print(" s, autoRampSpeedUsPerSec = ");
  Serial.print(autoRampSpeedUsPerSec);
  Serial.println(" us/s");
  Serial.println("Enter command:");
}

// ======================================================
// =========================== LOOP ======================
// ======================================================
void loop() {
  // ---- Serial commands ----
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd != '\n' && cmd != '\r') {
      Serial.print("Received: ");
      Serial.println(cmd);
    }

    switch (cmd) {
      case '0':
        // ramp down using auto speed
        startRampTo(SERVO_MIN_US, autoRampSpeedUsPerSec);
        break;

      case '1':
        // ramp up using auto speed
        startRampTo(SERVO_MAX_US, autoRampSpeedUsPerSec);
        break;

      case '2':
        moveServoUS(SERVO_MAX_US);
        break;

      case '3':
        moveServoUS(currentPulseWidth - SERVO_STEP);
        break;

      case '4':
        moveServoUS(currentPulseWidth + SERVO_STEP);
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.println("Invalid command. Use 0–4.");
        break;
    }

    Serial.println("Enter next command:");
  }

  // ---- Update speed-based ramp (non-blocking) ----
  updateRamp();

  // ---- Periodic current print (always, fixed format for your Python parser) ----
  if (millis() - lastCurrentPrint >= CURRENT_PRINT_PERIOD_MS) {
    lastCurrentPrint = millis();
    printINA226Data();
  }

  // ---- Periodic FSR live print (always, same "FSR Live:" format) ----
  if (millis() - lastFSRPrint >= FSR_PRINT_PERIOD_MS) {
    lastFSRPrint = millis();
    printFSRLive();
  }
}