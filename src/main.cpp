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
#define SERVO_STEP   300

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
int currentPulseWidth = 1450;
unsigned long lastPeriodicPrint = 0;

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

  delay(300); // keep exactly like your original
}

// ===============================
// MOVE ALL SERVOS (unchanged behavior)
// ===============================
void moveServoUS(int pulseWidth) {
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

  unsigned long start = millis();
  while (millis() - start < 1000) {
    printINA226Data(); // unchanged (includes delay(300))
  }
}

// ======================================================
// ====================== FSR SECTION ====================
// ======================================================

#define NUM_SENSORS 9

uint8_t fsrPins[NUM_SENSORS] = {
  PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
};

const char* fsrPinNames[NUM_SENSORS] = {
  "PB0", "PA7", "PA6", "PA5", "PA4", "PA3", "PA2", "PA1", "PA0"
};

const int samples = 80;
const int delay_ms = 3;
const float threshold = 15.0;
const int stableCyclesNeeded = 5;

float prevMean[NUM_SENSORS]  = {0};
float savedData[NUM_SENSORS] = {0};

int stableCounter = 0;
bool hasSaved = false;

// Non-blocking scheduler for FSR loop prints
unsigned long lastFSRRun = 0;
const unsigned long FSR_PERIOD_MS = 200;

// Run one full FSR update (same algorithm as your code)
void updateFSRSystem() {
  float meanVal[NUM_SENSORS] = {0};

  // ---- Take samples ----
  for (int i = 0; i < samples; i++) {
    for (int s = 0; s < NUM_SENSORS; s++) {
      meanVal[s] += analogRead(fsrPins[s]);
    }
    delay(delay_ms);
  }

  // ---- Compute mean ----
  for (int s = 0; s < NUM_SENSORS; s++) {
    meanVal[s] /= samples;
  }

  // ---- Stability detection ----
  bool allStable = true;
  for (int s = 0; s < NUM_SENSORS; s++) {
    float delta = fabs(meanVal[s] - prevMean[s]);
    if (delta > threshold) {
      allStable = false;
    }
  }

  if (allStable) {
    stableCounter++;
  } else {
    stableCounter = 0;
    hasSaved = false;
  }

  // ---- AUTO SAVE ----
  if (stableCounter >= stableCyclesNeeded && !hasSaved) {
    Serial.println("=====================================");
    Serial.println("STABLE → DATA SAVED");
    Serial.print("FSR Snapshot: ");

    for (int s = 0; s < NUM_SENSORS; s++) {
      savedData[s] = meanVal[s];
      Serial.print(fsrPinNames[s]);
      Serial.print("=");
      Serial.print(savedData[s], 2);
      if (s < NUM_SENSORS - 1) Serial.print(", ");
    }

    Serial.println();
    Serial.println("=====================================");
    hasSaved = true;
  }

  // ---- Print live readings ----
  Serial.print("FSR Live: ");
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.print(fsrPinNames[s]);
    Serial.print("=");
    Serial.print(meanVal[s], 2);
    if (s < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();

  // ---- Update last mean ----
  for (int s = 0; s < NUM_SENSORS; s++) {
    prevMean[s] = meanVal[s];
  }
}

// ======================================================
// ========================== SETUP ======================
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== STM32 Black Pill: 5 Servo + 5 INA226 (PCA ch0..4) + 9 FSR Auto-Save ===");

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
  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  Serial.println("Servos attached:");
  Serial.println("S0 PB13 (Pinky), S1 PB14 (Ring), S2 PB15 (Middle), S3 PA8 (Index), S4 PA11 (Thumb)");

  // -------- FSR SETUP --------
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(fsrPins[i], INPUT_ANALOG);
  }
  Serial.println("FSR Auto-Save System Ready...");

  Serial.println("\n📟 Serial Commands Reference");
  Serial.println("0 → Fully bent");
  Serial.println("1 → Mid position");
  Serial.println("2 → Fully straight");
  Serial.println("3 → Step −300 µs");
  Serial.println("4 → Step +300 µs");
  Serial.println("------------------------------");
  Serial.println("Enter command:");
}

// ======================================================
// =========================== LOOP ======================
// ======================================================
void loop() {
  // ---- Serial commands (UNCHANGED) ----
  if (Serial.available()) {
    char cmd = Serial.read();

    switch (cmd) {
      case '0': moveServoUS(SERVO_MIN_US); break;
      case '1': moveServoUS(1450); break;
      case '2': moveServoUS(SERVO_MAX_US); break;
      case '3': moveServoUS(currentPulseWidth - SERVO_STEP); break;
      case '4': moveServoUS(currentPulseWidth + SERVO_STEP); break;
      case '\n':
      case '\r':
        break;
      default:
        Serial.println("Invalid command. Use 0–4.");
        break;
    }

    Serial.println("Enter next command:");
  }

  // ---- Periodic INA print (UNCHANGED) ----
  if (millis() - lastPeriodicPrint >= 1000) {
    lastPeriodicPrint = millis();
    printINA226Data();
  }

  // ---- Periodic FSR update (added) ----
  // Note: updateFSRSystem() itself takes time (samples*delay_ms), same as your original code.
  if (millis() - lastFSRRun >= FSR_PERIOD_MS) {
    lastFSRRun = millis();
    updateFSRSystem();
  }
}
