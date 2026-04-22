# STM32 Hand Control System with Servo, INA226, and FSR Sensors

## 📌 Overview
This project implements a **multi-sensor embedded system** using an STM32 (Black Pill) board. It integrates:

- 5 Servo Motors (Finger actuation)
- 5 INA226 Current Sensors (via I2C Multiplexer)
- 9 FSR (Force Sensitive Resistor) Sensors
- Serial Command Interface for control

The system is designed for applications like:
- Robotic hand control
- Grip force analysis
- Embedded sensing and feedback systems

---

## 🧠 Features

### ✅ Servo Control
- Controls 5 servos simultaneously
- Uses microsecond precision (PWM)
- Supports incremental and preset positions
- Servos attach **only when needed** (safe startup)

### ✅ Current Monitoring (INA226)
- Reads current from 5 sensors
- Uses PCA9548A I2C multiplexer
- Detects I2C errors and halts on failure
- Outputs real-time current data

### ✅ FSR Sensor System
- Reads 9 analog FSR sensors
- Averages multiple samples for noise reduction
- Detects stability across readings
- Automatically saves stable snapshots

### ✅ Serial Command Interface
| Command | Action |
|--------|--------|
| 0 | Fully bent |
| 1 | Mid position |
| 2 | Fully straight |
| 3 | Step -200 µs |
| 4 | Step +200 µs |

---

## 🛠 Hardware Requirements

- STM32 Black Pill (or compatible)
- 5x Servo Motors
- 5x INA226 Current Sensors
- 1x PCA9548A I2C Multiplexer
- 9x FSR Sensors
- Power supply (adequate for servos)

---

## 🔌 Pin Configuration

### Servos
| Servo | Pin | Finger |
|------|-----|--------|
| S0 | PB13 | Pinky |
| S1 | PB14 | Ring |
| S2 | PB15 | Middle |
| S3 | PA8  | Index |
| S4 | PA11 | Thumb |

### I2C
- SDA → PB9
- SCL → PB8

### FSR Sensors
PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0

---

## ⚙️ System Behavior

### Startup
- Initializes I2C and INA sensors
- Verifies each sensor is connected
- Servos remain **disabled (safe state)**

### Runtime
- Periodically:
  - Prints INA current data
  - Reads FSR values
- Responds to serial commands for movement

### Safety Features
- Halts if INA sensor not detected
- Prevents sudden servo movement on boot
- Limits servo pulse range

---

## 📊 Output Format

### Current Data
```
time_ms,pulse_width,S0=xx mA, S1=xx mA, ...
```

### FSR Data
```
FSR Live: PA0=xxx, PA1=xxx, ...
```

### Stable Snapshot
```
STABLE → DATA SAVED
FSR Snapshot: ...
```

---

## 🚀 How to Use

1. Upload the code to STM32
2. Open Serial Monitor (115200 baud)
3. Send commands (0–4)
4. Observe:
   - Servo movement
   - Current readings
   - FSR data

---

## 🧩 Key Design Concepts

### I2C Multiplexing
Multiple INA226 sensors share the same address using PCA9548A.

### Noise Reduction
FSR readings are averaged across multiple samples.

### Stability Detection
Data is only saved when readings stabilize across cycles.

### Non-Blocking Loop
Uses `millis()` instead of delay for scheduling tasks.

---

## 📈 Possible Improvements

- Add PID control for force feedback
- Log data to SD card
- Add wireless communication (WiFi/Bluetooth)
- Implement GUI for visualization
- Add calibration routine for FSR sensors

---

## 🧑‍💻 Author Notes

This system is structured for:
- Scalability
- Safety
- Real-time embedded performance

---

## 📜 License
This project is open-source and free to use for educational and research purposes.
