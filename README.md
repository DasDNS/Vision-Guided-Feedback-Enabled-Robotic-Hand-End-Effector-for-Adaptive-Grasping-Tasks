# 🖐️ STM32 Robotic Hand Control System  
**Servo + Current Sensing + FSR Feedback + Python GUI**

---

## 📌 Overview
This project implements a robotic hand control system using STM32, INA226 current sensors, FSR sensors, and a Python GUI.

Features:
- Smooth servo control (non-blocking)
- Real-time current monitoring
- Real-time force sensing
- Serial-based control (0 = open, 1 = close)

---

## 🧱 System Architecture
Python GUI ↔ Serial ↔ STM32  
STM32 → Servos + INA226 (via PCA9548A) + FSR sensors

---

## ⚙️ Hardware
- STM32 Black Pill
- 5x Servo motors
- 5x INA226 current sensors
- PCA9548A I2C multiplexer
- 9x FSR sensors

---

## 🔌 Pin Mapping
Servos:
- PB13 → Pinky
- PB14 → Ring
- PB15 → Middle
- PA8  → Index
- PA11 → Thumb

I2C:
- SDA → PB9
- SCL → PB8

FSR:
PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0

---

## 🧠 Firmware Explanation

### Servo Control
- Uses pulse width (500–2400 µs)
- Smooth ramp motion using time-based updates

### Ramp System
- Non-blocking movement
- Each finger has independent speed
- Uses millis() instead of delay()

### I2C Multiplexer
- PCA9548A selects one INA226 at a time
- Allows multiple sensors with same address

### Current Sensing
- Reads current per finger
- Used for force/overload detection

### FSR Sensors
- Analog readings
- Provide pressure feedback

### Serial Commands
- '1' → close hand
- '0' → open hand

---

## 🖥️ Python GUI

### Features
- Serial connection manager
- Live FSR display
- Live current display
- Debug log

### Serial Handling
- Threaded reading (no UI freeze)
- Sends single-character commands

---

## ▶️ How to Run

### Flash STM32
Upload firmware using PlatformIO or Arduino IDE

### Install Python Dependencies
pip install PySide6 pyserial

### Run GUI
python3 main.py

---

## 📊 Output Format

Current:
millis,,Little=XX mA, Ring=XX mA ...

FSR:
FSR Live: Little=..., Ring=...

---

## ⚠️ Notes
- Servos disabled at startup
- External power recommended for servos
- Ensure correct I2C wiring

---

## 🔧 Future Improvements
- PID grip control
- Individual finger control
- Graph plotting in GUI
- ML-based grasp detection

---

## 📚 Concepts Covered
- Embedded systems
- I2C communication
- Real-time systems
- Sensor fusion
- GUI development
