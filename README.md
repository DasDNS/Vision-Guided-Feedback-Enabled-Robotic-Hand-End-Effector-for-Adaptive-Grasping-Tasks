# Robotic End Effector Control System

**STM32 + INA226 + FSR + ROS2 + PySide6 UI**

------------------------------------------------------------------------

## 📌 Overview

This project implements a **robotic hand / end effector control system**
with:

-   5 Servo-driven fingers
-   5 INA226 current sensors (via PCA9548A I2C multiplexer)
-   9 FSR (Force Sensitive Resistor) sensors
-   Finite State Machine (FSM) based grasp control
-   ROS 2 (Jazzy) pattern request/response integration
-   PySide6 desktop UI with live monitoring

The system supports:

-   Vision-based finger pattern selection (via ROS2)
-   Controlled multi-phase grasping
-   Current + force-based event detection
-   Safe reset from ANY state
-   Real-time visualization in a Python UI

------------------------------------------------------------------------

# 🧠 System Architecture

## Firmware (STM32)

The STM32 firmware handles:

-   Servo control (speed-based ramp, per finger)
-   Current sensing using INA226
-   FSR sensing with filtering + slope detection
-   FSM-based grasp logic
-   Serial command interface

## Desktop UI (Python + PySide6)

The UI provides:

-   Serial connection manager
-   ROS 2 integration
-   Pattern visualization
-   FSM state display
-   FSR live data window
-   Current data window
-   Debug + status log panels

------------------------------------------------------------------------

# 🔧 Hardware Components

  Component                    Quantity   Purpose
  ---------------------------- ---------- -------------------------------
  STM32 (BlackPill F401CE)     1          Main controller
  MG996R (or similar) Servos   5          Finger actuation
  INA226 current sensors       5          Per-servo current measurement
  PCA9548A I2C multiplexer     1          Multi-INA I2C routing
  FSR Sensors                  9          Contact detection
  External 5--6V supply        1          Servo power

------------------------------------------------------------------------

# ⚙️ Firmware Details

## Servo Control

-   Range: `500µs – 2400µs`
-   Full sweep time: `FULL_SWEEP_TIME_SEC = 8.0`
-   Speed automatically computed:

```{=html}
<!-- -->
```
    autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC

Each finger supports: - Independent enable/disable - Per-state speed
multipliers - Smooth ramp using speed-based control

------------------------------------------------------------------------

# 🧭 Finite State Machine (FSM)

States:

-   `IDLE`
-   `RESETTING`
-   `CLOSING_FAST`
-   `CLOSING_SLOW`
-   `TIGHTEN`
-   `HOLD`
-   `SETTLE`

## Grasp Sequence

1.  **CLOSING_FAST**
    -   Move enabled fingers to 2000µs
    -   Normal speed
2.  **CLOSING_SLOW**
    -   Move to 1400µs
    -   50% speed
    -   Monitor for:
        -   Huge FSR change → HOLD
        -   High current → HOLD
3.  **TIGHTEN**
    -   Ramp to 500µs
    -   30% speed
    -   Stops at full close
4.  **HOLD**
    -   Maintain position
    -   If contact disappears → TIGHTEN
5.  **SETTLE**
    -   Final state (no movement)

------------------------------------------------------------------------

# 📊 Sensor Processing

## Current Filtering

Low-pass filter:

    iFilt = iPrev + α * (iRaw - iPrev)

Slope detection enables event-based control.

## FSR Monitoring

Each FSR has: - Defined max range - Fraction-based huge-change detection

Threshold example:

    FSR_SLOW_HUGE_DELTA_FRAC = 0.30

------------------------------------------------------------------------

# 💻 Serial Command Interface

Allowed commands:

  Command         Meaning
  --------------- ----------------------------
  `00000–11111`   Finger activation pattern
  `2`             Start grasp
  `3`             Reset (works in ANY state)

Example workflow:

    11000
    2

------------------------------------------------------------------------

# 🖥 Python UI Features

## Connection Panel

-   Auto-detect STM32
-   Manual port selection
-   Safe disconnect handling

## ROS2 Integration

-   Topic: `finger_pattern_request`
-   Topic: `finger_pattern`
-   Request pattern from vision system
-   Status indicator shows ROS connectivity

## Data Panels

-   FSR Values (live)
-   Current Values (live)
-   MCU Status
-   Debug Log

## Safety Features

-   Command sanitization
-   Only allowed commands accepted
-   Pattern must be sent before grasp
-   Queue-based UI updates (prevents crash due to high data rate)

------------------------------------------------------------------------

# 🔄 Data Classification in UI

Incoming serial lines are classified as:

-   FSM state → displayed in large state box
-   `FSR Live:` → FSR panel
-   `millis,pulse,current...` → Current panel
-   Other → Status panel

------------------------------------------------------------------------

# 🚀 Running the Project

## 1️⃣ Flash STM32 Firmware

Use PlatformIO or Arduino framework for STM32.

Ensure: - SDA = PB9 - SCL = PB8 - All INA226 detected via PCA9548A

## 2️⃣ Install Python Dependencies

    pip install PySide6 pyserial rclpy

Source ROS 2 Jazzy before running:

    source /opt/ros/jazzy/setup.bash

## 3️⃣ Run UI

    python3 main.py

------------------------------------------------------------------------

# 🛡 Safety Considerations

-   Always power servos from external supply
-   Common ground required
-   Monitor current spikes
-   Avoid excessive serial print rates
-   Use UI queue batching to prevent UART overload

------------------------------------------------------------------------

# 🧪 Testing Workflow

1.  Connect STM32
2.  Request pattern via ROS
3.  Send pattern to MCU
4.  Press Start
5.  Observe FSM transitions
6.  Press Reset anytime if needed

------------------------------------------------------------------------

# 📈 Future Improvements

-   Closed-loop torque control
-   Adaptive grasp force tuning
-   Data logging to file
-   Real-time plotting
-   ROS2 action server integration
------------------------------------------------------------------------

**Author:** DasDNS
