# Robotic End Effector Control System  
**STM32 + FSR + INA226 + ROS 2 Jazzy + PySide6 UI**

---

## 📌 Overview

This project implements a **sensor-driven robotic end effector (prosthetic hand)** with:

- 5 Servo Motors (individual finger control)
- 5 × INA226 Current Sensors (via PCA9548A I2C multiplexer)
- 9 FSR Sensors (force feedback)
- Finite State Machine (FSM) grasp control
- Python GUI (PySide6)
- ROS 2 Jazzy integration for pattern input

The system performs event-driven grasping using:

Vision Pattern → FSM → Speed-Based Ramp Controller → Servos  
                        ↑  
                 FSR + Current Feedback

---

# 🏗 System Architecture

## MCU (STM32)
- Per-finger speed-based ramp control
- FSM-based grasp logic
- Real-time FSR filtering
- Current monitoring via INA226
- Pattern-based finger activation
- Reset available in ANY state

## Python UI
- Serial communication with STM32
- ROS 2 communication with vision system
- Pattern request + ACK
- Display of:
  - Active fingers
  - FSM state
  - FSR values
  - Current values
  - MCU status
  - Debug logs

## ROS 2
- Vision system publishes 5-bit pattern
- UI requests pattern
- UI sends ACK after receiving pattern

---

# 🔧 Hardware Requirements

- STM32 (BlackPill F401CE)
- 5× Servo Motors
- 5× INA226 Current Sensors
- PCA9548A I2C Multiplexer
- 9× FSR Sensors
- External power supply for servos
- USB-to-TTL adapter (if required)

---

# 📦 Software Requirements

## MCU Side
- PlatformIO or Arduino STM32 Core
- Libraries:
  - Servo
  - INA226_WE
  - Wire (I2C)

## PC Side (Ubuntu Recommended)
- Python 3.10+
- PySide6
- pyserial
- ROS 2 Jazzy

Install Python dependencies:
```
pip install PySide6 pyserial
```

---

# 🚀 Running the System

## 1️⃣ Open ROS 2 Environment
```
cd ~/ros2_ws
export ROS_DOMAIN_ID=10
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv_ros/bin/activate
```

## 2️⃣ Run Vision Pattern Node
Example:
```
ros2 run my_ros2_package pattern_publisher_node
```

Verify topics:
```
ros2 topic list
```

Expected topics:
- /finger_pattern_request
- /finger_pattern
- /finger_pattern_ack

## 3️⃣ Run the Python UI
```
python3 src/my_ros2_package/my_ros2_package/ROS_Serial_UI.py
```

## 4️⃣ Connect to STM32
1. Click Refresh Ports
2. Click Connect
3. Workflow:
   - Request Pattern (ROS)
   - Send Pattern to MCU
   - Start Grasping
   - Reset/Open anytime

---

# 🧠 Finite State Machine

States:
- IDLE
- CLOSING_FAST
- CLOSING_SLOW
- TIGHTEN
- HOLD
- SETTLE
- RECOVER
- RESETTING

Transitions triggered by:
- FSR huge change (>30% of sensor range)
- Current high (~800 mA)
- Contact drop
- Reset command (3)

---

# ⚙️ Speed-Based Ramp Control

Servo speed is controlled using:
```
#define FULL_SWEEP_TIME_SEC 12.0f
```

Auto-calculated speed:
```
autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC;
```

To change speed, modify FULL_SWEEP_TIME_SEC.

---

# 🔄 Serial Commands (MCU)

Allowed commands:
- 00000 – 11111 → Finger pattern
- 2 → Start grasp
- 3 → Reset (works in ANY state)

---

# 📊 Data Output Format

## Current Data
```
millis,pulse,Little=xx mA, Ring=xx mA, ...
```

## FSR Data
```
millis,FSR Live: Little=xx, RingPalm=xx, ...
```

The UI automatically classifies and displays:
- FSR values
- Current values
- FSM state
- Status messages

---

# 🛡 UART Stability Notes

If UART becomes unstable:
- Reduce print frequency
- Increase CURRENT_PRINT_PERIOD_MS
- Increase FSR_PRINT_PERIOD_MS
- Ensure baud rate is 115200
- Avoid multiple serial monitors simultaneously

---

# 📂 Example Project Structure
```
ros2_ws/
│
├── src/
│   └── my_ros2_package/
│       └── my_ros2_package/
│           ├── ROS_Serial_UI.py
│           ├── pattern_publisher_node.py
│
└── STM32_Firmware/
    └── main.cpp
```

---

# 🎓 Academic Relevance

This project demonstrates:
- Event-driven grasp control
- Mealy-type FSM implementation
- Sensor fusion (FSR + Current)
- Adaptive per-finger speed scaling
- Real-time feedback control
- ROS 2 distributed communication

---

# 📜 License

Educational / Research Use
