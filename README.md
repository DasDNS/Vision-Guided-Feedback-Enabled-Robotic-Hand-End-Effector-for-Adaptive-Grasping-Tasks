# STM32 Robotic Hand Serial Monitor + Filtered UI (FSR + Current + Servo/INA Control)

This repository contains **two cooperating programs**:

1. **STM32 firmware (Arduino framework)** controlling:
   - 5 servos (pinky→thumb)
   - 5 INA226 current sensors via PCA9548A I2C mux (channels 0..4)
   - 9 FSR sensors (analog inputs) on the palm
   - Smooth “speed-based ramp” motion (FULL_SWEEP_TIME_SEC configurable)
   - Continuous serial streaming of **Current** and **FSR** readings

2. **Python desktop UI (PySide6)** providing:
   - Serial connection management + auto-detect
   - “Quick Controls” buttons (0..4) for servo commands
   - Serial-monitor style command sending (custom line endings)
   - **Message filtering** into separate panels:
     - FSR panel (`FSR Live:` lines)
     - Current panel (fixed CSV-like INA format)
     - Status/Other panel (everything else)
   - Debug log + UI clearing
   - Line limiting to avoid UI slowdowns under heavy serial output

---

## 🧠 System Architecture

STM32 (UART @115200)  ⇄  USB-Serial  ⇄  PC (PySide6 UI)

The STM32 prints two periodic streams:
- **Current stream** (INA226 channels 0..4)
- **FSR stream** (9 analog channels)

The Python UI reads each line and routes it to the correct panel based on prefix/format.

---

# 1) STM32 Firmware

## ✅ Hardware Features Used

### Servos (5)
| Servo | Finger | Pin |
|------:|--------|-----|
| S0 | Pinky  | PB13 |
| S1 | Ring   | PB14 |
| S2 | Middle | PB15 |
| S3 | Index  | PA8  |
| S4 | Thumb  | PA11 |

### I2C Bus
- SDA: **PB9**
- SCL: **PB8**
- PCA9548A mux: **0x70**
- INA226 sensors: **0x40** on mux channels 0..4

### FSR Sensors (9 analog)
Pins:  
`PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

---

## 🌀 Servo Motion Model (Speed-based Ramp)

The firmware uses a **speed-based ramp** (smooth motion) instead of step+delay.

### Speed setting
Set desired full travel time:
```cpp
#define FULL_SWEEP_TIME_SEC 8.0f
```

Firmware computes ramp speed automatically:
```cpp
autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC;
```

`updateRamp()` runs in the main loop using `dt` integration.

### Why this is good
- Non-blocking (does not pause loop)
- Smooth motion independent of loop speed
- Changing one constant changes overall speed reliably

---

## 🧾 Firmware Serial Output Formats (Important)

### ✅ Current (INA226) output format (fixed)
This format is required for Python classification:

```
millis,pulse,S0=... mA, S1=... mA, S2=... mA, S3=... mA, S4=... mA
```

Example:
```
26780,2400,S0=5.25 mA, S1=5.32 mA, S2=4.90 mA, S3=5.25 mA, S4=5.30 mA
```

Printed every:
```cpp
#define CURRENT_PRINT_PERIOD_MS 200   // 5 Hz
```

### ✅ FSR live output format (fixed)
```
FSR Live: PB0=..., PA7=..., ..., PA0=...
```

Example:
```
FSR Live: PB0=0.00, PA7=15.00, PA6=0.00, ...
```

Printed every:
```cpp
#define FSR_PRINT_PERIOD_MS 200       // 5 Hz
```

---

## 🎮 Firmware Serial Commands (0–4)

> Firmware reads **one character at a time** using `Serial.read()`.

| Command | Action |
|--------:|--------|
| `0` | Ramp to **SERVO_MIN_US** (fully bent: 2400 → 500) |
| `1` | Ramp to **SERVO_MAX_US** (fully straight: 500 → 2400) |
| `2` | Instant move to **SERVO_MAX_US** |
| `3` | Instant step **−10 µs** |
| `4` | Instant step **+10 µs** |

**Servo attach-on-demand:** servos do not attach at boot. They attach only after first command.

---

# 2) Python PySide6 UI

## ✅ UI Features
- Port refresh + auto detect (`/dev/ttyUSB*` or `/dev/ttyACM*`)
- Connect/disconnect with “exclusive” serial lock
- Quick buttons for 0..4 commands (sends exactly 1 byte, no line ending)
- Manual send box with selectable line endings (NONE/LF/CR/CRLF)
- 4 output panels:
  - **FSR Values**
  - **Current Values**
  - **Status / Other**
  - **Debug Log**
- “Clear UI” button clears all panels
- Line limiting to keep UI responsive during 5 Hz + 5 Hz streaming

---

## 🧠 Message Filtering Rules

Python classifies each received line as:

1) **FSR**
- Starts with `FSR Live:`

2) **Current**
- Matches the INA output regex:
```text
^\d+,\d+,S0=.*mA, S1=.*mA, S2=.*mA, S3=.*mA, S4=.*mA$
```

3) **Status**
- Everything else (servo state messages, init logs, errors, etc.)

---

# 🛠️ Requirements

## Firmware
- STM32 “Black Pill” (or compatible)
- Arduino STM32 core (or PlatformIO)
- PCA9548A I2C mux + 5× INA226 sensors
- 5 servo motors
- 9 FSR sensors wired to analog pins

## Python
Install dependencies:
```bash
pip install PySide6 pyserial
```

---

# ▶️ How to Run

## 1) Flash STM32 firmware
Upload the C++ sketch to STM32.

Use serial at:
- **115200 baud**
- **No line ending** recommended for commands 0..4

## 2) Run the Python UI
```bash
python3 hand_control_ui.py
```

## 3) Connect and test
1. Click **Refresh Ports**
2. (Optional) **Auto-Detect**
3. Click **Connect**
4. Press quick buttons (0..4)
5. Watch:
   - FSR panel updating at 5 Hz
   - Current panel updating at 5 Hz
   - Status panel showing servo state + other messages

---

# ⚠️ Serial Flood / USB-TTL Stability Notes

If USB-TTL adapters disconnect/crash, common causes:

- Printing too fast (flooding buffers)
- Poor grounding / power noise from servos
- Cheap adapter overheating / brownouts
- Servo power sharing with USB without isolation

### This project reduces risk by:
✅ Fixed-rate printing (`200 ms` periods)  
✅ Line-based reading (`readline()`)  
✅ UI line limiting to keep the PC responsive

### Best practices
- Use a **separate power supply** for servos
- Tie grounds properly (servo PSU GND ↔ STM32 GND ↔ USB GND)
- Add decoupling on servo rail (470–1000 µF + 0.1 µF)
- If needed: reduce print rate or raise baud (e.g., 230400)

---

# 📁 Suggested File Layout

```
.
├── firmware/
│   └── stm32_hand_firmware.ino
├── ui/
│   └── hand_control_ui.py
└── README.md
```

---

# 👩‍💻 Author
**Dasuni Saparamadu**  
Embedded Software / Firmware Engineer  
Sri Lanka 🇱🇰
