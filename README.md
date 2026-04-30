# Robotic End Effector Control (STM32 + PySide6 UI)

This project contains **two cooperating programs** that communicate over a **UART serial link (115200 baud)**:

1. **STM32 firmware (Arduino framework)** for a robotic end-effector / hand:
   - 5 servos (Pinky → Thumb)
   - 5 INA226 current sensors via PCA9548A I2C mux (channels 0..4)
   - 9 FSR sensors (analog inputs) on the palm
   - A **finite state machine (FSM)** for grasping + disturbance recovery + tightening
   - Continuous streaming of **FSR** and **current** telemetry

2. **Python desktop UI (PySide6)**:
   - Serial connection (refresh/auto-detect/connect/disconnect)
   - Pattern entry (5-bit finger selection) + Start/Reset buttons
   - Filtering of MCU messages into **separate UI panels**
   - Throttled RX handling (queue + timer) to avoid UI slowdown at high message rates
   - Debug log + Clear UI

---

## 🧠 Overall Flow (End-to-End)

1. **User selects fingers** to actuate by sending a **5-bit pattern**:
   - Example: `11000` → Pinky + Ring enabled, others idle.

2. User presses **Start Grasping** (sends `2`):
   - MCU starts the FSM grasp sequence **only if** a pattern was received and the system is **IDLE**.

3. During grasping, the MCU continuously streams:
   - `FSR Live: ...`  (FSR telemetry)
   - `millis,pulse,Little=... mA, Ring=... mA, ...` (current telemetry)
   - `[STATE] <STATE_NAME>` updates when FSM changes state
   - Other status/info messages

4. User can press **Reset/Open** (sends `3`) at any time:
   - MCU opens the hand, clears the pattern, returns to IDLE, and waits for a new pattern.

---

# 1) STM32 Firmware

## ✅ Hardware Mapping

### Servos (5)
| Index | Finger | Pin |
|------:|--------|-----|
| 0 | Pinky  | PB13 |
| 1 | Ring   | PB14 |
| 2 | Middle | PB15 |
| 3 | Index  | PA8  |
| 4 | Thumb  | PA11 |

### I2C (INA226 + PCA9548A)
- SDA: **PB9**
- SCL: **PB8**
- PCA9548A address: **0x70**
- INA226 address: **0x40** behind mux channels **0..4**

### FSR Sensors (9 analog)
Pins:
`PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

Names printed by firmware:
- Little, LittlePalm, Ring, RingPalm, Middle, Index, IndexPalm, Thumb, ThumbPalm

---

## 🌀 Servo Motion: Speed-based Ramp (Per Finger)

Servo motion uses a **time-based speed calculation**:

```cpp
#define FULL_SWEEP_TIME_SEC 10.0f
autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC;
```

- Ramping is updated continuously using `dt` (non-blocking).
- Each finger has its own ramp target and ramp position.
- Disabled fingers are forced to open (`SERVO_MAX_US`).

---

## 🧩 FSM Grasp Control

The grasp process is implemented as an FSM:

- `IDLE`
- `CLOSING_FAST`
- `SETTLE`
- `CLOSING_SLOW`
- `HOLD`
- `TIGHTEN`
- `RECOVER`
- `OPEN`

FSM state changes are printed as:

```
[STATE] CLOSING_FAST
```

The UI detects this and displays it in the **FSM State** panel.

### Sensor-based logic (high-level)
- **Impact / disturbance** detection from current magnitude and current slope
- **Contact** detection from FSR + current
- **Secure grasp** detection from current band + stable slope
- **Load increase** detection from current baseline shift (e.g., pouring water)
- **Tighten** applies controlled small closing increments with a safety “budget”

---

## 📟 Firmware Serial Protocol (IMPORTANT)

### Allowed Commands (only these)
| Command | Meaning |
|--------:|---------|
| `00000` .. `11111` | 5-bit pattern (S0..S4) choosing which fingers are enabled |
| `2` | Start grasp FSM (only when IDLE and pattern exists) |
| `3` | Reset/Open immediately (go to OPEN then back to IDLE; clears pattern) |

The firmware is **line-based** and expects commands terminated by LF/CRLF.
It collects characters until newline then processes the command string.

To prevent junk bytes (e.g., `�11111`), firmware keeps only `0/1/2/3` characters while building the command line.

---

## 📊 Firmware Telemetry Output Formats

### FSR output
Starts with:
```
FSR Live:
```

Example:
```
FSR Live: Little=0.00, LittlePalm=12.00, Ring=0.00, ...
```

### Current output
Starts with:
```
millis,pulse,
```
and includes `mA` readings.

Example:
```
12345,2400,Little=10.2 mA, Ring=9.8 mA, Middle=11.0 mA, Index=8.7 mA, Thumb=12.5 mA
```

### Print rates
- Current: `CURRENT_PRINT_PERIOD_MS = 200` (5 Hz)
- FSR: `FSR_PRINT_PERIOD_MS = 200` (5 Hz)

---

# 2) Python PySide6 UI

## ✅ UI Features

### Serial Connection
- Refresh ports (`ttyUSB*` and `ttyACM*`)
- Auto-detect STM32 port
- Connect / Disconnect
- Status label

### Controls
- **Start Grasping** → sends command `2`
- **Reset/Open** → sends command `3`
- **Send box** → allows only:
  - 5-bit patterns `00000..11111`
  - `2`
  - `3`
- **Clear UI** button clears all panels

### Live Panels
- **Active Fingers** (decoded from the last 5-bit pattern)
- **FSM State** (parsed from `[STATE] ...`)
- **FSR Values** (lines starting with `FSR Live:`)
- **Current Values** (lines matching current format)
- **MCU Status / Other Messages** (everything else)
- **Debug Log** (TX/RX + connection events)

---

## 🧠 Message Filtering Logic

The UI classifies each received MCU line into one of:
- `fsr`    → starts with `FSR Live:`
- `current`→ matches `CURRENT_RE`
- `fsm`    → matches `[STATE] <STATE>`
- `status` → everything else

This allows sensor data to be shown in dedicated panels while keeping general logs readable.

---

## 🧯 High-Rate Serial Safety (Why the UI won’t freeze)

Because the MCU prints ~10 lines/sec (5 Hz current + 5 Hz FSR) **plus** FSM logs, the UI uses:

- A **deque RX queue** (`maxlen=2000`)
- A **QTimer flush** every 50 ms (20 fps)
- Flushes up to 200 lines per tick
- **Line limiting** per panel to cap memory growth

This prevents the Python UI from lagging under continuous telemetry.

---

# 🛠️ Requirements

## STM32 side
- STM32 board (Black Pill or similar)
- Arduino STM32 core / PlatformIO
- PCA9548A I2C mux
- 5× INA226 current sensors
- 5 servos
- 9 FSR sensors

## PC side
Install Python dependencies:

```bash
pip install PySide6 pyserial
```

---

# ▶️ How to Run

## 1) Flash firmware
Upload the STM32 code via Arduino IDE or PlatformIO.

Serial:
- **115200 baud**
- Commands must be terminated by newline (LF/CRLF)

## 2) Run the UI
```bash
python3 control_ui.py
```

## 3) Typical test sequence
1. Connect in UI
2. Enter pattern: `11000` (or any 5-bit selection)
3. Press **Start Grasping** (2)
4. Observe:
   - FSR panel updating
   - Current panel updating
   - FSM state transitions in the FSM panel
5. Press **Reset/Open** (3) anytime to return to IDLE

---

# ⚠️ Troubleshooting

### USB-TTL disconnects / “broke”
Common causes:
- Servo power noise / brownouts
- Missing common ground
- Too-high print rate (buffer overflow)
- Weak USB port / cable

Fixes:
- Power servos from a **separate supply**
- Common ground between STM32, sensors, servo PSU, and USB
- Add bulk capacitance on servo rail (470–1000 µF)
- Reduce print rate (increase print period)
- Use a higher quality USB-UART adapter

### Commands ignored
- Only `00000..11111`, `2`, `3` are allowed
- UI blocks invalid inputs

---

# 📁 Suggested Repository Layout

```
.
├── firmware/
│   └── stm32_fsm_grasp.ino
├── ui/
│   └── control_ui.py
└── README.md
```

---

# 👩‍💻 Credits / Title
UI window title in code:
**“Robotic End Effector Control UI by EGT/21/491 and EGT/21/546”**

---

# 👩‍💻 Author
**Dasuni Saparamadu**
