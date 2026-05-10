# Robotic End Effector Grasping Project (STM32 + PySide6 UI)

This repository contains:

- **Firmware (STM32 / Arduino C++)**
  - 5 servo outputs (Pinky/Ring/Middle/Index/Thumb)
  - 5 INA226 current sensors via PCA9548A I²C mux (channels 0..4)
  - 9 FSR sensors (analog) on the palm
  - A grasping **finite state machine (FSM)** controlled from serial commands

- **Desktop UI (Python / PySide6)**
  - Serial connect + safe command sending (pattern / start / reset)
  - Live display of **FSM state**, **active fingers**, **FSR**, and **current** logs

> Generated on 2026-02-20.

---

## Hardware

### MCU
- STM32 Black Pill (Arduino framework / PlatformIO recommended)

### Servos (5)
| Finger | Servo Index | STM32 Pin |
|---|---:|---|
| Pinky | S0 | `PB13` |
| Ring | S1 | `PB14` |
| Middle | S2 | `PB15` |
| Index | S3 | `PA8` |
| Thumb | S4 | `PA11` |

Servo pulse limits:
- `SERVO_MIN_US = 500` (closed)
- `SERVO_MAX_US = 2400` (open/spread)

### Current sensing (5 × INA226)
- INA226 address: `0x40`
- PCA9548A address: `0x70`
- Mux channel mapping:
  - ch0 → INA226 for Pinky
  - ch1 → INA226 for Ring
  - ch2 → INA226 for Middle
  - ch3 → INA226 for Index
  - ch4 → INA226 for Thumb

### FSR sensing (9)
Analog inputs:
- `PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

Printed names in firmware output:
- `Little, LittlePalm, Ring, RingPalm, Middle, Index, IndexPalm, Thumb, ThumbPalm`

---

## Serial commands (UI ↔ Firmware)

Firmware accepts only these commands (line-based, LF newline):

1) **Pattern**: `00000` .. `11111`  
Enables fingers for grasping. Bit order is:
`S0 S1 S2 S3 S4` = `Pinky Ring Middle Index Thumb`

- `1` → finger enabled (allowed to move)
- `0` → finger disabled (forced open/spread)

2) **Start**: `2`  
Starts grasping **only** if a valid pattern was stored and state is `IDLE`.

3) **Reset/Open**: `3`  
Works **in any state**. Immediately begins opening and returns to `IDLE`.

Example:
```text
11100   (enable Pinky + Ring + Middle)
2       (start grasp)
3       (reset/open)
```

---

## Firmware grasping FSM

### Key speed & target settings
- Base speed uses a time-based ramp:
  - `FULL_SWEEP_TIME_SEC = 8.0`
  - `autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC`

Two-step close targets:
- `FAST_TARGET_US = 2000`
- `SLOW_TARGET_US = 1400`

### States
- `IDLE`  
  Hand is open/spread; waits for a 5-bit pattern, then `2` to start.

- `RESETTING`  
  Triggered by `3`. Ramps all fingers open at normal speed, then returns to `IDLE`.

- `CLOSING_FAST`  
  One-time command:
  - enabled fingers ramp to **2000 µs** at normal speed
  - per-finger speed multipliers (`speedMul[]`) are applied (e.g., Ring can be faster)

- `CLOSING_SLOW`  
  One-time command:
  - enabled fingers ramp to **1400 µs** at **50%** base speed
  - safety: if **huge FSR change** happens → `HOLD`
  - safety: if **current becomes high during motion** → `HOLD`
  - when complete → `TIGHTEN`

- `TIGHTEN`  
  One-time command:
  - enabled fingers ramp toward **500 µs** at **30%** base speed
  - safety: huge FSR change or high current can move to `HOLD`
  - when 500 reached → `SETTLE`

- `HOLD`  
  No ramp updates (ramp flags stopped). PWM holds grasp.
  - If contact disappears (FSR and current low for a short debounce) → return to `TIGHTEN`

- `SETTLE`  
  No motion; maintains position.

---

## Filtering + detection rules

### Filters
- Current low-pass: `ALPHA_I = 0.20`
- FSR low-pass: `ALPHA_FSR = 0.20`

### Huge FSR change (used during slow/ tighten in this firmware)
- Each sensor has a max range estimate:
  - `[300, 600, 300, 600, 300, 300, 600, 300, 600]`
- Huge change trigger:
  - `abs(fsrFilt[s] - fsrSlowBase[s]) / fsrMaxRange[s] > FSR_SLOW_HUGE_DELTA_FRAC`
- Default:
  - `FSR_SLOW_HUGE_DELTA_FRAC = 0.30`

### Current high trigger
- If any enabled finger current rises above:
  - `I_TIGHTEN_HIGH_MA = 800 mA`
then motion can stop and go to `HOLD`.

### HOLD release trigger (debounced)
If **all** FSRs are low and **all enabled currents** are low continuously for:
- `HOLD_RELEASE_DEBOUNCE_MS = 150`

Thresholds:
- FSR: below `HOLD_RELEASE_FSR_FRAC = 0.20` of each sensor range
- Current: below `HOLD_RELEASE_I_MA = 100 mA`

Then FSM returns to `TIGHTEN`.

---

## Building & flashing (firmware)

### PlatformIO (recommended)
1. Create a PlatformIO project for your Black Pill board (example: `blackpill_f401ce`)
2. Add required libraries:
   - `INA226_WE`
   - Built-in: `Wire`, `Servo`
3. Upload:
```bash
pio run -t upload
```

### Important notes
- I²C pins are fixed:
  - `SDA = PB9`
  - `SCL = PB8`
- Firmware halts if any INA226 init fails or device is missing.

---

## Running the Python UI

### Requirements
- Python 3.10+
- Packages:
  - `PySide6`
  - `pyserial`

### Setup (Linux)
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install PySide6 pyserial
```

### Run
Save the UI as `ui.py` and run:
```bash
python3 ui.py
```

### Serial permission (Linux)
```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## UI usage

1. **Connect**
   - Refresh Ports → select device → Connect
2. **Send a pattern**
   - Type `00000..11111` → Send
   - Active finger dots update (green=enabled, red=disabled)
3. **Start**
   - Press *Start Grasping* (sends `2`)
4. **Reset/Open**
   - Press *Reset/Open* (sends `3`) anytime

UI shows:
- FSM state in large text (highlighted when it changes)
- FSR lines
- Current lines
- Status + debug logs

---

## Troubleshooting

- **“INA226 NOT FOUND … Halting.”**
  - Check PCA9548A address (0x70), INA226 address (0x40), wiring, power, and mux channels.

- **UI connects but no output**
  - STM32 may reset on connect; wait ~2 seconds.
  - Confirm baud rate is 115200.

- **Servo doesn’t move**
  - Ensure servo power supply can deliver enough current.
  - Ensure common ground between servo supply and STM32.

---

## Suggested repo layout
```text
project/
  firmware/
    main.cpp
    platformio.ini
  ui/
    ui.py
  README.md
```

---

## Credits
UI title in code: **EGT/21/491 and EGT/21/546**.
