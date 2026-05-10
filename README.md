# Robotic End Effector Control (STM32 + PySide6 UI)

This project contains:

- **MCU firmware (STM32 / Arduino framework)** implementing a **finite-state-machine (FSM)** grasp controller
- **Desktop UI (PySide6 + pyserial)** for selecting active fingers (5‑bit pattern), starting grasp, and resetting/opening

The system is designed for a robotic hand/end-effector with:
- **5 servo-driven fingers** (Pinky, Ring, Middle, Index, Thumb)
- **5 INA226 current sensors** (one per finger) connected through a **PCA9548A I2C multiplexer**
- **9 FSR sensors** on the palm/fingers (analog inputs)

---

## Repository layout (recommended)

You can organize your repo like:

```
.
├── mcu/
│   └── main.ino              # STM32 firmware (Arduino framework)
├── ui/
│   └── control_ui.py         # PySide6 desktop UI
└── README.md
```

---

## Hardware overview

### Servos
- Servo pins:
  - Pinky:  `PB13`
  - Ring:   `PB14`
  - Middle: `PB15`
  - Index:  `PA8`
  - Thumb:  `PA11`

Servo pulse limits (microseconds):
- `SERVO_MIN_US = 500`
- `SERVO_MAX_US = 2400`

### Current sensing (INA226 + PCA9548A)
- INA226 I2C address: `0x40`
- PCA9548A address: `0x70`
- STM32 I2C pins (do not change in firmware):
  - SDA: `PB9`
  - SCL: `PB8`

Each finger current sensor is behind a separate PCA channel:
- Channel 0 → Pinky INA226
- Channel 1 → Ring INA226
- Channel 2 → Middle INA226
- Channel 3 → Index INA226
- Channel 4 → Thumb INA226

### FSR sensors (9 analog channels)
Pins:
```
PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0
```

Names (printed as):
- `Little`, `LittlePalm`, `Ring`, `RingPalm`, `Middle`, `Index`, `IndexPalm`, `Thumb`, `ThumbPalm`

---

## Build & upload (MCU)

Typical PlatformIO / Arduino STM32 workflow (example):

- Ensure board is configured (e.g., BlackPill F401CE / similar).
- Build and upload with your normal toolchain.

> The firmware relies on `INA226_WE` library and Servo/I2C.

---

## Run the UI (Desktop)

### Dependencies
Python 3.9+ recommended.

Install:
```bash
pip install PySide6 pyserial
```

Run:
```bash
python3 ui/control_ui.py
```

---

## Serial protocol (UI ↔ MCU)

The UI only sends **line-based commands** terminated by `\n`.

Allowed commands:
- **5-bit finger pattern**: `00000` to `11111`
  - Bit order in firmware/UI: **[Pinky, Ring, Middle, Index, Thumb]**
  - Example: `10101` → Pinky + Middle + Thumb enabled
- **Start grasp**: `2`
- **Reset / Open**: `3`  *(works in any state)*

Firmware prints:
- Current line (periodic):  
  `millis,pulse,Little=... mA, Ring=... mA, ...`
- FSR line (periodic):  
  `millis,FSR Live: Little=..., LittlePalm=..., ...`
- State transitions:  
  `[STATE] IDLE` / `[STATE] CLOSING_FAST` / etc.
- Other `[UI]` / `[FSM]` messages

---

## Motion model (speed-based ramp)

The firmware uses a **speed-based ramp** per finger:

- Full travel: `SERVO_MAX_US - SERVO_MIN_US` (1900 µs)
- Desired full travel time: `FULL_SWEEP_TIME_SEC` (default 8.0 s)
- Base speed:
  ```
  autoRampSpeedUsPerSec = 1900 / FULL_SWEEP_TIME_SEC
  ```

Each state can apply per-finger multipliers (`speedMul[]`) to compensate different mechanical behavior per finger.

---

## Finite State Machine (FSM)

### States
- `STATE_IDLE`  
  Open/spread position. Waiting for a valid 5-bit pattern and Start command.
- `STATE_CLOSING_FAST`  
  Enabled fingers ramp toward **FAST_TARGET_US (1800 µs)** at normal speed (with per-finger multipliers).
- `STATE_CLOSING_SLOW`  
  Enabled fingers ramp toward **SLOW_TARGET_US (1000 µs)** at **half speed** (with per-finger multipliers).
- `STATE_TIGHTEN`  
  Enabled fingers ramp toward **SERVO_MIN_US (500 µs)** at **30% speed** to tighten the grasp.
- `STATE_HOLD`  
  Stop all ramps; maintain current servo positions (grasp hold). Optional release→tighten logic can re-engage.
- `STATE_SETTLE`  
  Stop all ramps; do not move (final settle/no motion).
- `STATE_RESETTING`  
  Reset/open: ramp all fingers to `SERVO_MAX_US` at normal speed; then return to IDLE.

---

## FSM transitions (all possibilities)

### Global transition (works from ANY state)
- **If command `3` received** → `STATE_RESETTING`

---

### IDLE
**Entry conditions**
- Power-on enters IDLE (after opening to SERVO_MAX_US).
- RESETTING completes → returns to IDLE.

**Transitions**
- Pattern received (`00000..11111`) → stored, still IDLE
- Start command `2` (only if pattern exists) → `STATE_CLOSING_FAST`

---

### RESETTING
**What happens**
- Clears stored pattern, enables all fingers during opening
- Starts a ramp to open/spread (`SERVO_MAX_US`)

**Transition**
- When all ramps complete (`!anyRampActive()`) → force exact SERVO_MAX_US → `STATE_IDLE`

---

### CLOSING_FAST
**What happens**
- If not commanded yet: start ramp enabled fingers to `FAST_TARGET_US` at base speed  
  (with per-finger multipliers, e.g., Ring faster, Middle/Index slower)

**Transition**
- When enabled fingers reached `FAST_TARGET_US` → `STATE_CLOSING_SLOW`

---

### CLOSING_SLOW
**What happens**
- If not commanded yet: start ramp enabled fingers to `SLOW_TARGET_US` at **0.50× base speed**

**Possible transitions**
1) **Huge FSR change detected** (relative to baseline captured at entry)  
   → `STATE_HOLD`  
   Prints: `"[FSM] Huge FSR change detected in CLOSING_SLOW -> HOLD"`

2) **High current during slow** (enabled finger current ≥ `I_TIGHTEN_HIGH_MA`, default 800 mA)  
   → `STATE_HOLD`  
   Prints: `"[FSM] Current high during CLOSING_SLOW -> HOLD"`

3) **Slow ramp completes normally** (enabled fingers reach `SLOW_TARGET_US`)  
   → `STATE_TIGHTEN`  
   Prints: `"[FSM] CLOSING_SLOW complete -> TIGHTEN"`

---

### TIGHTEN
**What happens**
- If not commanded yet: ramp enabled fingers toward `SERVO_MIN_US` at **0.30× base speed**

**Possible transitions**
1) **Huge FSR change detected**  
   → `STATE_HOLD`

2) **High current during tighten**  
   → `STATE_HOLD`

3) **Tighten completes** (enabled fingers reach `SERVO_MIN_US`)  
   → `STATE_SETTLE`  
   Prints: `"[FSM] Reached 500us -> SETTLE (no movement)"`

---

### HOLD
**What happens**
- `stopAllRamps()` is called continuously
- Servos remain at last commanded pulse widths (holding grasp)

**Optional transition: HOLD → TIGHTEN (release / contact lost rule)**
This project includes a *contact-lost re-tighten* condition:

If BOTH are true for at least `HOLD_RELEASE_DEBOUNCE_MS`:
- All FSR readings are below a fraction of each sensor’s configured max range:
  - `fsrFilt[s] / fsrMaxRange[s] <= HOLD_RELEASE_FSR_FRAC`  
  - Default: `HOLD_RELEASE_FSR_FRAC = 0.20` (20%)
- All enabled finger currents are below:
  - `iFilt[f] <= HOLD_RELEASE_I_MA`  
  - Default: `HOLD_RELEASE_I_MA = 100 mA`

Then:
- `STATE_HOLD` → `STATE_TIGHTEN`
- Prints: `"[FSM] HOLD: contact low (FSR<20% + I<100mA) -> TIGHTEN"`

If the condition stops being true before debounce completes, the timer resets and HOLD continues.

> Note: The provided implementation checks *all 9 FSR sensors* for the “low” condition.
> If you want this rule to consider only specific sensors (e.g., palm sensors only), modify `fsrLowEnoughForRelease()`.

---

### SETTLE
**What happens**
- No motion; all ramps stopped (`stopAllRamps()`)

**Transitions**
- Only global reset (`3`) → RESETTING  
(Otherwise stays in SETTLE.)

---

## Sensor processing

### Filtering
Low-pass filters are applied:
- Current filter: `ALPHA_I = 0.20`
- FSR filter: `ALPHA_FSR = 0.20`

### Huge FSR change during slow/tighten
At entry to `STATE_CLOSING_SLOW`, the firmware snapshots:
- `fsrSlowBase[s] = fsrFilt[s]`

Then at runtime:
- `delta = |fsrFilt[s] - fsrSlowBase[s]|`
- `frac = delta / fsrMaxRange[s]`
- If any sensor `frac > FSR_SLOW_HUGE_DELTA_FRAC` (default 0.30) → huge change.

---

## UI features (PySide6)

- Auto-detects serial ports (filters `ttyUSB*` / `ttyACM*`)
- Connects with `exclusive=True` (Linux) to avoid multiple programs opening same port
- Displays:
  - **Active finger pattern** as a dot/circle table
  - **FSM state** prominently (large text + subtle background color)
  - Separate panes for FSR / Current / Status / Debug log
- Prevents accidental commands:
  - Blocks anything other than **5-bit pattern**, `2`, or `3`
  - Sanitizes NUL bytes and junk characters
- Prevents “never-ending spam”:
  - Read thread exits on disconnect or errors
  - Error popups are rate-limited and deduplicated
  - Log output is trimmed to a maximum number of lines

---

## Safety notes & recommended limits

- Always test with the hand unloaded first.
- Verify `SERVO_MIN_US`/`SERVO_MAX_US` are safe for your mechanical limits.
- Consider adding:
  - A hard current cutoff (emergency open / stop)
  - A max time in TIGHTEN (avoid driving into hard stop forever if sensors fail)
  - A watchdog for I2C/INA226 failures (currently halts on I2C error)

---

## Credits / Title

UI window title in code:
> "Robotic End Effector Control UI by EGT/21/491 and EGT/21/546"

---

## Quick usage checklist

1. Power MCU and connect USB serial.
2. Start the UI.
3. Select port → Connect.
4. Send a 5-bit pattern (e.g., `11111`).
5. Press **Start** (sends `2`).
6. To stop and open at any time, press **Reset/Open** (sends `3`).

---

