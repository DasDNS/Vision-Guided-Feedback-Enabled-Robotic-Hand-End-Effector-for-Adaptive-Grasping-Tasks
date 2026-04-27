# STM32 Robotic Hand Control System
## Speed-Based Servo Ramp + INA226 Current Sensing + FSR Monitoring

This firmware runs on an **STM32F401 Blackpill** and controls:

- **5 Servo motors** (robotic fingers)
- **5 INA226 current sensors** (one per motor)
- **PCA9548A I2C multiplexer** (to access multiple INA226 devices sharing the same I2C address)
- **9 FSR force sensors** (analog inputs)
- **Serial command interface** for user control

---

# Motion Control Method Used

## Method 2 — Speed-Based Ramp (Emphasized)

This project uses a **speed-based ramp motion control method** for servo actuation.

Instead of moving servos in fixed steps with delays, the system:

1. Sets a **target pulse width** (µs)
2. Continuously moves the current pulse toward that target
3. Updates motion based on elapsed time (`dt`)
4. Controls speed in **µs per second**

This produces smoother, time-controlled motion.

---

## Concept Explanation

Traditional ramp (Method 1):

- `pulse += step`
- `delay(ms)`

Speed-based ramp (Method 2 — used here):

- `diff = target − current`
- `step = speed × dt`
- `current += step`

Where:

- `speed` = µs per second  
- `dt` = elapsed time since last update  
- Motion is continuous and time-scaled

---

# Why Method 2 Is Used

This method provides:

- Precise motion timing
- Smooth motion (no big “jumps”)
- Lower current spikes (important for MG996R / high-torque servos)
- Better grip stability (less vibration = cleaner FSR readings)
- Cleaner control math (speed is directly tunable)

Ideal for prosthetic and robotic hands.

---

# Auto Speed Calculation

Defined in code:

```cpp
#define FULL_SWEEP_TIME_SEC 8.0f
```

Full travel is:

- `SERVO_MAX_US (2400) → SERVO_MIN_US (500)`
- Travel distance = `1900 µs`

Automatic speed:

- `1900 / 8 ≈ 237.5 µs/sec`

So a full finger close/open takes approximately **8 seconds**.

---

# Serial Commands

Open Serial Monitor at **115200 baud** and send one character:

| Command | Action | Motion type |
|---:|---|---|
| `0` | Ramp to fully bent | speed-based ramp |
| `1` | Ramp to fully straight | speed-based ramp |
| `2` | Instant mid move (1450 µs) | instant |
| `3` | Step −10 µs | instant |
| `4` | Step +10 µs | instant |

---

# Sensor Monitoring (Frequency Emphasis)

## Current (INA226)
Printed every:

```cpp
#define CURRENT_PRINT_PERIOD_MS 200
```

- **200 ms → 5 Hz**
- Each line reads all 5 channels (S0–S4)

Format (fixed, good for Python parsing):

```
millis,pulse,S0=.. mA, S1=.. mA, S2=.. mA, S3=.. mA, S4=.. mA
```

## FSR (Analog)
Printed every:

```cpp
#define FSR_PRINT_PERIOD_MS 200
```

- **200 ms → 5 Hz**
- Each line prints all 9 FSR readings with pin labels:

```
FSR Live: PB0=..., PA7=..., ... , PA0=...
```

---

# Dependencies

- Arduino STM32 Core
- `Wire`
- `Servo`
- `INA226_WE`

---

# What the code does (step-by-step)

This section describes the **actual runtime sequence** of the firmware from boot to continuous operation.

## 1) Startup / Boot (`setup()`)

1. **Start Serial**
   - `Serial.begin(115200);`
   - Prints the banner and a command reference to the Serial Monitor.

2. **Initialize I2C bus**
   - Sets I2C pins:
     - SDA = **PB9**
     - SCL = **PB8**
   - Calls `Wire.begin()`.

3. **Initialize INA226 sensors through PCA9548A (channels 0..4)**
   For each channel `0 → 4`:
   - Select channel on PCA9548A using `selectPCAChannel(ch)`
   - Check if an INA226 responds at `0x40` using `inaPresentOnCurrentBus()`
   - Initialize that INA226 object using `ina226_x.init()`
   - If any sensor is missing or init fails:
     - Print an error message
     - **Halt forever** with `while(1){}` (safety behavior)

4. **Wait for conversions once (optional stabilization)**
   - Calls `waitUntilConversionCompleted()` for each INA226 channel.

5. **Servo safety at boot**
   - Servos are **not attached** during `setup()`.
   - This prevents sudden motion when the MCU powers on.
   - The code prints: “Servos DISABLED at boot…”

6. **FSR pin setup**
   - Configures 9 pins as `INPUT_ANALOG`:
     - `PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

7. **Print command menu + computed ramp speed**
   - Prints:
     - Available commands (0–4)
     - `FULL_SWEEP_TIME_SEC`
     - `autoRampSpeedUsPerSec`

---

## 2) Continuous operation (`loop()`)

The main loop performs **four tasks repeatedly**:

### Task A — Read user command (Serial)
- If a character arrives:
  1. Print `Received: <cmd>`
  2. Execute servo command:

**Command behavior**
- `0`: start ramp to `SERVO_MIN_US` (500 µs)
- `1`: start ramp to `SERVO_MAX_US` (2400 µs)
- `2`: instant move to **1450 µs**
- `3`: instant step down by **10 µs**
- `4`: instant step up by **10 µs**

After handling, prints: `Enter next command:`

### Task B — Update the speed-based ramp (non-blocking)
- `updateRamp()` runs every loop.
- If a ramp is active:
  1. Compute elapsed time `dt` from `millis()`
  2. Compute how far we are from the target (diff)
  3. Compute allowed movement:
     - `step = autoRampSpeedUsPerSec * dt`
  4. Move `rampPosUs` toward `rampTargetUs`
  5. Stop ramp when target is reached

✅ Because it is **non-blocking**, the MCU can still read sensors and print logs while moving.

### Task C — Print INA226 currents periodically (5 Hz)
- Every **200 ms**:
  - Reads channel 0..4 (select PCA channel each time)
  - Prints a single current line in fixed format

### Task D — Print FSR readings periodically (5 Hz)
- Every **200 ms**:
  - Reads all 9 analog pins using `analogRead()`
  - Prints one single-line message with pin labels

---

## 3) Summary (runtime behavior)

1. User sends command  
2. Servos attach (first command only)  
3. Ramp or instant motion is applied  
4. Currents continue printing at **5 Hz**  
5. FSR readings continue printing at **5 Hz**  
6. Ramp completes smoothly based on **FULL_SWEEP_TIME_SEC**  

---

## Troubleshooting notes

- If the output is “too fast”, increase:
  - `CURRENT_PRINT_PERIOD_MS`
  - `FSR_PRINT_PERIOD_MS`
- If INA226 init halts:
  - Check PCA9548A wiring and address `0x70`
  - Ensure each INA226 is connected to the correct PCA channel
  - Confirm the INA226 address is `0x40`
- If servos move unexpectedly:
  - Ensure you power servos from a stable 5V supply
  - Keep servo ground tied to MCU ground (common ground)
  
---

# License

Open-source for robotics and prosthetics research.

---
