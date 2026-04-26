# STM32 Robotic Hand — Servos + INA226 Current + FSR Live (PCA9548A)

This firmware runs on an **STM32F401 “Blackpill”** and provides:

- **5 finger servos** driven together (shared pulse width)
- **5 INA226 current sensors** (one per finger motor) behind a **PCA9548A I2C multiplexer**
- **9 FSR sensors** read as analog inputs and printed live
- **Non‑blocking servo sweeps** + **serial command control**
- **Fixed‑format current logging** designed for parsing (Python/CSV)

---

## 1) Hardware overview

### I2C bus
- **SCL:** PB8  
- **SDA:** PB9  
- **PCA9548A address:** `0x70`  
- **INA226 address (each channel):** `0x40`

### Servo pins (5 fingers)
| Finger | Servo Pin | PCA Channel (INA) |
|---|---:|---:|
| Pinky  | PB13 | 0 |
| Ring   | PB14 | 1 |
| Middle | PB15 | 2 |
| Index  | PA8  | 3 |
| Thumb  | PA11 | 4 |

### FSR analog pins (9 sensors)
`PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

---

## 2) Overall process (step‑by‑step)

### A) Boot / Initialization (`setup()`)
1. Start Serial at **115200**.
2. Configure I2C on **PB8/PB9** and call `Wire.begin()`.
3. Initialize INA226 sensors through the PCA9548A:
   - Select PCA channel **0 → 4**
   - Check device presence at `0x40`
   - Run `ina226.init()`
   - If any channel fails → print an error and **halt**
4. Servos start **DISABLED at boot** (not attached) to prevent startup jumps.
5. Configure all 9 FSR pins as `INPUT_ANALOG`.
6. Print the command menu.

### B) Main loop (`loop()`)
The loop continuously performs three tasks in parallel:

1. **Handle user serial commands** (`0–4`)
2. **Update servo sweep state machine** (non‑blocking)
3. **Print INA226 currents + FSR readings periodically** at fixed intervals

---

## 3) User commands (Serial Monitor)

Open Serial Monitor at **115200 baud** and send one character:

| Command | Action | Notes |
|---:|---|---|
| `0` | **Sweep** from **2400 → 1200 µs** | “Spread → bend” sweep |
| `1` | **Sweep** from **1200 → 2400 µs** | “Bend → spread” sweep |
| `2` | **Instant move** to **2400 µs** | No sweep |
| `3` | Step **−300 µs** | Limited by `SERVO_MIN_US` |
| `4` | Step **+300 µs** | Limited by `SERVO_MAX_US` |

### Servo attach behavior (important)
- The first time you send any servo command, the firmware calls **`attachServosOnce()`**.
- This attaches all 5 servos only once, then motion starts.
- This avoids sudden “jumping” at startup.

### Sweep behavior (non‑blocking)
Sweeps run in the background using a timer:
- Each sweep “step” happens every **`SWEEP_DELAY_MS = 3000 ms`**
- Step size is **`SERVO_STEP = 300 µs`**
- While sweeping, **FSR and current printing continue**.

---

## 4) Sensor read frequency (IMPORTANT)

This code is designed so **currents and FSR readings are printed at a controlled rate**.

### A) Current readings (INA226)
Current print period:
```cpp
#define CURRENT_PRINT_PERIOD_MS 200
```

- **Expected print rate:** ~**5 Hz** (one line every 200 ms)
- Each print reads **5 channels** by selecting PCA channels 0..4 and calling `getCurrent_mA()`.

**Fixed current log format (do not change if you parse in Python):**
```
millis,pulse,S0=.. mA, S1=.. mA, S2=.. mA, S3=.. mA, S4=.. mA
```

Example:
```
12345,2400,S0=420 mA, S1=390 mA, S2=410 mA, S3=405 mA, S4=398 mA
```

### B) FSR readings
FSR print period:
```cpp
#define FSR_PRINT_PERIOD_MS 200
```

- **Expected print rate:** ~**5 Hz** (one line every 200 ms)
- Each line reads **all 9 FSR pins** with `analogRead()` and prints in one line:

Example:
```
FSR Live: PB0=812.00, PA7=799.00, PA6=805.00, PA5=810.00, PA4=808.00, PA3=820.00, PA2=815.00, PA1=802.00, PA0=798.00
```

✅ You can change the rates by editing:
- `CURRENT_PRINT_PERIOD_MS`
- `FSR_PRINT_PERIOD_MS`

---

## 5) Key tunable parameters

### Servo motion
```cpp
#define SERVO_MIN_US 1200
#define SERVO_MAX_US 2400
#define SERVO_STEP   300
#define SWEEP_DELAY_MS 3000
```

### Print frequency
```cpp
#define CURRENT_PRINT_PERIOD_MS 200
#define FSR_PRINT_PERIOD_MS     200
```

---

## 6) Dependencies

- Arduino STM32 Core
- `Wire` (built‑in)
- `Servo`
- `INA226_WE`

Install `INA226_WE` via **PlatformIO Library Registry** or Arduino Library Manager.

---

## 7) Practical notes

- Servos can cause supply noise. Use a solid 5V supply and good bulk capacitors near servos.
- INA226 accuracy depends on your shunt resistor and layout.
- FSR readings are analog and may fluctuate; filtering/averaging can be added later.

---

## 8) License
Free to use and modify for academic, personal, and robotics prototyping work.
