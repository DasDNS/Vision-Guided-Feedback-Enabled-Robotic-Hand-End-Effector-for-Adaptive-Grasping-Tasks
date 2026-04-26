# STM32 Blackpill Robotic Hand Logger
### 5 Servos + 5 INA226 Current Sensors (via PCA9548A) + 9 FSR Live Readings

This firmware runs on an **STM32F401 “Blackpill”** using the **Arduino framework**.

It provides:
- **5 servos** (all fingers move together with the same pulse width)
- **5 INA226 current sensors** (one per servo motor) using a **PCA9548A I2C multiplexer**
- **9 FSR sensors** read as analog inputs and printed live
- **Non-blocking servo sweeps** controlled by **Serial commands (0–4)**
- **Fixed-format current logs** suitable for Python/CSV parsing

---

## Hardware mapping

### I2C bus (STM32 → PCA9548A → INA226)
- **SCL:** PB8  
- **SDA:** PB9  
- **PCA9548A address:** `0x70`  
- **INA226 address (each channel):** `0x40`

### Servo pins (5 fingers)
| Servo | Finger | MCU Pin | INA Channel |
|---:|---|---|---:|
| S0 | Pinky  | PB13 | 0 |
| S1 | Ring   | PB14 | 1 |
| S2 | Middle | PB15 | 2 |
| S3 | Index  | PA8  | 3 |
| S4 | Thumb  | PA11 | 4 |

### FSR pins (9 analog inputs)
`PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

---

## IMPORTANT: Reading/printing frequency

### Current readings (INA226)
```cpp
#define CURRENT_PRINT_PERIOD_MS 200
```
- Prints current data every **200 ms**
- That is **5 lines per second (5 Hz)**  
- Each line includes **all 5 INA channels** (S0…S4)

### FSR readings
```cpp
#define FSR_PRINT_PERIOD_MS 200
```
- Prints FSR data every **200 ms**
- That is **5 lines per second (5 Hz)**
- Each line includes **all 9 FSR pins**

### Sweep speed (when sweeping)
```cpp
#define SERVO_STEP     10      // µs per step
#define SWEEP_DELAY_MS 10      // ms between steps
```
- Sweep updates every **10 ms**
- Pulse changes by **10 µs** each update  
- Approx. sweep rate: **1000 µs per second**

---

## User commands (Serial Monitor)

Open Serial Monitor at **115200 baud** and send one character:

| Command | Action | Description |
|---:|---|---|
| `0` | Sweep bend | sweep **2400 → 500 µs** |
| `1` | Sweep straight | sweep **500 → 2400 µs** |
| `2` | Instant straight | move instantly to **2400 µs** |
| `3` | Step down | pulse **−10 µs** |
| `4` | Step up | pulse **+10 µs** |

The firmware prints `Received: X` when you type a command, then `Enter next command:`.

---

## Step-by-step code explanation

### 1) Libraries
```cpp
#include <Wire.h>
#include <Servo.h>
#include <INA226_WE.h>
```
- `Wire`: I2C communication for PCA9548A + INA226
- `Servo`: servo PWM generation
- `INA226_WE`: INA226 current sensor library

---

### 2) Constants (addresses, pins, timing)
- Sets the I2C addresses:
  - `PCA9548A_ADDRESS = 0x70`
  - `INA226_ADDRESS = 0x40`
- Defines the 5 servo pins and servo pulse range:
  - `SERVO_MIN_US = 500`
  - `SERVO_MAX_US = 2400`
- Sets sweep and print timing:
  - `SWEEP_DELAY_MS = 10`
  - `CURRENT_PRINT_PERIOD_MS = 200`
  - `FSR_PRINT_PERIOD_MS = 200`

---

### 3) Global objects
- Creates 5 `Servo` objects (`servo0…servo4`)
- Creates 5 `INA226_WE` objects (`ina226_0…ina226_4`)
  - They all use address `0x40`, but each one is reached by selecting a different PCA channel.

---

### 4) Servos do NOT move on boot (attach-on-demand)
```cpp
bool servosEnabled = false;
void attachServosOnce() { ... }
```
- Servos are attached only when a command needs motion.
- This helps prevent unexpected servo jumps at startup.

---

### 5) PCA9548A channel selection
```cpp
void selectPCAChannel(uint8_t channel)
```
- Writes `1 << channel` to PCA9548A
- This connects that channel to the I2C bus so the INA226 on that channel can be read.

---

### 6) INA226 presence + error handling
```cpp
bool inaPresentOnCurrentBus()
void checkForI2cErrors(INA226_WE &sensor)
```
- Presence check makes sure a device responds at `0x40`.
- `checkForI2cErrors()` halts the program if the INA library reports an I2C error code.

---

### 7) Printing current data (all 5 channels)
```cpp
void printINA226Data()
```
Steps:
1. Select PCA channel 0 → read current → error check
2. Repeat for channels 1..4
3. Print a single line in a fixed format:

**Current line format (fixed):**
```
millis,pulse,S0=.. mA, S1=.. mA, S2=.. mA, S3=.. mA, S4=.. mA
```

---

### 8) Servo movement
#### Instant move
```cpp
void moveServoUS(int pulseWidth)
```
- Attaches servos (first time only)
- Stops any sweep
- Applies one pulse width to all 5 servos

#### Non-blocking sweep
```cpp
void startSweep(...)
void updateSweep()
```
- `startSweep()` sets starting pulse, target, direction, and timing.
- `updateSweep()` advances the sweep only when the next step time is reached.
- Because it is non-blocking, sensor printing continues during sweeps.

---

### 9) FSR live printing (9 sensors)
```cpp
void printFSRLive()
```
- Reads all 9 analog pins using `analogRead()`
- Prints one single line:
```
FSR Live: PB0=..., PA7=..., ... , PA0=...
```

---

### 10) Setup sequence (`setup()`)
1. Start Serial (115200)
2. Set I2C pins (PB9 SDA, PB8 SCL) and start Wire
3. Initialize INA226 on PCA channels 0..4 (halt if any fail)
4. Print servo pin mapping and command menu
5. Configure FSR pins as analog inputs

---

### 11) Main loop (`loop()`) — what happens every cycle
1. **Check for Serial command**
2. **Update sweep state machine**
3. **Print INA currents every 200 ms (5 Hz)**
4. **Print FSR readings every 200 ms (5 Hz)**

---

## Dependencies
- Arduino STM32 core
- `Wire` (built-in)
- `Servo`
- `INA226_WE`

---

## Tips / troubleshooting
- If Serial output is too fast, increase the print periods:
  - `CURRENT_PRINT_PERIOD_MS`
  - `FSR_PRINT_PERIOD_MS`
- Keep I2C wiring short, and use proper pull-ups.
- Use a strong 5V supply for servos and good decoupling to avoid resets.

---

## License
Free to use and modify for academic/personal robotics projects.
