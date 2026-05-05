# STM32 Hand Control (0/1 Ramp) + PySide6 Serial UI

This project contains **two cooperating programs** that communicate over a **UART serial link (115200 baud)**:

1) **STM32 firmware (Arduino framework)**  
   - 5 servos (same pulse to all servos)  
   - 5 × INA226 current sensors behind a PCA9548A I2C mux (channels 0..4)  
   - 9 analog FSR sensors (live readout)  
   - Two commands only: **`1` = CLOSE ramp**, **`0` = OPEN ramp**  
   - Continuous streaming of **current** + **FSR** text lines for logging/monitoring

2) **Python desktop UI (PySide6)**  
   - Serial connect/disconnect with port refresh + auto-detect  
   - Buttons for **Close (1)** and **Open (0)** that send **single bytes** (no line ending)  
   - Serial-monitor style send box (configurable line ending)  
   - Automatically **filters MCU output into separate panels**: FSR, Current, Status  
   - Debug log + Clear UI

---

## 🧠 End-to-End Flow

1. Flash the STM32 firmware and connect the board over USB-UART at **115200 baud**.
2. Run the Python UI on your PC.
3. Click **Connect**.
4. Use:
   - **`1`** → ramps servos from **2400 → 500** (CLOSE) over **10 seconds**
   - **`0`** → ramps servos from **500 → 2400** (OPEN) over **10 seconds**
5. While running, the MCU continuously prints:
   - Current readings (INA226) in a fixed format
   - FSR live readings in a fixed format
   - Other status messages (“Received: …”, “Servos attached…”, errors, etc.)

The UI separates these into dedicated text areas.

---

# 1) STM32 Firmware

## ✅ Hardware / Pin Mapping

### Servos (all get the same commanded pulse width)
| Channel | Finger | Pin |
|--------:|--------|-----|
| S0 | Pinky  | PB13 |
| S1 | Ring   | PB14 |
| S2 | Middle | PB15 |
| S3 | Index  | PA8  |
| S4 | Thumb  | PA11 |

### I2C Bus
- SDA: **PB9**
- SCL: **PB8**
- PCA9548A address: **0x70**
- INA226 address: **0x40** (one per mux channel 0..4)

### FSR Sensors (9 analog inputs)
Pins: `PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

Names printed by firmware: `PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0`

---

## 🌀 Servo Motion (Speed-based Ramp, 10s Full Sweep)

Servo movement is **non-blocking** and based on elapsed time (`dt`):

- Servo range: `SERVO_MIN_US = 500`, `SERVO_MAX_US = 2400`
- Full travel: `1900 µs`
- Desired sweep time:  
  ```cpp
  #define FULL_SWEEP_TIME_SEC 10.0f
  ```
- Auto speed:
  ```cpp
  autoRampSpeedUsPerSec = (SERVO_MAX_US - SERVO_MIN_US) / FULL_SWEEP_TIME_SEC;
  ```

### Commands (ONLY 2 commands)
| Command | Action |
|--------:|--------|
| `1` | **CLOSE** ramp: 2400 → 500 |
| `0` | **OPEN**  ramp: 500 → 2400 |

> Note: Servos are **not attached on boot**. They attach only when the first command is sent.

---

## 📊 Firmware Telemetry Formats (Used by Python Filters)

### FSR output
Starts with:
```
FSR Live:
```
Example:
```
FSR Live: PB0=0.00, PA7=12.00, PA6=0.00, ...
```

### Current output (MUST remain exactly this style)
The Python UI detects current lines using a strict regex that matches this exact format:
```
millis,pulse,S0=.. mA, S1=.. mA, S2=.. mA, S3=.. mA, S4=.. mA
```
Example:
```
26780,2400,S0=5.25 mA, S1=5.32 mA, S2=4.90 mA, S3=5.25 mA, S4=5.30 mA
```

### Print rates
- Current: `CURRENT_PRINT_PERIOD_MS = 200` → **5 Hz**
- FSR: `FSR_PRINT_PERIOD_MS = 200` → **5 Hz**

---

# 2) Python PySide6 UI

## ✅ Features

### Serial connection
- Refresh ports (filters for `/dev/ttyUSB*` and `/dev/ttyACM*`)
- Auto-detect (first matching port)
- Connect / Disconnect
- Connection status label

### Control buttons
- **Close** button sends a single byte: `1`
- **Open** button sends a single byte: `0`
- Uses “send single char (no line ending)” to match firmware behavior (it reads `Serial.read()`)

### Serial-monitor send box
- Type a command manually
- Choose line ending: **NONE / LF / CR / CRLF**
- Warning: if you type multiple characters with NONE, the firmware will read them as multiple commands.

### Filtered display panels
The UI classifies each MCU line as:

- `fsr` → line starts with **`FSR Live:`**
- `current` → line matches strict current regex
- `status` → everything else

Panels:
- **FSR Values**
- **Current Values**
- **Current State / Other Messages**
- **Debug Log** (RX/TX/connection events)
- **Clear UI** button clears all panels

---

## 🧯 UI Performance Notes

The MCU prints about **10 lines/sec** continuously (FSR + current).
To prevent the UI from becoming slow:
- each panel uses a **max line limit** (default ~300–450 lines)
- older lines are automatically removed

---

# 🛠️ Requirements

## STM32 side
- STM32 (Black Pill or similar)
- PCA9548A I2C mux
- 5× INA226
- 5 servos
- 9 FSR sensors
- Arduino STM32 core / PlatformIO

## PC side (Python)
Install dependencies:
```bash
pip install PySide6 pyserial
```

---

# ▶️ How to Run

## 1) Flash the firmware
Upload the `.ino` / Arduino sketch to your STM32.

## 2) Run the UI
```bash
python3 stm32_hand_ui.py
```

## 3) Test sequence
1. Connect in the UI
2. Click **1 (Close)** → servo ramp to 500 µs over 10s
3. Click **0 (Open)** → servo ramp to 2400 µs over 10s
4. Watch:
   - **FSR Live** panel updating
   - **Current Values** panel updating

---

# ⚠️ Troubleshooting

## “USB TTL broke” / serial disconnects
Typical causes:
- Servo power brownouts/noise affecting the MCU or USB-UART
- No common ground between servo PSU and STM32/USB-UART
- Too much printing (buffer overflow) with weak adapters/cables

Suggested fixes:
- Power servos from a **separate supply**
- Ensure **common ground** (STM32 GND ↔ servo PSU GND ↔ USB-UART GND)
- Add bulk capacitor on servo rail (470–1000 µF)
- Use a better USB-UART adapter/cable
- Increase print periods (e.g., 500 ms) if needed

## Commands not working
- Buttons send **single bytes** (correct).
- If using the send box, choose **NONE** and send only `0` or `1` for the cleanest behavior.

---

# 📁 Suggested Repo Layout

```
.
├── firmware/
│   └── stm32_hand_ramp_01.ino
├── ui/
│   └── stm32_hand_ui.py
└── README.md
```

---

# 👩‍💻 Author
**Dasuni Saparamadu**
