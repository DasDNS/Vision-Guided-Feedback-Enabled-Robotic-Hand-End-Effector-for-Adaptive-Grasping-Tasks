# STM32 Robotic Hand Control System
### Servo Control + Current Sensing (INA226) + FSR Auto‑Save Feedback

This project implements a multi‑sensor robotic hand control system using an STM32F401 Blackpill microcontroller.

It integrates:

• 5x Servo motors (finger actuation)  
• 5x INA226 current sensors  
• PCA9548A I2C multiplexer  
• 9x FSR force sensors  
• Automatic grip stability detection  

---

## Features

### Servo Control
- Independent finger actuation
- Microsecond pulse control
- Step positioning
- Attach‑on‑demand startup safety

### Current Monitoring
- Real‑time motor current measurement
- Per‑finger sensing
- I2C error detection

### Force Feedback
- 9 FSR inputs
- Oversampling + averaging
- Stability detection
- Automatic data snapshot

---

## Hardware Mapping

### Servo Pins

| Finger | MCU Pin | INA Channel |
|-------|----------|-------------|
| Pinky | PB13 | 0 |
| Ring  | PB14 | 1 |
| Middle| PB15 | 2 |
| Index | PA8  | 3 |
| Thumb | PA11 | 4 |

### FSR Pins

PB0, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0

---

## Serial Commands

0 → Fully bent  
1 → Mid position  
2 → Fully straight  
3 → Step −300 µs  
4 → Step +300 µs  

---

## Example Outputs

Current logging:

timestamp,pulse,S0,S1,S2,S3,S4

FSR live:

FSR Live: PB0=812.3, PA7=799.1 ...

Stable detection:

STABLE → DATA SAVED

---

## Dependencies

- Arduino STM32 Core
- Wire
- Servo
- INA226_WE

---

## License

Open‑source for robotics research and development.
