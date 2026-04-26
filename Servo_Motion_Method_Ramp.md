# STM32 Robotic Hand Control System

This README documents the full firmware architecture including:

- Servo control
- INA226 current monitoring
- FSR sensing
- Non‑blocking sweep motion

---

## Servo Motion Method Used

### Step + Delay Ramp Method (Most Reliable)

The servo motion in this system is implemented using a **step‑increment ramp method**.

Instead of jumping directly from one position to another, the pulse width is:

1. Changed by a small increment (step)
2. Waited for a short time (delay)
3. Repeated until the target position is reached

This creates a controlled **ramp movement**.

---

### Why this method is used

Hobby servos internally run a position control loop.  
If commanded to jump large distances instantly, they:

- Draw high current spikes  
- Move aggressively  
- Overshoot or vibrate under load  
- Stress gears and linkages  

Using a ramp method:

- Reduces mechanical shock  
- Lowers peak current draw  
- Improves grip stability  
- Produces smoother motion  
- Is safer for prosthetic / robotic hands  

This makes it the **most predictable and reliable slow‑motion control method**.

---

### How the ramp works

Example pseudo‑flow:

```
Current Pulse = 2400 µs
Target Pulse  = 500 µs

Loop:
  Pulse -= Step
  Write pulse to servo
  Wait Delay
  Repeat until target reached
```

---

### Two ways to tune smoothness

#### 1) Step size

Smaller steps = smoother motion

| Step Size | Motion Behavior |
|-----------|-----------------|
| 300 µs    | Fast, jerky     |
| 100 µs    | Moderate        |
| 30 µs     | Smooth          |
| 10 µs     | Very smooth     |

Example:
```cpp
#define SERVO_STEP 10
```

---

#### 2) Delay between steps

Shorter delay = faster ramp  
Longer delay = slower ramp

| Delay | Motion Speed |
|-------|---------------|
| 500 ms | Very slow |
| 100 ms | Slow |
| 30 ms  | Smooth |
| 10 ms  | Fast & smooth |

Example:
```cpp
#define SWEEP_DELAY_MS 10
```

---

### Combining step + delay

| Step | Delay | Result |
|------|-------|--------|
| Large | Long  | Jerky & slow |
| Large | Short | Fast but rough |
| Small | Long  | Very slow & smooth |
| Small | Short | Smooth & responsive (ideal) |

---

### Why this is ideal for this robotic hand

- Prevents sudden current spikes  
- Improves FSR grip stability  
- Reduces mechanical stress  
- Produces predictable finger closure  

Therefore, the **step + delay ramp** provides the most reliable and controllable actuation method for prosthetic and robotic hand systems.
