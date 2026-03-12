#  Full Auto Steering for 4-Wheel Rover (ROS2)

## Overview

This ROS Python node (`drive_arc`) enables **manual and autonomous steering and drive control** for a 4-wheeled rover. It supports joystick input, encoder feedback, and autonomous commands to compute steering angles and drive PWM signals.

---

## Features

-  Manual and Autonomous control switching  
-  Runtime mode switching via joystick  
-  4 types of driving/steering logic  
-  Customizable joystick mappings  
-  Encoder feedback-based closed-loop steering  
-  Velocity and omega smoothing using `queue.Queue`  

---

## Node: `drive_arc`

### ✅ Publishers

| Topic        | Type               | Description                              |
|--------------|--------------------|------------------------------------------|
| `/motor_pwm` | `Int32MultiArray`  | [4x drive PWM, 4x steer PWM]             |
| `/state`     | `std_msgs/Bool`    | 0 → Manual, 1 → Autonomous               |

### 📥 Subscribers

| Topic        | Type                   | Used for                          |
|--------------|------------------------|-----------------------------------|
| `/joy`       | `sensor_msgs/Joy`      | Joystick input                    |
| `/enc_auto`  | `Float32MultiArray`    | Encoder readings                  |
| `/motion`    | `custom/WheelRpm`      | Autonomous velocity/omega         |
| `/rot`       | `std_msgs/Int8`        | Rotate-in-place trigger           |

---

## 🕹️ Joystick Mapping
![Joystick Layout](controller_image.png)
| Action                        | Button/Axis            | Description                                  |
|------------------------------|------------------------|----------------------------------------------|
| Mode up/down                 | `RB` / `LB`            | Increase/decrease drive mode                |
| Toggle autonomous            | `Start/Menu` button    | Switch between manual and autonomous         |
| Toggle steering lock         | Axis trigger           | Sets `steer_islocked`                        |
| Toggle full potential        | Axis trigger           | Sets `full_potential_islocked`               |
| Forward/Backward drive       | Left stick Y-axis      | Drive direction                              |
| Lateral movement             | Left stick X-axis      | Strafing                                     |
| Curve steering               | Axis 3                 | Adds angular motion                          |
| Same-direction PWM steering  | Right stick Y-axis     | Applies same PWM to all steering motors      |
| Opp-direction PWM steering   | Right stick X-axis     | Steers wheels in opposite PWM directions     |
| Individual wheel PWM axes    | Custom axes            | Controls each wheel's angle (4 axes)         |

---

## 🚗 Drive + Steering Modes

| Mode | `steer_islocked` | `full_potential_islocked` | Description                              |
|------|------------------|---------------------------|------------------------------------------|
| 1    |  True          |  True                   | Locked steering (forward, perp, rotate)  |
| 2    |  False         |  True                   | Unlocked steering using joystick axes    |
| 3    |  True          |  False                  | Full potential per-wheel PWM steering    |
| 4    |  False         |  False                  | Invalid / fallback                       |

---

## 🔁 Core Logic

### `joyCallback`
- Reads button and axis values  
- Controls mode switches and internal toggles (`state`, `steer_islocked`, etc.)

### `encoderCallback`
- Collects encoder values from subscribed topic

### `motionCallback`
- Updates `velocity` and `omega` from autonomous source

### `rotCallback`
- Triggers rotate-in-place action

---

## 🔧 Steer Function


---

## 🛞 Steering Control Function (`steer`)

The `steer(self, initial_angles, final_angles, mode)` function is responsible for rotating the steering motors of all four wheels to the desired angles using **closed-loop feedback** from encoders.

---

### 🧩 How It Works

**Inputs:**

- `initial_angles`: List of current encoder angles for each wheel
- `final_angles`: List of target angles (absolute or relative)
- `mode`:  
  - `0` → Relative Mode (offset from current)  
  - `1` → Absolute Mode (go to specific angle)

---

### 🔁 Process

1. For each wheel:
   - Compute `error = final_angle - current_angle`
   - Apply **proportional control**: `PWM = kp_steer * error`
   - Cap `PWM` to `max_steer_pwm` (to prevent overdrive)

2. Loop at 10 Hz:
   - Update encoder readings
   - Recalculate PWM commands
   - Check if all errors are below `error_thresh`
   - Exit when either all targets are met or timeout is hit

---

### 🧮 Modes Explained

#### 🌀 Relative Mode (`mode == 0`):
Each wheel rotates by an angle **relative** to its current position.

> Example:  
> `final_angles = [45, 45, 45, 45]`  
> → All wheels rotate +45° from where they are.

#### 🎯 Absolute Mode (`mode == 1`):
Each wheel rotates to an **absolute** target angle.

> Example:  
> `final_angles = [0, 90, 0, 90]`  
> → Wheels go to those exact angles (in degrees or encoder ticks).

---

### ⚙️ Flow of Control

```text
joyCallback
    ↓
encoderCallback
    ↓
MAIN
    ↓
STEERING is called
    ↓
STEER is called
    ↓
DRIVE is called
    ↓
PWM Message is published
