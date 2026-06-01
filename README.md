# omni_robot — BT Reactive Explorer V2

> Autonomous reactive navigation on a Raspberry Pi 4 using **ROS 2 Jazzy**
> and **BehaviorTree.CPP v3** — no map, no GPS, no encoders.

---

## Overview

This project implements a fully reactive obstacle-avoidance and autonomous
exploration system for a differential-drive (tank-drive) robot.
All decisions are made in real time from 4 ultrasonic sensors and an IMU,
processed through a **Behaviour Tree ticked at 20 Hz**.

```
HC-SR04 ×4  ──►  ultrasonic_node  ──►  /ultrasonic/{front,back,left,right}
ADXL345 IMU ──►  imu_node         ──►  /imu/data
                                              │
                                              ▼
                              bt_reactive_explorer_v2_node
                              (Behaviour Tree, 20 Hz tick)
                                              │
                                              ▼ /cmd_vel_safe
                              tank_drive_node ──► L298N ──► Motors
```

---

## Hardware Requirements

| Component        | Model                  | Role                        |
| ---------------- | ---------------------- | --------------------------- |
| Compute          | Raspberry Pi 4 Model B | Main CPU                    |
| Motor driver     | L298N H-bridge         | PWM motor control           |
| Distance sensors | 4× HC-SR04 ultrasonic  | Front / Back / Left / Right |
| IMU              | ADXL345 accelerometer  | Tilt & impact detection     |
| GPIO library     | pigpiod daemon         | Hardware PWM                |

### Wiring — HC-SR04 (GPIO pins)

| Sensor | TRIG    | ECHO    |
| ------ | ------- | ------- |
| Front  | GPIO 17 | GPIO 27 |
| Back   | GPIO 22 | GPIO 10 |
| Left   | GPIO 9  | GPIO 11 |
| Right  | GPIO 25 | GPIO 8  |

### Wiring — ADXL345 (I2C)

| ADXL345 pin | Raspberry Pi pin        |
| ----------- | ----------------------- |
| VCC         | 3.3V (Pin 1)            |
| GND         | GND (Pin 6)             |
| SDA         | GPIO 2 / SDA (Pin 3)    |
| SCL         | GPIO 3 / SCL (Pin 5)    |
| SDO         | GND → I2C address 0x53  |
| CS          | 3.3V (selects I2C mode) |

### Wiring — L298N (GPIO pins)

| L298N pin   | Raspberry Pi GPIO |
| ----------- | ----------------- |
| ENA (right) | GPIO 12 (PWM)     |
| ENB (left)  | GPIO 13 (PWM)     |
| IN1         | GPIO 5            |
| IN2         | GPIO 6            |
| IN3         | GPIO 16           |
| IN4         | GPIO 26           |

---

## Software Dependencies

### System packages

```bash
# pigpio — GPIO and hardware PWM via pigpiod daemon
sudo apt install libpigpio-dev pigpiod

# I2C support — kernel driver for ADXL345
sudo apt install i2c-tools libi2c-dev python3-smbus

# Enable I2C interface on Raspberry Pi
sudo raspi-config
# → Interface Options → I2C → Enable

# Verify ADXL345 is detected (should show 0x53)
i2cdetect -y 1
```

### ROS 2 packages

```bash
# BehaviorTree.CPP v3 framework
sudo apt install ros-jazzy-behaviortree-cpp-v3

# Keyboard teleoperation (optional, for manual control)
sudo apt install ros-jazzy-teleop-twist-keyboard
```

---

## Installation

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-username/omni_robot.git

# 2. Build only the required packages
cd ~/ros2_ws
colcon build --packages-select omni_robot_hardware omni_robot_bringup

# 3. Source the workspace
source install/setup.bash
```

---

## Usage

### Mode 1 — Autonomous Navigation (BT Reactive Explorer V2)

The robot drives forward, detects obstacles with ultrasonic sensors,
and reacts through the Behaviour Tree (backup, turn, wall-follow).

```bash
# Start pigpiod first
sudo systemctl start pigpiod

# Launch autonomous mode
ros2 launch omni_robot_bringup reactive_explore_v2.launch.py
```

### Mode 2 — Manual Teleoperation

Bring up all hardware drivers without the BT node, then drive with keyboard.

```bash
# Terminal 1 — start hardware drivers
ros2 launch omni_robot_bringup robot.launch.py

# Terminal 2 — keyboard control
# Note: we remap cmd_vel → cmd_vel_safe to reach the motor driver directly
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_safe
```

**Keyboard controls:**

| Key       | Action                            |
| --------- | --------------------------------- |
| `i`       | Forward                           |
| `,`       | Backward                          |
| `j`       | Turn left                         |
| `l`       | Turn right                        |
| `k`       | Stop                              |
| `q` / `z` | Increase / decrease all speeds    |
| `w` / `x` | Increase / decrease linear speed  |
| `e` / `c` | Increase / decrease angular speed |

---

## Configuration

All parameters are in `omni_robot_bringup/config/`.

### `bt_reactive_explore_v2.yaml` — Key parameters

| Parameter                 | Default | Description                           |
| ------------------------- | ------- | ------------------------------------- |
| `front_block_m`           | `0.20`  | Front obstacle detection distance (m) |
| `back_block_m`            | `0.06`  | Rear obstacle abort distance (m)      |
| `front_blocked_persist_s` | `3.0`   | Time before "stuck" escalation (s)    |
| `forward_speed_mps`       | `0.15`  | Forward cruise speed (m/s)            |
| `turn_speed_rads`         | `1.0`   | Angular speed for turns (rad/s)       |
| `turn_time_s`             | `0.785` | Duration of 45° recovery turn (s)     |
| `spin_time_s`             | `6.28`  | Duration of full 360° stuck spin (s)  |
| `backup_time_s`           | `1.4`   | Backup duration after obstacle (s)    |
| `impact_accel_mps2`       | `25.0`  | IMU impact detection threshold (m/s²) |
| `tilt_stop_deg`           | `85.0`  | Tilt safety stop angle (degrees)      |

### `drive.yaml` — Motor driver parameters

| Parameter          | Default | Description                             |
| ------------------ | ------- | --------------------------------------- |
| `max_pwm`          | `200`   | Maximum PWM duty cycle                  |
| `min_pwm`          | `80`    | Minimum PWM to overcome static friction |
| `max_linear_mps`   | `0.5`   | Maximum linear velocity                 |
| `max_angular_rads` | `1.0`   | Maximum angular velocity                |

---

## Behaviour Tree — Priority Logic

The BT runs at **20 Hz** (50 ms/tick). On every tick, branches are evaluated
top to bottom and higher-priority branches always pre-empt lower ones.

| Priority     | Condition           | Action                  |
| ------------ | ------------------- | ----------------------- |
| P1 (highest) | Tilt unsafe         | Hard stop               |
| P2           | IMU impact detected | Backup + turn           |
| P3           | Front blocked > 3s  | Backup + full 360° spin |
| P4           | Front blocked       | Backup + 45° turn       |
| P5           | Side wall detected  | Wall follow             |
| P6 (default) | Path clear          | DriveForwardSteered     |

---

## Repository Structure

```
omni_robot/
├── README.md
├── .gitignore
├── omni_robot_hardware/          # C++ driver nodes
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/omni_robot_hardware/
│   │   ├── hc_sr04.hpp
│   │   ├── pigpio_client.hpp
│   │   └── tank_drive.hpp
│   └── src/
│       ├── hc_sr04.cpp
│       ├── pigpio_client.cpp
│       ├── tank_drive.cpp
│       ├── tank_drive_node.cpp
│       ├── ultrasonic_node.cpp
│       ├── imu_node.cpp
│       └── bt_reactive_explorer_v2_node.cpp
└── omni_robot_bringup/           # Launch files and config
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   ├── reactive_explore_v2.launch.py   # Autonomous mode
    │   └── robot.launch.py                 # Teleoperation mode
    └── config/
        ├── sensors.yaml
        ├── drive.yaml
        ├── bt_reactive_explore_v2.yaml
        └── bt_reactive_explore.xml         # BT XML tree
```

---

## Troubleshooting

**pigpio_start failed:**

```bash
sudo systemctl start pigpiod
sudo systemctl enable pigpiod   # auto-start on boot
```

**ADXL345 not detected (`i2cdetect` shows nothing):**

```bash
# Check I2C is enabled
ls /dev/i2c*           # should show /dev/i2c-1
# Check wiring — SDA→Pin3, SCL→Pin5, SDO→GND for address 0x53
```

**Robot spins but does not rotate physically:**

```bash
# Increase turn_speed_rads in bt_reactive_explore_v2.yaml
# Default 1.0 rad/s gives full PWM (200) — check min_pwm in drive.yaml
```

**FrontBlocked triggers immediately at startup:**

```bash
# front_block_m is too large or sensor reads noise
# Check sensor wiring and set front_block_m: 0.20 in bt_reactive_explore_v2.yaml
```

---

## Authors

- **Mohamed Louay SAIDI** — ICE4, 2025–2026
- **Syrine MEKSI** — ICE4, 2025–2026
