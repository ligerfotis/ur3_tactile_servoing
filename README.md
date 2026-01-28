# UR3e Tactile Servoing System

**Real-time closed-loop control of a Universal Robots UR3e using GelSight tactile feedback.**

This repository implements a tactile servoing pipeline that enables a UR3e manipulator to actively respond to contact forces. The system creates a "virtual spring" behavior where the robot continuously tries to return to a home position, but yields to physical touch - like a compliant sponge.

---

## 🏗️ System Architecture

### 1. Tactile Perception
- **Sensor**: GelSight Mini (Optical-Tactile Sensor)
- **Processing**: Tracks marker displacement on the elastomeric surface using optical flow
- **Output**: 3D force vector (`Vector3Stamped`) representing contact direction

### 2. Compliance Control (`tactile_servo_node`)
- **Virtual Spring**: Robot continuously pulls toward home position
- **Touch Response**: When touched, the robot yields in the direction of the force
- **Safety Limits**: Max deviation from home (0.3 rad), stronger spring when far from home
- **Direct Position Control**: Bypasses MoveIt Servo for fast spring-back (~100Hz)

### 3. Motion Control
- **Spring-back**: Uses `forward_position_controller` directly (fast, no Servo limits)
- **Touch motion**: Uses MoveIt Servo for Cartesian velocity from touch forces
- **Singularity handling**: Relaxed thresholds to prevent emergency stops

---

## 🚀 Quickstart Guide

### Prerequisites
- **OS**: Ubuntu 22.04 LTS (Jammy)
- **ROS 2**: Humble Hawksbill
- **Hardware**: UR3e Robot + GelSight Mini

### 1. Installation
```bash
cd ~/ur3_teleop
colcon build --symlink-install
source install/setup.bash
```

### 2. Running on Real Robot

**Terminal 1: Start the robot driver + MoveIt**
```bash
./run_real.sh
```
*Wait for "Robot Connected" and the robot will move to home position.*

**Terminal 2: Activate tactile servoing**
```bash
source install/setup.bash
ros2 launch ur3_tactile_servoing_utils tactile_servoing.launch.py
```

The robot will now behave like a spring - push it and it yields, release and it snaps back!

### 3. Running in Simulation
```bash
./run_simulation.sh
# Then in another terminal:
ros2 launch ur3_tactile_servoing_utils tactile_servoing.launch.py
```

---

## ⚙️ Configuration

### Home Position
Edit in `src/ur3_tactile_servoing_utils/ur3_tactile_servoing_utils/tactile_servo.py`:
```python
self.home_joints = [0.105, -2.35, -1.0, 0.145, 1.57, 0.785]
# Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
```

Also update `move_to_home.py` to match!

### Control Parameters
In `tactile_servo.py`:
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `tactile_speed` | `0.3` | Speed when touched (m/s) |
| `spring_speed` | `0.3` | Max spring return speed (rad/s) |
| `dead_zone` | `0.005` | Min force to trigger motion |
| `max_deviation` | `0.3` | Max distance from home (rad) |

### Servo Config
In `config/my_servo_config.yaml`:
- `lower_singularity_threshold`: 10000 (relaxed to avoid emergency stops)
- `hard_stop_singularity_threshold`: 20000

---

## 🔧 Troubleshooting

**Robot moves slowly / doesn't snap back**
- Check that `forward_position_controller` is active
- Run: `ros2 control list_controllers`

**"Very close to a singularity" warnings**
- The current thresholds are very relaxed; if still occurring, adjust home pose away from singularities

**MoveIt won't plan/execute**
- The tactile servoing uses `forward_position_controller`; MoveIt needs `joint_trajectory_controller`
- Switch: `ros2 control switch_controllers --activate joint_trajectory_controller --deactivate forward_position_controller`

**GelSight camera not found**
- Check: `ls /dev/video*` and `sudo chmod 666 /dev/video*`

---

*Verified by Antigravity - January 28, 2026*
