# UR3e Tactile Servoing System

**Real-time closed-loop control of a Universal Robots UR3e using GelSight tactile feedback.**

This repository implements a tactile servoing pipeline that enables a UR3e manipulator to actively respond to contact forces. By closing the loop between a **GelSight Mini** sensor and the **MoveIt Servo** controller, the system enables compliant, force-aware manipulation where the robot yields to physical interaction in real-time.

---

## 🏗️ System Architecture

The system is composed of three primary subsystems operating in a closed feedback loop:

### 1. Tactile Perception
*   **Sensor**: GelSight Mini (Optical-Tactile Sensor).
*   **Processing**: The perception node tracks high-contrast marker displacement on the elastomeric surface.
*   **Output**: This flow field is mapped to a 3-dimensional force vector ($F_x, F_y, F_z$) published as a `Vector3Stamped` message.

### 2. High-Level Control Logic
*   **Node**: `tactile_servo_node`
*   **Function**: Transforms raw force vectors into Cartesian velocity commands for the robot end-effector.
*   **Anisotropic Gain Scheduling**: To improve controllability, the system applies independent gains for lateral (shear) and normal (pressure) forces. This allows for sensitive "steering" responsiveness while maintaining stable contact pressure.

### 3. Motion Control
*   **Controller**: `forward_position_controller` driven by **MoveIt Servo**.
*   **Inverse Kinematics**: Converts the Cartesian velocity command into joint-space velocities using the Inverse Jacobian, while actively avoiding singularities and joint limits.

---

## 🚀 Quickstart Guide

### Prerequisites
*   **OS**: Ubuntu 22.04 LTS (Jammy)
*   **ROS 2**: Humble Hawksbill
*   **Hardware**: UR3e Robot (or Gazebo Simulation) + GelSight Mini

### 1. Installation
Clone the repository and build the colcon workspace:
```bash
cd ~/ur3_teleop  # Project folder
colcon build --symlink-install
source install/setup.bash
```

### 2. Launching the Simulation
We provide a unified launch script that initializes the Gazebo environment, loads the robot description, and starts the MoveIt planning pipeline.

**Terminal 1: Simulation Backend**
```bash
./run_simulation.sh
```
*Wait for the MoveGroup interface to initialize and the robot to settle in its home position.*

### 3. Activating Tactile Control
Once the simulation is running, launch the tactile servoing pipeline. This script performs an atomic controller switch to prepare the robot for velocity streaming.

**Terminal 2: Control Node**
```bash
source install/setup.bash
ros2 launch ur3_tactile_servoing_utils tactile_servoing.launch.py
```

**System Status**: The robot should remain rigid in its initial pose. Interacting with the GelSight sensor (real or simulated signal) will now drive the robot end-effector away from the applied force vector.

---

## ⚙️ Configuration Parameters

The control behavior can be tuned in `src/ur3_tactile_servoing_utils/launch/tactile_servoing.launch.py`:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `lateral_velocity_gain` | `15.0` | Sensitivity to shear forces ($X, Y$). Higher values increase steering responsiveness. |
| `normal_velocity_gain` | `2.0` | Sensitivity to normal force ($Z$). Lower values provide "heavier" resistance to pushing. |
| `max_velocity` | `0.5` | Saturation limit (m/s) for the output velocity command. |
| `dead_zone` | `0.005` | Minimum force magnitude required to trigger motion (noise filtering). |

### Initial Robot Pose
The robot's home configuration is defined in `src/Universal_Robots_ROS2_Description/config/initial_positions.yaml`. The default is a **Horizontal Flange** pose optimized to maximize the workspace and strictly avoid wrist singularities.

---

## 🔧 Troubleshooting

**Error: "No GelSight cameras found"**
*   **Context**: The perception node cannot access the USB video device.
*   **Solution**: Verify the USB connection and check permissions: `sudo chmod 666 /dev/video*`.

**Warning: "Very close to a singularity"**
*   **Context**: The robot has reached a geometric configuration where it loses a degree of freedom (e.g., wrist alignement).
*   **Solution**: The servoing loop will halt to prevent unstable motion. Restart the simulation or jog the robot away from the singularity.

---

*Verified by Antigravity - January 28, 2026*
