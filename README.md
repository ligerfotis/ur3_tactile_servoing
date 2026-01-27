# UR3e Teleoperation Project

This project simulates a UR3e robot in Gazebo and provides a keyboard teleoperation interface using MoveIt Servo.

## 🚀 Quick Start

## 📦 Installation

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install python3-vcstool \
                 ros-humble-moveit-servo \
                 ros-humble-ur-description \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-ur-robot-driver
```

### 2. Import Repositories
```bash
vcs import src < ur3_teleop.repos
```

### 3. Build the Workspace
```bash
cd /home/fotis/ur3_teleop
colcon build --symlink-install
```

## 🚀 Quick Start / Usage

### 1. Run Simulation (Terminal 1)
Starts Gazebo and RViz. A "ready" stance is automatically set.
```bash
source install/setup.bash
./run_simulation.sh
```
*Wait ~10-15 seconds for Gazebo and RViz to fully load.*

### 2. Run Teleoperation (Terminal 2)
Starts the servo node, switches controllers, and launches the keyboard interface.
```bash
source install/setup.bash
ros2 run ur3_teleop_utils start_teleop.sh
```

### 3. Run Tactile Perception (Terminal 3)
Starts the tactile sensor nodes within the conda environment.
```bash
./start_tactile.sh
```

### 4. Run Tactile Servoing (Terminal 4)
Starts the node that translates force vectors to robot movement.
```bash
source install/setup.bash
ros2 run ur3_teleop_utils tactile_servo
```

## 🎮 Controls

Click inside the **Teleoperation Terminal (Terminal 2)** to control the robot:

| Key | Action |
| :--- | :--- |
| **w / s** | Move Forward / Backward (X-axis) |
| **a / d** | Move Left / Right (Y-axis) |
| **q / e** | Move Up / Down (Z-axis) |
| **1 / 2** | Decrease / Increase Speed |

**Note**: The default speed is set to `0.1` m/s for safety.

**Tactile Mode**: Apply force to the sensor to move the end-effector.

## 🛠 Project Structure

*   `ur3_teleop.repos`: Dependency definitions.
*   `run_simulation.sh`: Utility script to clean up processes and launch the simulation.
*   `src/ur3_teleop_utils/`: Main ROS 2 package.
    *   `scripts/start_teleop.sh`: Main script for teleoperation.
    *   `ur3_teleop_utils/keyboard_servo.py`: Python module for keyboard input.
    *   `launch/start_servo.launch.py`: Launcher for the MoveIt Servo node.
    *   `config/`: Configuration files.

## ⚠️ Troubleshooting

*   **Robot "Drops"**: This is fixed by `start_teleop.sh`. If it happens, ensure you are using the script and not running commands manually.
*   **Keys not working**: Ensure the teleoperation terminal has focus.
*   **Simulation Glitches**: If Gazebo hangs, `run_simulation.sh` automatically kills stale processes (gzserver/gzclient) before starting.
