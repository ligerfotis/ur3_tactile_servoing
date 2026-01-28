#!/bin/bash
# Helper script to launch the REAL UR3e robot driver
# Usage: ./run_real.sh [ROBOT_IP]

# Default Robot IP if not provided
ROBOT_IP=${1:-"192.168.1.201"}

echo "=================================================="
echo "🚀 Launching UR3e Driver for Real Robot"
echo "🤖 Robot IP: $ROBOT_IP"
echo "💻 Your PC IP (must match External Control): 192.168.1.100"
echo "=================================================="

# Source the workspace
source install/setup.bash

echo "🧹 Cleaning up old processes..."
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f ur_robot_driver 2>/dev/null
pkill -9 -f controller_manager 2>/dev/null
pkill -9 -f move_group 2>/dev/null
sleep 1

# Launch the driver
# - ur_type: ur3e
# - robot_ip: The IP of the robot
# - launch_rviz: Open RViz to see the robot
# - headless_mode: true (prevents URCap from taking over screen, optional)
# - initial_joint_controller: joint_trajectory_controller (Standard for startup)

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=$ROBOT_IP \
    kinematics_params_file:=${HOME}/ur3_teleop/ur3e_calibration.yaml \
    controllers_file:=${HOME}/ur3_teleop/my_controllers.yaml \
    launch_rviz:=true \
    initial_joint_controller:=joint_trajectory_controller
