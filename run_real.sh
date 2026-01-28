#!/bin/bash
# Helper script to launch the REAL UR3e robot driver
# Usage: ./run_real.sh [ROBOT_IP]

# Default Robot IP if not provided
ROBOT_IP=${1:-"192.168.1.201"}

echo "=================================================="
echo "🧹 Cleaning up old processes..."
pkill -9 -f rviz2 2>/dev/null
pkill -f ur_robot_driver 2>/dev/null
pkill -f tactile_servo 2>/dev/null
pkill -f tactile_flow 2>/dev/null
pkill -f gelsight 2>/dev/null
sleep 2

echo "🚀 Launching UR3e Driver for Real Robot"
echo "🤖 Robot IP: $ROBOT_IP"
echo "💻 Your PC IP (must match External Control): 192.168.1.100"
echo "=================================================="

# Source the workspace
source install/setup.bash

# Launch the driver
# - ur_type: ur3e
# - robot_ip: The IP of the robot
# - launch_rviz: Open RViz to see the robot
# - headless_mode: true (prevents URCap from taking over screen, optional)
# - initial_joint_controller: joint_trajectory_controller (Standard for startup)

# Launch the driver in background
echo "Starting Driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=$ROBOT_IP \
    kinematics_params_file:=${HOME}/ur3_teleop/ur3e_calibration.yaml \
    controllers_file:=${HOME}/ur3_teleop/my_controllers.yaml \
    launch_rviz:=false \
    initial_joint_controller:=joint_trajectory_controller &
DRIVER_PID=$!

sleep 5

# Launch MoveIt! (Provides Planning Scene & Collision Check)
echo "Starting MoveIt..."
ros2 launch my_ur_moveit.launch.py \
    ur_type:=ur3e \
    launch_rviz:=true \
    use_sim_time:=false &
MOVEIT_PID=$!

echo "✅ Full System Launched!"
echo "   - Driver PID: $DRIVER_PID"
echo "   - MoveIt PID: $MOVEIT_PID"
echo "👉 Press Play on the Robot Tablet Now!"
echo "   (Ctrl+C to stop everything)"


# Background waiter to move to Home Pose once ready
(
    echo "⏳ Waiting for Robot to be Ready (Press Play on Tablet)..."
    # Wait for controller to be active
    until ros2 control list_controllers | grep "joint_trajectory_controller" | grep "active" > /dev/null; do
        sleep 2
    done
    echo "✅ Robot Connected! Moving to Home Pose in 5 seconds..."
    sleep 5
    ./move_to_home.py
) &

# Wait for process to exit
wait $DRIVER_PID
