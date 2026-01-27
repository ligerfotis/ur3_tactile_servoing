#!/bin/bash
# Cleanup and run simulation
# This script kills any leftover processes before starting

echo "Cleaning up any leftover processes..."
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f servo_node 2>/dev/null
pkill -9 -f move_group 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f keyboard_servo 2>/dev/null
pkill -9 -f test_pub 2>/dev/null
sleep 2

echo "Starting simulation..."
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur3e initial_positions_file:=/home/fotis/ur3_teleop/src/Universal_Robots_ROS2_Description/config/initial_positions.yaml
