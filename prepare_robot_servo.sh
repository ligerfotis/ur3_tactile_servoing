#!/bin/bash
# Prepares the UR3e to receive Servo commands (Tactile or Keyboard)

echo "=== Preparing UR3e for Servo Commands ==="

# Step 1: Start the Servo Node
echo "Step 1: Starting servo node..."
ros2 launch ur3_teleop_utils start_servo.launch.py &
SERVO_PID=$!
sleep 4

# Step 2: Start the Servo Service
echo "Step 2: Starting servo service..."
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} > /dev/null

# Step 3: Switch Controllers
echo "Step 3: Switching controllers (Trajectory -> Forward Position)..."
ros2 control load_controller forward_position_controller 2>/dev/null
ros2 control set_controller_state joint_trajectory_controller inactive
ros2 control set_controller_state forward_position_controller active

echo ""
echo "=== Robot Ready! ==="
echo "You can now run your teleop or tactile nodes."

# Wait for the background process
wait $SERVO_PID
