#!/bin/bash
# Complete teleop startup script
# Handles the timing correctly to avoid robot "dropping"

echo "=== UR3 Teleop Startup ==="

# Cleanup any leftover processes
echo "Cleaning up leftover processes..."
pkill -9 -f servo_node 2>/dev/null
pkill -9 -f keyboard_servo 2>/dev/null
pkill -9 -f test_pub 2>/dev/null
sleep 1

# Step 1: Launch servo node FIRST (in background)
echo "Step 1: Starting servo node..."
# Use ros2 launch with package
ros2 launch ur3_tactile_servoing_utils start_servo.launch.py &
SERVO_PID=$!
sleep 4

# Step 2: Start servo service
echo "Step 2: Starting servo service..."
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {} > /dev/null

# Step 3: Load and configure forward_position_controller
echo "Step 3: Loading controller..."
ros2 control load_controller forward_position_controller 2>/dev/null
ros2 control set_controller_state forward_position_controller inactive 2>/dev/null

# Step 4: Deactivate trajectory controller FIRST to free joints
echo "Step 4: Deactivating trajectory controller..."
ros2 control set_controller_state joint_trajectory_controller inactive 2>/dev/null

# Step 5: Start position hold BEFORE activating forward controller
echo "Step 5: Starting position hold..."
python3 -c "
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

rclpy.init()
node = Node('hold_position')
pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

for i in range(200):  # 4 seconds at 50Hz
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'tool0'
    pub.publish(msg)
    time.sleep(0.02)

node.destroy_node()
rclpy.shutdown()
" &
HOLD_PID=$!

sleep 0.5

# Step 6: NOW activate forward_position_controller (joints are free now)
echo "Step 6: Activating forward_position_controller..."
ros2 control set_controller_state forward_position_controller active

# Wait for hold publisher to finish
wait $HOLD_PID 2>/dev/null

# Verify controller is active
echo "Verifying controller..."
ros2 control list_controllers | grep forward

echo ""
echo "Step 7: Starting keyboard teleop..."
echo "=== Ready! Press keys to move the robot ==="
# Use ros2 run with package
ros2 run ur3_tactile_servoing_utils keyboard_servo

# Cleanup on exit
echo "Cleaning up..."
kill $SERVO_PID 2>/dev/null
ros2 control set_controller_state forward_position_controller inactive 2>/dev/null
ros2 control set_controller_state joint_trajectory_controller active 2>/dev/null
echo "Done."
