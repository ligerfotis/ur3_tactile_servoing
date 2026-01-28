#!/bin/bash
echo "🔄 Switching Robot to MoveIt Mode (Trajectory Control)..."

# Deactivate Servoing Controller
ros2 control set_controller_state forward_position_controller inactive

# Activate MoveIt Controller
ros2 control set_controller_state joint_trajectory_controller active

echo "✅ Robot is ready for MoveIt Planning!"
