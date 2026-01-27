#!/bin/bash
# Helper script to launch tactile perception within its conda environment

# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Source this workspace
source /home/fotis/ur3_teleop/install/setup.bash

# 3. Activate Conda Environment
# We use the full path to conda to be safe
source /home/fotis/anaconda3/etc/profile.d/conda.sh
conda activate tactile_perception_ros

# 4. Launch the tactile flow
echo "Starting Tactile Perception Flow..."
ros2 launch tactile_perception_ros gelsight_tactile_flow.launch.py
