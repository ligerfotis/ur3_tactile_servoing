from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    teleop_share = get_package_share_directory('ur3_teleop_utils')
    tactile_share = get_package_share_directory('tactile_perception_ros')

    # 1. Tactile Perception
    tactile_flow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tactile_share, 'launch', 'gelsight_tactile_flow.launch.py')
        )
    )

    # 2. MoveIt Servo Node
    moveit_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_share, 'launch', 'start_servo.launch.py')
        )
    )

    # 3. Robust Controller Switch & Servo Start
    # This script ensures:
    # - forward_position_controller is loaded (if not already)
    # - Atomic switch happens from trajectory -> position
    # - Servo node is started via service call
    setup_robot = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ros2 control load_controller forward_position_controller > /dev/null 2>&1 || true; '
            'sleep 1; '
            'ros2 control set_controller_state forward_position_controller inactive > /dev/null 2>&1 || true; '
            'sleep 1; '
            'ros2 control switch_controllers --activate forward_position_controller --deactivate joint_trajectory_controller; '
            'sleep 1; '
            'ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}'
        ],
        output='screen'
    )

    # 4. Tactile Bridge Node (The Brain)
    tactile_bridge = Node(
        package='ur3_teleop_utils',
        executable='tactile_servo',
        name='tactile_servo_node',
        parameters=[{
            'lateral_velocity_gain': 15.0,
            'normal_velocity_gain': 2.0,
            'dead_zone': 0.005,
            'max_velocity': 0.5,
            'invert_direction': True
        }],
        output='screen'
    )

    return LaunchDescription([
        tactile_flow,
        moveit_servo,
        # Give nodes time to initialize before switching controllers
        TimerAction(period=5.0, actions=[setup_robot]),
        # Start bridge shortly after setup is complete (Total ~10s)
        TimerAction(period=10.0, actions=[tactile_bridge]),
    ])
