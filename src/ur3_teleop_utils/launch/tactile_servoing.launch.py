from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    teleop_share = get_package_share_directory('ur3_teleop_utils')
    tactile_share = get_package_share_directory('tactile_perception_ros')

    # 1. Tactile Perception (Gelsight + Flow)
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

    # 3. Step 1: Load and Activate the position controller
    # We use a spawner which is more robust than manual ros2 control calls in launch
    activate_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4. Step 2: Atomic switch once the spawner has finished its job (ensuring it's loaded)
    # This command deactivates the trajectory controller and ensures position is active
    controller_switch = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', 
             '--deactivate', 'joint_trajectory_controller', 
             '--activate', 'forward_position_controller'],
        output='screen'
    )

    # 5. Step 3: Start Servo service
    start_servo_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen'
    )

    # 6. Step 4: Start the bridge node (The Brain)
    tactile_bridge = Node(
        package='ur3_teleop_utils',
        executable='tactile_servo',
        name='tactile_servo_node',
        parameters=[{
            'velocity_gain': 10.0,
            'dead_zone': 0.005,
            'max_velocity': 0.5
        }],
        output='screen'
    )

    return LaunchDescription([
        tactile_flow,
        moveit_servo,
        activate_position_controller,
        
        # Sequence logic:
        # After spawner finishes -> Switch controllers -> Start Servo Service -> Start Bridge
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=activate_position_controller,
                on_exit=[
                    controller_switch,
                    TimerAction(period=1.0, actions=[start_servo_service]),
                    TimerAction(period=2.0, actions=[tactile_bridge]),
                ],
            )
        ),
    ])
