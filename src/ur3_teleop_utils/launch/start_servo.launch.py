import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Parameters
    ur_type = "ur3e"

    # 1. Load Servo Parameters from YAML
    pkg_share = get_package_share_directory('ur3_teleop_utils')
    config_file = os.path.join(pkg_share, 'config', 'my_servo_config.yaml')
    
    with open(config_file, 'r') as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = servo_yaml['servo_node']['ros__parameters']
    
    # Standard MoveIt Servo requirement: parameters must be under 'moveit_servo' namespace
    # OR the node itself must be named 'servo_node'.
    # We'll provide both for robustness.
    final_params = {}
    for key, val in servo_params.items():
        final_params[key] = val
        final_params[f"moveit_servo.{key}"] = val

    # 2. Robot Description (URDF)
    # Get the package share directory
    pkg_share = get_package_share_directory('ur3_teleop_utils')
    config_file = os.path.join(pkg_share, 'config', 'my_servo_config.yaml')
    robot_description_config = xacro.process_file(
        os.path.join(get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"),
        mappings={"name": "ur", "ur_type": ur_type}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # 3. Robot Description Semantic (SRDF)
    srdf_path = os.path.join(get_package_share_directory("ur_moveit_config"), "srdf", "ur.srdf.xacro")
    robot_description_semantic_config = xacro.process_file(
        srdf_path,
        mappings={"name": "ur", "ur_type": ur_type}
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config.toxml()}

    # Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        # name="servo_node",
        parameters=[
            final_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )
    
    return LaunchDescription([
        servo_node,
    ])
