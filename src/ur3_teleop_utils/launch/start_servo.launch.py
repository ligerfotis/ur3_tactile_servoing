import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Parameters
    ur_type = "ur3e"

    # Define Servo Parameters explicitly - standard keys
    base_params = {
        "move_group_name": "ur_manipulator",
        "planning_frame": "base_link",
        "ee_frame_name": "tool0",
        "robot_link_command_frame": "tool0",
        
        "use_gazebo": True,
        
        "command_in_type": "speed_units",
        "command_out_type": "std_msgs/Float64MultiArray",
        
        "publish_period": 0.004,
        "publish_joint_positions": True,
        "publish_joint_velocities": False,
        "publish_joint_accelerations": False,
        
        "command_out_topic": "/forward_position_controller/commands",
        "cartesian_command_in_topic": "/servo_node/delta_twist_cmds", # Absolute topic
        "joint_command_in_topic": "/servo_node/delta_joint_cmds",
        "joint_topic": "/joint_states",
        "status_topic": "~/status",
        
        "check_collisions": True,
        "collision_check_rate": 5.0,
        "collision_check_type": "threshold_distance",
        "self_collision_proximity_threshold": 0.01,
        "scene_collision_proximity_threshold": 0.02,
        "collision_distance_safety_factor": 1000.0,
        "min_allowable_collision_distance": 0.01,
        
        "scale.linear": 0.6,
        "scale.rotational": 0.3,
        "scale.joint": 0.01,
        
        "incoming_command_timeout": 0.1,
        "num_outgoing_halt_msgs_to_publish": 4,
        "low_latency_mode": False,
        
        "smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
        "is_primary_planning_scene_monitor": False,
        "low_pass_filter_coeff": 10.0,
    }
    
    # Generate Namespaced Params
    final_params = {}
    for key, val in base_params.items():
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
