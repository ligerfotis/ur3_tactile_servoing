#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveToHome(Node):
    def __init__(self):
        super().__init__('move_to_home_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Hardcoded HOME Joint Values (Radians)
        # This setup usually puts the flange pointing DOWN (Horizontal to ground)
        # Shoulder Pan: 0 (Facing forward)
        # Shoulder Lift: -1.57 (-90 deg, Horizontal arm)
        # Elbow: -1.57 (-90 deg, Forearm down) -- Wait, standard home is all zeros (Vertical).
        # Let's try a known "Table Inspection" pose:
        # shoulder_pan: 0.0
        # shoulder_lift: -1.57 (Horizontal)
        # elbow: 0.0 (Straight out) -> -1.57 (Bent 90)
        # wrist_1: -1.57 (Pointing Down)
        # wrist_2: 1.57
        # wrist_3: 0.0
        
        # Actually, let's target:
        # Pan: 0
        # Lift: -1.57
        # Elbow: 1.57 (Up?) No.
        # Let's use a safe "Candle" then bend.
        
        # "Natural" Observation Pose (High & Looking Down) - SAFER FRONT REACH
        # Shoulder Lift -1.60, Elbow -0.60 (Bent more to avoid singularity), Wrist 1 -0.94 (Compensate)
        # Updated home pose: lower, forward, straight flange, 45° rotated
        self.home_joints = [0.105, -2.35, -1.0, 0.145, 1.57, 0.785]
        # Names must match URDF
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.get_logger().info('Moving to Home Pose (Horizontal Flange)...')
        self.send_goal()

    def send_goal(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.home_joints
        point.time_from_start.sec = 8  # 8 Seconds = Slow & Safe Speed
        
        msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command sent! Duration: {point.time_from_start.sec}s')

def main(args=None):
    rclpy.init(args=args)
    node = MoveToHome()
    # Give time for publisher to connect
    import time
    time.sleep(1)
    node.send_goal()
    time.sleep(1) # Wait for send
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
