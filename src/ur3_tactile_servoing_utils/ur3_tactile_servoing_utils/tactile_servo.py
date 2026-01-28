#!/usr/bin/env python3
"""
Tactile Servoing Node - Compliance Control
Implements virtual spring behavior: robot continuously tries to return to home,
touch forces can push it away, release = snap back.

Uses DIRECT position control for fast spring-back (bypasses MoveIt Servo limits).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
import time

class TactileServoNode(Node):
    def __init__(self):
        super().__init__('tactile_servo_node')
        
        # Parameters
        self.declare_parameter('force_topic', '/tactile/force_vector')
        self.declare_parameter('tactile_speed', 0.3)        # Speed when touched (m/s)
        self.declare_parameter('spring_speed', 0.3)         # Speed returning to home (rad/s per joint)
        self.declare_parameter('dead_zone', 0.005)
        self.declare_parameter('invert_direction', False)
        
        self.tactile_speed = self.get_parameter('tactile_speed').value
        self.spring_speed = self.get_parameter('spring_speed').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.invert = self.get_parameter('invert_direction').value
        
        # Home position - user-defined pose
        # Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        # wrist_2=1.57 (90°) for straight flange, wrist_3=0.785 (45°) rotation
        # shoulder_lift lowered to bring EE down and forward
        self.home_joints = [0.105, -2.35, -1.0, 0.145, 1.57, 0.785]
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        # State
        self.current_joints = list(self.home_joints)  # Start at home
        self.target_joints = list(self.home_joints)   # Target position
        self.last_force_time = 0.0
        self.last_touch_direction = [0.0, 0.0, 0.0]
        
        # Jacobian approximation for touch->joint mapping (simplified)
        # This maps end-effector velocity to joint velocities
        # For UR3e, wrist joints mainly affect orientation, shoulder/elbow affect position
        
        # Force subscriber
        self.force_sub = self.create_subscription(
            Vector3Stamped,
            self.get_parameter('force_topic').value,
            self.force_callback,
            10
        )
        
        # Twist publisher for MoveIt Servo (tactile control)
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Direct position publisher (BYPASSES SERVO for spring-back!)
        # Queue size = 1 to prevent command buffering/delay
        self.pos_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            1  # IMPORTANT: No queuing - only latest command matters
        )
        
        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Fast control loop (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Tactile Servo Node started (DIRECT POSITION CONTROL for fast spring-back)')
    
    def joint_callback(self, msg: JointState):
        """Track current joint positions"""
        temp_joints = {}
        for name, pos in zip(msg.name, msg.position):
            temp_joints[name] = pos
        try:
            self.current_joints = [temp_joints[n] for n in self.joint_names]
        except KeyError:
            pass
    
    def force_callback(self, msg: Vector3Stamped):
        """Store tactile force direction for combining with spring"""
        fx, fy, fz = msg.vector.x, msg.vector.y, msg.vector.z
        magnitude = math.sqrt(fx**2 + fy**2 + fz**2)
        
        if magnitude >= self.dead_zone:
            direction = -1.0 if self.invert else 1.0
            # Normalize
            self.last_touch_direction = [
                fx / magnitude * direction,
                fy / magnitude * direction,
                fz / magnitude * direction
            ]
            self.last_force_time = time.time()
        else:
            # Clear touch direction immediately when not touching
            self.last_touch_direction = [0.0, 0.0, 0.0]
    
    def timer_callback(self):
        """Main control loop - combines spring + touch using DIRECT position control"""
        now = time.time()
        dt = 0.01  # 100Hz loop
        
        is_touched = (now - self.last_force_time) < 0.15
        
        # Calculate target: blend between spring-return and touch-push
        new_target = []
        max_deviation = 0.3  # Max allowed deviation from home (radians, ~17 degrees)
        
        for i, (current, home) in enumerate(zip(self.current_joints, self.home_joints)):
            # Spring force: always pull toward home
            error = home - current
            deviation = abs(error)
            
            # Spring gets STRONGER when far from home (prevents runaway)
            if deviation > max_deviation * 0.5:
                spring_strength = 10.0  # Double strength when getting far
            else:
                spring_strength = 5.0
            
            spring_delta = error * spring_strength * dt
            spring_delta = max(min(spring_delta, self.spring_speed * dt), -self.spring_speed * dt)
            
            if is_touched:
                # When touched: move AWAY from home in touch direction
                # Map touch direction to joints - REBALANCED for better lateral response
                touch_effect = 0.0
                if i == 0:  # shoulder_pan -> X direction (INCREASED)
                    touch_effect = -self.last_touch_direction[0] * self.tactile_speed * dt * 3.0
                elif i == 1:  # shoulder_lift -> Z direction (REDUCED)
                    touch_effect = -self.last_touch_direction[2] * self.tactile_speed * dt * 0.5
                elif i == 2:  # elbow -> Z direction (REDUCED)
                    touch_effect = -self.last_touch_direction[2] * self.tactile_speed * dt * 0.3
                elif i == 3:  # wrist_1 -> Y direction (INCREASED)
                    touch_effect = self.last_touch_direction[1] * self.tactile_speed * dt * 2.0
                
                # SAFETY: Reduce touch effect if too far from home (prevents divergence)
                if deviation > max_deviation * 0.7:
                    touch_effect *= 0.3  # Dampen touch when near limits
                
                # Combine: spring is reduced when touched (but not too much)
                target_delta = spring_delta * 0.5 + touch_effect
            else:
                # Not touched: full spring return
                target_delta = spring_delta
            
            new_pos = current + target_delta
            
            # HARD CLAMP: Never go beyond max deviation from home
            new_pos = max(min(new_pos, home + max_deviation), home - max_deviation)
            
            new_target.append(new_pos)
        
        # Publish direct position command (FAST!)
        pos_msg = Float64MultiArray()
        pos_msg.data = new_target
        self.pos_pub.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TactileServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
