#!/usr/bin/env python3
"""
Tactile Servoing Node
Subscribes to tactile force vector and controls robot via MoveIt Servo
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import math

class TactileServoNode(Node):
    def __init__(self):
        super().__init__('tactile_servo_node')
        
        # Parameters
        self.declare_parameter('force_topic', '/tactile/force_vector')
        self.declare_parameter('velocity_gain', 0.1)  # m/s per unit force
        self.declare_parameter('max_velocity', 0.2)   # m/s max
        self.declare_parameter('dead_zone', 0.05)     # Ignore forces below this
        self.declare_parameter('invert_direction', False)  # True to push back against force
        
        self.velocity_gain = self.get_parameter('velocity_gain').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.invert = self.get_parameter('invert_direction').value
        
        # Subscriber to tactile force
        force_topic = self.get_parameter('force_topic').value
        self.force_sub = self.create_subscription(
            Vector3Stamped,
            force_topic,
            self.force_callback,
            10
        )
        
        # Publisher to servo
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        self.get_logger().info(f'Tactile Servo Node started')
        self.get_logger().info(f'  Force topic: {force_topic}')
        self.get_logger().info(f'  Velocity gain: {self.velocity_gain}')
        self.get_logger().info(f'  Max velocity: {self.max_velocity}')
        self.get_logger().info(f'  Dead zone: {self.dead_zone}')
        self.get_logger().info(f'  Invert direction: {self.invert}')
    
    def force_callback(self, msg: Vector3Stamped):
        """Convert force vector to velocity command"""
        # Extract force vector
        fx = msg.vector.x
        fy = msg.vector.y
        fz = msg.vector.z
        
        # Calculate magnitude
        magnitude = math.sqrt(fx**2 + fy**2 + fz**2)
        
        # Apply dead zone
        if magnitude < self.dead_zone:
            # Publish zero velocity
            self.publish_zero_velocity()
            return
        
        # Normalize and scale
        if magnitude > 0:
            direction = -1.0 if self.invert else 1.0
            scale = min(magnitude * self.velocity_gain, self.max_velocity) / magnitude
            
            vx = fx * scale * direction
            vy = fy * scale * direction
            vz = fz * scale * direction
        else:
            vx = vy = vz = 0.0
        
        # Publish velocity command
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'tool0'
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        self.twist_pub.publish(twist)
        
        # Log occasionally
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
            
        if self._log_counter % 50 == 0:  # Log every 50 messages
            self.get_logger().info(
                f'Force: [{fx:.3f}, {fy:.3f}, {fz:.3f}] (mag: {magnitude:.3f}) '
                f'-> Vel: [{vx:.3f}, {vy:.3f}, {vz:.3f}]'
            )
    
    def publish_zero_velocity(self):
        """Publish zero velocity to stop robot"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'tool0'
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TactileServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
