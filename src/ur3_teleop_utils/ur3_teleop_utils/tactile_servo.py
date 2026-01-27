#!/usr/bin/env python3
"""
Tactile Servoing Node
Subscribes to tactile force vector and controls robot via MoveIt Servo
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import math
import time

class TactileServoNode(Node):
    def __init__(self):
        super().__init__('tactile_servo_node')
        
        # Parameters
        self.declare_parameter('force_topic', '/tactile/force_vector')
        self.declare_parameter('lateral_velocity_gain', 15.0)  # XY m/s per unit force
        self.declare_parameter('normal_velocity_gain', 5.0)   # Z m/s per unit force
        self.declare_parameter('max_velocity', 0.5)           # m/s max
        self.declare_parameter('dead_zone', 0.005)            # Ignore forces below this
        self.declare_parameter('invert_direction', False)
        
        self.lateral_gain = self.get_parameter('lateral_velocity_gain').value
        self.normal_gain = self.get_parameter('normal_velocity_gain').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.invert = self.get_parameter('invert_direction').value
        
        # State
        self.last_twist = TwistStamped()
        self.last_force_time = 0.0
        
        # Subscriber
        self.force_sub = self.create_subscription(
            Vector3Stamped,
            self.get_parameter('force_topic').value,
            self.force_callback,
            10
        )
        
        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Timer to ensure steady command stream (holds robot in place)
        # 50Hz = 0.02s
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Tactile Servo Node started (Steady-stream enabled)')
    
    def force_callback(self, msg: Vector3Stamped):
        """Convert tactile force to a pending velocity command"""
        fx, fy, fz = msg.vector.x, msg.vector.y, msg.vector.z
        magnitude = math.sqrt(fx**2 + fy**2 + fz**2)
        
        twist = TwistStamped()
        twist.header.frame_id = 'tool0'
        
        if magnitude >= self.dead_zone:
            direction = -1.0 if self.invert else 1.0
            
            # Apply different gains for lateral (XY) and normal (Z)
            vx = fx * self.lateral_gain * direction
            vy = fy * self.lateral_gain * direction
            vz = fz * self.normal_gain * direction
            
            # Clamp total velocity magnitude while preserving direction
            v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
            if v_mag > self.max_velocity:
                scale = self.max_velocity / v_mag
                vx *= scale
                vy *= scale
                vz *= scale
                
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = vz
            self.last_force_time = time.time()
        else:
            # Below dead zone = Zero velocity
            pass
            
        self.last_twist = twist

    def timer_callback(self):
        """Consistently publish the last known command (or zero) to keep Servo active"""
        # If no force for 0.5s, force a zero command
        if time.time() - self.last_force_time > 0.5:
            self.last_twist = TwistStamped()
            self.last_twist.header.frame_id = 'tool0'
            
        self.last_twist.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(self.last_twist)

def main(args=None):
    rclpy.init(args=args)
    node = TactileServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop command
        stop = TwistStamped()
        node.twist_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
