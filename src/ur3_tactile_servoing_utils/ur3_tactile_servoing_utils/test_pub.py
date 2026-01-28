#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.get_logger().info('Test publisher started - publishing z=0.1 velocity')
        
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tool0'
        msg.twist.linear.z = 0.1  # Move up in tool frame
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
