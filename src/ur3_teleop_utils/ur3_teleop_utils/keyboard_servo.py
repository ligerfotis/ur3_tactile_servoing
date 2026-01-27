#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to TwistStamped!
---------------------------
Planar Movement:
   w / s : Forward / Backward (X+/X-)
   a / d : Left / Right (Y+/Y-)

Height Control:
   q / e : Up / Down (Z+/Z-)

Speed Control:
   1 / 2 : Decrease / Increase Max Speed

anything else : stop

CTRL-C to quit
"""

# Mappings: (x, y, z, th)
moveBindings = {
    'w': (1, 0, 0, 0),       # Forward
    's': (-1, 0, 0, 0),      # Backward
    'a': (0, 1, 0, 0),       # Left
    'd': (0, -1, 0, 0),      # Right
    'q': (0, 0, 1, 0),       # Up
    'e': (0, 0, -1, 0),      # Down
}

speedBindings = {
    '2': (1.1, 1.1),
    '1': (0.9, 0.9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('keyboard_servo_teleop')
    pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    
    speed = 0.1
    turn = 0.5
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey(settings)
            
            # Debugging print to confirm key press
            if key != '':
                 print(f"Key Pressed: {key!r}", flush=True)

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(f"currently:\tspeed {speed:.2f}", flush=True)
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                continue
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'): # CTRL-C
                    break
            
            # Publish command
            twist = TwistStamped()
            twist.header.stamp = node.get_clock().now().to_msg()
            twist.header.frame_id = "tool0"
            twist.twist.linear.x = x * speed
            twist.twist.linear.y = y * speed
            twist.twist.linear.z = z * speed
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)
    finally:
        twist = TwistStamped()
        twist.twist.linear.x = 0.0; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
