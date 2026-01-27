#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
import time

msg = """
Reading from the keyboard and Publishing to TwistStamped!
---------------------------
Multi-Key Support Enabled (Diagonal Movement!)

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

HOLD_TIMEOUT = 0.5  # Keys are considered "held" for 500ms after last press

def getKeys(settings):
    """
    Reads all available keys from stdin without blocking.
    Returns a string of characters.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)  # Short poll
    keys = ''
    if rlist:
        try:
            # Drain the buffer
            while True:
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.0) 
                if rlist2:
                    keys += sys.stdin.read(1)
                else:
                    break
        except Exception:
            pass
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keys

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('keyboard_servo_teleop')
    pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    
    speed = 0.1
    turn = 0.5
    status = 0
    
    # State tracking: key -> last_press_time
    key_states = {}

    try:
        print(msg)
        while(1):
            current_time = time.time()
            key_batch = getKeys(settings)
            
            # Update state for any keys received
            for k in key_batch:
                if k in moveBindings.keys():
                    key_states[k] = current_time
                elif k in speedBindings.keys():
                    speed = speed * speedBindings[k][0]
                    turn = turn * speedBindings[k][1]
                    print(f"currently:\tspeed {speed:.2f}", flush=True)
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                elif k == '\x03':  # CTRL-C
                    raise KeyboardInterrupt()

            # Calculate movement vector from all "active" keys
            x = 0
            y = 0
            z = 0
            th = 0
            
            active_keys = []
            for key, binding in moveBindings.items():
                if key in key_states and (current_time - key_states[key]) < HOLD_TIMEOUT:
                    x += binding[0]
                    y += binding[1]
                    z += binding[2]
                    th += binding[3]
                    active_keys.append(key)
            
            # Clamp values to -1, 0, 1
            x = max(min(x, 1), -1)
            y = max(min(y, 1), -1)
            z = max(min(z, 1), -1)

            if active_keys:
                print(f"Active keys: {active_keys} -> Move: [{x}, {y}, {z}]", flush=True)

            # Publish command
            twist = TwistStamped()
            twist.header.stamp = node.get_clock().now().to_msg()
            twist.header.frame_id = "tool0"
            twist.twist.linear.x = float(x) * speed
            twist.twist.linear.y = float(y) * speed
            twist.twist.linear.z = float(z) * speed
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = float(th) * turn
            pub.publish(twist)

    except KeyboardInterrupt:
        print("\nCTRL-C")

    finally:
        twist = TwistStamped()
        twist.twist.linear.x = 0.0; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
