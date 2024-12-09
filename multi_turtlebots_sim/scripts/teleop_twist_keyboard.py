#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Control Your Robot:
   W : move forward
   S : move backward
   A : turn left
   D : turn right
   X : stop

Adjust Speed:
   + : increase linear speed
   - : decrease linear speed
   < : increase angular speed
   > : decrease angular speed

CTRL-C to quit
"""

# Movement bindings for 'WASD'
move_bindings = {
    'w': (1, 0),   # Forward
    's': (-1, 0),  # Backward
    'a': (0, 1),   # Turn left
    'd': (0, -1),  # Turn right
    'x': (0, 0),   # Stop
}

# Speed adjustment bindings
speed_bindings = {
    '+': (1.1, 1.0),  # Increase linear speed -> to handle shift + =
    '=': (1.1, 1.0),  # Increase linear speed
    '-': (0.9, 1.0),  # Decrease linear speed -> to handle shift + -
    '_': (0.9, 1.0),  # Decrease linear speed
    '<': (1.0, 1.1),  # Increase angular speed
    '>': (1.0, 0.9),  # Decrease angular speed
}

def get_key():
    """Get keyboard input."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(speed, turn):
    """Print current speed settings."""
    return f"Current Speed:\tlinear {speed:.2f}\tangular {turn:.2f}"

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)

    speed = 0.1  # Default linear speed
    turn = 0.1   # Default angular speed
    x = 0
    th = 0

    try:
        print(msg)
        print(print_vels(speed, turn))
        while True:
            key = get_key()
            if key in move_bindings.keys():
                x, th = move_bindings[key]
            elif key in speed_bindings.keys():
                speed *= speed_bindings[key][0]
                turn *= speed_bindings[key][1]
                print(print_vels(speed, turn))
            else:
                # Stop on unknown key
                x, th = 0, 0
                if key == '\x03':  # CTRL-C
                    break

            # Publish the Twist message
            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
