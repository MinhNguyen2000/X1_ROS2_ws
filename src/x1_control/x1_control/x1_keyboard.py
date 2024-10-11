#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : switch x/y axis speed
s/S : stop keyboard control
space key, k : force stop
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),    'I': (1, 0),
    'o': (1, -1),   'O': (1, -1),
    'j': (0, 1),    'J': (0, 1),
    'l': (0, -1),   'L': (0, -1),
    'u': (1, 1),    'U': (1, 1),
    ',': (-1, 0),   
    '.': (-1, 1),
    'm': (-1, -1),  'M': (-1, -1),
}

speedBindings = {
    'C': (1, .9),
    'q': (1.1, 1.1),    'Q': (1.1, 1.1),
    'z': (.9, .9),      'Z': (.9, .9),
    'w': (1.1, 1),      'W': (1.1, 1),
    'x': (.9, 1),       'X': (.9, 1),
    'e': (1, 1.1),      'E': (1, 1.1),
    'c': (1, .9),       'C': (1, .9),
}

def getKey(settings):
    """
    Function to read from terminal without waiting for the Enter/return key
    """
    
    # Sets the terminal to "raw" mode to capture key press without Enter
    tty.setraw(sys.stdin.fileno())

    # Read the key press every 0.1 second
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist: key = sys.stdin.read(1)
    else: key = ''

    # Restore the terminal's original setting after the keycap is captured
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Declare parameters with default values
        self.linear_limit = self.declare_parameter('linear_speed_limit', 1.0).value
        self.angular_limit = self.declare_parameter('angular_speed_limit', 5.0).value

        # Publisher
        self.vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Node variables
        self.xspeed_switch_ = True
        self.speed_ = 0.2
        self.turn_ = 1.0
        self.x_ = 0
        self.th_ = 0
        self.stop_ = False
        self.count_ = 0

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    kb_ctrl_node = KeyboardControlNode()

    try:
        print(msg)
        print(vels(kb_ctrl_node.speed_,kb_ctrl_node.turn_))

        while True:
            key = getKey(settings)
            # Change motion
            if key in moveBindings:
                kb_ctrl_node.x_, kb_ctrl_node.th_ = moveBindings[key]
                kb_ctrl_node.count_ = 0

            # Change speed
            elif key in speedBindings:
                kb_ctrl_node.speed_ *= speedBindings[key][0]
                kb_ctrl_node.turn_ *= speedBindings[key][1]
                if kb_ctrl_node.speed_ > kb_ctrl_node.linear_limit: 
                    kb_ctrl_node.speed_ = kb_ctrl_node.linear_limit
                if kb_ctrl_node.turn_ > kb_ctrl_node.angular_limit:
                    kb_ctrl_node.turn_ = kb_ctrl_node.angular_limit
                print(vels(kb_ctrl_node.speed_, kb_ctrl_node.turn_))
                kb_ctrl_node.count_ = 0
            
            # Force stop
            elif key == " ":
                kb_ctrl_node.x_ = 0
                kb_ctrl_node.th_ = 0

            else:
                kb_ctrl_node.count+= 1
                if kb_ctrl_node > 4: 
                    kb_ctrl_node.x_ = 0
                    kb_ctrl_node.th_ = 0
                if key == "\x03": break

            twist = Twist()
            if kb_ctrl_node.xspeed_switch__:
                twist.linear.x = kb_ctrl_node.speed_ * kb_ctrl_node.x
            else:
                twist.linear.y = kb_ctrl_node.speed_ * kb_ctrl_node.x
            twist.angular.z = kb_ctrl_node.turn_ * kb_ctrl_node.th

            kb_ctrl_node.vel_pub_.publish(twist)
    except Exception as e:
        print(e)
    finally: # Stop the robot by publishing an empty Twist() message and reset the terminal settings
        print('Stopping the robot.')
        kb_ctrl_node.pub.publish(Twist())  # Stop the robot by sending zero velocity
        print('Restoring terminal settings.')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings
        print('Destroying node.')
        kb_ctrl_node.destroy_node()  # Explicitly destroy the node (optional)
        print('Shutting down ROS2 node.')
        rclpy.shutdown()  # Shut down ROS2

if __name__ == "__main__":
    main() 