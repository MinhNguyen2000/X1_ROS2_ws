#!/usr/bin/env python3
# encoding: utf-8
import getpass
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('turtlebot_joy')
        self.add_on_shutdown(self.cancel)
        self.user_name = getpass.getuser()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.Joy_time = time.time()
        self.linear_speed_limit = self.declare_parameter('linear_speed_limit', 0.3).value
        self.angular_speed_limit = self.declare_parameter('angular_speed_limit', 1.0).value

        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_Joy = self.create_subscription(Joy, '/joy', self.buttonCallback, 10)

        self.timer = self.create_timer(0.05, self.publish_velocity)  # Timer to publish velocities at regular intervals

    def cancel(self):
        self.pub_cmdVel.destroy()
        self.sub_Joy.destroy()

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy): return
        self.Joy_time = time.time()
        if self.user_name == "jetson":
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[2] * self.angular_speed_limit
        else:
            self.linear_speed = joy_data.axes[1] * self.linear_speed_limit
            self.angular_speed = joy_data.axes[3] * self.angular_speed_limit

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.pub_cmdVel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    joy = JoyTeleop()
    try:
        rclpy.spin(joy)
    except rclpy.exceptions.ROSInterruptException:
        joy.get_logger().info('Exception occurred')
    finally:
        joy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
