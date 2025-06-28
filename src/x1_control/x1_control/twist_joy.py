#!/usr/bin/env python3
import getpass
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('twist_joy')

        self.user_name_ = getpass.getuser()
        self.linear_speed_ = 0.0
        self.angular_speed_ = 0.0
        self.Joy_state_ = False
        self.velocity_ = Twist()
        self.Joy_time_ = time.time()

        self.linear_speed_limit_ = self.declare_parameter('linear_speed_limit', 2.0).value
        self.angular_speed_limit_ = self.declare_parameter('angular_speed_limit', 2.0).value

        self.get_logger().info("User name: %s" % self.user_name_)

        self.pub_cmdVel_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub_Joy_ = self.create_subscription(Joy, '/joy', self.button_callback, 10)

        vel_thread = threading.Thread(target=self.pub_vel)
        vel_thread.setDaemon(True)
        vel_thread.start()

        self.timer = self.create_timer(0.05, self.pub_vel)  # 20 Hz equivalent to rospy.Rate(20)
        self.get_logger().info('JoyTeleop Node started')

    def pub_vel(self):
        now_time = time.time()
        if now_time - self.Joy_time_ > 1:
            if self.Joy_state_:
                self.pub_cmdVel_.publish(Twist())  # Stop the robot
                self.Joy_state_ = False
            self.Joy_time_ = now_time

        if self.linear_speed_ == 0 and self.angular_speed_ == 0:
            if self.Joy_state_:
                self.pub_cmdVel_.publish(Twist())  # Stop the robot
                self.Joy_state_ = False
        else:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed_
            twist_msg.angular.z = self.angular_speed_
            self.pub_cmdVel_.publish(twist_msg)
            self.Joy_state_ = True

    def cancel(self):
        self.pub_cmdVel_.destroy()
        self.sub_Joy_.destroy()

    def button_callback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        self.Joy_time_ = time.time()

        if self.user_name_ == "jetson":
            self.linear_speed_ = joy_data.axes[1] * self.linear_speed_limit_
            self.angular_speed_ = joy_data.axes[2] * self.angular_speed_limit_
        else:
            self.linear_speed_ = joy_data.axes[1] * self.linear_speed_limit_
            self.angular_speed_ = joy_data.axes[3] * self.angular_speed_limit_


def main():
    rclpy.init()
    joy_teleop = JoyTeleop()

    try:
        rclpy.spin(joy_teleop)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        joy_teleop.cancel()
        joy_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()