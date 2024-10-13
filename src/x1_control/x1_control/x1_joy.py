#!/usr/bin/env python3

import os
import time
import getpass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Joy
from action_msgs.msg import GoalInfo

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_ctrl')

        # Declare parameter with default values
        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)

        # Node variables
        self.Joy_active_ = False
        self.Buzzer_active_ = False
        self.RGBLight_index_ = 0
        self.cancel_time_ = time.time()
        self.user_name_ = getpass.getuser()
        self.linear_Gear_ = 1
        self.angular_Gear_ = 1
        self.xspeed_limit_ = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit_ = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit_ = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # Publisher objects
        self.pub_goal_ = self.create_publisher(GoalInfo, "move_base/cancel", 10)
        self.pub_cmdVel_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer_ = self.create_publisher(Bool, "Buzzer", 10)
        self.pub_JoyState_ = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight_ = self.create_publisher(Int32, "RGBLight", 10)

        # Subscriber objects
        self.sub_Joy_ = self.create_subscription(Joy, 'joy', self.buttonCallback, 10)

        self.timer = self.create_timer(0.05, self.spin_once)

    def buttonCalback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.user_name_ == "jetson":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        if joy_data.axes[4] == -1:
            self.cancel_nav()

        if joy_data.buttons[7] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight_.publish(self.RGBLight_index_)
            else:
                self.RGBLight_index_ = 0
            self.RGBLight_index_ += 1
        
        if joy_data.buttons[11] == 1:
            self.Buzzer_active_ = not self.Buzzer_active_
            self.pub_Buzzer_.publish(Bool(data = self.Buzzer_active_))

        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit_ * self.linear_Gear_
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit_ * self.linear_Gear_
        angular_speed = self.filter_date(joy_data.axes[2]) * self.angular_speed_limit_ * self.angular_Gear_

        twist_msg_ = Twist()
        twist_msg_.linear.x = xlinear_speed
        twist_msg_.linear.y - ylinear_speed
        twist_msg_.angular.z = angular_speed
        self.pub_cmdVel_.publish(twist_msg_)
    def user_pc(self, joy_data):
