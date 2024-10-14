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
        super().__init__("joy_ctrl")

        # Declare parameter with default values
        self.declare_parameter("xspeed_limit", 1.0)
        self.declare_parameter("yspeed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)

        # Node variables
        self.Joy_active_ = False
        self.Buzzer_active_ = False
        self.RGBLight_index_ = 0
        self.cancel_time_ = time.time()
        self.user_name_ = getpass.getuser()
        self.linear_Gear_ = 1
        self.angular_Gear_ = 1
        self.xspeed_limit_ = self.get_parameter("xspeed_limit").get_parameter_value().double_value
        self.yspeed_limit_ = self.get_parameter("yspeed_limit").get_parameter_value().double_value
        self.angular_speed_limit_ = self.get_parameter("angular_speed_limit").get_parameter_value().double_value

        self.get_logger().info("User name: %s" % self.user_name_)

        # Publisher objects for robot sensing and actuation
        self.pub_goal_ = self.create_publisher(GoalInfo, "move_base/cancel", 10)
        self.pub_cmdVel_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_Buzzer_ = self.create_publisher(Bool, "Buzzer", 10)
        self.pub_JoyState_ = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight_ = self.create_publisher(Int32, "RGBLight", 10)

        # Subscriber objects that listens to joystick command
        self.sub_Joy_ = self.create_subscription(Joy, "joy", self.buttonCallback, 10)

        self.timer = self.create_timer(0.05, self.spin_once)

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.user_name_ == "jetson":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        """ jetson joy_data axis and button mapping:
            axes 8: [0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0]
            Left joystick (left positive, right negative): axes[0]
            Left joystick (up positive, down negative): axes[1]
            Right joystick (left positive, right negative): axes[2]
            Right joystick (up positive, down negative): axes[3]
            R2 (press negative, release positive): axes[4]
            L2 (press negative, release positive): axes[5]
            Left D-pad (left positive, right negative): axes[6]
            Left D-pad (up positive, down negative): axes[7]
            buttons 15:  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7]
            SELECT: buttons[10]
            START: buttons[11]
            Left joystick pressed: buttons[13]
            Right joystick pressed: buttons[14]
        """

        # Cancel navigation if R2 is pressed
        if joy_data.axes[4] == -1:
            self.cancel_nav()

        # RGB Light cycing when R1 is pressed
        if joy_data.buttons[7] == 1:
            if self.RGBLight_index < 6:
                for i in range(3):
                    self.pub_RGBLight_.publish(self.RGBLight_index_)
            else:
                self.RGBLight_index_ = 0
            self.RGBLight_index_ += 1
        
        # Buzzer toggling when START is pressed
        if joy_data.buttons[11] == 1:
            self.Buzzer_active_ = not self.Buzzer_active_
            self.pub_Buzzer_.publish(Bool(data = self.Buzzer_active_))

        # Linear Gear Cycling (3 levels) when the left joystick is pressed
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1

        # Angular Gear Cycling (4 levels) when the right joystick is pressed
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0

        # Speed calculation and limitation
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit_ * self.linear_Gear_
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit_ * self.linear_Gear_
        angular_speed = self.filter_date(joy_data.axes[2]) * self.angular_speed_limit_ * self.angular_Gear_

        if xlinear_speed > self.xspeed_limit: 
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: 
            xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: 
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: 
            ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: 
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit

        # Twist message on the "/cmd_vel" topic
        twist_msg_ = Twist()
        twist_msg_.linear.x = xlinear_speed
        twist_msg_.linear.y - ylinear_speed
        twist_msg_.angular.z = angular_speed
        for i in range(3):
            self.pub_cmdVel_.publish(twist_msg_)

    def user_pc(self, joy_data):
        """pc joy_data axis and button mapping:
            axes 8: [ -0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0 ]
            Left joystick (left positive, right negative): axes[0]
            Left joystick (up positive, down negative): axes[1]
            L2 (press negative, release positive): axes[2]
            Right joystick (left positive, right negative): axes[3]
            Right joystick (up positive, down negative): axes[4]
            R2 (press negative, release positive): axes[5]
            Left D-pad (left positive, right negative): axes[6]
            Left D-pad (up positive, down negative): axes[7]
            buttons 11: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            A: buttons[0]
            B: buttons[1]
            X: buttons[2]
            Y: buttons[3]
            L1: buttons[4]
            R1: buttons[5]
            SELECT: buttons[6]
            MODE: buttons[7]
            START: buttons[8]
            Left joystick pressed: buttons[9]
            Right joystick pressed: buttons[10]
        """

        # Cancel navigation when R2 is pressed
        if joy_data.axes[5] == -1: self.cancel_nav()

        # RGB Light cycing when R1 is pressed
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight.publish(self.RGBLight_index)
            else: self.RGBLight_index = 0
            self.RGBLight_index += 1

        # Buzzer toggling when START is pressed
        if joy_data.buttons[7] == 1:
            self.Buzzer_active=not self.Buzzer_active
            self.pub_Buzzer.publish(self.Buzzer_active)

        # Linear Gear Cycling (3 levels) when the left joystick is pressed
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1

        # Angular Gear Cycling (4 levels) when the right joystick is pressed
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0

        # Speed calculation and limitation
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit_ * self.linear_Gear_
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit_ * self.linear_Gear_
        angular_speed = self.filter_date(joy_data.axes[3]) * self.angular_speed_limit_ * self.angular_Gear_

        if xlinear_speed > self.xspeed_limit: 
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: 
            xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit: 
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: 
            ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit: 
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit

        # Twist message on the "/cmd_vel" topic
        twist_msg_ = Twist()
        twist_msg_.linear.x = xlinear_speed
        twist_msg_.linear.y - ylinear_speed
        twist_msg_.angular.z = angular_speed
        for i in range(3):
            self.pub_cmdVel_.publish(twist_msg_)

    def filter_data(self, value):
        """Low pass filter on the data such that noisy values and drifts are filtered"""
        if abs(value) < 0.2: value = 0
        return value
    
    def cancel_nav(self):
        # Issue the move_base cancel command
        now_time = time.time()
        if now_time - self.cancel_time_ > 1:
            self.Joy_active_ = not self.Joy_active_
            for i in range(3):
                self.pub_JoyState_.publish(Bool(self.Joy_active_))
                self.pub_goal_.publish(GoalInfo())
                self.pub_cmdVel_.publish(Twist())
            self.cancel_time = now_time

    def cancel(self):
        self.pub_cmdVel_.destroy()
        self.pub_goal_.destroy()
        self.pub_JoyState_.destroy()
        self.sub_Joy_.destroy()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

def main():
    rclpy.init()
    joy_teleop = JoyTeleop()

    try:
        rclpy.spin(joy_teleop)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        joy_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()