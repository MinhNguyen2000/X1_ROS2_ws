#!/usr/bin/env python3

import os
import time
import getpass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Joy
from action_msgs.msg import GoalInfo

class JoyTeleop(Node):
    def __init__(self):
        super().__init__("x1_joy")

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
        self.pub_cmdVel_ = self.create_publisher(TwistStamped, "/x1_controller/cmd_vel", 10)
        self.pub_Buzzer_ = self.create_publisher(Bool, "Buzzer", 10)
        self.pub_JoyState_ = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight_ = self.create_publisher(Int32, "RGBLight", 10)

        # Subscriber objects that listens to joystick command
        self.sub_Joy_ = self.create_subscription(Joy, "joy", self.buttonCallback, 10)
    


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
            if self.RGBLight_index_ < 6:
                for i in range(3):
                    self.pub_RGBLight_.publish(Int32(data=self.RGBLight_index_))
            else:
                self.RGBLight_index_ = 0
            self.RGBLight_index_ += 1
        
        # Buzzer toggling when START is pressed
        if joy_data.buttons[11] == 1:
            self.Buzzer_active_ = not self.Buzzer_active_
            self.pub_Buzzer_.publish(Bool(data = self.Buzzer_active_))

        # Linear Gear Cycling (3 levels) when the left joystick is pressed
        if joy_data.buttons[13] == 1:
            if self.linear_Gear_ == 1.0: self.linear_Gear_ = 1.0 / 3
            elif self.linear_Gear_ == 1.0 / 3: self.linear_Gear_ = 2.0 / 3
            elif self.linear_Gear_ == 2.0 / 3: self.linear_Gear_ = 1
            self.get_logger().info(f"Current linear gear: {self.linear_Gear_}")

        # Angular Gear Cycling (4 levels) when the right joystick is pressed
        if joy_data.buttons[14] == 1:
            if self.angular_Gear_ == 1.0: self.angular_Gear_ = 1.0 / 4
            elif self.angular_Gear_ == 1.0 / 4: self.angular_Gear_ = 1.0 / 2
            elif self.angular_Gear_ == 1.0 / 2: self.angular_Gear_ = 3.0 / 4
            elif self.angular_Gear_ == 3.0 / 4: self.angular_Gear_ = 1.0
            self.get_logger().info(f"Current angular gear: {self.angular_Gear_}")

        # Speed calculation and limitation
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit_ * self.linear_Gear_
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit_ * self.linear_Gear_
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit_ * self.angular_Gear_

        if xlinear_speed > self.xspeed_limit_: 
            xlinear_speed = self.xspeed_limit_
        elif xlinear_speed < -self.xspeed_limit_: 
            xlinear_speed = -self.xspeed_limit_
        if ylinear_speed > self.yspeed_limit_: 
            ylinear_speed = self.yspeed_limit_
        elif ylinear_speed < -self.yspeed_limit_: 
            ylinear_speed = -self.yspeed_limit_
        if angular_speed > self.angular_speed_limit_: 
            angular_speed = self.angular_speed_limit_
        elif angular_speed < -self.angular_speed_limit_: 
            angular_speed = -self.angular_speed_limit_

        # Twist message on the "/cmd_vel" topic
        # TODO - adjust this to customize sending Twist or TwistStamped messages
        twist_msg_ = TwistStamped()
        twist_msg_.header.stamp = self.get_clock().now().to_msg()
        twist_msg_.header.frame_id = "base_link"
        twist_msg_.twist.linear.x = xlinear_speed
        twist_msg_.twist.linear.y = ylinear_speed
        twist_msg_.twist.angular.z = angular_speed
        for i in range(3):
            self.pub_cmdVel_.publish(twist_msg_)

    def user_pc(self, joy_data):
        """pc joy_data axis and button mapping:
            axes 8: [ -0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0 ]
            Left joystick (left positive, right negative): axes[0]
            Left joystick (up positive, down negative): axes[1]
            Right joystick (left positive, right negative): axes[2]
            Right joystick (up positive, down negative): axes[3]
            R2 (press negative, release positive): axes[4]
            L2 (press negative, release positive): axes[5]
            Left D-pad (left positive, right negative): axes[6]
            Left D-pad (up positive, down negative): axes[7]
            buttons 11: [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
            A: buttons[0]
            B: buttons[1]
            X: buttons[3]
            Y: buttons[4]
            L1: buttons[6]
            R1: buttons[7]
            L2: buttons[8]
            R2: buttons[9]
            SELECT: buttons[10]
            START: buttons[11]
            Left joystick pressed: buttons[13]
            Right joystick pressed: buttons[14]
        """

        # Cancel navigation when R2 is pressed
        if joy_data.axes[4] == -1: self.cancel_nav()

        # RGB Light cycing when R1 is pressed
        if joy_data.buttons[7] == 1:
            if self.RGBLight_index_ < 6:
                self.pub_RGBLight_.publish(Int32(data=self.RGBLight_index_))
            else: self.RGBLight_index_ = 0
            self.RGBLight_index_ += 1

        # Buzzer toggling when START is pressed
        if joy_data.buttons[11] == 1:
            self.Buzzer_active_ = not self.Buzzer_active_
            self.pub_Buzzer_.publish(Bool(data=self.Buzzer_active_))

        # Linear Gear Cycling (3 levels) when the left joystick is pressed
        if joy_data.buttons[13] == 1:
            if self.linear_Gear_ == 1.0: self.linear_Gear_ = 1.0 / 3
            elif self.linear_Gear_ == 1.0 / 3: self.linear_Gear_ = 2.0 / 3
            elif self.linear_Gear_ == 2.0 / 3: self.linear_Gear_ = 1
            self.get_logger().info(f"Current linear gear: {self.linear_Gear_}")

        # Angular Gear Cycling (4 levels) when the right joystick is pressed
        if joy_data.buttons[14] == 1:
            if self.angular_Gear_ == 1.0: self.angular_Gear_ = 1.0 / 4
            elif self.angular_Gear_ == 1.0 / 4: self.angular_Gear_ = 1.0 / 2
            elif self.angular_Gear_ == 1.0 / 2: self.angular_Gear_ = 3.0 / 4
            elif self.angular_Gear_ == 3.0 / 4: self.angular_Gear_ = 1.0
            self.get_logger().info(f"Current angular gear: {self.angular_Gear_}")

        # Speed calculation and limitation
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit_ * self.linear_Gear_
        # ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit_ * self.linear_Gear_
        angular_speed = self.filter_data(joy_data.axes[3]) * self.angular_speed_limit_ * self.angular_Gear_

        if xlinear_speed > self.xspeed_limit_: 
            xlinear_speed = self.xspeed_limit_
        elif xlinear_speed < -self.xspeed_limit_: 
            xlinear_speed = -self.xspeed_limit_
        # if ylinear_speed > self.yspeed_limit_: 
        #     ylinear_speed = self.yspeed_limit_
        # elif ylinear_speed < -self.yspeed_limit_: 
        #     ylinear_speed = -self.yspeed_limit_
        if angular_speed > self.angular_speed_limit_: 
            angular_speed = self.angular_speed_limit_
        elif angular_speed < -self.angular_speed_limit_: 
            angular_speed = -self.angular_speed_limit_

        # Twist message on the "/cmd_vel" topic
        twist_msg_ = TwistStamped()
        twist_msg_.header.stamp = self.get_clock().now().to_msg()
        twist_msg_.header.frame_id = "base_footprint"
        twist_msg_.twist.linear.x = xlinear_speed
        # twist_msg_.twist.linear.y = ylinear_speed
        twist_msg_.twist.angular.z = angular_speed
        
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
                self.pub_JoyState_.publish(Bool(data=self.Joy_active_))
                self.pub_goal_.publish(GoalInfo())
                self.pub_cmdVel_.publish(TwistStamped() if self.use_twist_stamped else Twist())
            self.cancel_time = now_time

    def cancel(self):
        self.pub_cmdVel_.destroy()
        self.pub_goal_.destroy()
        self.pub_JoyState_.destroy()
        self.sub_Joy_.destroy()

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

if __name__ == "__main__":
    main()