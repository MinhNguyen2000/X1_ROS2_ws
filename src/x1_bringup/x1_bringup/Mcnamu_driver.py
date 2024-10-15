import math
import random
import threading
from Rosmaster_Lib import Rosmaster

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState

# from dynamic_reconfigure.server import Server
# from yahboomcar_bringup.cfg import PIDparamConfig  # Assuming you have a similar package in ROS2
from time import sleep

class X1Driver(Node):
    def __init__(self):
        super().__init__("driver_node")
        
        self.RA2DE = 180 / math.pi
        self.car = Rosmaster()
        self.car.set_car_type(4)  # 1-X3, 2-X3Plus, 4-X1, 5-R2
        
        # Declare parameters with default values
        self.car_type_arg_ = self.declare_parameter("car_type","X1")
        self.imu_link_arg_ = self.declare_parameter("imu_link", "imu_link")
        self.Prefix_arg_ = self.declare_parameter("prefix", "")
        self.xlinear_limit_arg = self.declare_parameter("xlinear_speed_limit", 1.0)
        self.ylinear_limit_arg_ = self.declare_parameter("ylinear_speed_limit", 1.0)
        self.angular_limit_arg_ = self.declare_parameter("angular_speed_limit", 5.0)
        self.nav_use_rotvel_arg_ = self.declare_parameter("nav_use_rotvel", False)

        self.car_type_ = self.get_parameter("car_type").get_parameter_value().double_value()
        self.imu_link_ = self.get_parameter("imu_link").get_parameter_value().double_value()
        self.Prefix_ = self.get_parameter("prefix").get_parameter_value().double_value()
        self.xlinear_limit_ = self.get_parameter("xlinear_speed_limit").get_parameter_value().double_value()
        self.ylinear_limit_ = self.get_parameter("ylinear_speed_limit").get_parameter_value().double_value()
        self.angular_limit_ = self.get_parameter("angular_speed_limit").get_parameter_value().double_value()
        self.nav_use_rotvel_ = self.get_parameter("nav_use_rotvel").get_parameter_value().double_value()

        # Create the subscriber objects
        self.vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.RGBLight_sub_ = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)
        self.Buzzer_sub_ = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)
        
        # Create the publisher objects
        self.Edi_pub_ = self.create_publisher(Float32, "edition", 100)
        self.vol_pub_ = self.create_publisher(Float32, "voltage", 100)
        self.state_pub_ = self.create_publisher(JointState, "joint_states", 100)
        self.vel_pub_ = self.create_publisher(Twist, "/vel_raw", 100)
        self.imu_pub_ = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.mag_pub_ = self.create_publisher(MagneticField, "/imu/mag", 100)

        # self.dyn_server = Server(PIDparamConfig, self.dynamic_reconfigure_callback)

        #create timer
        self.timer = self.create_timer(0.1, self.pub_data)

        #create and init variable
        self.edition = Float32()
        self.edition.data = 1.0
        self.car.create_receive_threading()

    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist): return
        vx = msg.linear.x
        vy = msg.linear.y
        angular = msg.angular.z
        self.car.set_car_motion(vx, vy, angular)

    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32): return
        for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        if not isinstance(msg, Bool): return
        if msg.data: 
            for i in range(3): self.car.set_beep(1)
        else:
            for i in range(3): self.car.set_beep(0)
    
    def pub_data(self):
        if not rclpy.ok():
            return

        # Configure message interface
        imu = Imu()
        twist = Twist()
        battery = Float32()
        edition = Float32()
        mag = MagneticField()
        state = JointState()

        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "joint_states"
        if len(self.Prefix)==0:
            state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
        else:
            state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]

        # Obtain sensor data from the car
        edition.data = self.car.get_version()
        battery.data = self.car.get_battery_voltage()
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        mx *= 1.0; my *= 1.0; mz *= 1.0
        vx, vy, angular = self.car.get_motion_data()

        # Preparing the message for publishing
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular

        # Publishing the real robot data on corresponding topics
        self.vel_pub_.publish(twist)
        self.imu_pub_.publish(imu)
        self.mag_pub_.publish(mag)
        self.vol_pub_.publish(battery)
        self.Edi_pub_.publish(edition)

        state.position = [0, 0, 0, 0]
        if not (vx == vy == angular == 0):
            i = random.uniform(-3.14, 3.14)
            state.position = [i, i, i, i]
        
        self.state_pub_.publish(state)

    

    # def dynamic_reconfigure_callback(self, config, level):
    #     self.linear_max = config["linear_max"]
    #     self.linear_min = config["linear_min"]
    #     self.angular_max = config["angular_max"]
    #     self.angular_min = config["angular_min"]
    #     return config

def main():
    rclpy.init()
    x1_driver = X1Driver()
    rclpy.spin(x1_driver)
    x1_driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
