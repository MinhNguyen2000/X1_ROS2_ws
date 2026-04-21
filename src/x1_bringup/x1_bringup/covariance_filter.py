import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import copy
import signal


class CovarianceFilter(Node):
    def __init__(self):
        super().__init__("covariance_filter")

        # ===== ROS Interface =====
        self.imu_sub        = self.create_subscription(Imu, "imu/data_raw", self.imu_callback, 10)
        self.lidar_sub      = self.create_subscription(Odometry, "odom_rf2o", self.lidar_callback, 10)
        self.wheel_vel_sub  = self.create_subscription(TwistStamped, "vel_raw", self.wheel_vel_callback, 10)

        self.imu_pub        = self.create_publisher(Imu, "imu/data_covariance", 10)
        self.lidar_pub      = self.create_publisher(Odometry, "odom_rf2o_covariance", 10)
        self.wheel_vel_pub  = self.create_publisher(TwistWithCovarianceStamped, "vel_covariance", 10)

        # IMU covariances - as per https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
        # orientation covariance row majors, in the order of (roll, pitch, yaw)
        self.imu_orientation_covariance = [1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1e-2]
        # angular velocity covariance row majors, in the order of (vroll, vpitch, vyaw)
        self.imu_angular_velocity_covariance = [1.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1e-2]
        # linear acceleratioin covariance row majors, in the order of (ax, ay, az)
        self.imu_linear_acceleration_covariance = [1e-2, 0.0, 0.0,
                                                   0.0, 1e-2, 0.0,
                                                   0.0, 0.0, 1e-2]
        
        # LiDAR covariances - as per https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        # pose covariance, in the order of (x, y, z, r, p, y)
        self.lidar_pose_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 1e-2]
        # twist covariance, in the order of (vx, vy, vz, vr, vp, vy)
        self.lidar_twist_covariance = [1e-2, 0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  1e-2, 0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  1.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  1.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  1e-3]

        # Wheel velocity covariances
        # as per https://docs.ros.org/en/humble/p/geometry_msgs/msg/TwistWithCovarianceStamped.html
        # twist covariance, in the order of (vx, vy, vz, vr, vp, vy)
        self.wheel_twist_covariance = [0.1,  0.0,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.1,  0.0,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.1,  0.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  1.0,  0.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
                                       0.0,  0.0,  0.0,  0.0,  0.0,  1.0]

        self.get_logger().info('Covariance filter node is running')
        

    def imu_callback(self, msg: Imu):
        msg_covariance = copy.deepcopy(msg)
        msg_covariance.header.frame_id = "imu_link"
        msg_covariance.orientation_covariance = self.imu_orientation_covariance
        msg_covariance.angular_velocity_covariance = self.imu_angular_velocity_covariance
        msg_covariance.linear_acceleration_covariance = self.imu_linear_acceleration_covariance
        self.imu_pub.publish(msg_covariance)

    def lidar_callback(self, msg: Odometry):
        msg_covariance = copy.deepcopy(msg)
        msg_covariance.pose.covariance = self.lidar_pose_covariance
        msg_covariance.twist.covariance = self.lidar_twist_covariance
        self.lidar_pub.publish(msg_covariance)

    def wheel_vel_callback(self, msg: TwistStamped):
        msg_twiststamped = copy.deepcopy(msg)
        msg_covariance = TwistWithCovarianceStamped()
        msg_covariance.header.stamp = self.get_clock().now().to_msg()
        msg_covariance.twist.twist = msg_twiststamped.twist
        msg_covariance.twist.covariance = self.wheel_twist_covariance
        self.wheel_vel_pub.publish(msg_covariance)

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceFilter()

    # catch sigterm from GUI:
    signal.signal(signal.SIGTERM, lambda *args: rclpy.shutdown())

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()