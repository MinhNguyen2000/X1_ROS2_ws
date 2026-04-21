from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node 

import os

def generate_launch_description():
    # ===== SET REQUIRED PATHS =====
    pkg_path = get_package_share_directory("x1_bringup")
    ekf_params_path = os.path.join(pkg_path, "config", "ekf_params.yaml")

    # ===== DEFINE LAUNCH ARGUMENTS =====

    # ===== NODES & LAUNCH DESCRIPTION =====

    # LiDAR scan matcher package
    laser_scan_matcher_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic" : "/scan",
            "odom_topic" : "/odom_rf2o",
            "publish_tf" : True,
            "base_frame_id" : "base_footprint",
            "odom_frame_id" : "odom",
            "init_pose_from_topic" : "",
            "freq" : 20.0}],
    )

    # Covariance filter node to publish IMU + LiDAR + wheel encoder covariance
    covariance_filter_node = Node(
        package="x1_bringup",
        executable="covariance_filter_node",
        name="covariance_filter",
        output="screen"
    )

    # EKF node
    ekf_odom_node = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_odom_node",
        output = "screen",
        parameters = [ekf_params_path],
        remappings = [("odometry/filtered", "odom")]
    )

    return LaunchDescription([
        laser_scan_matcher_node,
        covariance_filter_node,
        ekf_odom_node
    ])

