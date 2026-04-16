import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    '''
    Function to launch the following functionalities on the robot
      1. LiDAR - using the ydlidar package launch file
      2. Camera - using the orbbec camera package
      3. The IMU
      4. Odometry components - LiDAR scan matching, EKF sensor fusion
    '''
    # --- Define required paths
    camera_pkg_dir = get_package_share_directory("orbbec_camera")
    camera_params_path = os.path.join(camera_pkg_dir, "config", "camera_params.yaml")

    # --- Define the launch arguments
    ydlidar_launch_file_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )   # path to the ydlidar_ros2_driver launch file

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name namespace'
    )

    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_params_path,
        description='Full path to the YAML parameters file to load',
    )

    camera_name = LaunchConfiguration("camera_name")
    camera_params = LaunchConfiguration('camera_params_file')

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    # lidar node
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file_dir)
    )

    # camera nodes
    compose_node = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name=camera_name,
        namespace="",
        parameters=[camera_params],
    )

    # Define the ComposableNodeContainer
    camera_container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            compose_node,
        ],
        output="screen",
    )

    # Low level driver node
    driver_node = Node(
        package='x1_bringup',
        executable='Mcnamu_driver',
    )

    return LaunchDescription([
        camera_name_arg,
        camera_params_file_arg,
        ydlidar,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("camera_name")), 
            camera_container
        ]),
        driver_node
    ])