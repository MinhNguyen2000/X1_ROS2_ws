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
      3. Low level hardware - IMU, wheel encoder, wheel motors
      4. Odometry components - LiDAR scan matching, EKF sensor fusion
    '''
    # --- Define required paths
    description_pkg_dir = get_package_share_directory("x1_description")
    model_dir = os.path.join(description_pkg_dir, "urdf", "x1.urdf.xacro")

    camera_pkg_dir = get_package_share_directory("orbbec_camera")
    camera_params_path = os.path.join(camera_pkg_dir, "config", "camera_params.yaml")

    lidar_pkg_dir = get_package_share_directory('ydlidar_ros2_driver')
    lidar_launch_file_dir = os.path.join(lidar_pkg_dir, "launch", "ydlidar_launch.py")   # path to the ydlidar_ros2_driver launch file

    bringup_pkg_dir = get_package_share_directory("x1_bringup")
    odom_launch_path = os.path.join(bringup_pkg_dir, "launch", "odom_launch.py")

    # ===== DECLARE LAUNCH ARGUMENTS =====
    model_arg = DeclareLaunchArgument(
        name="robot_model",
        default_value=model_dir,
        description="Absolute path to robot URDF/xacro file"
    )

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

    robot_model = LaunchConfiguration("robot_model")
    camera_name = LaunchConfiguration("camera_name")
    camera_params = LaunchConfiguration('camera_params_file')

    robot_description = ParameterValue(
        Command(["xacro ", robot_model]),
        value_type = str
    )

    # ===== NODES & LAUNCH DESCRIPTIONS =====
    # robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # lidar node
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file_dir)
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

    # Low level driver node - IMU, wheel encoder, and wheel motors
    driver_node = Node(
        package='x1_bringup',
        executable='Mcnamu_driver',
    )

    # Launch the odometry nodes
    odom_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([odom_launch_path])
    )

    return LaunchDescription([
        model_arg,
        camera_name_arg,
        camera_params_file_arg,
        robot_state_publisher_node,
        ydlidar,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("camera_name")), 
            camera_container
        ]),
        driver_node,
        odom_nodes
    ])