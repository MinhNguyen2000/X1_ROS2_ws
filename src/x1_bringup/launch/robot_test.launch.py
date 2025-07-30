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
    # Path to the ydlidar_ros2_driver launch file
    ydlidar_launch_file_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name namespace'
    )

    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=os.path.join(
            get_package_share_directory('orbbec_camera'),
            'config',
            'camera_params.yaml'
        ),
        description='Full path to the YAML parameters file to load',
    )

    camera_name = LaunchConfiguration("camera_name")
    camera_params = LaunchConfiguration('camera_params_file')


    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file_dir)
    )

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

    return LaunchDescription([
        camera_name_arg,
        camera_params_file_arg,
        ydlidar,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("camera_name")), 
            camera_container
        ])
    ])