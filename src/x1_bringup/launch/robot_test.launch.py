import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch the ydlidar_ros2_driver launch file
    ydlidar_launch_file_dir = os.path.join(get_package_share_directory('ydlidar_ros2_driver'),'launch','ydlidar_launch.py')
    
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file_dir)
    )

    return LaunchDescription([
        ydlidar
    ])