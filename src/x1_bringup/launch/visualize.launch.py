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
    
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
            get_package_share_directory("x1_description"),"urdf","x1.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    model = LaunchConfiguration("model")
    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type = str
    )

    # 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters = [{"robot_description": robot_description}],
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    image_transport_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed','raw'],
        remappings=[
            ('in/compressed','/camera/color/image_raw/compressed'),
            ('out','/camera/color/image_raw/uncompressed')
        ],
        parameters=[
            {'qos_overrides./camera/color/image_raw/compressed.reliability': 'best_effort'}
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("x1_description"),
            "rviz","real.rviz")]
    )


    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        image_transport_node,
        rviz
    ])