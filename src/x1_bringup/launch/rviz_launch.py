import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== DECLARE LAUNCH ARGUMENTS =====
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
            get_package_share_directory("x1_description"),"urdf","x1.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    use_gazebo_arg = DeclareLaunchArgument(
        name="use_gazebo",
        default_value="true",
        description="Set to false when running on real hardware (robot publishes joint states itself)"
    )

    model = LaunchConfiguration("model")
    use_gazebo = LaunchConfiguration("use_gazebo")

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type = str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters = [{"robot_description": robot_description}],
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(use_gazebo)
    )

    image_transport_node = Node(
        package='x1_bringup',
        executable='image_republisher',
        name='image_republisher',
        namespace='/camera'
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
        use_gazebo_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        image_transport_node,
        rviz
    ])