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
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
            get_package_share_directory("x1_description"),"urdf","x1.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    model = LaunchConfiguration("model")
    robot_description = ParameterValue(
        Command(["xacro ", model," is_ignition:=",is_ignition]),
        value_type = str
    )

    # 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters = [{"robot_description": robot_description}],
    )

    # Launch the ydlidar_ros2_driver launch file
    ydlidar_launch_file_dir = os.path.join(get_package_share_directory('ydlidar_ros2_driver'),'launch','ydlidar_launch.py')
    
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file_dir)
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("x1_description"),
            "rviz","real.rviz")]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )

    joint_state_broadcaster_spawner = Node(
            package = "controller_manager",
            executable = "spawner",
            arguments = [
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
            ]
    )

    x1_diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "x1_controller",
            "--controller-manager",
            "/controller_manager",
            # "--ros-args",
            # "--log-level",
            # "debug"
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        # joint_state_publisher,
        # joint_state_publisher_gui,
        joint_state_broadcaster_spawner,
        # x1_diffdrive_controller_spawner,
        # ydlidar,
        rviz
    ])