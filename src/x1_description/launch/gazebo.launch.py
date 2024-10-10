import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ========== HELPER VARIABLES ==========
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Paths to URDF and Gazebo launch files
    gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch'
    )
    # yahboomcar_urdf = os.path.join(
    #     get_package_share_directory('yahboomcar_description'), 'urdf', 'yahboomcar.urdf'
    # )

    x1_description_dir = get_package_share_directory("x1_description")          # Path to package
    x1_urdf_dir = os.path.join(x1_description_dir, 'urdf', 'x1.urdf.xacro')     # Path to URDF file

    # ========== ARGUMENT DEFINITION ==========
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = x1_urdf_dir,
        description = "Absolute path to robot URDF file"
    )

    # Launch configuration definition
    model = LaunchConfiguration("model")

    robot_description = ParameterValue(
        Command(["xacro ", model," is_ignition:=",is_ignition]),
        value_type = str
    )

    # Set an environmental variable in Linux from a launch file
    gazebo_resource_path = SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH", 
        value = str(Path(x1_description_dir).parent.resolve())
    )

    # Include Gazebo empty world launch file
    gazebo = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource([gazebo_launch_file_dir, '/empty_world.launch.py']),
        PythonLaunchDescriptionSource([gazebo_launch_file_dir, '/gz_sim.launch.py']),
        launch_arguments = [("gz_args",["-v 4"," -r"])]
    )

    # ========== NODE DEFINITION ========== 
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters = [{"robot_description": robot_description}],
    )

    # Static transform publisher (for base_link to base_footprint)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
    )

    # Spawn X1 model in Gazebo
    gz_spawn_entity = Node(
        name='spawn_model',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', 'ROSMaster X1'
        ]
    )

    # # Fake joint calibration (using ros2 topic pub instead of topic_tools)
    # fake_joint_calibration = Node(
    #     package='ros2',
    #     executable=FindExecutable(name='topic'),
    #     arguments=['pub', '/calibrated', 'std_msgs/Bool', 'true'],
    #     output='screen'
    # )

    # Launch description to run everything
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        static_tf,
        gz_spawn_entity
        # fake_joint_calibration
    ])
