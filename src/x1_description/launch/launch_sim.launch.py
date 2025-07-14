import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition              # Check condition whether to run Gazebo controller
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
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    # yahboomcar_urdf = os.path.join(
    #     get_package_share_directory('yahboomcar_description'), 'urdf', 'yahboomcar.urdf'
    # )

    default_world = os.path.join(
        get_package_share_directory('x1_description'),
        'worlds',
        'empty.world')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description="World to load"
    )

    world = LaunchConfiguration('world')


    x1_description_dir = get_package_share_directory("x1_description")          # Path to package
    x1_urdf_dir = os.path.join(x1_description_dir, 'urdf', 'x1.urdf.xacro')     # Path to URDF file

    # ========== ARGUMENT DEFINITION ==========
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = x1_urdf_dir,
        description = "Absolute path to robot URDF file"
    )

    use_joy_arg = DeclareLaunchArgument(
        name = 'use_joy',
        default_value="True",
        description='Determine whether to use joystick or keyboard to contol the robot in Gazebo simulation'
    )

    # Launch configuration definition
    model = LaunchConfiguration("model")
    use_joy = LaunchConfiguration("use_joy")

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
        PythonLaunchDescriptionSource([gazebo_launch_file_dir]),
        launch_arguments = 
        {'gz_args': ['-r -v4 ', world]}.items() 
    )

    # ========== NODE DEFINITION ========== 
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters = [{"robot_description": robot_description}],
    )

    # Static transform publisher (from base_footprint to base_link)
    static_tf = Node(
        name='tf_footprint_base',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
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


    # Launch the physical controller nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        respawn=False,
        parameters = [os.path.join(get_package_share_directory("x1_control"),"config","joy_config.yaml")],
        condition=IfCondition(use_joy)
    )

    x1_joy_node =  Node(
        package='x1_control',
        executable='x1_joy.py',
        name='x1_joy',
        output='screen',
        parameters=[
            {'linear_speed_limit': 1.0},
            {'angular_speed_limit': 5.0}
        ],
        condition=IfCondition(use_joy)
    )

    # Launch ROS2 controller nodes
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

    bridge_params = os.path.join(get_package_share_directory('x1_description'), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', 
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    # Launch description to run everything
    return LaunchDescription([
        world_arg,
        model_arg,
        use_joy_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        static_tf,
        gz_spawn_entity,
        joy_node,
        x1_joy_node,
        joint_state_broadcaster_spawner,
        x1_diffdrive_controller_spawner,
        ros_gz_bridge
    ])
