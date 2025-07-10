import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration    
from launch.conditions import IfCondition, UnlessCondition              # Check condition whether to run Gazebo controller
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Launch the nodes to control the robot using PS joystick

def generate_launch_description():
    use_diffdrive_arg = DeclareLaunchArgument(
        name = "use_diffdrive",
        default_value = "True",     # by default, simulate with Gazebo and spawn a DiffDriveController to simulate driving
        description="If use Gazebo simulation, spawn a controller to control the robot from /cmd_vel"
    )

    use_diffdrive = LaunchConfiguration("use_diffdrive")

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        respawn=False,
        parameters = [os.path.join(get_package_share_directory("x1_control"),"config","joy_config.yaml")]
    )

    x1_joy_node =  Node(
        package='x1_control',
        executable='x1_joy.py',
        name='x1_joy',
        output='screen',
        parameters=[
            {'linear_speed_limit': 1.0},
            {'angular_speed_limit': 5.0}
        ]
        # parameters = [os.path.join(get_package_share_directory("x1_control"),"config","joy_teleop.yaml")]
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
        ],
        condition = IfCondition(use_diffdrive)
    )

    x1_jointvel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition = UnlessCondition(use_diffdrive)
    )

    # Addition - spawn a controller to map from /cmd_vel to Gazebo simulation

    return LaunchDescription([
        use_diffdrive_arg,
        joy_node,
        x1_joy_node,
        joint_state_broadcaster_spawner,
        x1_diffdrive_controller_spawner,
        x1_jointvel_controller_spawner
    ])
