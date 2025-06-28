import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration    
from launch.conditions import IfCondition               # Check condition whether to run Gazebo controller
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Launch the nodes to control the robot using PS joystick

def generate_launch_description():
    use_gazebo_arg = DeclareLaunchArgument(
        name = "use_gazebo",
        default_value = "True",     # by default, simulate with Gazebo and spawn a DiffDriveController to simulate driving
        description="If use Gazebo simulation, spawn a controller to control the robot from /cmd_vel"
    )

    use_gazebo = LaunchConfiguration("use_gazebo")

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        respawn=False,
        parameters = [os.path.join(get_package_share_directory("x1_control"),"config","joy_config.yaml")]
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

    x1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "x1_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition = IfCondition(use_gazebo)
    )

    # Addition - spawn a controller to map from /cmd_vel to Gazebo simulation

    return LaunchDescription([
        use_gazebo_arg,
        joy_node,
        x1_joy_node,
        joint_state_broadcaster_spawner,
        x1_controller_spawner
    ])
