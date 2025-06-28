from launch import LaunchDescription
from launch_ros.actions import Node

# Launch the files to control turtlesim with a PS joystick

def generate_launch_description():

    turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )

    # Start the node for obtaining wireless controller (joy) information
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            respawn=False
        )
    
    # Start the wireless controller node to use a controller to control the turtle
    twist_joy_node = Node(
            package='x1_control',
            executable='twist_joy.py', 
            name='twist_joy',
            output='screen',
            parameters=[
                {'linear_speed_limit': 2.0},
                {'angular_speed_limit': 2.0}
            ]
        )
    return LaunchDescription([
        turtlesim_node,
        joy_node,
        twist_joy_node
    ])