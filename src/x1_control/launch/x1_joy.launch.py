from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        respawn=False
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
    )
    return LaunchDescription([
        joy_node,
        x1_joy_node
    ])
