from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    x1_keyboard_node = Node(
        package='x1_control',
        executable='x1_keyboard.py',
        name='x1_keyboard',
        output='screen',
        parameters=[
            {'linear_speed_limit': 1.0},
            {'angular_speed_limit': 5.0}
        ]
    )
    return LaunchDescription([
        x1_keyboard_node
    ])
