import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
            'robot_type',
            default_value=EnvironmentVariable('ROBOT_TYPE'),
            description='robot_type [X1,X3,X3plus,R2,X7]'
        ),

    robot_type = LaunchConfiguration(robot_type_arg)
        
    x1_bringup_node = Node(
        package='x1_bringup',
        executable='Mcnamu_driver.py',
        name='driver_node',
        output='screen',
        condition=LaunchConfigurationEquals('robot_type', 'X1'),
        parameters=[{
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0,
            'angular_speed_limit': 5.0,
            'imu_link': 'imu_link',
        }],
        # remappings=[
        #     ('/pub_vel', '/vel_raw'),
        #     ('/pub_imu', '/imu/imu_raw'),
        #     ('/pub_mag', '/mag/mag_raw'),
        # ]
    ),

    return LaunchDescription([
      robot_type_arg,
      x1_bringup_node
    ])
