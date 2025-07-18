from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = x1---------------------")
def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('x1_description')
    default_model_path = urdf_tutorial_path / 'urdf/x1.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/display.rviz'
    print(default_model_path)
    print(default_rviz_config_path)

    gui_arg = DeclareLaunchArgument(
        name='gui', 
        default_value='true', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', 
        default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    # ========== ROBOT MODEL SIMULATION ==========
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # ========== RVIZ VISUALIZATION ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # ========== LOW LEVEL DRIVER ==========
    driver_node = Node(
        package='x1_bringup',
        executable='Mcnamu_driver',
    )

    # ========== ODOMETER DATA PUBLISH ==========
    base_node = Node(
        package='x1_base_node',
        executable='base_node_x1',
        # When using EKF fusion, this tf is published by EKF.
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # ========== IMU DATA FILTERING AND FUSION ==========
    imu_filter_config = os.path.join(              # Complete path of the imu parameter file
        get_package_share_directory('x1_bringup'),
        'config',
        'imu_filter_param.yaml'
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config]
    )

    # ========== EXTENDED KF FILTER ==========
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    # ========== JOYSTICK HANDLE CONTROL ==========
    x1_joy_node = Node(
        package='x1_control',
        executable='x1_joy.py',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        pub_odom_tf_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        driver_node,
        base_node,
        imu_filter_node,
        # ekf_node,
        x1_joy_node
    ])
