import os # To manipulate the file paths

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare launch arguments
    ns_arg = DeclareLaunchArgument(
        "ns",
        default_value="robot1",
        description="Namespace for the robot"
    )
    
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", 
        default_value="true",
        description="Flag to use the GUI for joint state publisher"
    )
    
    robot_type_arg = DeclareLaunchArgument(
        "robot_type", 
        # default_value=EnvironmentVariable("ROBOT_TYPE"),
        description="Robot type [X1,X3,X3plus,R2,X7]"
    )

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(
            get_package_share_directory("x1_description"),"urdf","x1.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )


    ns = LaunchConfiguration("ns")
    use_gui = LaunchConfiguration("use_gui")
    robot_type = LaunchConfiguration("robot_type")
    model = LaunchConfiguration("model")

    # The robot_description parameter contains the urdf file of the robot to be published by robot_state_publisher
    robot_description = ParameterValue(Command(["xacro ", model]))

    # Node definition
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters= [{"robot_description": robot_description}]
    ) 

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui", 
        condition= IfCondition(use_gui)
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition= UnlessCondition(use_gui)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("x1_description"),
            "rviz","display.rviz")]
    )

    return LaunchDescription([
        ns_arg,
        use_gui_arg,
        # robot_type_arg,
        model_arg,
        robot_state_publisher,
        # joint_state_publisher,
        # joint_state_publisher_gui,
        rviz_node
    ])