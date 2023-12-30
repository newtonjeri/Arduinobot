import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_filename = "arduinobot.xacro"
    xacro_file = os.path.join(get_package_share_directory("arduinobot_description"), "urdf", xacro_filename)

    model_args = DeclareLaunchArgument(
        name = "model",
        default_value = xacro_file,
        description = "Absolue path to robot urdf"
    )

    robot_description = ParameterValue(
        Command(
            ['xacro ', LaunchConfiguration("model")]    
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[xacro_file],
    )



    joint_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_controller",
                "--controller-manager", "/controller_manager",
            ]

    )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ]
        )
    
    gripper_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--controller-manager", "/controller_manager",
            ]
    )

    return LaunchDescription(
        [
            # model_args,
            # robot_state_publisher,
            joint_state_broadcaster,
            joint_trajectory_controller,
            gripper_controller, 
        ]
    )
