import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arduinobot_description"), "launch", "arduinobot_gazebo.launch.py")
        )
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arduinobot_control"), "launch", "arduinobot_controllers.launch.py")
        )
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("arduinobot_moveit2"), "launch", "arduinobot_moveit.launch.py")
        )
    )



    return LaunchDescription([
        gazebo,
        controllers,
        moveit
    ])