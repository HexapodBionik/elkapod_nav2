import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
import yaml

def generate_launch_description():
    # Find the package
    urdf_launch_package = FindPackageShare(package="elkapod_description").find(
        "elkapod_description"
    )

    default_rviz_config_path = os.path.join(urdf_launch_package, "config/elkapod.rviz")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path],
        )
    )

    return ld

