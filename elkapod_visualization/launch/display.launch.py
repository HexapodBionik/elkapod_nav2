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
    urdf_launch_package = FindPackageShare(package="elkapod_description").find(
        "elkapod_description"
    )

    vis_package = FindPackageShare(package="elkapod_visualization").find(
        "elkapod_visualization"
    )

    default_rviz_config_path = os.path.join(vis_package, "config/elkapod.rviz")


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

