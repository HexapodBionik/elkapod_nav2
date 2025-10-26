from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('sim_mode')

    elkapod_slam_dir = get_package_share_directory('elkapod_slam')
    ekf_config = os.path.join(
        elkapod_slam_dir,
        'config',
        'ekf.yaml'
    )
    
    leg_odometry_launch_path = os.path.join(
        get_package_share_directory('elkapod_odometry'),
        'launch',
        'odom.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leg_odometry_launch_path),
            launch_arguments={'sim_mode': use_sim_time,
                              'odom_filtered_topic': '/odometry/filtered_leg'}.items()
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name='fusion_ekf',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            output='screen',
            emulate_tty=True),
    ])
