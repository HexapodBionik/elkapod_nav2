from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('sim_mode', default=True)
    namespace_ekf = LaunchConfiguration('namespace', default='')

    elkapod_slam_dir = get_package_share_directory('elkapod_slam')
    elkapod_odometry_dir = get_package_share_directory('elkapod_odometry')
    ekf_config = os.path.join(elkapod_odometry_dir, 'config', 'ekf_config.yaml')
    odom_config = os.path.join(elkapod_odometry_dir, 'config', 'elkapod_odometry_params.yaml')

    relay_node = Node(
        package="elkapod_odometry",
        executable="elkapod_relay",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )

    odom_node = Node(
        package="elkapod_odometry",
        executable="elkapod_odom",
        parameters=[odom_config, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
        remappings=[('/tf', '/tf_garbage'), ('/tf_static', '/tf_static_garbage')]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name='ekf_filter_node', 
        parameters=[ekf_config, {'use_sim_time': use_sim_time, 'publish_tf': False}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode', default_value='true',
            description='Use sim_time'),
        relay_node,
        odom_node,
        ekf_node,
        # final_ekf_node
    ])
