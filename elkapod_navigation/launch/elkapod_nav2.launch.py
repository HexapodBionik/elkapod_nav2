from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
# from nav2_common.launch import LaunchConfigAsBool, RewrittenYaml
import datetime
import os


def launch_setup(context: LaunchContext, *args, **kwargs):
    elkapod_navigation_dir = get_package_share_directory('elkapod_navigation')

    nav2_config_path = os.path.join(
        elkapod_navigation_dir,
        'config',
        'nav_params.yaml'
    )

    use_respawn = LaunchConfiguration('use_respawn').perform(context)
    use_respawn = use_respawn == 'true' or use_respawn == "True"
    log_level = LaunchConfiguration('log_level')

    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        # 'smoother_server',
        # 'route_server',
        # 'velocity_smoother',
        # 'collision_monitor',
        # 'waypoint_follower',
        # 'docking_server',
        # 'following_server',
    ]
    nodes = [
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_config_path],
            arguments=['--ros-args', '--log-level', log_level],
            # remappings=remappings,
            namespace='navigation'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_config_path],
            arguments=['--ros-args', '--log-level', log_level],
            # remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            remappings=[('cmd_vel', '/cmd_vel')],
            namespace='navigation'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_config_path],
            arguments=['--ros-args', '--log-level', log_level],
            # remappings=remappings,
            namespace='navigation'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_config_path],
            arguments=['--ros-args', '--log-level', log_level],
            # remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            namespace='navigation'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}],
            namespace='navigation'
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated clock.'),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='log level'
        ),
        OpaqueFunction(function=launch_setup),
    ])
