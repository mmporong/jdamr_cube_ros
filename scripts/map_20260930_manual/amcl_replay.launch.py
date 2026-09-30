"""map_server + amcl + lifecycle manager on sim time, for amcl_replay.py run."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('map'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': LaunchConfiguration('map'), **sim}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[LaunchConfiguration('params'), sim]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_replay', output='screen',
             parameters=[{'node_names': ['map_server', 'amcl'], 'autostart': True,
                          'bond_timeout': 0.0, **sim}]),
    ])
