import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    jdamr200_cartographer_prefix = get_package_share_directory('jdamr200_cartographer')
    jdamr200_config_dir = LaunchConfiguration('jdamr200_config_dir', default=os.path.join(jdamr200_cartographer_prefix, 'config'))

    configuration_basename = LaunchConfiguration('configuration_basename', default='jdamr200_lidar.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    ld14lidar_launch_file_dir = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'),
        'launch'
    )

    cartographer_launch_file_dir = os.path.join(
        get_package_share_directory('jdamr200_cartographer'),
        'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=jdamr200_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

                
        # Include the launch file that starts the ydlidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ld14lidar_launch_file_dir, '/ld14.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        
        
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # map TF to odom TF
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),

        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            # odom TF to base_footprint
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'body_link']
        ),
        
        
    
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),


        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid map'),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}],
            arguments=[ '-configuration_directory', jdamr200_config_dir,
                        '-configuration_basename', configuration_basename]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_launch_file_dir, '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time' : use_sim_time,'resolution': resolution, 'publish_period_sec': publish_period_sec}.items(),
        ),

        

    ])
