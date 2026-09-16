"""Launch RTAB-Map for the JD-AMR Astra S RGB-D topic contract."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def generate_launch_description():
    """Build a camera-only metric RGB-D SLAM launch description."""
    rtabmap_launch = PathJoinSubstitution([
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py',
    ])

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path',
                              default_value='~/.ros/jdamr_rgbd.db'),
        DeclareLaunchArgument('delete_database', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
    ]

    database_args = [
        '--Mem/IncrementalMemory true',
        '--Mem/InitWMWithAllNodes false',
        '--RGBD/NeighborLinkRefining true',
        '--RGBD/ProximityBySpace true',
        '--RGBD/OptimizeMaxError 2.0',
        '--Reg/Strategy 0',
        '--Reg/Force3DoF true',
        '--Vis/MinInliers 15',
        '--Grid/Sensor 1',
        '--Grid/3D true',
        '--Grid/RangeMax 5.0',
        '--Rtabmap/DetectionRate 2.0',
    ]
    mapping_args = ' '.join(database_args)

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            'args': [
                PythonExpression([
                    "'-d ' if '",
                    LaunchConfiguration('delete_database'),
                    "'.lower() == 'true' else ''",
                ]),
                mapping_args,
            ],
            'database_path': LaunchConfiguration('database_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'camera_link',
            'map_frame_id': 'map',
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth': 'true',
            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'subscribe_scan': 'false',
            'rgbd_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync': 'true',
            'qos': '2',
            'topic_queue_size': '30',
            'sync_queue_size': '30',
            'approx_sync_max_interval': '0.03',
            'odom_always_process_most_recent_frame': 'false',
            'wait_for_transform': '0.3',
            'rviz': LaunchConfiguration('rviz'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [include])
