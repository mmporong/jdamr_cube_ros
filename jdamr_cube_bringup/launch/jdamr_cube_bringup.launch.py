"""검증된 JD-AMR 실기 브링업의 하위 호환 진입점."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """기본 런치 이름을 real_bringup.launch.py에 연결한다."""
    real_launch = os.path.join(
        get_package_share_directory('jdamr_cube_bringup'),
        'launch',
        'real_bringup.launch.py',
    )

    defaults = {
        'base_port': '/dev/ttyS0',
        'lidar_port': '/dev/ydlidar_g4',
        'wheel_radius': '0.0329',
        'wheel_separation': '0.510',
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    forwarded_arguments = {
        name: LaunchConfiguration(name)
        for name in defaults
    }

    return LaunchDescription([
        *declarations,
        LogInfo(msg=(
            'jdamr_cube_bringup.launch.py는 실기용 real_bringup.launch.py를 '
            '실행합니다. 시뮬레이션은 jdamr_cube_gazebo 패키지를 사용하세요.'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments=forwarded_arguments.items(),
        ),
    ])
