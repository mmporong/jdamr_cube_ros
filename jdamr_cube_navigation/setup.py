"""Package and console-entry configuration for jdamr_cube_navigation."""

from glob import glob
import os

from setuptools import setup

package_name = 'jdamr_cube_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'evaluation'),
            glob('evaluation/*.*')),
        (os.path.join('share', package_name, 'evaluation', 'assets',
                      'nav_obstacle'),
            glob('evaluation/assets/nav_obstacle/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu.kr@gmail.com',
    description='Nav2 기반 좌표 이동(goto pose) 컨트롤러. 저장된 맵을 로드해 목표 좌표까지 자율주행한다.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'box_parking_start = jdamr_cube_navigation.box_parking_start:main',
            'frontier_explorer = jdamr_cube_navigation.frontier_explorer:main',
            'corridor_route = jdamr_cube_navigation.corridor_route:main',
            'goto_pose = jdamr_cube_navigation.goto_pose:main',
            'keepout_mask = jdamr_cube_navigation.keepout_mask:main',
            'keepout_zone_capture = '
            'jdamr_cube_navigation.keepout_zone_capture:main',
            'tf_replay_filter = '
            'jdamr_cube_navigation.tf_replay_filter:main',
            'soak_metrics = jdamr_cube_navigation.soak_metrics:main',
            'nav2_liveness_guard = '
            'jdamr_cube_navigation.nav2_liveness_guard:main',
            'sim_slam_route = '
            'jdamr_cube_navigation.sim_slam_route:main',
            'sim_fault_injector = '
            'jdamr_cube_navigation.sim_fault_injector:main',
            'traction_velocity_guard = '
            'jdamr_cube_navigation.traction_velocity_guard:main',
            'sim_nav_obstacle_scenario = '
            'jdamr_cube_navigation.sim_nav_obstacle_scenario:main',
            'sim_scan_gate = '
            'jdamr_cube_navigation.sim_scan_gate:main',
            'sim_collision_monitor_scenario = '
            'jdamr_cube_navigation.sim_collision_monitor_scenario:main',
            'sim_restaurant_replay_scenario = '
            'jdamr_cube_navigation.sim_restaurant_replay_scenario:main',
            'g005_ground_truth_localization = '
            'jdamr_cube_navigation.g005_ground_truth_localization:main',
            'g005_frontier_observer = '
            'jdamr_cube_navigation.g005_frontier_observer:main',
            'g005_frontier_coordinator = '
            'jdamr_cube_navigation.g005_frontier_coordinator:main',
            'depth_box_parking = '
            'jdamr_cube_navigation.depth_box_parking:main',
            'box_approach_shadow = '
            'jdamr_cube_navigation.box_approach_shadow:main',
            'box_approach_execution = '
            'jdamr_cube_navigation.box_approach_execution:main',
            'depth_obstacle_filter = '
            'jdamr_cube_navigation.depth_obstacle_filter:main',
            'restaurant_service = '
            'jdamr_cube_navigation.restaurant_service:main',
        ],
    },
)
