"""Package and console-entry configuration for jdamr_cube_vslam."""

from glob import glob
import os

from setuptools import setup


package_name = 'jdamr_cube_vslam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'evaluation'),
         glob('evaluation/*.md')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    scripts=glob('scripts/*.sh'),
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu.kr@gmail.com',
    description=(
        'JD-AMR Astra S RGB-D 기록, RTAB-Map 3D 매핑, '
        '궤적 정확도 평가 도구.'),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_csv_recorder = '
            'jdamr_cube_vslam.trajectory_csv_recorder:main',
            'rgbd_snapshot_ply = '
            'jdamr_cube_vslam.rgbd_snapshot_ply:main',
            'vslam_accuracy = '
            'jdamr_cube_vslam.trajectory_accuracy:main',
            'downsample_rgbd_bag = '
            'jdamr_cube_vslam.downsample_rgbd_bag:main',
            'fuse_rgbd_odom = '
            'jdamr_cube_vslam.fuse_rgbd_odom:main',
        ],
    },
)
