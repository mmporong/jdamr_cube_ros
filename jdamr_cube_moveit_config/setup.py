from setuptools import setup
import os
from glob import glob

package_name = 'jdamr_cube_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu.kr@gmail.com',
    description='jdamr_cube SO-101 팔 MoveIt2 설정(SRDF, kinematics, controllers, move_group launch)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
