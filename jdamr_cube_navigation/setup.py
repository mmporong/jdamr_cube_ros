from setuptools import setup
import os
from glob import glob

package_name = 'jdamr_cube_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu.kr@gmail.com',
    description='Nav2 기반 좌표 이동(goto pose) 컨트롤러. 저장된 맵을 로드해 목표 좌표까지 자율주행한다.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_pose = jdamr_cube_controller.goto_pose:main',
        ],
    },
)
