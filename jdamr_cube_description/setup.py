from setuptools import setup
import os
from glob import glob

package_name = 'jdamr_cube_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 설치 경로 설정 [cite: 37, 58]
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF 파일 설치 경로 설정 [cite: 37, 58]
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Mesh 파일 설치 경로 설정 (so101 팔 STL 포함)
        (os.path.join('share', package_name, 'meshes', 'so101'), glob('meshes/so101/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'so101'), glob('meshes/so101/*.md')),
        # nav2 등 컨트롤러 파라미터 파일
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz 설정 파일 설치 경로 설정 (있을 경우) [cite: 49]
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@todo.todo',
    description='Description package for jdamr_cube using Python setup',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행 노드가 있다면 여기에 추가
        ],
    },
)