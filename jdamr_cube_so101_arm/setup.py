from setuptools import setup

package_name = 'jdamr_cube_so101_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu.kr@gmail.com',
    description='SO-101 팔 관절 각도 제어 CLI. 관절 목표값을 주면 arm_controller/gripper_controller로 이동시킨다.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control = jdamr_cube_so101_arm.joint_control:main',
        ],
    },
)
