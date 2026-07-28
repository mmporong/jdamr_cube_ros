from setuptools import find_packages, setup

package_name = 'capstone_pick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmporong',
    maintainer_email='mmporong@gmail.com',
    description='비전 기반 물체 판단 + 캘리브레이션 파지 파이프라인',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick = capstone_pick.pick_node:main',
        ],
    },
)
