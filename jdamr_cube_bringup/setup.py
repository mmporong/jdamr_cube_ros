from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'jdamr_cube_bringup'


def collect_systemd_files():
    """Preserve systemd unit and drop-in directory layout when installed."""
    files_by_destination = {}
    for source in glob('systemd/**/*', recursive=True):
        if not os.path.isfile(source):
            continue
        relative_directory = os.path.relpath(
            os.path.dirname(source), 'systemd')
        destination = os.path.join('share', package_name, 'systemd')
        if relative_directory != '.':
            destination = os.path.join(destination, relative_directory)
        files_by_destination.setdefault(destination, []).append(source)
    return [
        (destination, sorted(sources))
        for destination, sources in sorted(files_by_destination.items())
    ]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. 런치 파일 설치 설정 [cite: 93]
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ] + collect_systemd_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdedu',
    maintainer_email='jdedu@todo.todo',
    description='Bringup package for jdamr_cube hardware and TF',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
