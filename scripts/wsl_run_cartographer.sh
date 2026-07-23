#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
exec ros2 launch jdamr_cube_cartographer cartographer.launch.py
