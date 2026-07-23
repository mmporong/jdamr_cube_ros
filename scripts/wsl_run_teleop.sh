#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
exec ros2 run jdamr_cube_teleop jdamr_cube_teleop
