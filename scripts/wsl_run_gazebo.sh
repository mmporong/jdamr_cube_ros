#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
ROOM_WORLD="$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world"
exec ros2 launch jdamr_cube_gazebo gazebo.launch.py world:="$ROOM_WORLD"
