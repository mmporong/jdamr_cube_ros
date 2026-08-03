#!/bin/bash
# 캡스톤 시뮬 스택 기동 — systemd 유저 유닛으로 (세션이 죽어도 살아남음)
# 사용: ~/capstone_tools/start_stack.sh   / 중지: systemctl --user stop capstone-sim capstone-gui capstone-ui
systemctl --user stop capstone-sim capstone-gui capstone-ui 2>/dev/null
sleep 1
systemd-run --user --collect --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world'
systemd-run --user --collect --unit=capstone-gui bash -c \
  'export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia; source /opt/ros/jazzy/setup.bash; sleep 10; exec gz sim -g'
systemd-run --user --collect --unit=capstone-ui bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash; sleep 18; exec python3 ~/capstone_tools/pick_ui.py'
echo "capstone-sim / capstone-gui / capstone-ui 유닛 기동"
