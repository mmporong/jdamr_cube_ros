#!/bin/bash
# 캡스톤 시뮬 스택 기동 — systemd 유저 유닛으로 (세션이 죽어도 살아남음)
# 사용: ~/capstone_tools/start_stack.sh   / 중지: systemctl --user stop capstone-sim capstone-gui capstone-ui
systemctl --user stop capstone-sim capstone-gui capstone-ui 2>/dev/null
sleep 1
systemd-run --user --collect --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world'
systemd-run --user --collect --unit=capstone-gui bash -c \
  'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GALLIUM_DRIVER=d3d12 DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000; source /opt/ros/jazzy/setup.bash; sleep 10; exec gz sim -g'
systemd-run --user --collect --unit=capstone-ui bash -c \
  'export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000; source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash; sleep 18; exec python3 ~/capstone_tools/pick_ui.py'
echo "capstone-sim / capstone-gui / capstone-ui 유닛 기동"
