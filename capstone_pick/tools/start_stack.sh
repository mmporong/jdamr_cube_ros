#!/bin/bash
# 캡스톤 시뮬 스택 기동 — systemd 유저 유닛으로 (세션이 죽어도 살아남음)
# 사용: ~/capstone_tools/start_stack.sh   / 중지: systemctl --user stop capstone-sim capstone-gui capstone-ui
#
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: DDS 디스커버리를 이 기기로 한정.
# 기본값(SUBNET)이면 같은 LAN(강의실)의 다른 머신 스택과 /clock·/odom·/cmd_vel이
# 섞인다 — 실측: /clock 발행자 6개(서로 다른 에폭)로 컨트롤러 시간이 널뛰어
# 3초 팔 궤적이 0.27초 스냅·무시로 갈렸고, /odom은 남의 로봇이 섞여 425mm 허위
# 이동을 만들었다. WSL2는 NAT라 우연히 격리됐던 것이고 네이티브는 명시해야 한다.
# 모든 노드·셸이 같은 범위를 써야 서로 보인다(~/.bashrc에도 동일 설정).
# GZ_PARTITION: gz-transport 디스커버리도 LAN 멀티캐스트라 같은 경로가 하나 더 있다.
# 다른 머신의 gz 서버가 이 머신의 브리지를 통해 /clock으로 들어오는 것을 파티션으로
# 차단한다. sim·gui·CLI 셸이 전부 같은 값이어야 서로 붙는다(~/.bashrc에도 동일 설정).
ISOLATE=(--setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone)
systemctl --user stop capstone-sim capstone-gui capstone-ui 2>/dev/null
systemctl --user reset-failed capstone-sim capstone-gui capstone-ui 2>/dev/null
sleep 2
systemd-run --user --collect "${ISOLATE[@]}" --unit=capstone-sim bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && exec ros2 launch jdamr_cube_gazebo gazebo.launch.py gui:=false world:=$HOME/jdamr_cube_ws/install/jdamr_cube_gazebo/share/jdamr_cube_gazebo/worlds/room.world'
systemd-run --user --collect "${ISOLATE[@]}" --unit=capstone-gui bash -c \
  'export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia; source /opt/ros/jazzy/setup.bash; sleep 10; exec gz sim -g'
systemd-run --user --collect "${ISOLATE[@]}" --unit=capstone-ui bash -c \
  'source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash; sleep 18; exec python3 ~/capstone_tools/pick_ui.py'
echo "capstone-sim / capstone-gui / capstone-ui 유닛 기동 (DDS LOCALHOST + GZ_PARTITION=lim-capstone)"
