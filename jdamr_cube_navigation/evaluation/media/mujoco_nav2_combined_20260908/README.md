# Nav2 고정·동적 장애물 통합 증거

Gazebo에서 실행한 `detour_sudden_stop_resume` PASS 기록을 MuJoCo 3D 장면으로
재생한 포트폴리오 미디어다. 복도 측면에서 18° 비스듬히 보는 3D 카메라가
주행 거리의 75%만 따라가므로, 로봇이 화면 왼쪽에서 오른쪽으로 이동한다.
출발 전부터 복도 중앙을 막은 박스를 Nav2가
우회하고, 이후 경로로 횡단하는 보행자에 대해 Collision Monitor가 정지한 뒤
보행자가 통로 한쪽 `y=+1.0 m`에서 반대쪽 `y=-1.0 m`까지 완전히 지나가면
같은 목표로 재개해 도착하는 순서를 담았다. 반투명 벽은 차량과 LiDAR 광선을
함께 보기 위한 시각 표현이며 센서용 충돌 형상은 유지된다.

- `mujoco_nav2_obstacle_challenge.mp4`: 1280×720, 24fps, 24초 대표 영상
- `mujoco_nav2_obstacle_challenge.gif`: 웹 미리보기
- `mujoco_nav2_obstacle_challenge.jpg`: 고정 박스 우회 장면
- `jdamr_nav2_portfolio_scene.xml`: 영상에 사용한 MuJoCo 장면
- `mujoco_portfolio_manifest.json`: 원본·출력 SHA-256, 검증 수치, 렌더링 자원 기록

주행 궤적·Nav2 plan·상태 전환은 ROS 2 MCAP에서 읽었고, 화면의 LiDAR
광선은 MuJoCo 장면에서 다시 계산했다. 따라서 이 영상은 MuJoCo 독립 물리
검증이 아니며, 원본 Gazebo/ROS 2 통합 실행을 읽기 쉽게 보여주는 증거
재생본이다.

실측 결과와 재현 명령은
[`20260908_DYNAMIC_OBSTACLE_READINESS.md`](../../20260908_DYNAMIC_OBSTACLE_READINESS.md)에 있다.
