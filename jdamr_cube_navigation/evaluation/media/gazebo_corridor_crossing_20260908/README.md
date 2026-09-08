# Gazebo 보행자 통로 완전 횡단 증거

저장 지도에 없는 사람형 장애물이 통로 한쪽 `y=+1.0 m`에서 나타나 로봇 앞을
지난 뒤 반대쪽 `y=-1.0 m`까지 횡단한 Gazebo/ROS 2 통합 실행의 미디어다.
Collision Monitor가 로봇을 정지했고, 장애물이 통로를 벗어나면 기존 Nav2 목표를
취소하거나 다시 보내지 않고 주행을 재개해 도착했다.

- `gazebo_dynamic_obstacle_highlight.mp4`: 횡단·정지·재개·도착 하이라이트
- `gazebo_dynamic_obstacle_highlight.gif`: 웹 미리보기
- `gazebo_obstacle_stop.png`: 보행자 정지 장면
- `gazebo_goal_arrival.png`: 최종 목표 도착 장면
- `portfolio_media_manifest.json`: 원본·출력 SHA-256과 검증 수치

편집 전 68.6초 Gazebo 카메라 원본과 전체 오버레이 영상은 저장소에 넣지 않고
`$HOME/jdamr_artifacts/onboard_candidate_combined_20260908_v05`와
`$HOME/jdamr_artifacts/gazebo_corridor_crossing_20260908_v01`에 보존한다.
사람 형태는 센서에 잡히는 장애물 proxy를 보여주는 시각 모델이며, 사람 인식이나
실차 제동거리 검증을 뜻하지 않는다.
