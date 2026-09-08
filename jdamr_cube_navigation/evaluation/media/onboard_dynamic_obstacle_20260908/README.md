# 보행자 횡단 정지 후 동일 목표 재개

사람 형태의 장애물이 좌측과 우측에서 각각 직선 주행 경로로 들어오는 두 실행을
검증했다. Collision Monitor가 정지 명령을 냈고, 장애물을 치운 뒤에는 새 목표를 보내지
않고 기존 NavigateToPose 목표를 이어서 수행했다. 영상 하단의 수치는 각 실행에서 함께
기록한 MCAP과 Contact sensor 결과다.

- `gazebo_pedestrian_bidirectional_reel.mp4`: 좌측·우측 진입 실행을 이은 34초 대표 영상
- `gazebo_dynamic_obstacle_highlight.mp4`: 좌측 진입·정지·도착 17초 영상
- `gazebo_dynamic_obstacle_right.mp4`: 우측 진입·정지·도착 17초 영상
- `gazebo_dynamic_obstacle_highlight.gif`: 웹 문서용 800×450 미리보기
- `gazebo_obstacle_stop.png`, `gazebo_obstacle_stop_right.png`: 방향별 정지 포스터
- `gazebo_goal_arrival.png`: 목표점 도착 포스터
- `portfolio_media_manifest.json`: 두 원본 실행과 출력물의 SHA-256, 표시 수치

카메라는 모바일 베이스를 따라가는 3인칭 사선 시점이다. 현재 LiDAR 주행 범위에 없는
SO-101 상체와 RGB-D 마스트는 촬영본에서만 숨겼고, collision·inertial·joint XML은
원본과 동일하다. 하이라이트의 긴 순항 구간만 8배속으로 줄였으며 횡단·정지·도착은
원래 프레임 순서를 유지한다. 용량을 줄이기 위해 저장소에는 편집 전 원본과 전체 길이
영상을 넣지 않았으며, manifest에 적은 `$HOME/jdamr_artifacts` 경로에 보존했다. 이
결과는 독립된 단일 보행자 Gazebo 통합 검증 2회이며, 동시 다중 보행자나 실차 제동거리·
사람 안전을 입증하지 않는다.
