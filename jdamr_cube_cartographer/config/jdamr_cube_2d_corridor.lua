-- Cartographer 2D — JDAMR Cube + LD14 + 복도/방 (검증된 TB3 레시피 이식)
--
-- 2026-08-24 재작성: 종전 복도 프로파일은 거울반전 버그 시절에 손댄 미검증 값
-- 덩어리였다(상관매칭 off·가중치 1e5/1e1·motion_filter 0.1°). 그 상태로 방을
-- 찍으니 상관매칭 off 탓에 발산해 부챗살이 나왔다. 그래서 실측으로 성공한
-- 터틀봇 복도 레시피(~/tb3_maps/carto_lds01.lua, 51.2m 직선벽 성공)를 그대로
-- 이식하고, JDAMR에서 강제로 다른 것 하나만 바꿨다 → IMU.
--
-- IMU 차이(유일한 강제 이탈):
--   TB3는 use_imu_data=true·tracking_frame=imu_link 로 자이로가 회전을 받쳤다.
--   JDAMR은 가속도계가 데이터를 안 준다(정지 |a|=362 또는 0). 카토그래퍼 IMU는
--   가속도로 중력을 먼저 잡으므로, 켜면 오히려 발산한다. 그래서 IMU는 끈다.
--   → 복도 퇴화 방어가 TB3보다 약하다. 방에서는 문제없고, 긴 복도는
--     폐루프 주행(출발점 복귀)으로 메운다. IMU가 살면 이 두 줄만 되돌린다.
--
-- 측정 근거(LD14): 0.15~8 m, 6 Hz. 오도메트리 720도 회전 오차 1.3%(검증됨).
include "map_builder.lua"
include "trajectory_builder.lua"
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",     -- IMU 미사용이라 base_link (TB3는 imu_link)
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,       -- 드라이버가 odom->base 발행, 카토는 map->odom만
  publish_frame_projected_to_2d = true,
  use_odometry = true,              -- 바퀴 오도메트리를 사전정보로 (검증된 1.3% 오차)
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,   -- TB3 성공값. 회전왜곡 나오면 그때 올린다
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,    -- 스캔 stamp가 +140ms 과거라 여유
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2   -- Pi 4에서 돌므로 노트북(3)보다 낮춤
-- ── 라이다 실측 사거리 (LD14) ──
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 3.5           -- JDAMR 강의실 성공값(5.49%)과 동일
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0   -- 자유공간을 넉넉히 비움(TB3값)
-- ── IMU 끔 (JDAMR 강제 이탈) ──
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- ── 스캔매칭: 상관매칭 ON (이 설정의 핵심 — 끄면 발산) ──
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.
-- ── 노드 생성 간격 (motion_filter) — Pi 부하 방지 ──
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)   -- 종전 0.1°는 노드폭증
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
-- ── 포즈 그래프 ──
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 6.
-- 오도메트리를 균형 있게 신뢰 (종전 1e5/1e1 극단 편중 폐기)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
return options
