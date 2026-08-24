-- jdamr_cube_2d_real.lua — 실기용 v2 (2026-08-14)
-- TB3 검증 레시피(turtlebot3_lds_2d.lua) 전체 이식 — 같은 강의실에서
-- 실증된 구성을 그대로 쓴다. 바꾼 것은 프레임뿐:
--   tracking_frame: imu_link → base_link (IMU 미사용은 TB3 도 동일)
-- 유지한 TB3 값: 전역매칭 켬 · motion_filter 0.1° · max_range 3.5
-- (max_range 를 8 로 넓혔던 v1 은 다리 격자·원거리 반복 패턴을 끌어들여
--  스냅 점프를 유발한 것으로 판단 — 3.5 로 근거리 구조만 쓴다)
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,  -- 2026-08-21: 6Hz 라이다 회전왜곡(167ms/스캔) 보정 — 1이면 스캔 전체를 한 순간으로 취급해 회전 중 부챗살 번짐
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
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
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- 8/14 에 직각 코너를 만든 값. 복도용 6m 는 corridor 프로파일에 있다.
TRAJECTORY_BUILDER_2D.max_range = 3.5
-- 2026-08-18 실측 대응: 빔의 24%가 무응답으로 돌아온다(검은 다리·유리·틈새).
-- 그 빔마다 3m 를 자유공간으로 칠하면 초당 950개 광선이 정보 없는 방향을
-- 비우면서 이미 그려둔 벽까지 지운다(벽/자유 0.36%, 정상 2~6%).
-- 이 값은 "무응답 빔을 얼마나 믿고 비울 것인가"이지 관측 거리가 아니다 —
-- 실제로 맞고 돌아온 빔은 max_range 6m 까지 그대로 벽으로 찍힌다.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 2026-08-18 최종: 켠다. 실제 지도 품질이 켰을 때 더 좋았다(사용자 판독).
-- ✎ 한때 "0.4m 이동을 66도 틀리게 본다"는 프로브 결과로 껐으나 그 시험이
--   틀렸다 — map 변위와 odom 변위의 일치를 채점했는데, 오도메트리 오차를
--   교정하는 것이 SLAM 의 본업이라 제대로 교정할수록 짧은 구간에서 크게
--   어긋나 보인다. 정합 품질의 심판은 지도이지 두 계층의 일치가 아니다.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- TB3 기본값. 사전추정을 과신하면 스캔이 자세를 못 고쳐 벽이 번진다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 2026-08-16 RFC 0018(cartographer-project/rfcs) 검증 설정 이식.
-- 후단 최적화에서 바퀴 오도메트리의 병진과 회전을 갈라 신뢰한다:
--   병진 1e5 — 엔코더 적산은 고해상도이고 우리 것은 실측 배율 0.986
--   회전 1e1 — 스키드 조향은 회전에서 미끄러진다. 회전은 스캔 매칭이 맡는다
-- 두 값의 1만 배 격차가 요점이다. 같이 높이면 복도에서 잘못된 회전까지
-- 믿게 되고, 같이 낮추면 무특징 구간에서 전진이 기각된다.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1

-- 2026-08-18: 벽 셀이 점유로 굳지 못하는 문제(확신 셀 0.28%) 대응.
-- 기본 0.55 는 0.5→0.8 로 가는 데 히트 7회가 필요해, 포즈가 조금만 흔들려
-- 미스가 끼면 되돌아간다. 0.65 면 2~3회로 굳는다.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65

return options
