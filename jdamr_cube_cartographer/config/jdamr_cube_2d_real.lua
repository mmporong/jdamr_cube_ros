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
  num_subdivisions_per_laser_scan = 1,
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
-- 2026-08-14 복도 실측 반영: 캐비닛 간격이 3.5m 를 넘어 시야 크롭이
-- 무특징 구간을 인위로 만들었다(사용자 관측). 6m 로 앵커를 잡는다.
TRAJECTORY_BUILDER_2D.max_range = 6.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 2026-08-14 최종: 전역 상관매칭 OFF — 복도 세로축에서 틀린 정렬로 점프
-- (가중치는 ceres 에만 적용돼 상관매칭 점프를 못 막는다). 캘리브레이션된
-- 오도메트리(0.986/0.999)가 씨앗을 공급한다.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 2026-08-14 복도 대응: 세로 방향 무특징 구간에서 매처가 전진을 기각
-- (제자리 정지→턴 후 점프 실측). 오도메트리가 1% 캘리브레이션 상태라
-- TB3 기본(10/40)보다 사전추정을 강하게 믿는다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 40
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
