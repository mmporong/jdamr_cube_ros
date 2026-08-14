-- jdamr_cube_2d_real.lua — 실기용 (시뮬용 jdamr_cube_2d.lua 와 분리)
--
-- 시뮬판과의 차이와 근거:
--  * use_odometry = true
--      시뮬판이 false 였던 이유는 gz DiffDrive 첫 메시지 타임스탬프 문제였다.
--      실기는 jdamr_base_driver 가 유효한 시각·공분산의 /odom 을 50Hz 로 내므로
--      회전 중 스캔 왜곡 보정에 쓴다. 강사원본 E3(초기 추정 전무)의 반대 처방.
--      효과 판정은 ROADMAP_MAP 6절대로 RPE 온/오프 비교로 한다.
--  * max_range 3.5 → 8.0
--      3.5 는 TB3 LDS-01 값의 잔재. LD14 는 8m 급 — 복도·큰 방에서 원거리
--      벽을 버리면 루프 클로저 재료가 준다. (실측 스캔으로 상한 재확인할 것)
--  * min_range 0.12 → 0.15
--      강사원본 E9 지적 자리. LD14 근거리 노이즈 실측 전 보수값.
--  * online correlative 는 켜 둔다 — odometry 검증 전 안전망. 오도메트리가
--    RPE 로 검증되면 꺼서 파이4 CPU 를 아낀다 (E4 의 반대 방향 여유).
--
-- E10 메모: IMU 를 살리려면 use_imu_data=true + tracking_frame 을 IMU 프레임으로
-- 함께 바꿔야 한다. 지금은 둘 다 끔/base_link 로 정합.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",       -- 드라이버가 odom→base TF 담당, 여긴 map→odom 만 (E1 방지)
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
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

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- motion_filter 는 기본값 유지 — 강사원본 E4(0.1° 과밀)를 반복하지 않는다

return options
