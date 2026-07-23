-- jdamr_cube_2d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", -- jdamr_cube의 중심 프레임 [cite: 20, 35]
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false, -- published_frame과 odom_frame이 둘 다 "odom"이라 true로 두면
                               -- cartographer가 odom -> odom 자기자신에게 TF를 발행하려다
                               -- TF_SELF_TRANSFORM 에러가 발생함. map -> odom만 발행하도록 false.
  publish_frame_projected_to_2d = false,
  use_odometry = false, -- gz-sim DiffDrive의 /odom 첫 메시지가 트래젝토리 시작 시점과
                         -- 동일한 타임스탬프로 들어와 cartographer_node가 죽는 문제 회피.
                         -- online_correlative_scan_matching으로 스캔 매칭만으로 충분함.
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
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false -- IMU 미사용 시 false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options