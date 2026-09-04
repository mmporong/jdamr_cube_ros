# 센서·시간 프로파일 — `corridor_localdds_armed_20260904T152036`

## 입력과 범위

- MCAP: `$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036/corridor_localdds_armed_20260904T152036_0.mcap`
- SHA-256: `65105177e43eca6153d2543f3cf8e9cc9f7c62b77f258975fdcdbd10b5e2e16a`
- 전체 캡처: 756.229초
- 분석 구간: successful_route / 558.690초
- 정적 노이즈 구간: pre_route_prelude / 60.0초
- 아래 수치는 실물 주행 MCAP의 관측값이며 외부 ground truth가 아니다.

## 스트림 시간 특성

| 스트림 | 메시지 | 관측 Hz | header 주기 p50 | header jitter p99 | recorder−header p95 | 최대 gap |
|---|---:|---:|---:|---:|---:|---:|
| LiDAR | 5,394 | 9.653 | 0.103561초 | 0.002164초 | 0.107408초 | 0.111922초 |
| wheel odom | 27,934 | 50.0 | 0.020001초 | 0.001432초 | 0.002507초 | 0.029342초 |
| IMU | 27,934 | 50.0 | 0.020001초 | 0.001432초 | 0.002997초 | 0.028942초 |
| odom→base TF | 11,173 | 20.0 | 0.042817초 | 0.017793초 | 0.002592초 | 0.066263초 |
| map→odom TF | 5,394 | 9.653 | 0.103561초 | 0.002164초 | -0.879918초 | 0.166762초 |

## 센서 관측

- LiDAR: 5,394 frames, 5,023,326 rays 중 60.50%가 선언 범위 안의 유효값이었다.
- LiDAR 유효 거리: p05 0.896m, p50 2.07m, p95 7.0503m.
- wheel odom: decimated path 79.259m, start-to-end 31.9285m.
- 주행 구간에서 odom이 정지로 분류한 IMU 표본: 14개.
- 주행 직전 정적 구간 IMU z축: median -0.0001745rad/s, robust sigma 0.0018113rad/s.

## 시뮬레이션 초기값 후보

이 값들은 실측 분포의 재현 시작점이다. 튜닝 완료값이나 센서 고유 사양으로 취급하지 않는다.

```yaml
measured_not_tuned: true
scan_nonfinite_ray_fraction: 0.395006
scan_period_s: 0.103561
scan_period_abs_jitter_p99_s: 0.002164
scan_recorder_minus_header_p95_s: 0.107408
imu_stationary_z_bias_radps: -0.0001745
imu_stationary_z_robust_sigma_radps: 0.0018113
odom_stationary_translation_increment_p99_m: 0.0
odom_stationary_abs_yaw_increment_p99_rad: 0.0
```

## 해석 한계

- The real-robot bag has no external ground truth, so additive LiDAR range noise and odometry accuracy are not identifiable.
- Recorder latency includes driver, scheduling, DDS, and recorder effects; it is not a sensor-only latency measurement.
- Non-finite LiDAR rays are no-return observations, not evidence of transport-level message dropout.
- The map-to-odom TF header is future-dated by localization tolerance, so recorder-minus-header is not latency for that TF.
- Stationary segments are inferred from wheel-odometry twist thresholds and are not an independently labelled experiment.
- Simulation candidates are observed initial envelopes, not tuned or validated sim-to-real parameters.
