// Copyright 2026 The ldlidar_sl_ros2 Authors

#ifndef SCAN_UTILS_HPP_
#define SCAN_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ldlidar_ros2 {

constexpr double kDefaultScanTime = 1.0 / 6.0;
constexpr double kMinScanTime = 0.05;
constexpr double kMaxScanTime = 0.5;
constexpr double kTwoPi = 2.0 * M_PI;

struct ScanTiming {
  uint64_t start_stamp_ns;
  double scan_time;
  double time_increment;
  bool uses_sensor_timestamps;
};

inline double FallbackScanTime(double lidar_spin_freq) {
  const double period = lidar_spin_freq > 0.0 ? 1.0 / lidar_spin_freq : kDefaultScanTime;
  if (!std::isfinite(period)) {
    return kDefaultScanTime;
  }
  return std::max(kMinScanTime, std::min(period, kMaxScanTime));
}

inline ScanTiming ComputeScanTiming(
    uint64_t first_stamp_ns, uint64_t last_stamp_ns, std::size_t beam_count,
    double lidar_spin_freq, uint64_t publish_stamp_ns) {
  const double fallback = FallbackScanTime(lidar_spin_freq);
  double scan_time = fallback;
  bool valid = false;

  if (first_stamp_ns > 0 && last_stamp_ns > first_stamp_ns) {
    const double measured = static_cast<double>(last_stamp_ns - first_stamp_ns) * 1e-9;
    if (std::isfinite(measured) && measured >= kMinScanTime && measured <= kMaxScanTime) {
      scan_time = measured;
      valid = true;
    }
  }

  const uint64_t fallback_ns = static_cast<uint64_t>(scan_time * 1e9);
  const uint64_t start_stamp_ns = valid
      ? first_stamp_ns
      : (publish_stamp_ns > fallback_ns ? publish_stamp_ns - fallback_ns : 0);
  const double time_increment = beam_count > 1
      ? scan_time / static_cast<double>(beam_count - 1)
      : 0.0;
  return {start_stamp_ns, scan_time, time_increment, valid};
}

struct ScanGeometry {
  double angle_min;
  double angle_max;
  double angle_increment;
};

inline double NormalizeAngle(double angle) {
  angle = std::fmod(angle, kTwoPi);
  return angle < 0.0 ? angle + kTwoPi : angle;
}

inline double PublishedAngle(double sensor_angle, bool reverse_angle_axis) {
  const double normalized = NormalizeAngle(sensor_angle);
  return reverse_angle_axis ? NormalizeAngle(kTwoPi - normalized) : normalized;
}

inline ScanGeometry ComputeScanGeometry(
    std::size_t beam_count, bool chronological_clockwise, double first_angle) {
  if (beam_count <= 1) {
    return {chronological_clockwise ? NormalizeAngle(first_angle) : 0.0,
            chronological_clockwise ? NormalizeAngle(first_angle) : kTwoPi, 0.0};
  }

  if (chronological_clockwise) {
    const double angle_min = NormalizeAngle(first_angle);
    return {angle_min, angle_min - kTwoPi,
            -kTwoPi / static_cast<double>(beam_count - 1)};
  }
  return {0.0, kTwoPi, kTwoPi / static_cast<double>(beam_count - 1)};
}

inline int AngleToIndex(
    double angle, const ScanGeometry& geometry, std::size_t beam_count) {
  if (beam_count <= 1 || geometry.angle_increment == 0.0) {
    return 0;
  }

  double offset;
  if (geometry.angle_increment < 0.0) {
    offset = NormalizeAngle(geometry.angle_min - NormalizeAngle(angle));
  } else {
    offset = NormalizeAngle(angle - geometry.angle_min);
  }
  const double step = std::abs(geometry.angle_increment);
  const int index = static_cast<int>(std::ceil(offset / step - 1e-9));
  return std::min(index, static_cast<int>(beam_count - 1));
}

}  // namespace ldlidar_ros2

#endif  // SCAN_UTILS_HPP_
