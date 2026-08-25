// Copyright 2026 Lim
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace ydlidar_g4_ros2
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct RawPoint
{
  double angle_rad;
  float range;
  float intensity;
};

struct ScanTiming
{
  std::uint64_t stamp_ns;
  double scan_time;
  float time_increment;
  std::uint64_t scan_end_ns;
  std::uint64_t stamp_adjustment_ns;
  bool used_scan_time_fallback;
};

struct BinnedScan
{
  double angle_first;
  double angle_last;
  double angle_increment;
  std::vector<float> ranges;
  std::vector<float> intensities;
};

inline double normalize_angle(double angle)
{
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

inline double clockwise_delta(double first_ros_angle, double ros_angle)
{
  double delta = std::fmod(first_ros_angle - ros_angle, kTwoPi);
  if (delta < 0.0) {
    delta += kTwoPi;
  }
  return delta;
}

inline bool has_sdk_scan_period(double sdk_scan_time)
{
  return std::isfinite(sdk_scan_time) && sdk_scan_time > 0.0;
}

inline ScanTiming resolve_timing(
  std::uint64_t sdk_stamp_ns, double sdk_scan_time, double frequency,
  std::uint64_t now_ns, std::size_t beam_count,
  std::uint64_t previous_scan_end_ns)
{
  constexpr double kMinScanTime = 0.05;
  constexpr double kMaxScanTime = 0.5;
  const double safe_frequency =
    std::isfinite(frequency) && frequency > 0.0 ? frequency : 10.0;
  const double fallback = std::clamp(1.0 / safe_frequency, kMinScanTime, kMaxScanTime);
  // A missed revolution produces roughly 2x the configured period. The G4's
  // normal fixed-speed jitter is far inside this 20% acceptance window.
  const double minimum_expected = std::max(kMinScanTime, 0.8 * fallback);
  const double maximum_expected = std::min(kMaxScanTime, 1.2 * fallback);
  const bool sdk_scan_time_is_valid =
    std::isfinite(sdk_scan_time) && sdk_scan_time >= minimum_expected &&
    sdk_scan_time <= maximum_expected;
  const double scan_time =
    sdk_scan_time_is_valid ? sdk_scan_time : fallback;
  const auto duration_ns = static_cast<std::uint64_t>(scan_time * 1.0e9);
  const std::uint64_t raw_stamp_ns =
    sdk_stamp_ns != 0 ? sdk_stamp_ns : (now_ns > duration_ns ? now_ns - duration_ns : 0);
  constexpr std::uint64_t kTimestampRoundingGuardNs = 1'000;
  std::uint64_t minimum_stamp_ns = previous_scan_end_ns;
  if (minimum_stamp_ns != 0) {
    minimum_stamp_ns =
      minimum_stamp_ns >
      std::numeric_limits<std::uint64_t>::max() - kTimestampRoundingGuardNs ?
      std::numeric_limits<std::uint64_t>::max() :
      minimum_stamp_ns + kTimestampRoundingGuardNs;
  }
  const std::uint64_t stamp_ns = std::max(raw_stamp_ns, minimum_stamp_ns);
  const std::size_t safe_beam_count = std::max<std::size_t>(2, beam_count);
  const float time_increment =
    static_cast<float>(scan_time / static_cast<double>(safe_beam_count));
  const long double scan_span_ns =
    static_cast<long double>(safe_beam_count - 1) * time_increment * 1.0e9L;
  const auto span_ns = static_cast<std::uint64_t>(std::ceil(scan_span_ns));
  const std::uint64_t scan_end_ns = stamp_ns >
    std::numeric_limits<std::uint64_t>::max() - span_ns ?
    std::numeric_limits<std::uint64_t>::max() : stamp_ns + span_ns;
  return {
    stamp_ns, scan_time, time_increment, scan_end_ns,
    stamp_ns - raw_stamp_ns, !sdk_scan_time_is_valid};
}

inline BinnedScan bin_chronological_scan(const std::vector<RawPoint> & points)
{
  const std::size_t bin_count = std::max<std::size_t>(2, points.size());
  const double angle_increment = -kTwoPi / static_cast<double>(bin_count);
  const float infinity = std::numeric_limits<float>::infinity();

  BinnedScan result;
  result.angle_first = points.empty() ? kPi : normalize_angle(-points.front().angle_rad);
  result.angle_increment = angle_increment;
  result.angle_last = result.angle_first + angle_increment * static_cast<double>(bin_count - 1);
  result.ranges.assign(bin_count, infinity);
  result.intensities.assign(bin_count, 0.0F);

  for (const auto & point : points) {
    const double ros_angle = normalize_angle(-point.angle_rad);
    const double delta = clockwise_delta(result.angle_first, ros_angle);
    std::size_t index = static_cast<std::size_t>(std::lround(delta / -angle_increment));
    if (index == bin_count) {
      index = 0;
    }
    if (index >= bin_count || !std::isfinite(point.range) || point.range <= 0.0F) {
      continue;
    }
    if (!std::isfinite(result.ranges[index]) || point.range < result.ranges[index]) {
      result.ranges[index] = point.range;
      result.intensities[index] = point.intensity;
    }
  }
  return result;
}

}  // namespace ydlidar_g4_ros2
