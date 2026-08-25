// Copyright 2026 The ldlidar_sl_ros2 Authors

#include <gtest/gtest.h>

#include <cmath>

#include "scan_utils.hpp"

namespace {

TEST(ScanTimingTest, PublishStallDoesNotExtendScanTime) {
  constexpr uint64_t first = 10000000000ULL;
  constexpr uint64_t last = first + 160000000ULL;
  constexpr uint64_t publish_after_stall = last + 5000000000ULL;

  const auto timing = ldlidar_ros2::ComputeScanTiming(
      first, last, 161, 6.0, publish_after_stall);

  EXPECT_TRUE(timing.uses_sensor_timestamps);
  EXPECT_EQ(timing.start_stamp_ns, first);
  EXPECT_NEAR(timing.scan_time, 0.16, 1e-9);
  EXPECT_NEAR(timing.time_increment, 0.001, 1e-9);
}

TEST(ScanGeometryTest, ClockwiseAnglesStayChronologicalAcrossWrap) {
  const auto geometry = ldlidar_ros2::ComputeScanGeometry(
      361, true, 1.0 * M_PI / 180.0);

  EXPECT_LT(geometry.angle_increment, 0.0);
  EXPECT_NEAR(geometry.angle_min, 1.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(geometry.angle_max, geometry.angle_min - 2.0 * M_PI, 1e-12);
  EXPECT_EQ(ldlidar_ros2::AngleToIndex(1.0 * M_PI / 180.0, geometry, 361), 0);
  EXPECT_EQ(ldlidar_ros2::AngleToIndex(0.0, geometry, 361), 1);
  EXPECT_EQ(ldlidar_ros2::AngleToIndex(359.0 * M_PI / 180.0, geometry, 361), 2);
  EXPECT_EQ(ldlidar_ros2::AngleToIndex(358.0 * M_PI / 180.0, geometry, 361), 3);
  EXPECT_NEAR(ldlidar_ros2::NormalizeAngle(
      geometry.angle_min + 2.0 * geometry.angle_increment),
      359.0 * M_PI / 180.0, 1e-12);
}

TEST(ScanGeometryTest, ReversedSensorAnglesUseRosCoordinateDirection) {
  EXPECT_NEAR(
      ldlidar_ros2::PublishedAngle(1.0 * M_PI / 180.0, true),
      359.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(ldlidar_ros2::PublishedAngle(0.0, true), 0.0, 1e-12);
  EXPECT_NEAR(
      ldlidar_ros2::PublishedAngle(1.0 * M_PI / 180.0, false),
      1.0 * M_PI / 180.0, 1e-12);
}

TEST(ScanTimingTest, InvalidTimestampsUseBoundedFrequencyFallback) {
  constexpr uint64_t publish = 20000000000ULL;
  const auto reversed = ldlidar_ros2::ComputeScanTiming(
      10000000000ULL, 9000000000ULL, 101, 100.0, publish);
  const auto excessive = ldlidar_ros2::ComputeScanTiming(
      10000000000ULL, 12000000000ULL, 101, 0.0, publish);

  EXPECT_FALSE(reversed.uses_sensor_timestamps);
  EXPECT_DOUBLE_EQ(reversed.scan_time, ldlidar_ros2::kMinScanTime);
  EXPECT_EQ(reversed.start_stamp_ns, publish - 50000000ULL);
  EXPECT_FALSE(excessive.uses_sensor_timestamps);
  EXPECT_NEAR(excessive.scan_time, ldlidar_ros2::kDefaultScanTime, 1e-12);
  EXPECT_LT(excessive.scan_time, ldlidar_ros2::kMaxScanTime);
}

}  // namespace
