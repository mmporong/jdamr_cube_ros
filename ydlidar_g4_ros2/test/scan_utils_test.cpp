// Copyright 2026 Lim
// SPDX-License-Identifier: MIT

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "ydlidar_g4_ros2/scan_utils.hpp"

namespace
{

using ydlidar_g4_ros2::RawPoint;
using ydlidar_g4_ros2::bin_chronological_scan;
using ydlidar_g4_ros2::has_sdk_scan_period;
using ydlidar_g4_ros2::kPi;
using ydlidar_g4_ros2::resolve_timing;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

std::vector<RawPoint> clockwise_scan_from(int start_degrees)
{
  std::vector<RawPoint> points;
  points.reserve(360);
  for (int i = 0; i < 360; ++i) {
    const int angle = (start_degrees + i) % 360;
    points.push_back(
      {radians(static_cast<double>(angle)), static_cast<float>(i + 1),
        static_cast<float>(1000 + i)});
  }
  return points;
}

TEST(ScanUtils, ClockwiseWrapBecomesNegativeRosIncrement)
{
  const auto scan = bin_chronological_scan(clockwise_scan_from(359));

  EXPECT_LT(scan.angle_increment, 0.0);
  EXPECT_NEAR(scan.angle_first, radians(1.0), 1e-9);
  EXPECT_NEAR(scan.angle_increment, radians(-1.0), 1e-9);
  EXPECT_FLOAT_EQ(scan.ranges.at(0), 1.0F);
  EXPECT_FLOAT_EQ(scan.ranges.at(1), 2.0F);
  EXPECT_FLOAT_EQ(scan.ranges.at(2), 3.0F);
}

TEST(ScanUtils, BinsPreserveChronologyAcrossRosAngleWrap)
{
  const auto scan = bin_chronological_scan(clockwise_scan_from(179));

  EXPECT_NEAR(scan.angle_first, radians(-179.0), 1e-9);
  EXPECT_FLOAT_EQ(scan.ranges.at(0), 1.0F);
  EXPECT_FLOAT_EQ(scan.ranges.at(1), 2.0F);
  EXPECT_FLOAT_EQ(scan.ranges.at(2), 3.0F);
  EXPECT_FLOAT_EQ(scan.intensities.at(2), 1002.0F);
}

TEST(ScanUtils, InvalidTimingUsesFrequencyAndBackdatesMissingStamp)
{
  constexpr std::uint64_t now_ns = 2'000'000'000ULL;
  const auto timing = resolve_timing(
    0, std::numeric_limits<double>::quiet_NaN(), 10.0, now_ns, 360, 0);

  EXPECT_DOUBLE_EQ(timing.scan_time, 0.1);
  EXPECT_FLOAT_EQ(timing.time_increment, static_cast<float>(0.1 / 360.0));
  EXPECT_EQ(timing.stamp_ns, 1'900'000'000ULL);
  EXPECT_TRUE(timing.used_scan_time_fallback);
}

TEST(ScanUtils, InitialPartialRotationHasNoUsableSdkPeriod)
{
  EXPECT_FALSE(has_sdk_scan_period(0.0));
  EXPECT_FALSE(has_sdk_scan_period(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(has_sdk_scan_period(0.103));
}

TEST(ScanUtils, ValidSdkTimingAndFirstRayStampArePreserved)
{
  const auto timing = resolve_timing(
    123456789ULL, 0.11, 10.0, 2'000'000'000ULL, 1000, 0);

  EXPECT_DOUBLE_EQ(timing.scan_time, 0.11);
  EXPECT_FLOAT_EQ(timing.time_increment, 0.00011F);
  EXPECT_EQ(timing.stamp_ns, 123456789ULL);
  EXPECT_FALSE(timing.used_scan_time_fallback);
}

TEST(ScanUtils, DoubledSdkPeriodFallsBackToConfiguredFrequency)
{
  const auto timing = resolve_timing(
    1'000'000'000ULL, 0.206, 10.0, 1'210'000'000ULL, 929, 0);

  EXPECT_DOUBLE_EQ(timing.scan_time, 0.1);
  EXPECT_FLOAT_EQ(timing.time_increment, static_cast<float>(0.1 / 929.0));
  EXPECT_TRUE(timing.used_scan_time_fallback);
  EXPECT_LT(timing.scan_end_ns, 1'100'000'000ULL);
}

TEST(ScanUtils, BorderlineSlowSdkPeriodAlsoFallsBack)
{
  const auto timing = resolve_timing(
    1'000'000'000ULL, 0.149, 10.0, 1'160'000'000ULL, 929, 0);

  EXPECT_DOUBLE_EQ(timing.scan_time, 0.1);
  EXPECT_TRUE(timing.used_scan_time_fallback);
}

TEST(ScanUtils, NextStampIsClampedToPreviousPublishedScanEnd)
{
  constexpr std::uint64_t previous_end_ns = 1'100'000'000ULL;
  const auto timing = resolve_timing(
    1'099'800'000ULL, 0.1, 10.0, 1'210'000'000ULL, 1000,
    previous_end_ns);

  EXPECT_EQ(timing.stamp_ns, previous_end_ns + 1'000ULL);
  EXPECT_EQ(timing.stamp_adjustment_ns, 201'000ULL);
  EXPECT_GT(timing.scan_end_ns, timing.stamp_ns);
  EXPECT_FALSE(timing.used_scan_time_fallback);
}

}  // namespace
