#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "jdamr_base_driver/geometry.hpp"

TEST(DrivetrainGeometry, AcceptsMeasuredRobotValues)
{
  EXPECT_NO_THROW(jdamr::validate_drivetrain_geometry(
    0.0329, 0.510, 1.0, 4096));
  EXPECT_NO_THROW(jdamr::validate_drivetrain_geometry(
    0.031628, 0.491708, 1.011889, 4096));
}

TEST(DrivetrainGeometry, RejectsUnsafeOrNonFiniteValues)
{
  EXPECT_THROW(jdamr::validate_drivetrain_geometry(
    0.0, 0.510, 1.0, 4096), std::invalid_argument);
  EXPECT_THROW(jdamr::validate_drivetrain_geometry(
    0.0329, -0.510, 1.0, 4096), std::invalid_argument);
  EXPECT_THROW(jdamr::validate_drivetrain_geometry(
    0.0329, 0.510, -1.0, 4096), std::invalid_argument);
  EXPECT_THROW(jdamr::validate_drivetrain_geometry(
    0.0329, 0.510, std::numeric_limits<double>::quiet_NaN(), 4096),
    std::invalid_argument);
  EXPECT_THROW(jdamr::validate_drivetrain_geometry(
    0.0329, 0.510, 1.0, 0), std::invalid_argument);
}
