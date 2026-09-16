#pragma once

#include <cmath>
#include <stdexcept>

namespace jdamr
{

inline void validate_drivetrain_geometry(
  double wheel_radius, double wheel_separation,
  double wheel_radius_ratio, int counts_per_rev)
{
  if (!std::isfinite(wheel_radius) ||
    wheel_radius < 0.01 || wheel_radius > 0.25)
  {
    throw std::invalid_argument("wheel_radius must be within [0.01, 0.25] m");
  }
  if (!std::isfinite(wheel_separation) ||
    wheel_separation < 0.05 || wheel_separation > 2.0)
  {
    throw std::invalid_argument("wheel_separation must be within [0.05, 2.0] m");
  }
  if (!std::isfinite(wheel_radius_ratio) ||
    wheel_radius_ratio < 0.5 || wheel_radius_ratio > 2.0)
  {
    throw std::invalid_argument("wheel_radius_ratio must be within [0.5, 2.0]");
  }
  if (counts_per_rev < 1 || counts_per_rev > 1000000) {
    throw std::invalid_argument("counts_per_rev must be within [1, 1000000]");
  }
}

}  // namespace jdamr
