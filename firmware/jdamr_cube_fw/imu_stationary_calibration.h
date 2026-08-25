#ifndef JDAMR_IMU_STATIONARY_CALIBRATION_H_
#define JDAMR_IMU_STATIONARY_CALIBRATION_H_

#include <cstddef>

namespace jdamr_fw
{

struct ImuVector3
{
  float x;
  float y;
  float z;
};

struct ImuStationaryOffsets
{
  ImuVector3 accel_mg;
  ImuVector3 gyro_dps;
};

inline bool compute_stationary_offsets(
  const ImuVector3 & gyro_sum_dps,
  std::size_t sample_count,
  ImuStationaryOffsets * offsets)
{
  if (sample_count == 0 || offsets == nullptr) {
    return false;
  }

  // A single stationary pose cannot separate accelerometer bias from gravity.
  // Keep acceleration unmodified until a proper six-face calibration is added.
  offsets->accel_mg = {0.0f, 0.0f, 0.0f};

  const float divisor = static_cast<float>(sample_count);
  offsets->gyro_dps = {
    gyro_sum_dps.x / divisor,
    gyro_sum_dps.y / divisor,
    gyro_sum_dps.z / divisor,
  };
  return true;
}

}  // namespace jdamr_fw

#endif  // JDAMR_IMU_STATIONARY_CALIBRATION_H_
