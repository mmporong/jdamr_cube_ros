#ifndef JDAMR_IMU_PACKET_UNITS_H_
#define JDAMR_IMU_PACKET_UNITS_H_

#include <cmath>
#include <cstdint>
#include <limits>

namespace jdamr_fw
{

inline int16_t pack_scaled_i16(float value, float scale)
{
  const float scaled = value * scale;
  if (!std::isfinite(scaled)) {
    return 0;
  }

  constexpr float kMax = static_cast<float>(std::numeric_limits<int16_t>::max());
  constexpr float kMin = static_cast<float>(std::numeric_limits<int16_t>::min());
  if (scaled >= kMax) {
    return std::numeric_limits<int16_t>::max();
  }
  if (scaled <= kMin) {
    return std::numeric_limits<int16_t>::min();
  }
  return static_cast<int16_t>(std::lround(scaled));
}

inline int16_t pack_accel_mg(float accel_mg)
{
  // QMI8658::read_sensor_data() already returns milligravity.
  return pack_scaled_i16(accel_mg, 1.0f);
}

inline int16_t pack_gyro_dps(float gyro_dps)
{
  return pack_scaled_i16(gyro_dps, 100.0f);
}

inline int16_t pack_magnetic_ut(float magnetic_ut)
{
  return pack_scaled_i16(magnetic_ut, 10.0f);
}

}  // namespace jdamr_fw

#endif  // JDAMR_IMU_PACKET_UNITS_H_
