#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "../jdamr_cube_fw/imu_packet_units.h"
#include "../jdamr_cube_fw/imu_stationary_calibration.h"

int main()
{
  using jdamr_fw::pack_accel_mg;
  using jdamr_fw::pack_gyro_dps;
  using jdamr_fw::pack_magnetic_ut;

  assert(pack_accel_mg(1000.0f) == 1000);
  assert(pack_accel_mg(-1000.0f) == -1000);
  assert(pack_accel_mg(980.665f) == 981);
  assert(pack_gyro_dps(0.625f) == 63);
  assert(pack_gyro_dps(-0.625f) == -63);
  assert(pack_magnetic_ut(12.34f) == 123);

  assert(pack_accel_mg(std::numeric_limits<float>::infinity()) == 0);
  assert(pack_accel_mg(std::numeric_limits<float>::quiet_NaN()) == 0);
  assert(pack_gyro_dps(1000.0f) == std::numeric_limits<int16_t>::max());
  assert(pack_gyro_dps(-1000.0f) == std::numeric_limits<int16_t>::min());

  jdamr_fw::ImuStationaryOffsets offsets{};
  assert(jdamr_fw::compute_stationary_offsets(
    {90.0f, -135.0f, 15.0f}, 50, &offsets));
  assert(offsets.accel_mg.x == 0.0f);
  assert(offsets.accel_mg.y == 0.0f);
  assert(offsets.accel_mg.z == 0.0f);
  assert(std::fabs(offsets.gyro_dps.x - 1.8f) < 1.0e-6f);
  assert(std::fabs(offsets.gyro_dps.y + 2.7f) < 1.0e-6f);
  assert(std::fabs(offsets.gyro_dps.z - 0.3f) < 1.0e-6f);

  assert(!jdamr_fw::compute_stationary_offsets(
    {0.0f, 0.0f, 0.0f}, 0, &offsets));
  assert(!jdamr_fw::compute_stationary_offsets(
    {0.0f, 0.0f, 0.0f}, 1, nullptr));
  return 0;
}
