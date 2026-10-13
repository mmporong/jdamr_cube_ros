#include <cassert>
#include <cstdint>
#include <limits>

#include "../jdamr_cube_fw/wheel_layout.h"

int main()
{
  using namespace jdamr_fw;

  assert(WHEEL_COUNT == 4);
  assert(WHEEL_IDS[0] == 2 && WHEEL_IDS[1] == 1);
  assert(WHEEL_IDS[2] == 3 && WHEEL_IDS[3] == 4);
  assert(servo_velocity(0, 120, 80, 3400) == -120);
  assert(servo_velocity(1, 120, 80, 3400) == 80);
  assert(servo_velocity(2, 120, 80, 3400) == -120);
  assert(servo_velocity(3, 120, 80, 3400) == 80);
  assert(servo_velocity(0, 4000, 0, 3400) == -3400);
  assert(servo_velocity(3, 0, -4000, 3400) == -3400);

  assert(average_clamped(100, 102) == 101);
  assert(average_clamped(-101, -103) == -102);
  assert(average_clamped(
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * 2,
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * 2) ==
    std::numeric_limits<int32_t>::max());
  assert(average_clamped(
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) * 2,
    static_cast<int64_t>(std::numeric_limits<int32_t>::min()) * 2) ==
    std::numeric_limits<int32_t>::min());
  assert(average_clamped(
    std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::max()) ==
    std::numeric_limits<int32_t>::max());

  WheelUnwrapState left_position{};
  assert(update_unwrap(left_position, 4090, -1, 300));
  assert(left_position.total == 0);
  assert(update_unwrap(left_position, 4, -1, 300));
  assert(left_position.total == -10);

  // A failed read resets initialization. The recovered absolute position is
  // a new baseline, not a large odometry delta.
  reset_unwrap(left_position);
  assert(update_unwrap(left_position, 2000, -1, 300));
  assert(left_position.total == -10);
  assert(update_unwrap(left_position, 2005, -1, 300));
  assert(left_position.total == -15);

  assert(!update_unwrap(left_position, 3000, -1, 300));
  assert(!left_position.initialized);
  assert(left_position.total == -15);
  return 0;
}
