#ifndef JDAMR_WHEEL_LAYOUT_H
#define JDAMR_WHEEL_LAYOUT_H

#include <stdint.h>
#include <limits.h>

namespace jdamr_fw
{

static const uint8_t WHEEL_COUNT = 4;
static const uint8_t SIDE_LEFT = 0;
static const uint8_t SIDE_RIGHT = 1;

// Front left, front right, rear left, rear right.
static const uint8_t WHEEL_IDS[WHEEL_COUNT] = {2, 1, 3, 4};
static const uint8_t WHEEL_SIDES[WHEEL_COUNT] = {
  SIDE_LEFT, SIDE_RIGHT, SIDE_LEFT, SIDE_RIGHT};
static const int8_t WHEEL_FORWARD_SIGNS[WHEEL_COUNT] = {-1, 1, -1, 1};

struct WheelUnwrapState
{
  int64_t total;
  int16_t last;
  bool initialized;
};

inline int16_t clamp_velocity(int16_t velocity, int16_t limit)
{
  if (velocity > limit) return limit;
  if (velocity < -limit) return -limit;
  return velocity;
}

inline int16_t servo_velocity(uint8_t wheel_index,
                              int16_t left_forward,
                              int16_t right_forward,
                              int16_t limit)
{
  const int16_t side_velocity =
    WHEEL_SIDES[wheel_index] == SIDE_LEFT ? left_forward : right_forward;
  return WHEEL_FORWARD_SIGNS[wheel_index] * clamp_velocity(side_velocity, limit);
}

inline int32_t average_clamped(int64_t first, int64_t second)
{
  const int64_t average = first / 2 + second / 2 +
                          (first % 2 + second % 2) / 2;
  if (average > INT32_MAX) return INT32_MAX;
  if (average < INT32_MIN) return INT32_MIN;
  return static_cast<int32_t>(average);
}

inline void reset_unwrap(WheelUnwrapState &state)
{
  state.initialized = false;
}

inline bool update_unwrap(WheelUnwrapState &state,
                          int16_t raw_position,
                          int8_t forward_sign,
                          int16_t maximum_delta)
{
  const int16_t position = static_cast<int16_t>(raw_position & 0x0FFF);
  if (!state.initialized) {
    state.last = position;
    state.initialized = true;
    return true;
  }

  int16_t delta = static_cast<int16_t>((position - state.last) & 0x0FFF);
  if (delta > 2048) delta -= 4096;
  if (delta > maximum_delta || delta < -maximum_delta) {
    state.initialized = false;
    return false;
  }
  state.last = position;
  state.total += forward_sign * static_cast<int64_t>(delta);
  return true;
}

}  // namespace jdamr_fw

#endif
