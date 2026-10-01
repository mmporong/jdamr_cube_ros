#pragma once

#include <cmath>

namespace jdamr
{

// 이보다 작은 지령은 정지 요청이지 이동이 아니다.
constexpr double kStillLinearMps = 1e-3;
constexpr double kStillAngularRadps = 1e-3;

inline bool commands_motion(bool fresh, double v_mps, double w_radps)
{
  return fresh && (std::abs(v_mps) > kStillLinearMps || std::abs(w_radps) > kStillAngularRadps);
}

// 래치 비상정지 (ISO 13850 방식): 걸리면 의도적인 리셋 전까지 바퀴를 0으로 묶고,
// 리셋 자체가 이동을 시작하면 안 된다. 그래서 0이 아닌 지령이 아직 들어오는 동안
// (Nav2 목표가 살아 있으면 래치가 풀리는 순간 출발한다) 리셋을 거부한다.
class EmergencyStopLatch
{
public:
  // 풀려 있던 래치를 이 호출이 걸었으면 true.
  bool engage()
  {
    const bool changed = !engaged_;
    engaged_ = true;
    return changed;
  }

  // 이동 지령이 없을 때만 푼다. 풀었으면 true.
  bool reset(bool motion_commanded)
  {
    if (motion_commanded) {
      return false;
    }
    engaged_ = false;
    return true;
  }

  bool engaged() const {return engaged_;}

private:
  bool engaged_{false};
};

}  // namespace jdamr
