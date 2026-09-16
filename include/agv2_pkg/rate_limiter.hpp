#pragma once

#include "agv2_pkg/kinematics.hpp"

namespace agv2_pkg {

struct RateLimits {
  double max_d_speed;  // m/s per step (per-frame speed slew cap)
};

// Last commanded values. Steering angles are held here for the next CAN emit,
// but they are not rate-limited; wheel speeds are independently clamped.
struct LimiterState {
  double theta_front;   // rad
  double theta_rear;    // rad
  double v_front;       // m/s, signed
  double v_rear;        // m/s, signed
};

// Advance one control step. Steering angles pass through to the target
// immediately; wheel speeds slew toward their targets.
LimiterState step(const LimiterState& cur,
                  const WheelTargets& target,
                  const RateLimits& lim);

}  // namespace agv2_pkg
