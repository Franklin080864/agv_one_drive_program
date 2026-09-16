#include "agv2_pkg/rate_limiter.hpp"

#include <algorithm>

namespace agv2_pkg {

LimiterState step(const LimiterState& cur,
                  const WheelTargets& target,
                  const RateLimits& lim) {
  LimiterState next = cur;

  next.theta_front = target.theta_front;
  next.theta_rear  = target.theta_rear;

  const double dvf =
      std::clamp(target.v_front - cur.v_front, -lim.max_d_speed, lim.max_d_speed);
  const double dvr =
      std::clamp(target.v_rear - cur.v_rear, -lim.max_d_speed, lim.max_d_speed);
  next.v_front = cur.v_front + dvf;
  next.v_rear  = cur.v_rear  + dvr;

  return next;
}

}  // namespace agv2_pkg
