#include <gtest/gtest.h>

#include <cmath>

#include "agv2_pkg/rate_limiter.hpp"

using agv2_pkg::LimiterState;
using agv2_pkg::RateLimits;
using agv2_pkg::WheelTargets;
using agv2_pkg::step;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

TEST(RateLimiter, SteeringAnglesPassThroughImmediately) {
  const LimiterState cur{0.0, 0.0, 0.0, 0.0};
  const WheelTargets target{kPi / 2.0, -kPi / 2.0, 0.4, -0.4, false};
  const RateLimits lim{0.01};

  const auto next = step(cur, target, lim);

  EXPECT_NEAR(next.theta_front, target.theta_front, 1e-12);
  EXPECT_NEAR(next.theta_rear, target.theta_rear, 1e-12);
}

TEST(RateLimiter, PositiveWheelSpeedsSlewByConfiguredStep) {
  const LimiterState cur{0.1, -0.2, 0.0, 0.0};
  const WheelTargets target{0.3, -0.4, 0.4, 0.4, false};
  const RateLimits lim{0.05};

  const auto next = step(cur, target, lim);

  EXPECT_NEAR(next.theta_front, target.theta_front, 1e-12);
  EXPECT_NEAR(next.theta_rear, target.theta_rear, 1e-12);
  EXPECT_NEAR(next.v_front, 0.05, 1e-12);
  EXPECT_NEAR(next.v_rear, 0.05, 1e-12);
}

TEST(RateLimiter, NegativeWheelSpeedsSlewByConfiguredStep) {
  const LimiterState cur{0.0, 0.0, 0.0, 0.0};
  const WheelTargets target{0.0, 0.0, -0.4, -0.4, false};
  const RateLimits lim{0.05};

  const auto next = step(cur, target, lim);

  EXPECT_NEAR(next.v_front, -0.05, 1e-12);
  EXPECT_NEAR(next.v_rear, -0.05, 1e-12);
}

TEST(RateLimiter, WheelSpeedsDoNotOvershootSmallTargets) {
  const LimiterState cur{0.0, 0.0, 0.0, 0.0};
  const WheelTargets target{0.0, 0.0, 0.03, -0.02, false};
  const RateLimits lim{0.05};

  const auto next = step(cur, target, lim);

  EXPECT_NEAR(next.v_front, target.v_front, 1e-12);
  EXPECT_NEAR(next.v_rear, target.v_rear, 1e-12);
}
