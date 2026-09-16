#include <gtest/gtest.h>

#include <cmath>

#include "agv2_pkg/kinematics.hpp"

using agv2_pkg::WheelGeometry;
using agv2_pkg::solve_ik;

namespace {
constexpr double kPi = 3.14159265358979323846;
WheelGeometry make_geom() {
  return WheelGeometry{0.25, 0.0, -0.25, 0.0};
}
WheelGeometry make_diagonal_geom() {
  return WheelGeometry{0.155, -0.169, -0.155, 0.169};
}
constexpr double kSing = 0.001;
}  // namespace

TEST(Kinematics, PureForwardBothWheelsZeroAngleEqualSpeed) {
  const auto g = make_geom();
  auto t = solve_ik(0.3, 0.0, 0.0, g, kSing, 0.0, 0.0);
  EXPECT_NEAR(t.theta_front, 0.0, 1e-9);
  EXPECT_NEAR(t.theta_rear,  0.0, 1e-9);
  EXPECT_NEAR(t.v_front, 0.3, 1e-9);
  EXPECT_NEAR(t.v_rear,  0.3, 1e-9);
  EXPECT_FALSE(t.singular);
}

TEST(Kinematics, PureSidewaysBothWheelsHalfPiEqualSpeed) {
  const auto g = make_geom();
  auto t = solve_ik(0.0, 0.2, 0.0, g, kSing, 0.0, 0.0);
  EXPECT_NEAR(t.theta_front, kPi / 2.0, 1e-9);
  EXPECT_NEAR(t.theta_rear,  kPi / 2.0, 1e-9);
  EXPECT_NEAR(t.v_front, 0.2, 1e-9);
  EXPECT_NEAR(t.v_rear,  0.2, 1e-9);
}

TEST(Kinematics, PureRotationOppositeAnglesEqualSpeed) {
  const auto g = make_geom();
  auto t = solve_ik(0.0, 0.0, 0.5, g, kSing, 0.0, 0.0);
  // Wheels at (+0.25, 0) and (-0.25, 0), pure omega:
  //   v_front = (0, 0.125) -> theta = +pi/2
  //   v_rear  = (0, -0.125) -> theta = -pi/2
  EXPECT_NEAR(t.theta_front, +kPi / 2.0, 1e-9);
  EXPECT_NEAR(t.theta_rear,  -kPi / 2.0, 1e-9);
  EXPECT_NEAR(t.v_front, 0.125, 1e-9);
  EXPECT_NEAR(t.v_rear,  0.125, 1e-9);
}

TEST(Kinematics, DiagonalPureRotationPositiveOmegaFoldsRearAndReversesSpeed) {
  const auto g = make_diagonal_geom();
  auto t = solve_ik(0.0, 0.0, 0.5, g, kSing, 0.0, 0.0);

  const double expected_angle = std::atan2(0.155, 0.169);
  const double expected_speed = std::hypot(0.5 * 0.169, 0.5 * 0.155);
  EXPECT_NEAR(t.theta_front, expected_angle, 1e-9);
  EXPECT_NEAR(t.theta_rear,  expected_angle, 1e-9);
  EXPECT_NEAR(t.v_front, expected_speed, 1e-9);
  EXPECT_NEAR(t.v_rear, -expected_speed, 1e-9);
}

TEST(Kinematics, DiagonalPureRotationNegativeOmegaKeepsTangentAndFlipsSpeeds) {
  const auto g = make_diagonal_geom();
  auto t = solve_ik(0.0, 0.0, -0.5, g, kSing, 0.0, 0.0);

  const double expected_angle = std::atan2(0.155, 0.169);
  const double expected_speed = std::hypot(0.5 * 0.169, 0.5 * 0.155);
  EXPECT_NEAR(t.theta_front, expected_angle, 1e-9);
  EXPECT_NEAR(t.theta_rear,  expected_angle, 1e-9);
  EXPECT_NEAR(t.v_front, -expected_speed, 1e-9);
  EXPECT_NEAR(t.v_rear, expected_speed, 1e-9);
}

TEST(Kinematics, ArcMotionWheelSpeedsDifferAsExpected) {
  const auto g = make_geom();
  // Forward + slight CCW rotation: outer (rear-left side... here both y=0
  // so radii from ICR are |x_i - 0| = 0.25 each); but with vy=0 the IK gives
  // both wheels pointing slightly off-axis with same magnitude due to symmetry.
  auto t = solve_ik(0.2, 0.0, 0.4, g, kSing, 0.0, 0.0);
  // v_front_y = +0.4*0.25 = 0.1 ; v_front_x = 0.2 -> mag = sqrt(0.04+0.01)
  // v_rear_y  = -0.1         ; v_rear_x  = 0.2 -> same mag
  const double expected_mag = std::hypot(0.2, 0.1);
  EXPECT_NEAR(t.v_front, expected_mag, 1e-9);
  EXPECT_NEAR(t.v_rear,  expected_mag, 1e-9);
  // Angles symmetric.
  EXPECT_NEAR(t.theta_front, +std::atan2(0.1, 0.2), 1e-9);
  EXPECT_NEAR(t.theta_rear,  -std::atan2(0.1, 0.2), 1e-9);
}

TEST(Kinematics, SingularityHoldsLastAngle) {
  const auto g = make_geom();
  const double last_f = 0.7, last_r = -0.4;
  auto t = solve_ik(0.0, 0.0, 0.0, g, kSing, last_f, last_r);
  EXPECT_NEAR(t.theta_front, last_f, 1e-9);
  EXPECT_NEAR(t.theta_rear,  last_r, 1e-9);
  EXPECT_NEAR(t.v_front, 0.0, 1e-9);
  EXPECT_NEAR(t.v_rear,  0.0, 1e-9);
  EXPECT_TRUE(t.singular);
}

TEST(Kinematics, ReverseProducesPiAngle) {
  const auto g = make_geom();
  auto t = solve_ik(-0.2, 0.0, 0.0, g, kSing, 0.0, 0.0);
  EXPECT_NEAR(t.theta_front, 0.0, 1e-9);
  EXPECT_NEAR(t.theta_rear,  0.0, 1e-9);
  EXPECT_NEAR(t.v_front, -0.2, 1e-9);
  EXPECT_NEAR(t.v_rear,  -0.2, 1e-9);
}
