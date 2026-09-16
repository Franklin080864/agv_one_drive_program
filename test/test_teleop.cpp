#include <gtest/gtest.h>

#include <cmath>

#include "agv2_pkg/teleop.hpp"

using agv2_pkg::TeleopParams;
using agv2_pkg::TeleopInput;
using agv2_pkg::WheelGeometry;
using agv2_pkg::make_teleop_target;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSing = 0.001;

WheelGeometry diagonal_geom() {
  return WheelGeometry{0.155, -0.169, -0.155, 0.169};
}

TeleopParams params() {
  return TeleopParams{0.1, 0.87, 1.5708, 0.1};
}

TeleopInput input(double left_linear,
                  double left_angular,
                  double fallback_speed,
                  double fallback_angular,
                  bool sideways_left = false,
                  bool sideways_right = false) {
  return TeleopInput{left_linear, left_angular, fallback_speed, fallback_angular,
                     sideways_left, sideways_right};
}
}  // namespace

TEST(Teleop, RotateAxisUsesDiagonalGeometryInsteadOfDifferentialTwist) {
  const auto t = make_teleop_target(input(0.0, 0.0, 0.0, 1.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);

  const double expected_angle = std::atan2(0.155, 0.169);
  const double expected_speed = 0.87 * std::hypot(0.169, 0.155);
  EXPECT_NEAR(t.theta_front, expected_angle, 1e-9);
  EXPECT_NEAR(t.theta_rear, expected_angle, 1e-9);
  EXPECT_NEAR(t.v_front, expected_speed, 1e-9);
  EXPECT_NEAR(t.v_rear, -expected_speed, 1e-9);
  EXPECT_GT(std::fabs(t.theta_front), 0.7);
}

TEST(Teleop, RotateAxisDirectionOnlyFlipsWheelSpeeds) {
  const auto left = make_teleop_target(input(0.0, 0.0, 0.0, 1.0), params(),
                                       diagonal_geom(), kSing, 0.0, 0.0);
  const auto right = make_teleop_target(input(0.0, 0.0, 0.0, -1.0), params(),
                                        diagonal_geom(), kSing, 0.0, 0.0);

  EXPECT_NEAR(right.theta_front, left.theta_front, 1e-9);
  EXPECT_NEAR(right.theta_rear, left.theta_rear, 1e-9);
  EXPECT_NEAR(right.v_front, -left.v_front, 1e-9);
  EXPECT_NEAR(right.v_rear, -left.v_rear, 1e-9);
}

TEST(Teleop, NegativeSteerDirectionSendsOldHardwareSign) {
  const auto t = make_teleop_target(input(0.0, 0.0, 0.0, 1.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);
  const int steer_direction = -1;

  EXPECT_NEAR(t.theta_front * steer_direction, -42.53 * kPi / 180.0, 0.002);
  EXPECT_NEAR(t.theta_rear * steer_direction, -42.53 * kPi / 180.0, 0.002);
}

TEST(Teleop, RotateDeadbandCommandsStop) {
  const auto t = make_teleop_target(input(0.0, 0.0, 0.0, 0.05), params(),
                                    diagonal_geom(), kSing, 0.2, -0.2);

  EXPECT_NEAR(t.theta_front, 0.0, 1e-12);
  EXPECT_NEAR(t.theta_rear, 0.0, 1e-12);
  EXPECT_NEAR(t.v_front, 0.0, 1e-12);
  EXPECT_NEAR(t.v_rear, 0.0, 1e-12);
}

TEST(Teleop, LeftStickForwardUsesContinuousModel) {
  const auto t = make_teleop_target(input(1.0, 0.0, 0.0, 0.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);

  EXPECT_NEAR(t.theta_front, 0.0, 1e-12);
  EXPECT_NEAR(t.theta_rear, 0.0, 1e-12);
  EXPECT_NEAR(t.v_front, 0.1, 1e-12);
  EXPECT_NEAR(t.v_rear, 0.1, 1e-12);
}

TEST(Teleop, LeftStickYawUsesGeometryBeforeFallbackYaw) {
  const auto t = make_teleop_target(input(0.0, 1.0, 0.0, -1.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);

  const double expected_angle = std::atan2(0.155, 0.169);
  const double expected_speed = 0.87 * std::hypot(0.169, 0.155);
  EXPECT_NEAR(t.theta_front, expected_angle, 1e-9);
  EXPECT_NEAR(t.theta_rear, expected_angle, 1e-9);
  EXPECT_NEAR(t.v_front, expected_speed, 1e-9);
  EXPECT_NEAR(t.v_rear, -expected_speed, 1e-9);
}

TEST(Teleop, LeftStickArcUsesVxAndOmegaTogether) {
  const auto t = make_teleop_target(input(1.0, 1.0, 0.0, 0.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);

  const double vfx = 0.1 + 0.87 * 0.169;
  const double vfy = 0.87 * 0.155;
  const double vrx = 0.1 - 0.87 * 0.169;
  const double vry = -0.87 * 0.155;
  EXPECT_NEAR(t.theta_front, std::atan2(vfy, vfx), 1e-9);
  EXPECT_NEAR(t.v_front, std::hypot(vfx, vfy), 1e-9);
  EXPECT_NEAR(t.theta_rear, std::atan2(vry, vrx) + kPi, 1e-9);
  EXPECT_NEAR(t.v_rear, -std::hypot(vrx, vry), 1e-9);
  EXPECT_GT(std::fabs(t.theta_front), 0.3);
}

TEST(Teleop, SidewaysButtonOverridesLeftStick) {
  const auto t = make_teleop_target(input(1.0, 1.0, 0.0, 0.0, true, false),
                                    params(), diagonal_geom(), kSing, 0.0, 0.0);

  EXPECT_NEAR(t.theta_front, params().steer_lock, 1e-12);
  EXPECT_NEAR(t.theta_rear, params().steer_lock, 1e-12);
  EXPECT_NEAR(t.v_front, params().max_speed, 1e-12);
  EXPECT_NEAR(t.v_rear, params().max_speed, 1e-12);
}

TEST(Teleop, FallbackSpeedWorksWhenLeftStickAndFallbackYawAreIdle) {
  const auto t = make_teleop_target(input(0.05, 0.05, -1.0, 0.0), params(),
                                    diagonal_geom(), kSing, 0.0, 0.0);

  EXPECT_NEAR(t.theta_front, 0.0, 1e-12);
  EXPECT_NEAR(t.theta_rear, 0.0, 1e-12);
  EXPECT_NEAR(t.v_front, -params().max_speed, 1e-12);
  EXPECT_NEAR(t.v_rear, -params().max_speed, 1e-12);
}
