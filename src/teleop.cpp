#include "agv2_pkg/teleop.hpp"

#include <cmath>

namespace agv2_pkg {

namespace {
bool outside_deadband(double v, double deadband) {
  return std::fabs(v) > deadband;
}
}  // namespace

WheelTargets make_teleop_target(const TeleopInput& input,
                                const TeleopParams& params,
                                const WheelGeometry& geom,
                                double singularity_speed,
                                double last_theta_front,
                                double last_theta_rear) {
  WheelTargets t{};

  if (input.sideways_left) {
    t.theta_front = +params.steer_lock;
    t.theta_rear  = +params.steer_lock;
    t.v_front = +params.max_speed;
    t.v_rear  = +params.max_speed;
  } else if (input.sideways_right) {
    t.theta_front = -params.steer_lock;
    t.theta_rear  = -params.steer_lock;
    t.v_front = +params.max_speed;
    t.v_rear  = +params.max_speed;
  } else if (outside_deadband(input.left_linear_axis, params.deadband) ||
             outside_deadband(input.left_angular_axis, params.deadband)) {
    const double vx = input.left_linear_axis * params.max_speed;
    const double omega = input.left_angular_axis * params.rotate_omega;
    t = solve_ik(vx, 0.0, omega, geom, singularity_speed,
                 last_theta_front, last_theta_rear);
  } else if (outside_deadband(input.fallback_angular_axis, params.deadband)) {
    const double omega = (input.fallback_angular_axis > 0.0 ? +1.0 : -1.0) *
                         params.rotate_omega;
    t = solve_ik(0.0, 0.0, omega, geom, singularity_speed,
                 last_theta_front, last_theta_rear);
  } else if (outside_deadband(input.fallback_speed_axis, params.deadband)) {
    const double s = (input.fallback_speed_axis > 0.0) ? +1.0 : -1.0;
    t.theta_front = 0.0;
    t.theta_rear  = 0.0;
    t.v_front = s * params.max_speed;
    t.v_rear  = s * params.max_speed;
  }

  return t;
}

}  // namespace agv2_pkg
