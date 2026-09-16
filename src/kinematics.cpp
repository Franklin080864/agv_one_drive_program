#include "agv2_pkg/kinematics.hpp"

#include <cmath>

namespace agv2_pkg {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

void normalize_steer_target(double& theta, double& speed) {
  if (theta > kHalfPi) {
    theta -= kPi;
    speed = -speed;
  } else if (theta < -kHalfPi) {
    theta += kPi;
    speed = -speed;
  }
}
}  // namespace

WheelTargets solve_ik(double vx,
                      double vy,
                      double omega,
                      const WheelGeometry& g,
                      double singularity_speed,
                      double last_theta_front,
                      double last_theta_rear) {
  const double vfx = vx - omega * g.front_y;
  const double vfy = vy + omega * g.front_x;
  const double vrx = vx - omega * g.rear_y;
  const double vry = vy + omega * g.rear_x;

  const double mag_f = std::hypot(vfx, vfy);
  const double mag_r = std::hypot(vrx, vry);

  WheelTargets out{};
  const bool f_sing = mag_f < singularity_speed;
  const bool r_sing = mag_r < singularity_speed;

  if (f_sing) {
    out.theta_front = last_theta_front;
    out.v_front = 0.0;
  } else {
    out.theta_front = std::atan2(vfy, vfx);
    out.v_front = mag_f;
    normalize_steer_target(out.theta_front, out.v_front);
  }

  if (r_sing) {
    out.theta_rear = last_theta_rear;
    out.v_rear = 0.0;
  } else {
    out.theta_rear = std::atan2(vry, vrx);
    out.v_rear = mag_r;
    normalize_steer_target(out.theta_rear, out.v_rear);
  }

  out.singular = f_sing && r_sing;
  return out;
}

}  // namespace agv2_pkg
