#pragma once

#include "agv2_pkg/kinematics.hpp"

namespace agv2_pkg {

struct TeleopParams {
  double max_speed;
  double rotate_omega;
  double steer_lock;
  double deadband;
};

struct TeleopInput {
  double left_linear_axis;
  double left_angular_axis;
  double fallback_speed_axis;
  double fallback_angular_axis;
  bool sideways_left;
  bool sideways_right;
};

WheelTargets make_teleop_target(const TeleopInput& input,
                                const TeleopParams& params,
                                const WheelGeometry& geom,
                                double singularity_speed,
                                double last_theta_front,
                                double last_theta_rear);

}  // namespace agv2_pkg
