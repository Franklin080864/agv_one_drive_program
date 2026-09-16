#pragma once

namespace agv2_pkg {

struct WheelGeometry {
  double front_x;
  double front_y;
  double rear_x;
  double rear_y;
};

// Per-wheel target produced by IK or supplied by WheelCommand path.
// v_* is signed body-frame line speed; the wheel rolls forward along its
// current heading when v_* > 0 and the heading matches that direction.
struct WheelTargets {
  double theta_front;   // rad
  double theta_rear;    // rad
  double v_front;       // m/s
  double v_rear;        // m/s
  bool   singular;      // both wheel speeds below threshold; angles held at last value
};

// Strict dual-steer inverse kinematics (A-route).
// Each wheel velocity vector is v_i = (vx - omega * y_i, vy + omega * x_i);
// theta_i starts as atan2(v_iy, v_ix) and v_i starts as ||v_i||. If theta_i
// falls outside the steering range [-pi/2, pi/2], theta_i is folded by pi and
// v_i is negated so the same ground velocity is achieved without over-steering.
// No equal-speed approximation.
//
// When |v_i| < singularity_speed, we keep last_theta_i to avoid angle thrash
// at standstill. last_theta_* are the previously commanded wheel angles.
WheelTargets solve_ik(double vx,
                      double vy,
                      double omega,
                      const WheelGeometry& g,
                      double singularity_speed,
                      double last_theta_front,
                      double last_theta_rear);

}  // namespace agv2_pkg
