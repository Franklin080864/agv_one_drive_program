#pragma once

#include <chrono>

namespace agv2_pkg {

enum class FsmState { Normal, RampDown, Locked };

struct FsmConfig {
  std::chrono::milliseconds watchdog;     // any input idle longer than this -> RampDown
  std::chrono::milliseconds lock_settle;  // both wheels at ~0 for this long -> Locked
  double stop_speed_eps;                  // m/s threshold for "stopped"
};

// Three-state machine for the agv2 driver.
//   Normal     — at least one input source is fresh.
//   RampDown   — all inputs stale; let rate_limiter slew speeds toward 0.
//   Locked     — speeds settled near 0, hold position.
//
// Inputs are reported via on_active_input(now); call tick(now, |v_f|, |v_r|)
// each control loop with the *most recent commanded* wheel speeds.
class StateMachine {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit StateMachine(FsmConfig cfg);

  void on_active_input(TimePoint now);
  void on_lock_request();

  FsmState tick(TimePoint now, double v_front_abs, double v_rear_abs);

  FsmState state() const { return state_; }

 private:
  FsmConfig cfg_;
  FsmState state_;
  TimePoint last_input_;
  TimePoint settle_since_;
  bool input_seen_;
  bool counting_settle_;
};

}  // namespace agv2_pkg
