#include "agv2_pkg/fsm.hpp"

namespace agv2_pkg {

StateMachine::StateMachine(FsmConfig cfg)
    : cfg_(cfg),
      state_(FsmState::Locked),
      last_input_(),
      settle_since_(),
      input_seen_(false),
      counting_settle_(false) {}

void StateMachine::on_active_input(TimePoint now) {
  last_input_ = now;
  input_seen_ = true;
  state_ = FsmState::Normal;
  counting_settle_ = false;
}

void StateMachine::on_lock_request() {
  state_ = FsmState::Locked;
  counting_settle_ = false;
  input_seen_ = false;
}

FsmState StateMachine::tick(TimePoint now, double v_front_abs, double v_rear_abs) {
  switch (state_) {
    case FsmState::Normal: {
      if (input_seen_ && (now - last_input_) > cfg_.watchdog) {
        state_ = FsmState::RampDown;
        counting_settle_ = false;
      }
      break;
    }
    case FsmState::RampDown: {
      const bool stopped =
          v_front_abs <= cfg_.stop_speed_eps && v_rear_abs <= cfg_.stop_speed_eps;
      if (stopped) {
        if (!counting_settle_) {
          counting_settle_ = true;
          settle_since_ = now;
        } else if ((now - settle_since_) >= cfg_.lock_settle) {
          state_ = FsmState::Locked;
          counting_settle_ = false;
        }
      } else {
        counting_settle_ = false;
      }
      break;
    }
    case FsmState::Locked:
      break;
  }
  return state_;
}

}  // namespace agv2_pkg
