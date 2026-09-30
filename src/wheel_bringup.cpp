#include "agv2_pkg/wheel_bringup.hpp"

#include <limits>

namespace agv2_pkg {

void note_wheel_status(WheelBringupState& state, int64_t now_ms) {
  state.status_seen = true;
  state.last_status_ms = now_ms;
}

void reset_wheel_bringup(WheelBringupState& state) {
  state = WheelBringupState{};
}

bool is_wheel_status_stale(const WheelBringupState& state,
                           const WheelBringupConfig& cfg,
                           int64_t now_ms) {
  if (!state.status_seen) return true;
  if (now_ms < state.last_status_ms) return true;
  if (cfg.status_stale_ms <= 0) return false;
  return (now_ms - state.last_status_ms) > cfg.status_stale_ms;
}

namespace {

bool should_emit_periodic(int64_t last_ms, int64_t period_ms, int64_t now_ms) {
  if (last_ms < 0) return true;
  if (now_ms < last_ms) return true;
  if (period_ms <= 0) return true;
  return (now_ms - last_ms) >= period_ms;
}

bool should_warn(WheelBringupState& state,
                 const WheelBringupConfig& cfg,
                 int64_t now_ms) {
  if (cfg.warn_period_ms < 0) return false;
  if (!should_emit_periodic(state.last_warn_ms, cfg.warn_period_ms, now_ms)) {
    return false;
  }
  state.last_warn_ms = now_ms;
  return true;
}

}  // namespace

WheelBringupDecision update_wheel_bringup(WheelBringupState& state,
                                          const WheelStatus& status,
                                          const WheelBringupConfig& cfg,
                                          bool can_sender_ready,
                                          int64_t now_ms) {
  WheelBringupDecision out;
  out.status_stale = !status.present || is_wheel_status_stale(state, cfg, now_ms);

  if (cfg.wait_for_can_sender && !can_sender_ready) {
    out.phase = WheelBringupPhase::WaitingForCanSender;
    out.warn = should_warn(state, cfg, now_ms);
    return out;
  }

  // A reported device fault must never be treated as an enabled drive, even
  // if its enabled bit is still set. Recovery/reset is an explicit operator
  // action; do not continually reconfigure or auto-reset a faulted motor.
  if (status.present && (status.fault || status.error_code != 0)) {
    out.phase = WheelBringupPhase::Fault;
    out.warn = should_warn(state, cfg, now_ms);
    return out;
  }

  // Re-run the existing mapping sequence after lost feedback as well as at
  // startup. A nonpositive retry period permits the first attempt only, never
  // an unbounded every-control-tick SDO burst.
  if (out.status_stale &&
      (state.last_sdo_ms < 0 || cfg.sdo_retry_period_ms > 0) &&
      should_emit_periodic(state.last_sdo_ms, cfg.sdo_retry_period_ms, now_ms)) {
    out.send_sdo_init = true;
    state.last_sdo_ms = now_ms;
    if (state.sdo_retry_count < std::numeric_limits<int>::max()) {
      ++state.sdo_retry_count;
    }
  }

  if (out.status_stale) {
    out.phase = state.status_seen ? WheelBringupPhase::StaleFeedback
                                  : WheelBringupPhase::Shutdown;
    out.send_controlword = true;
    out.controlword_step = 0;
  } else if (!status.ready) {
    out.phase = WheelBringupPhase::Shutdown;
    out.send_controlword = true;
    out.controlword_step = 0;
  } else if (!status.switched_on) {
    out.phase = WheelBringupPhase::SwitchOn;
    out.send_controlword = true;
    out.controlword_step = 1;
  } else if (!status.enabled) {
    out.phase = WheelBringupPhase::EnableOperation;
    out.send_controlword = true;
    out.controlword_step = 2;
  } else {
    out.phase = WheelBringupPhase::Enabled;
    out.allow_velocity = true;
  }

  if ((!out.allow_velocity) &&
      should_warn(state, cfg, now_ms)) {
    out.warn = true;
  }
  return out;
}

const char* wheel_bringup_phase_name(WheelBringupPhase phase) {
  switch (phase) {
    case WheelBringupPhase::WaitingForCanSender:
      return "waiting_for_can_sender";
    case WheelBringupPhase::Shutdown:
      return "shutdown";
    case WheelBringupPhase::SwitchOn:
      return "switch_on";
    case WheelBringupPhase::EnableOperation:
      return "enable_operation";
    case WheelBringupPhase::Enabled:
      return "enabled";
    case WheelBringupPhase::StaleFeedback:
      return "stale_feedback";
    case WheelBringupPhase::Fault:
      return "fault";
  }
  return "unknown";
}

}  // namespace agv2_pkg
