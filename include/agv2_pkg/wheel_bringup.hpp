#pragma once

#include <cstdint>

#include "agv2_pkg/can_codec.hpp"

namespace agv2_pkg {

struct WheelBringupConfig {
  bool wait_for_can_sender;
  int64_t sdo_retry_period_ms;
  int64_t status_stale_ms;
  int64_t warn_period_ms;
};

struct WheelBringupState {
  bool status_seen{false};
  int64_t last_status_ms{-1};
  int64_t last_sdo_ms{-1};
  int64_t last_warn_ms{-1};
  int sdo_retry_count{0};
};

enum class WheelBringupPhase {
  WaitingForCanSender,
  Shutdown,
  SwitchOn,
  EnableOperation,
  Enabled,
};

struct WheelBringupDecision {
  WheelBringupPhase phase{WheelBringupPhase::Shutdown};
  bool send_sdo_init{false};
  bool send_controlword{false};
  uint8_t controlword_step{0};
  bool allow_velocity{false};
  bool warn{false};
  bool status_stale{false};
};

void note_wheel_status(WheelBringupState& state, int64_t now_ms);

bool is_wheel_status_stale(const WheelBringupState& state,
                           const WheelBringupConfig& cfg,
                           int64_t now_ms);

WheelBringupDecision update_wheel_bringup(WheelBringupState& state,
                                          const WheelStatus& status,
                                          const WheelBringupConfig& cfg,
                                          bool can_sender_ready,
                                          int64_t now_ms);

const char* wheel_bringup_phase_name(WheelBringupPhase phase);

}  // namespace agv2_pkg
