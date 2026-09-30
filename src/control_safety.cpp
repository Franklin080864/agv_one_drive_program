#include "agv2_pkg/control_safety.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agv2_pkg {

bool all_finite(std::initializer_list<double> values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool finite_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool finite_in_range(double value, double minimum, double maximum) {
  return all_finite({value, minimum, maximum}) && minimum <= maximum &&
         value >= minimum && value <= maximum;
}

CommandArbitrator::CommandArbitrator(std::chrono::milliseconds motion_timeout,
                                   std::chrono::milliseconds owner_timeout)
    : motion_timeout_(motion_timeout), owner_timeout_(owner_timeout) {
  if (motion_timeout_.count() <= 0 || owner_timeout_ < motion_timeout_) {
    throw std::invalid_argument(
        "command timeouts require 0 < motion_timeout <= owner_timeout");
  }
}

void CommandArbitrator::note_input(CommandSource source, TimePoint now) {
  if (source == CommandSource::None) return;
  auto& input = source == CommandSource::WheelCmd ? wheel_ : twist_;
  // Reject a timestamp regression rather than allowing old input to claim a
  // new session. The node supplies receive times from steady_clock.
  if (input.seen && now < input.last) return;
  input = {true, now};
  if (owner_ == CommandSource::None) owner_ = source;
}

bool CommandArbitrator::fresh(const InputTime& input, TimePoint now,
                              std::chrono::milliseconds timeout) const {
  return input.seen && now >= input.last && now - input.last <= timeout;
}

const CommandArbitrator::InputTime& CommandArbitrator::input_for(
    CommandSource source) const {
  return source == CommandSource::WheelCmd ? wheel_ : twist_;
}

CommandSelection CommandArbitrator::select(TimePoint now) {
  if (owner_ != CommandSource::None &&
      !fresh(input_for(owner_), now, owner_timeout_)) {
    owner_ = CommandSource::None;
  }
  if (owner_ == CommandSource::None) {
    const bool wheel_fresh = fresh(wheel_, now, motion_timeout_);
    const bool twist_fresh = fresh(twist_, now, motion_timeout_);
    if (wheel_fresh && twist_fresh) {
      owner_ = wheel_.last <= twist_.last ? CommandSource::WheelCmd
                                         : CommandSource::CmdVel;
    } else if (wheel_fresh) {
      owner_ = CommandSource::WheelCmd;
    } else if (twist_fresh) {
      owner_ = CommandSource::CmdVel;
    }
  }
  if (owner_ == CommandSource::None) return {};
  const auto& input = input_for(owner_);
  return {owner_, fresh(input, now, motion_timeout_), input.last};
}

void CommandArbitrator::reset() {
  wheel_ = {};
  twist_ = {};
  owner_ = CommandSource::None;
}

JoyEnableAction JoyEnableGate::update(bool emergency, bool enable,
                                      bool auto_enable, bool axes_neutral) {
  const bool enable_rise = initialized_ && enable && !enable_;
  const bool auto_rise = initialized_ && auto_enable && !auto_enable_;
  initialized_ = true;
  emergency_ = emergency;
  enable_ = enable;
  auto_enable_ = auto_enable;

  if (emergency) return JoyEnableAction::Emergency;
  if (enable_rise) {
    return axes_neutral ? JoyEnableAction::TeleopEnable
                        : JoyEnableAction::TeleopRejected;
  }
  if (auto_rise) return JoyEnableAction::AutoEnable;
  return JoyEnableAction::None;
}

void FaultLatch::update(uint32_t active) {
  active_ = active;
  latched_ |= active;
}

bool FaultLatch::clear() {
  if (active_ != 0) return false;
  latched_ = 0;
  return true;
}

}  // namespace agv2_pkg
