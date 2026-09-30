#pragma once

#include <chrono>
#include <cstdint>
#include <initializer_list>

namespace agv2_pkg {

bool all_finite(std::initializer_list<double> values);
bool finite_positive(double value);
bool finite_nonnegative(double value);
bool finite_in_range(double value, double minimum, double maximum);

enum class CommandSource { None, WheelCmd, CmdVel };

struct CommandSelection {
  CommandSource source{CommandSource::None};
  bool fresh_for_motion{false};
  std::chrono::steady_clock::time_point last_input{};
};

// An input session has two separate deadlines. Losing motion freshness stops
// motion; it does not immediately surrender ownership to a competing publisher.
// Call note_input only for a validated command in the current Auto session.
class CommandArbitrator {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  CommandArbitrator(std::chrono::milliseconds motion_timeout,
                    std::chrono::milliseconds owner_timeout);

  void note_input(CommandSource source, TimePoint now);
  CommandSelection select(TimePoint now);
  void reset();
  CommandSource active_source() const { return owner_; }

 private:
  struct InputTime {
    bool seen{false};
    TimePoint last{};
  };

  bool fresh(const InputTime& input, TimePoint now,
             std::chrono::milliseconds timeout) const;
  const InputTime& input_for(CommandSource source) const;

  std::chrono::milliseconds motion_timeout_;
  std::chrono::milliseconds owner_timeout_;
  InputTime wheel_;
  InputTime twist_;
  CommandSource owner_{CommandSource::None};
};

enum class JoyEnableAction {
  None,
  Emergency,
  TeleopEnable,
  AutoEnable,
  TeleopRejected
};

// Establishes a startup baseline and detects enable edges. An emergency level
// always wins, including on the first frame. Enable presses during emergency
// are consumed, requiring a new press after emergency is released.
class JoyEnableGate {
 public:
  JoyEnableAction update(bool emergency, bool enable, bool auto_enable,
                         bool axes_neutral);
  bool emergency_active() const { return emergency_; }

 private:
  bool initialized_{false};
  bool emergency_{false};
  bool enable_{false};
  bool auto_enable_{false};
};

// Only fault bits requiring explicit acknowledgement belong in this latch.
// Resolving a current fault does not by itself restore motion permission.
class FaultLatch {
 public:
  void update(uint32_t active);
  uint32_t active() const { return active_; }
  uint32_t latched() const { return latched_; }
  bool clear();

 private:
  uint32_t active_{0};
  uint32_t latched_{0};
};

}  // namespace agv2_pkg
