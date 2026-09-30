#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>

#include "agv2_pkg/control_safety.hpp"
#include "agv2_pkg/fsm.hpp"

using namespace agv2_pkg;
using namespace std::chrono_literals;

namespace {
CommandArbitrator::TimePoint at(int milliseconds) {
  return CommandArbitrator::TimePoint{} + std::chrono::milliseconds(milliseconds);
}
}  // namespace

TEST(ControlValidation, RejectsNonfiniteValuesBeforeTheyReachClamping) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(all_finite({0.0, -1.0, 1.0}));
  EXPECT_FALSE(all_finite({0.0, nan}));
  EXPECT_FALSE(all_finite({inf}));
  EXPECT_FALSE(all_finite({-inf}));
  EXPECT_FALSE(finite_positive(inf));
  EXPECT_FALSE(finite_nonnegative(nan));
  EXPECT_FALSE(finite_in_range(nan, -1.0, 1.0));
}

TEST(ControlValidation, DistinguishesPositiveNonnegativeAndBoundedParameters) {
  EXPECT_TRUE(finite_positive(0.01));
  EXPECT_FALSE(finite_positive(0.0));
  EXPECT_FALSE(finite_positive(-0.01));
  EXPECT_TRUE(finite_nonnegative(0.0));
  EXPECT_FALSE(finite_nonnegative(-0.01));
  EXPECT_TRUE(finite_in_range(-1.0, -1.0, 1.0));
  EXPECT_TRUE(finite_in_range(1.0, -1.0, 1.0));
  EXPECT_FALSE(finite_in_range(1.01, -1.0, 1.0));
  EXPECT_FALSE(finite_in_range(0.0, 1.0, -1.0));
  EXPECT_FALSE(finite_in_range(0.0, -1.0,
                             std::numeric_limits<double>::infinity()));
}

TEST(CommandArbitrator, RejectsInvalidTimeoutConfiguration) {
  EXPECT_THROW(CommandArbitrator(0ms, 1000ms), std::invalid_argument);
  EXPECT_THROW(CommandArbitrator(-1ms, 1000ms), std::invalid_argument);
  EXPECT_THROW(CommandArbitrator(200ms, 199ms), std::invalid_argument);
  EXPECT_NO_THROW(CommandArbitrator(200ms, 200ms));
}

TEST(CommandArbitrator, StartsWithoutSourceAndAcceptsClockEpochAsAValidTime) {
  CommandArbitrator arb(200ms, 1000ms);
  EXPECT_EQ(arb.select(at(0)).source, CommandSource::None);
  arb.note_input(CommandSource::None, at(0));
  EXPECT_EQ(arb.active_source(), CommandSource::None);
  arb.note_input(CommandSource::WheelCmd, at(0));
  const auto selection = arb.select(at(0));
  EXPECT_EQ(selection.source, CommandSource::WheelCmd);
  EXPECT_TRUE(selection.fresh_for_motion);
  EXPECT_EQ(selection.last_input, at(0));
}

TEST(CommandArbitrator, FirstArrivalOwnsEvenAfterItsTimestampBecomesMoreRecent) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::CmdVel, at(0));
  arb.note_input(CommandSource::WheelCmd, at(10));
  arb.note_input(CommandSource::CmdVel, at(20));
  EXPECT_EQ(arb.select(at(20)).source, CommandSource::CmdVel);
}

TEST(CommandArbitrator, ArrivalOrderBreaksEqualTimestampTies) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::CmdVel, at(0));
  arb.note_input(CommandSource::WheelCmd, at(0));
  EXPECT_EQ(arb.select(at(0)).source, CommandSource::CmdVel);
}

TEST(CommandArbitrator, OtherSourceDoesNotRefreshOwnersMotionDeadline) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::WheelCmd, at(0));
  EXPECT_TRUE(arb.select(at(200)).fresh_for_motion);
  arb.note_input(CommandSource::CmdVel, at(201));
  const auto selection = arb.select(at(201));
  EXPECT_EQ(selection.source, CommandSource::WheelCmd);
  EXPECT_FALSE(selection.fresh_for_motion);
  EXPECT_EQ(selection.last_input, at(0));
}

TEST(CommandArbitrator, OwnerIsRetainedUntilTheSeparateReleaseDeadline) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::WheelCmd, at(0));
  arb.note_input(CommandSource::CmdVel, at(1000));
  const auto waiting = arb.select(at(1000));
  EXPECT_EQ(waiting.source, CommandSource::WheelCmd);
  EXPECT_FALSE(waiting.fresh_for_motion);
  const auto takeover = arb.select(at(1001));
  EXPECT_EQ(takeover.source, CommandSource::CmdVel);
  EXPECT_TRUE(takeover.fresh_for_motion);
  EXPECT_EQ(takeover.last_input, at(1000));
}

TEST(CommandArbitrator, DoesNotSelectAMotionStaleCandidateAtTakeover) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::WheelCmd, at(0));
  arb.note_input(CommandSource::CmdVel, at(700));
  const auto selection = arb.select(at(1001));
  EXPECT_EQ(selection.source, CommandSource::None);
  EXPECT_FALSE(selection.fresh_for_motion);
}

TEST(CommandArbitrator, SameOwnerCanResumeBeforeItsReleaseDeadline) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::CmdVel, at(0));
  EXPECT_FALSE(arb.select(at(201)).fresh_for_motion);
  arb.note_input(CommandSource::CmdVel, at(300));
  EXPECT_TRUE(arb.select(at(300)).fresh_for_motion);
  EXPECT_EQ(arb.select(at(300)).source, CommandSource::CmdVel);
}

TEST(CommandArbitrator, ResetDiscardsCommandsFromThePreviousEnableSession) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::CmdVel, at(0));
  arb.note_input(CommandSource::WheelCmd, at(1));
  arb.reset();
  EXPECT_EQ(arb.active_source(), CommandSource::None);
  EXPECT_EQ(arb.select(at(2)).source, CommandSource::None);
  arb.note_input(CommandSource::WheelCmd, at(3));
  EXPECT_EQ(arb.select(at(3)).source, CommandSource::WheelCmd);
}

TEST(CommandArbitrator, TimestampRegressionCannotRollBackAnInput) {
  CommandArbitrator arb(200ms, 1000ms);
  arb.note_input(CommandSource::CmdVel, at(100));
  arb.note_input(CommandSource::CmdVel, at(10));
  EXPECT_EQ(arb.select(at(100)).last_input, at(100));
  // Unexpected future timestamps fail closed rather than becoming fresh.
  EXPECT_FALSE(arb.select(at(50)).fresh_for_motion);
}

TEST(CommandArbitrator, SelectedTimestampPreservesTheFsmWatchdogUnderOtherTraffic) {
  CommandArbitrator arb(200ms, 1000ms);
  StateMachine fsm({200ms, 100ms, 0.01});
  arb.note_input(CommandSource::WheelCmd, at(0));
  auto selection = arb.select(at(0));
  ASSERT_TRUE(selection.fresh_for_motion);
  fsm.on_active_input(selection.last_input);
  for (int t = 50; t <= 300; t += 50) {
    arb.note_input(CommandSource::CmdVel, at(t));
    selection = arb.select(at(t));
    if (selection.fresh_for_motion) fsm.on_active_input(selection.last_input);
    fsm.tick(at(t), 0.3, 0.3);
  }
  EXPECT_EQ(selection.source, CommandSource::WheelCmd);
  EXPECT_EQ(fsm.state(), FsmState::RampDown);
}

TEST(JoyEnableGate, HeldStartupButtonsDoNotEnableActuation) {
  JoyEnableGate gate;
  EXPECT_EQ(gate.update(false, true, true, true), JoyEnableAction::None);
  EXPECT_EQ(gate.update(false, true, true, true), JoyEnableAction::None);
  EXPECT_EQ(gate.update(false, false, false, true), JoyEnableAction::None);
  EXPECT_EQ(gate.update(false, false, true, true), JoyEnableAction::AutoEnable);
}

TEST(JoyEnableGate, EmergencyIsEffectiveOnTheFirstFrameAndEveryHeldFrame) {
  JoyEnableGate gate;
  EXPECT_EQ(gate.update(true, false, false, true), JoyEnableAction::Emergency);
  EXPECT_TRUE(gate.emergency_active());
  EXPECT_EQ(gate.update(true, true, true, true), JoyEnableAction::Emergency);
  EXPECT_EQ(gate.update(true, true, true, true), JoyEnableAction::Emergency);
}

TEST(JoyEnableGate, EnableHeldAcrossEmergencyReleaseDoesNotRearm) {
  JoyEnableGate gate;
  gate.update(false, false, false, true);
  EXPECT_EQ(gate.update(true, true, true, true), JoyEnableAction::Emergency);
  EXPECT_EQ(gate.update(false, true, true, true), JoyEnableAction::None);
  EXPECT_FALSE(gate.emergency_active());
  gate.update(false, false, false, true);
  EXPECT_EQ(gate.update(false, true, false, true), JoyEnableAction::TeleopEnable);
}

TEST(JoyEnableGate, TeleopRequiresNeutralAxesAndANewPressAfterRejection) {
  JoyEnableGate gate;
  gate.update(false, false, false, true);
  EXPECT_EQ(gate.update(false, true, false, false), JoyEnableAction::TeleopRejected);
  EXPECT_EQ(gate.update(false, true, false, true), JoyEnableAction::None);
  gate.update(false, false, false, true);
  EXPECT_EQ(gate.update(false, true, false, true), JoyEnableAction::TeleopEnable);
}

TEST(JoyEnableGate, SimultaneousEnableEdgesKeepTeleopPriority) {
  JoyEnableGate gate;
  gate.update(false, false, false, true);
  EXPECT_EQ(gate.update(false, true, true, true), JoyEnableAction::TeleopEnable);
}

TEST(FaultLatch, CannotClearAnyActiveFault) {
  FaultLatch faults;
  EXPECT_TRUE(faults.clear());
  faults.update(0x1);
  EXPECT_EQ(faults.active(), 0x1u);
  EXPECT_EQ(faults.latched(), 0x1u);
  EXPECT_FALSE(faults.clear());
  EXPECT_EQ(faults.latched(), 0x1u);
}

TEST(FaultLatch, ResolutionRequiresAcknowledgementAndRetainsAllObservedFaults) {
  FaultLatch faults;
  faults.update(0x1);
  faults.update(0x4);
  EXPECT_EQ(faults.active(), 0x4u);
  EXPECT_EQ(faults.latched(), 0x5u);
  faults.update(0);
  EXPECT_EQ(faults.active(), 0u);
  EXPECT_EQ(faults.latched(), 0x5u);
  EXPECT_TRUE(faults.clear());
  EXPECT_EQ(faults.latched(), 0u);
}
