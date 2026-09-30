#include <gtest/gtest.h>

#include <chrono>

#include "agv2_pkg/fsm.hpp"

using namespace agv2_pkg;
using namespace std::chrono_literals;

namespace {
StateMachine::TimePoint at(int milliseconds) {
  return StateMachine::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

FsmConfig cfg() { return {200ms, 100ms, 0.01}; }
}  // namespace

TEST(StateMachine, StartsLockedUntilValidInputIsDelivered) {
  StateMachine fsm(cfg());
  EXPECT_EQ(fsm.tick(at(0), 0.0, 0.0), FsmState::Locked);
  EXPECT_EQ(fsm.tick(at(10000), 0.0, 0.0), FsmState::Locked);
  fsm.on_active_input(at(10000));
  EXPECT_EQ(fsm.tick(at(10000), 0.0, 0.0), FsmState::Normal);
}

TEST(StateMachine, WatchdogUsesTheCommandTimestampWithoutExtendingItAtEachTick) {
  StateMachine fsm(cfg());
  fsm.on_active_input(at(0));
  EXPECT_EQ(fsm.tick(at(200), 0.4, 0.4), FsmState::Normal);
  EXPECT_EQ(fsm.tick(at(201), 0.4, 0.4), FsmState::RampDown);
}

TEST(StateMachine, LocksOnlyAfterBothWheelsRemainStoppedForTheSettleDuration) {
  StateMachine fsm(cfg());
  fsm.on_active_input(at(0));
  fsm.tick(at(201), 0.3, 0.3);
  EXPECT_EQ(fsm.tick(at(250), 0.0, 0.02), FsmState::RampDown);
  EXPECT_EQ(fsm.tick(at(300), 0.0, 0.0), FsmState::RampDown);
  EXPECT_EQ(fsm.tick(at(399), 0.0, 0.0), FsmState::RampDown);
  EXPECT_EQ(fsm.tick(at(400), 0.0, 0.0), FsmState::Locked);
}

TEST(StateMachine, MotionDuringSettlingRestartsTheSettleTimer) {
  StateMachine fsm(cfg());
  fsm.on_active_input(at(0));
  fsm.tick(at(201), 0.3, 0.3);
  fsm.tick(at(250), 0.0, 0.0);
  fsm.tick(at(300), 0.02, 0.0);
  fsm.tick(at(350), 0.0, 0.0);
  EXPECT_EQ(fsm.tick(at(449), 0.0, 0.0), FsmState::RampDown);
  EXPECT_EQ(fsm.tick(at(450), 0.0, 0.0), FsmState::Locked);
}

TEST(StateMachine, LockRequestIsImmediateAndDiscardsPreviousInput) {
  StateMachine fsm(cfg());
  fsm.on_active_input(at(0));
  fsm.on_lock_request();
  EXPECT_EQ(fsm.state(), FsmState::Locked);
  EXPECT_EQ(fsm.tick(at(1), 0.4, 0.4), FsmState::Locked);
  EXPECT_EQ(fsm.tick(at(1000), 0.0, 0.0), FsmState::Locked);
}

TEST(StateMachine, NewValidInputCanResumeFromWatchdogRampDown) {
  StateMachine fsm(cfg());
  fsm.on_active_input(at(0));
  fsm.tick(at(201), 0.4, 0.4);
  fsm.on_active_input(at(250));
  EXPECT_EQ(fsm.state(), FsmState::Normal);
  EXPECT_EQ(fsm.tick(at(450), 0.4, 0.4), FsmState::Normal);
  EXPECT_EQ(fsm.tick(at(451), 0.4, 0.4), FsmState::RampDown);
}
