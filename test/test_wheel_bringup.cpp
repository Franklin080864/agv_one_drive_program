#include <gtest/gtest.h>

#include "agv2_pkg/wheel_bringup.hpp"

using namespace agv2_pkg;

namespace {

WheelBringupConfig cfg() {
  return WheelBringupConfig{/*wait_for_can_sender=*/true,
                            /*sdo_retry_period_ms=*/1000,
                            /*status_stale_ms=*/500,
                            /*warn_period_ms=*/2000};
}

}  // namespace

TEST(WheelBringup, WaitsForCanSenderBeforeSdo) {
  WheelBringupState state;
  WheelStatus status{};

  auto d0 = update_wheel_bringup(state, status, cfg(), /*can_sender_ready=*/false,
                                 /*now_ms=*/0);
  EXPECT_EQ(d0.phase, WheelBringupPhase::WaitingForCanSender);
  EXPECT_FALSE(d0.send_sdo_init);
  EXPECT_FALSE(d0.send_controlword);
  EXPECT_FALSE(d0.allow_velocity);
  EXPECT_EQ(state.sdo_retry_count, 0);

  auto d1 = update_wheel_bringup(state, status, cfg(), /*can_sender_ready=*/true,
                                 /*now_ms=*/10);
  EXPECT_TRUE(d1.send_sdo_init);
  EXPECT_TRUE(d1.send_controlword);
  EXPECT_EQ(d1.controlword_step, 0);
  EXPECT_EQ(state.sdo_retry_count, 1);
}

TEST(WheelBringup, RetriesSdoUntilFreshStatus) {
  WheelBringupState state;
  WheelStatus status{};
  auto c = cfg();

  auto d0 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/0);
  EXPECT_TRUE(d0.send_sdo_init);
  EXPECT_EQ(state.sdo_retry_count, 1);

  auto d1 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/500);
  EXPECT_FALSE(d1.send_sdo_init);
  EXPECT_EQ(state.sdo_retry_count, 1);

  auto d2 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/1000);
  EXPECT_TRUE(d2.send_sdo_init);
  EXPECT_EQ(state.sdo_retry_count, 2);

  status.present = true;
  note_wheel_status(state, /*now_ms=*/1100);
  auto d3 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/1300);
  EXPECT_FALSE(d3.send_sdo_init);
}

TEST(WheelBringup, ProgressesControlwordFromStatusBits) {
  WheelBringupState state;
  auto c = cfg();

  WheelStatus status{};
  status.present = true;
  note_wheel_status(state, /*now_ms=*/0);
  auto d0 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/0);
  EXPECT_FALSE(d0.send_sdo_init);
  EXPECT_TRUE(d0.send_controlword);
  EXPECT_EQ(d0.controlword_step, 0);
  EXPECT_FALSE(d0.allow_velocity);

  status.ready = true;
  note_wheel_status(state, /*now_ms=*/10);
  auto d1 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/10);
  EXPECT_TRUE(d1.send_controlword);
  EXPECT_EQ(d1.controlword_step, 1);

  status.switched_on = true;
  note_wheel_status(state, /*now_ms=*/20);
  auto d2 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/20);
  EXPECT_TRUE(d2.send_controlword);
  EXPECT_EQ(d2.controlword_step, 2);

  status.enabled = true;
  note_wheel_status(state, /*now_ms=*/30);
  auto d3 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/30);
  EXPECT_FALSE(d3.send_controlword);
  EXPECT_TRUE(d3.allow_velocity);
}

TEST(WheelBringup, StaleEnabledFeedbackBlocksVelocityAndRetriesAtConfiguredRate) {
  WheelBringupState state;
  WheelStatus status{};
  const auto c = cfg();
  EXPECT_TRUE(update_wheel_bringup(state, status, c, true, 0).send_sdo_init);

  status.present = true;
  status.ready = true;
  status.switched_on = true;
  status.enabled = true;
  note_wheel_status(state, 100);
  EXPECT_TRUE(update_wheel_bringup(state, status, c, true, 600).allow_velocity);

  auto stale = update_wheel_bringup(state, status, c, true, 601);
  EXPECT_TRUE(stale.status_stale);
  EXPECT_FALSE(stale.allow_velocity);
  EXPECT_EQ(stale.phase, WheelBringupPhase::StaleFeedback);
  EXPECT_TRUE(stale.send_controlword);
  EXPECT_EQ(stale.controlword_step, 0);
  EXPECT_FALSE(stale.send_sdo_init);
  EXPECT_TRUE(update_wheel_bringup(state, status, c, true, 1000).send_sdo_init);
  EXPECT_FALSE(update_wheel_bringup(state, status, c, true, 1050).send_sdo_init);
  EXPECT_FALSE(update_wheel_bringup(state, status, c, true, 1999).send_sdo_init);
  EXPECT_TRUE(update_wheel_bringup(state, status, c, true, 2000).send_sdo_init);
  EXPECT_FALSE(update_wheel_bringup(state, status, c, true, 60000).allow_velocity);

  // Fresh, healthy feedback allows bring-up again; an external safety latch
  // must still require explicit reauthorization before sending motion.
  note_wheel_status(state, 60010);
  const auto recovered = update_wheel_bringup(state, status, c, true, 60010);
  EXPECT_FALSE(recovered.status_stale);
  EXPECT_TRUE(recovered.allow_velocity);
  EXPECT_FALSE(recovered.send_sdo_init);
}

TEST(WheelBringup, FaultOrErrorCodeWinsOverEnabledAndDoesNotAutoReset) {
  for (int kind = 0; kind < 2; ++kind) {
    WheelBringupState state;
    WheelStatus status{};
    status.present = true;
    status.ready = true;
    status.switched_on = true;
    status.enabled = true;
    status.fault = kind == 0;
    status.error_code = kind == 1 ? 0x1234 : 0;
    note_wheel_status(state, 0);

    for (const int64_t now : {int64_t{0}, int64_t{100}, int64_t{60000}}) {
      const auto d = update_wheel_bringup(state, status, cfg(), true, now);
      EXPECT_EQ(d.phase, WheelBringupPhase::Fault);
      EXPECT_FALSE(d.allow_velocity);
      EXPECT_FALSE(d.send_sdo_init);
      EXPECT_FALSE(d.send_controlword);
    }
    EXPECT_EQ(state.sdo_retry_count, 0);
  }
}

TEST(WheelBringup, WarningBitAloneDoesNotChangeValidatedMotionBehavior) {
  WheelBringupState state;
  WheelStatus status{};
  status.present = true;
  status.ready = true;
  status.switched_on = true;
  status.enabled = true;
  status.warning = true;
  note_wheel_status(state, 10);
  EXPECT_TRUE(update_wheel_bringup(state, status, cfg(), true, 10).allow_velocity);
}

TEST(WheelBringup, MissingStatusCannotAuthorizeFromEnabledBits) {
  WheelBringupState state;
  WheelStatus status{};
  status.ready = true;
  status.switched_on = true;
  status.enabled = true;
  note_wheel_status(state, 10);
  const auto d = update_wheel_bringup(state, status, cfg(), true, 10);
  EXPECT_TRUE(d.status_stale);
  EXPECT_FALSE(d.allow_velocity);
  EXPECT_EQ(d.controlword_step, 0);
}

TEST(WheelBringup, InvalidStatusFrameDoesNotRefreshFeedbackAge) {
  WheelBringupState state;
  can_msgs::msg::Frame frame;
  frame.id = kWheelFrontStatusId;
  frame.dlc = 8;
  frame.data = {0x07, 0, 0, 0, 0, 0, 0, 0};
  auto status = decode_wheel_status(frame);
  ASSERT_TRUE(status.present);
  note_wheel_status(state, 0);

  frame.is_error = true;
  const auto invalid_status = decode_wheel_status(frame);
  if (invalid_status.present) {
    status = invalid_status;
    note_wheel_status(state, 1000);
  }
  EXPECT_EQ(state.last_status_ms, 0);
  EXPECT_FALSE(update_wheel_bringup(state, status, cfg(), true, 1000).allow_velocity);
}

TEST(WheelBringup, DisabledRetryPeriodDoesNotFloodCanBus) {
  for (const int64_t period : {int64_t{0}, int64_t{-1}}) {
    WheelBringupState state;
    WheelStatus status{};
    auto c = cfg();
    c.sdo_retry_period_ms = period;
    EXPECT_TRUE(update_wheel_bringup(state, status, c, true, 0).send_sdo_init);
    EXPECT_FALSE(update_wheel_bringup(state, status, c, true, 50).send_sdo_init);
    EXPECT_FALSE(update_wheel_bringup(state, status, c, true, 60000).send_sdo_init);
    EXPECT_EQ(state.sdo_retry_count, 1);
  }
}

TEST(WheelBringup, ClockRegressionInvalidatesStatus) {
  WheelBringupState state;
  note_wheel_status(state, 1000);
  EXPECT_TRUE(is_wheel_status_stale(state, cfg(), 999));
}

TEST(WheelBringup, ExplicitResetRequiresNewFeedback) {
  WheelBringupState state;
  WheelStatus status{};
  status.present = true;
  status.ready = true;
  status.switched_on = true;
  status.enabled = true;
  note_wheel_status(state, 1000);
  EXPECT_TRUE(update_wheel_bringup(state, status, cfg(), true, 1000).allow_velocity);
  reset_wheel_bringup(state);

  const auto d = update_wheel_bringup(state, status, cfg(), true, 1050);
  EXPECT_FALSE(state.status_seen);
  EXPECT_TRUE(d.status_stale);
  EXPECT_FALSE(d.allow_velocity);
  EXPECT_TRUE(d.send_sdo_init);
  EXPECT_EQ(d.controlword_step, 0);
}
