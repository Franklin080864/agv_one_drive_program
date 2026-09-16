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

TEST(WheelBringup, RetriesSdoUntilFirstStatus) {
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

  note_wheel_status(state, /*now_ms=*/1100);
  auto d3 = update_wheel_bringup(state, status, c, /*can_sender_ready=*/true,
                                 /*now_ms=*/2100);
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
