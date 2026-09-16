#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "agv2_pkg/can_codec.hpp"

using namespace agv2_pkg;

namespace {
constexpr double kPi = 3.14159265358979323846;

int32_t read_int32_le(const can_msgs::msg::Frame& f, size_t offset) {
  uint32_t u =
      (static_cast<uint32_t>(f.data[offset + 3]) << 24) |
      (static_cast<uint32_t>(f.data[offset + 2]) << 16) |
      (static_cast<uint32_t>(f.data[offset + 1]) << 8)  |
      static_cast<uint32_t>(f.data[offset + 0]);
  return static_cast<int32_t>(u);
}
}  // namespace

TEST(CanCodec, SteerPositionCmdLayout) {
  // 1.0 rad ~= 57.2958 deg; in 0.01-deg units that's 5730 (rounded).
  auto f = encode_steer_position_cmd(kSteerRearId, 1.0);
  EXPECT_EQ(f.id, kSteerRearId);
  EXPECT_EQ(f.data[0], 0xa4);
  EXPECT_EQ(f.data[1], 0x00);
  // data[2-3] is the speed limit field (default kSteerSpeedDps == 300).
  const uint16_t speed = static_cast<uint16_t>(f.data[2]) |
                         (static_cast<uint16_t>(f.data[3]) << 8);
  EXPECT_EQ(speed, kSteerSpeedDps);
  const int32_t centideg = read_int32_le(f, 4);
  EXPECT_NEAR(static_cast<double>(centideg) / 100.0, 1.0 * 180.0 / kPi, 0.02);
}

TEST(CanCodec, SteerNegativeAngleEncodedAsTwosComplement) {
  auto f = encode_steer_position_cmd(kSteerFrontId, -1.0);
  const int32_t centideg = read_int32_le(f, 4);
  EXPECT_LT(centideg, 0);
  EXPECT_NEAR(static_cast<double>(centideg) / 100.0, -1.0 * 180.0 / kPi, 0.02);
}

TEST(CanCodec, SteerPositionCmdUsesConfiguredSpeedField) {
  auto f = encode_steer_position_cmd(kSteerFrontId, 0.0, 30);
  EXPECT_EQ(f.data[0], 0xa4);
  EXPECT_EQ(f.data[2], 0x1e);
  EXPECT_EQ(f.data[3], 0x00);
}

TEST(CanCodec, SteerDirectionAppliedBeforePositionEncoding) {
  const int front_steer_direction = -1;
  auto f = encode_steer_position_cmd(kSteerFrontId, (kPi / 4.0) * front_steer_direction);
  const int32_t centideg = read_int32_le(f, 4);
  EXPECT_NEAR(static_cast<double>(centideg) / 100.0, -45.0, 0.02);
}

TEST(CanCodec, NormalizeSingleTurnFeedbackAroundZero) {
  EXPECT_NEAR(normalize_single_turn_angle_deg(358.38), -1.62, 1e-9);
  EXPECT_NEAR(normalize_single_turn_angle_deg(1.24), 1.24, 1e-9);
  EXPECT_NEAR(normalize_single_turn_angle_deg(360.0), 0.0, 1e-9);
  EXPECT_NEAR(normalize_single_turn_angle_deg(-358.0), 2.0, 1e-9);
}

TEST(CanCodec, SteerQueryAndEnableLayout) {
  auto q = encode_steer_position_query(kSteerRearId);
  EXPECT_EQ(q.data[0], 0x94);
  EXPECT_EQ(q.dlc, 8);
  for (size_t i = 1; i < 8; ++i) EXPECT_EQ(q.data[i], 0);

  auto e = encode_steer_enable(kSteerFrontId);
  EXPECT_EQ(e.data[0], 0x88);
  for (size_t i = 1; i < 8; ++i) EXPECT_EQ(e.data[i], 0);
}

TEST(CanCodec, WheelVelocityForwardWithReversedFrontMotor) {
  // Front motor mounted reversed (direction = -1): a +0.5 m/s body-forward
  // command must translate to a NEGATIVE on-wire RPM target.
  const WheelLimits lim{200.0, 0.10};  // 200 RPM cap, 10cm radius.
  auto f = encode_wheel_velocity(kWheelFrontId, 0.5, /*direction=*/-1, lim);
  EXPECT_EQ(f.id, kWheelFrontId);
  EXPECT_EQ(f.data[0], 0x0f);
  EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x03);
  EXPECT_EQ(f.data[7], 0x00);
  const int32_t rpm10 = read_int32_le(f, 3);
  EXPECT_LT(rpm10, 0);
  // Expected magnitude: v = omega*r => omega = v/r = 5 rad/s; rpm = 5*60/(2pi) ~= 47.75
  // In 0.1-RPM units that's ~478. Negative because direction=-1.
  const double expected_rpm = (0.5 / 0.10) * 60.0 / (2.0 * kPi);
  EXPECT_NEAR(static_cast<double>(rpm10) / 10.0, -expected_rpm, 0.5);
}

TEST(CanCodec, WheelVelocityRearMotorPositiveDirection) {
  const WheelLimits lim{200.0, 0.10};
  auto f = encode_wheel_velocity(kWheelRearId, 0.5, /*direction=*/+1, lim);
  const int32_t rpm10 = read_int32_le(f, 3);
  EXPECT_GT(rpm10, 0);
}

TEST(CanCodec, WheelVelocityClampedToMaxRpm) {
  const WheelLimits lim{50.0, 0.10};  // very low cap to force clamp
  auto f = encode_wheel_velocity(kWheelFrontId, 5.0, /*direction=*/+1, lim);
  const int32_t rpm10 = read_int32_le(f, 3);
  EXPECT_LE(std::abs(rpm10), 500 + 1);  // 50 RPM * 10 = 500
}

TEST(CanCodec, WheelControlwordSequence) {
  auto a = encode_wheel_controlword(kWheelFrontId, 0);
  auto b = encode_wheel_controlword(kWheelFrontId, 1);
  auto c = encode_wheel_controlword(kWheelFrontId, 2);
  EXPECT_EQ(a.data[0], 0x06);
  EXPECT_EQ(b.data[0], 0x07);
  EXPECT_EQ(c.data[0], 0x0f);
  EXPECT_EQ(c.data[2], 0x03);  // sub-index byte preserved
}

TEST(CanCodec, DecodeWheelStatusBits) {
  can_msgs::msg::Frame f;
  f.id = kWheelFrontStatusId;
  f.dlc = 8;
  f.data = {0x00, 0, 0, 0, 0, 0, 0, 0};
  auto s0 = decode_wheel_status(f);
  EXPECT_TRUE(s0.present);
  EXPECT_FALSE(s0.ready);
  EXPECT_FALSE(s0.switched_on);
  EXPECT_FALSE(s0.enabled);

  f.data = {0x07, 0, 0, 0, 0, 0, 0, 0};
  auto s = decode_wheel_status(f);
  EXPECT_TRUE(s.present);
  EXPECT_TRUE(s.ready);
  EXPECT_TRUE(s.switched_on);
  EXPECT_TRUE(s.enabled);

  f.data[0] = 0x03;
  auto s2 = decode_wheel_status(f);
  EXPECT_TRUE(s2.ready);
  EXPECT_TRUE(s2.switched_on);
  EXPECT_FALSE(s2.enabled);
}

TEST(CanCodec, DecodeWheelStatusWithCustomIds) {
  can_msgs::msg::Frame f;
  f.id = 0x482;
  f.dlc = 8;
  f.data = {0x07, 0, 0, 0, 0, 0, 0, 0};

  auto default_decode = decode_wheel_status(f);
  EXPECT_FALSE(default_decode.present);

  auto custom_decode = decode_wheel_status(f, /*front_status_id=*/0x482,
                                           /*rear_status_id=*/0x481);
  EXPECT_TRUE(custom_decode.present);
  EXPECT_TRUE(custom_decode.enabled);
}

TEST(CanCodec, DecodeSteerPositionRoundTrip) {
  auto cmd = encode_steer_position_cmd(kSteerRearId, 0.5);  // ~28.65 deg
  // Reuse the same byte layout as a fake feedback frame (data[0]=0x94).
  can_msgs::msg::Frame fb;
  fb.id = kSteerRearFbId;
  fb.dlc = 8;
  fb.data.fill(0);
  fb.data[0] = 0x94;
  for (size_t i = 4; i < 8; ++i) fb.data[i] = cmd.data[i];
  auto p = decode_steer_position(fb);
  EXPECT_TRUE(p.present);
  EXPECT_NEAR(p.position_deg, 0.5 * 180.0 / kPi, 0.02);
}

TEST(CanCodec, DecodeSteerPositionWithCustomIds) {
  auto cmd = encode_steer_position_cmd(0x555, -0.25);
  can_msgs::msg::Frame fb;
  fb.id = 0x241;
  fb.dlc = 8;
  fb.data.fill(0);
  fb.data[0] = 0x94;
  for (size_t i = 4; i < 8; ++i) fb.data[i] = cmd.data[i];

  auto default_decode = decode_steer_position(fb);
  EXPECT_FALSE(default_decode.present);

  auto custom_decode = decode_steer_position(fb, /*rear_fb_id=*/0x241,
                                             /*front_fb_id=*/0x242);
  EXPECT_TRUE(custom_decode.present);
  EXPECT_NEAR(custom_decode.position_deg, -0.25 * 180.0 / kPi, 0.02);
}

TEST(CanCodec, DecodeCompleteWheelFeedback) {
  can_msgs::msg::Frame f;
  f.id = kWheelRearStatusId;
  f.dlc = 8;
  f.data = {
      0x8f, 0x12,  // status word 0x128f: enabled, fault, warning
      0x34, 0x12,  // error code 0x1234
      0x18, 0xfc, 0xff, 0xff,  // actual velocity -1000
  };

  const auto status = decode_wheel_status(f);
  ASSERT_TRUE(status.present);
  EXPECT_EQ(status.status_word, 0x128f);
  EXPECT_EQ(status.error_code, 0x1234);
  EXPECT_EQ(status.actual_velocity_raw, -1000);
  EXPECT_TRUE(status.ready);
  EXPECT_TRUE(status.switched_on);
  EXPECT_TRUE(status.enabled);
  EXPECT_TRUE(status.fault);
  EXPECT_TRUE(status.warning);
}

TEST(CanCodec, RejectsShortOrRemoteFeedbackFrames) {
  can_msgs::msg::Frame wheel;
  wheel.id = kWheelFrontStatusId;
  wheel.dlc = 7;
  wheel.data.fill(0xff);
  EXPECT_FALSE(decode_wheel_status(wheel).present);

  can_msgs::msg::Frame steer;
  steer.id = kSteerFrontFbId;
  steer.dlc = 8;
  steer.is_rtr = true;
  steer.data.fill(0);
  steer.data[0] = 0x94;
  EXPECT_FALSE(decode_steer_position(steer).present);
}

TEST(CanCodec, TerminatorPayload) {
  auto f = encode_terminator();
  EXPECT_EQ(f.id, 0x001u);
  for (size_t i = 0; i < 7; ++i) EXPECT_EQ(f.data[i], 0xFF);
  EXPECT_EQ(f.data[7], 0xFD);
}
