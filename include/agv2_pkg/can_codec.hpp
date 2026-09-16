#pragma once

#include <can_msgs/msg/frame.hpp>
#include <cstdint>
#include <vector>

namespace agv2_pkg {

// CAN IDs for the dual-steer chassis (matches agv_pkg).
constexpr uint32_t kSteerRearId    = 0x141;  // rear steering motor
constexpr uint32_t kSteerFrontId   = 0x142;  // front steering motor
constexpr uint32_t kWheelFrontId   = 0x201;  // front hub motor (CANopen RPDO target velocity)
constexpr uint32_t kWheelRearId    = 0x202;  // rear hub motor
constexpr uint32_t kSteerRearFbId  = 0x141;  // feedback shares request id; matched on data[0]==0x94
constexpr uint32_t kSteerFrontFbId = 0x142;
constexpr uint32_t kWheelFrontStatusId = 0x181;  // CANopen TPDO1 from front wheel
constexpr uint32_t kWheelRearStatusId  = 0x182;  // CANopen TPDO1 from rear wheel
constexpr uint32_t kBatterySocId       = 0x191;
constexpr uint32_t kSdoFrontId         = 0x601;  // SDO write to front wheel motor
constexpr uint32_t kSdoRearId          = 0x602;  // SDO write to rear wheel motor
constexpr uint32_t kNmtBroadcastId     = 0x000;  // NMT broadcast

// MyActuator-style steering motor: query speed parameter for cmd 0xa4 (multi-turn position).
constexpr uint16_t kSteerSpeedDps = 300;  // 0.01 deg/s units? agv_pkg uses 300 as a steering speed limit.

// Per-wheel sign correction. -1 if motor's positive rotation drives the body
// rearward; +1 if it matches body-forward.
struct WheelDirections {
  int front;  // expected -1 or +1
  int rear;   // expected -1 or +1
};

struct WheelLimits {
  double max_rpm;
  double wheel_radius;  // m
};

// --- Encoders --------------------------------------------------------------

// Steering motor: 0x88 enable (single open frame).
can_msgs::msg::Frame encode_steer_enable(uint32_t can_id);
// Steering motor: 0x94 read single-turn position.
can_msgs::msg::Frame encode_steer_position_query(uint32_t can_id);
// Steering motor: 0xa4 multi-turn position command. angle_rad is converted to
// 0.01-degree units internally; speed_field is the on-wire speed limit
// (typically kSteerSpeedDps).
can_msgs::msg::Frame encode_steer_position_cmd(uint32_t can_id,
                                               double angle_rad,
                                               uint16_t speed_field = kSteerSpeedDps);

// Normalize single-turn feedback to the local steering range around zero.
// Example: 358.38 deg becomes -1.62 deg.
double normalize_single_turn_angle_deg(double angle_deg);

// Wheel motor (CANopen RPDO): set Target Velocity (0x60FFh, sub 0x03).
// speed_mps is body-frame; sign correction (direction) is applied here.
// Output value is int32 in 0.1 RPM units (CANopen standard).
can_msgs::msg::Frame encode_wheel_velocity(uint32_t can_id,
                                           double speed_mps,
                                           int direction,
                                           const WheelLimits& lim);

// Wheel motor CANopen state-machine bring-up (CIA 402): one of three frames
// emitted in sequence based on the current TPDO1 status bits.
//   step 0: shutdown                 (0x06)
//   step 1: switch on                (0x07)
//   step 2: enable operation         (0x0f)
can_msgs::msg::Frame encode_wheel_controlword(uint32_t can_id, uint8_t step);

// Wheel motor SDO bring-up sequence: one-shot init frames written to 0x601/0x602.
// Emit all once on startup; matches the agv_pkg setup block byte-for-byte.
std::vector<can_msgs::msg::Frame> encode_wheel_sdo_init(uint32_t sdo_id, uint8_t node_id);

// Shutdown: terminator frame (0x001 with all-FF FD payload), used on Ctrl+C.
can_msgs::msg::Frame encode_terminator();

// --- Decoders --------------------------------------------------------------

// Wheel TPDO1 status bits (data[0] bit0/1/2 = ready/switch_on/enable_op).
struct WheelStatus {
  bool present;
  bool ready;
  bool switched_on;
  bool enabled;
  bool fault;
  bool warning;
  uint16_t status_word;
  uint16_t error_code;
  int32_t actual_velocity_raw;
};
WheelStatus decode_wheel_status(
    const can_msgs::msg::Frame& f,
    uint32_t front_status_id = kWheelFrontStatusId,
    uint32_t rear_status_id = kWheelRearStatusId);

// Steering motor 0x94 reply: single-turn position in degrees.
struct SteerPosition {
  bool present;
  double position_deg;  // raw controller value; normalize before control use
};
SteerPosition decode_steer_position(
    const can_msgs::msg::Frame& f,
    uint32_t rear_fb_id = kSteerRearFbId,
    uint32_t front_fb_id = kSteerFrontFbId);

// Battery SOC (0x191 byte 6, 0..100).
struct BatterySoc {
  bool present;
  double soc;  // percent
};
BatterySoc decode_battery_soc(const can_msgs::msg::Frame& f);

}  // namespace agv2_pkg
