#include "agv2_pkg/can_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace agv2_pkg {

namespace {
constexpr double kPi = 3.14159265358979323846;

can_msgs::msg::Frame make_frame(uint32_t id) {
  can_msgs::msg::Frame f;
  f.id = id;
  f.dlc = 8;
  f.is_rtr = false;
  f.is_extended = false;
  f.is_error = false;
  f.data = {0, 0, 0, 0, 0, 0, 0, 0};
  return f;
}

void write_int32_le(can_msgs::msg::Frame& f, size_t offset, int32_t v) {
  f.data[offset + 0] = static_cast<uint8_t>(v & 0xFF);
  f.data[offset + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  f.data[offset + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  f.data[offset + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

double mps_to_rpm(double mps, double wheel_radius) {
  // v = omega * r => omega = v / r [rad/s]; rpm = omega * 60 / (2*pi).
  return (mps / wheel_radius) * 60.0 / (2.0 * kPi);
}

bool valid_standard_data_frame(const can_msgs::msg::Frame& frame,
                               uint8_t minimum_dlc) {
  return !frame.is_error && !frame.is_extended && !frame.is_rtr &&
         frame.id <= 0x7ff && frame.dlc >= minimum_dlc && frame.dlc <= 8;
}

int32_t rounded_int32(double value) {
  // Saturate before lround/conversion, whose out-of-range results are not
  // suitable for an actuator target. Existing in-range rounding is unchanged.
  value = std::clamp(value,
                     static_cast<double>(std::numeric_limits<int32_t>::min()),
                     static_cast<double>(std::numeric_limits<int32_t>::max()));
  return static_cast<int32_t>(std::lround(value));
}
}  // namespace

can_msgs::msg::Frame encode_steer_enable(uint32_t can_id) {
  auto f = make_frame(can_id);
  f.data[0] = 0x88;
  return f;
}

can_msgs::msg::Frame encode_steer_position_query(uint32_t can_id) {
  auto f = make_frame(can_id);
  f.data[0] = 0x94;
  return f;
}

can_msgs::msg::Frame encode_steer_position_cmd(uint32_t can_id,
                                               double angle_rad,
                                               uint16_t speed_field) {
  // Invalid angles must not become an arbitrary actuator position or force a
  // recenter. The node rejects such input; this is a final codec safeguard.
  if (!std::isfinite(angle_rad)) return encode_steer_position_query(can_id);
  auto f = make_frame(can_id);
  f.data[0] = 0xa4;
  f.data[1] = 0x00;
  f.data[2] = static_cast<uint8_t>(speed_field & 0xFF);
  f.data[3] = static_cast<uint8_t>((speed_field >> 8) & 0xFF);
  const double angle_deg = angle_rad * 180.0 / kPi;
  const int32_t angle_centideg = rounded_int32(angle_deg * 100.0);
  write_int32_le(f, 4, angle_centideg);
  return f;
}

double normalize_single_turn_angle_deg(double angle_deg) {
  return std::remainder(angle_deg, 360.0);
}

can_msgs::msg::Frame encode_wheel_velocity(uint32_t can_id,
                                           double speed_mps,
                                           int direction,
                                           const WheelLimits& lim) {
  double rpm = 0.0;
  if (std::isfinite(speed_mps) &&
      std::isfinite(lim.wheel_radius) && lim.wheel_radius > 0.0 &&
      std::isfinite(lim.max_rpm) && lim.max_rpm >= 0.0 &&
      (direction == -1 || direction == 1)) {
    const double signed_mps = speed_mps * static_cast<double>(direction);
    rpm = mps_to_rpm(signed_mps, lim.wheel_radius);
    rpm = std::clamp(rpm, -lim.max_rpm, lim.max_rpm);
  }
  const int32_t target = rounded_int32(rpm * 10.0);  // 0.1 RPM units

  auto f = make_frame(can_id);
  f.data[0] = 0x0f;
  f.data[1] = 0x00;
  f.data[2] = 0x03;
  write_int32_le(f, 3, target);
  f.data[7] = 0x00;
  return f;
}

can_msgs::msg::Frame encode_wheel_controlword(uint32_t can_id, uint8_t step) {
  auto f = make_frame(can_id);
  // Sub-index 0x03 used for control byte position in agv_pkg's mapped PDO.
  f.data[2] = 0x03;
  switch (step) {
    case 0: f.data[0] = 0x06; break;
    case 1: f.data[0] = 0x07; break;
    default: f.data[0] = 0x0f; break;
  }
  return f;
}

std::vector<can_msgs::msg::Frame> encode_wheel_sdo_init(uint32_t sdo_id, uint8_t node_id) {
  std::vector<can_msgs::msg::Frame> out;
  // Twelve SDO writes that configure RPDO/TPDO mapping plus Modes-of-Operation
  // and acceleration limits. Byte sequences inherited verbatim from agv_pkg.
  const std::vector<std::array<uint8_t, 8>> sdo_payloads = {
      {0x23, 0x00, 0x16, 0x01, 0x10, 0x00, 0x40, 0x60},
      {0x23, 0x00, 0x16, 0x02, 0x08, 0x00, 0x60, 0x60},
      {0x23, 0x00, 0x16, 0x03, 0x20, 0x00, 0xff, 0x60},
      {0x2f, 0x00, 0x16, 0x00, 0x03, 0x00, 0x00, 0x00},
      {0x2f, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00},
      {0x23, 0x00, 0x1a, 0x01, 0x10, 0x00, 0x41, 0x60},
      {0x23, 0x00, 0x1a, 0x02, 0x10, 0x00, 0x3f, 0x60},
      {0x23, 0x00, 0x1a, 0x03, 0x20, 0x00, 0x69, 0x60},
      {0x2f, 0x00, 0x1a, 0x00, 0x03, 0x00, 0x00, 0x00},
      {0x2f, 0x00, 0x18, 0x05, 0x32, 0x00, 0x00, 0x00},
      {0x2f, 0x00, 0x18, 0x03, 0x32, 0x00, 0x00, 0x00},
      {0x2f, 0x00, 0x18, 0x02, 0xfe, 0x00, 0x00, 0x00},
  };
  out.reserve(sdo_payloads.size() + 1);
  for (const auto& p : sdo_payloads) {
    auto f = make_frame(sdo_id);
    for (size_t i = 0; i < 8; ++i) f.data[i] = p[i];
    out.push_back(f);
  }
  // NMT broadcast: start node by id (data[0]=0x01, data[1]=node_id).
  auto nmt = make_frame(kNmtBroadcastId);
  nmt.data[0] = 0x01;
  nmt.data[1] = node_id;
  out.push_back(nmt);
  return out;
}

can_msgs::msg::Frame encode_terminator() {
  auto f = make_frame(0x001);
  f.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  return f;
}

WheelStatus decode_wheel_status(const can_msgs::msg::Frame& f,
                                uint32_t front_status_id,
                                uint32_t rear_status_id) {
  WheelStatus s{};
  if (f.id != front_status_id && f.id != rear_status_id) return s;
  if (!valid_standard_data_frame(f, 8)) return s;
  s.status_word =
      static_cast<uint16_t>(f.data[0]) |
      (static_cast<uint16_t>(f.data[1]) << 8);
  s.error_code =
      static_cast<uint16_t>(f.data[2]) |
      (static_cast<uint16_t>(f.data[3]) << 8);
  const uint32_t velocity_raw =
      static_cast<uint32_t>(f.data[4]) |
      (static_cast<uint32_t>(f.data[5]) << 8) |
      (static_cast<uint32_t>(f.data[6]) << 16) |
      (static_cast<uint32_t>(f.data[7]) << 24);
  s.actual_velocity_raw = static_cast<int32_t>(velocity_raw);
  s.present = true;
  s.ready       = (s.status_word & 0x0001) != 0;
  s.switched_on = (s.status_word & 0x0002) != 0;
  s.enabled     = (s.status_word & 0x0004) != 0;
  s.fault       = (s.status_word & 0x0008) != 0;
  s.warning     = (s.status_word & 0x0080) != 0;
  return s;
}

SteerPosition decode_steer_position(const can_msgs::msg::Frame& f,
                                    uint32_t rear_fb_id,
                                    uint32_t front_fb_id) {
  SteerPosition out{};
  if (f.id != rear_fb_id && f.id != front_fb_id) return out;
  if (!valid_standard_data_frame(f, 8)) return out;
  if (f.data[0] != 0x94) return out;
  const uint32_t raw =
      (static_cast<uint32_t>(f.data[7]) << 24) |
      (static_cast<uint32_t>(f.data[6]) << 16) |
      (static_cast<uint32_t>(f.data[5]) << 8)  |
      static_cast<uint32_t>(f.data[4]);
  // agv_pkg treats the 32-bit value as signed (int) before scaling by 0.01.
  const int32_t signed_raw = static_cast<int32_t>(raw);
  out.position_deg = 0.01 * static_cast<double>(signed_raw);
  out.present = true;
  return out;
}

BatterySoc decode_battery_soc(const can_msgs::msg::Frame& f) {
  BatterySoc out{};
  if (f.id != kBatterySocId) return out;
  if (!valid_standard_data_frame(f, 7)) return out;
  out.soc = static_cast<double>(f.data[6]);
  out.present = true;
  return out;
}

}  // namespace agv2_pkg
