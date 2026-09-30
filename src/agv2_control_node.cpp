// agv2_control_node — single-process driver replacing the agv_pkg command path
// + the steering_adapter intermediate process. See plan file for design notes.

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sstream>
#include <set>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include "agv2_pkg/can_codec.hpp"
#include "agv2_pkg/fsm.hpp"
#include "agv2_pkg/control_safety.hpp"
#include "agv2_pkg/kinematics.hpp"
#include "agv2_pkg/rate_limiter.hpp"
#include "agv2_pkg/teleop.hpp"
#include "agv2_pkg/wheel_bringup.hpp"
#include "agv2_pkg/msg/chassis_telemetry.hpp"
#include "agv2_pkg/msg/wheel_command.hpp"

namespace agv2_pkg {

// Keep ROS alive after SIGINT/SIGTERM long enough to drain the final CAN frame.
std::atomic<bool> g_shutdown_requested{false};
void async_safe_log(const char* msg) {
  ssize_t n = write(STDERR_FILENO, msg, std::strlen(msg));
  (void)n;
}
void shutdown_signal_handler(int /*sig*/) {
  if (g_shutdown_requested.exchange(true)) return;
  async_safe_log("\n[agv2] shutdown requested; draining final CAN frame.\n");
}

enum class Mode { Idle, Teleop, Auto };
using ActiveSource = CommandSource;

struct SteerDirections {
  int front;  // expected -1 or +1
  int rear;   // expected -1 or +1
};

struct CanIds {
  uint32_t steer_front;
  uint32_t steer_rear;
  uint32_t wheel_front;
  uint32_t wheel_rear;
  uint32_t wheel_front_status;
  uint32_t wheel_rear_status;
  uint32_t sdo_front;
  uint32_t sdo_rear;
  uint8_t front_node_id;
  uint8_t rear_node_id;
};

class Agv2Control : public rclcpp::Node {
 public:
  Agv2Control() : Node("agv2_control_node") {
    declare_and_load_params();

    rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", cmd_qos,
        std::bind(&Agv2Control::on_cmd_vel, this, std::placeholders::_1));
    wheel_cmd_sub_ = create_subscription<msg::WheelCommand>(
        "wheel_command", cmd_qos,
        std::bind(&Agv2Control::on_wheel_command, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(1),
        std::bind(&Agv2Control::on_joy, this, std::placeholders::_1));
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
        "from_can_bus", rclcpp::QoS(1000),
        std::bind(&Agv2Control::on_can, this, std::placeholders::_1));

    can_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS(1000));
    soc_pub_ = create_publisher<std_msgs::msg::Float64>("Battery_SOC_STATE", rclcpp::QoS(10));
    telemetry_pub_ = create_publisher<msg::ChassisTelemetry>(
        "agv2/chassis_telemetry", rclcpp::QoS(100));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "agv2/diagnostics", rclcpp::QoS(10));
    reset_faults_srv_ = create_service<std_srvs::srv::Trigger>(
        "agv2/reset_faults",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          reset_faults(*res);
        });
    reinitialize_srv_ = create_service<std_srvs::srv::Trigger>(
        "agv2/reinitialize_transport",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          if (mode_ != Mode::Idle || shutdown_prepared_ || g_shutdown_requested.load() ||
              !actuation_ever_started_ || !can_sender_ready_for_bringup()) {
            res->success = false;
            res->message = "requires an already-started, locked chassis and a connected CAN sender";
            return;
          }
          if (wheel_front_status_.fault || wheel_rear_status_.fault ||
              wheel_front_status_.error_code != 0 || wheel_rear_status_.error_code != 0) {
            res->success = false;
            res->message = "resolve drive faults first; automatic drive fault-reset is prohibited";
            return;
          }
          stop_motion("transport_reinitializing");
          startup_interlock_released_ = true;  // Recovery remains fault-latched.
          reset_wheel_bringup(wheel_front_bringup_);
          reset_wheel_bringup(wheel_rear_bringup_);
          recovery_requested_ = true;
          recovery_started_ms_ = now_steady_ms();
          recovery_status_ = "waiting_feedback";
          fault_latch_.update(fault_latch_.active() | TransportRecovery);
          res->success = true;
          res->message = "communication recovery scheduled with shutdown controlwords only; verify diagnostics, reset faults, then re-enable";
          publish_diagnostics(true);
        });

    lock_srv_ = create_service<std_srvs::srv::Trigger>(
        "agv2/lock",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          {
            std::lock_guard<std::mutex> lk(mu_);
            stop_motion("operator_lock");
          }
          res->success = true;
          res->message = "locked";
        });
    prepare_shutdown_srv_ = create_service<std_srvs::srv::Trigger>(
        "agv2/prepare_shutdown",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          {
            std::lock_guard<std::mutex> lk(mu_);
            stop_motion("shutdown_requested");
          }

          if (shutdown_prepared_) {
            res->success = true;
            res->message = "shutdown already prepared (DDS delivery only)";
            return;
          }
          if (!actuation_ever_started_) {
            shutdown_prepared_ = true;
            res->success = true;
            res->message = "actuation was never brought up; no final CAN frame required";
            return;
          }
          if (!can_sender_ready_for_bringup()) {
            res->success = false;
            res->message = "CAN sender is not connected; final frames were not sent";
            return;
          }

          publish_zero_wheels();
          const auto terminator = encode_terminator();
          RCLCPP_INFO(get_logger(),
                      "Preparing terminator id=0x%03x dlc=%u tail=0x%02x.",
                      static_cast<unsigned int>(terminator.id),
                      static_cast<unsigned int>(terminator.dlc),
                      static_cast<unsigned int>(terminator.data[7]));
          can_pub_->publish(terminator);
          const bool acknowledged = wait_for_can_delivery();
          shutdown_prepared_ = acknowledged;
          res->success = acknowledged;
          res->message = acknowledged
              ? "final frames acknowledged at DDS layer; physical stop still requires verification"
              : "DDS delivery was not confirmed; control remains locked, keep CAN running";
          publish_diagnostics(true);
        });

    const auto period = std::chrono::milliseconds(
        static_cast<int>(std::round(1000.0 / loop_hz_)));
    timer_ = create_wall_timer(period, std::bind(&Agv2Control::tick, this));

    RCLCPP_INFO(get_logger(),
        "agv2_control_node up: loop=%.1fHz, geom F(%.3f,%.3f) R(%.3f,%.3f), "
        "wheel_radius=%.3f, motor_dir(F=%d,R=%d), steer_dir(F=%d,R=%d), "
        "can steer(F=0x%03x,R=0x%03x,speed_field=%u), wheel(F=0x%03x,R=0x%03x), "
        "max_v=%.2f m/s, max_w=%.2f rad/s, teleop_rotate_w=%.2f rad/s",
        loop_hz_, geom_.front_x, geom_.front_y, geom_.rear_x, geom_.rear_y,
        wheel_lim_.wheel_radius, dirs_.front, dirs_.rear,
        steer_dirs_.front, steer_dirs_.rear,
        static_cast<unsigned int>(can_ids_.steer_front),
        static_cast<unsigned int>(can_ids_.steer_rear),
        static_cast<unsigned int>(steer_speed_field_),
        static_cast<unsigned int>(can_ids_.wheel_front),
        static_cast<unsigned int>(can_ids_.wheel_rear),
        max_linear_speed_, max_angular_speed_, joy_.teleop_rotate_omega);
  }

  // Called by the control timer after a process signal as a fallback when the
  // explicit prepare-shutdown service was not used.
  void emit_terminator() {
    if (shutdown_prepared_) {
      RCLCPP_INFO(get_logger(),
                  "Graceful shutdown was already prepared; suppressing duplicate frame.");
      return;
    }
    if (!actuation_ever_started_) {
      RCLCPP_INFO(get_logger(),
                  "Actuation was never brought up; suppressing terminator CAN frame.");
      return;
    }
    stop_motion("process_shutdown");
    publish_zero_wheels();
    can_pub_->publish(encode_terminator());
    const bool acknowledged = wait_for_can_delivery();
    if (acknowledged) {
      RCLCPP_INFO(get_logger(), "Terminator CAN frame acknowledged by ROS subscribers.");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Terminator queued but DDS acknowledgment timed out; verify CAN capture.");
    }
  }

 private:
  // -------- params --------
  void declare_and_load_params() {
    geom_.front_x = declare_parameter<double>("chassis.front_wheel_x", 0.25);
    geom_.front_y = declare_parameter<double>("chassis.front_wheel_y", 0.0);
    geom_.rear_x  = declare_parameter<double>("chassis.rear_wheel_x", -0.25);
    geom_.rear_y  = declare_parameter<double>("chassis.rear_wheel_y", 0.0);
    wheel_lim_.wheel_radius = declare_parameter<double>("chassis.wheel_radius", 0.10);
    dirs_.front = declare_parameter<int>("chassis.front_motor_direction", -1);
    dirs_.rear  = declare_parameter<int>("chassis.rear_motor_direction",   1);
    steer_dirs_.front = declare_parameter<int>("chassis.front_steer_direction", 1);
    steer_dirs_.rear  = declare_parameter<int>("chassis.rear_steer_direction",  1);

    can_ids_.steer_front = declare_can_id_parameter("can.steer_front_id", kSteerFrontId);
    can_ids_.steer_rear = declare_can_id_parameter("can.steer_rear_id", kSteerRearId);
    can_ids_.wheel_front = declare_can_id_parameter("can.wheel_front_id", kWheelFrontId);
    can_ids_.wheel_rear = declare_can_id_parameter("can.wheel_rear_id", kWheelRearId);
    can_ids_.wheel_front_status = declare_can_id_parameter(
        "can.wheel_front_status_id", kWheelFrontStatusId);
    can_ids_.wheel_rear_status = declare_can_id_parameter(
        "can.wheel_rear_status_id", kWheelRearStatusId);
    can_ids_.sdo_front = declare_can_id_parameter("can.sdo_front_id", kSdoFrontId);
    can_ids_.sdo_rear = declare_can_id_parameter("can.sdo_rear_id", kSdoRearId);
    can_ids_.front_node_id = declare_node_id_parameter("can.front_node_id", 0x01);
    can_ids_.rear_node_id = declare_node_id_parameter("can.rear_node_id", 0x02);
    steer_speed_field_ = declare_uint16_parameter(
        "can.steer_speed_field", kSteerSpeedDps, /*minimum=*/1);

    bringup_cfg_.wait_for_can_sender =
        declare_parameter<bool>("wheel_bringup.wait_for_can_sender", true);
    can_sender_node_name_ = declare_parameter<std::string>(
        "wheel_bringup.can_sender_node_name", "socket_can_sender");
    bringup_cfg_.sdo_retry_period_ms =
        declare_ms_parameter("wheel_bringup.sdo_retry_period_ms", 1000);
    bringup_cfg_.status_stale_ms =
        declare_ms_parameter("wheel_bringup.status_stale_ms", 500);
    bringup_cfg_.warn_period_ms =
        declare_ms_parameter("wheel_bringup.warn_period_ms", 2000);

    max_linear_speed_  = declare_parameter<double>("limits.max_linear_speed",  0.5);
    max_angular_speed_ = declare_parameter<double>("limits.max_angular_speed", 0.5);
    max_steer_angle_   = declare_parameter<double>("limits.max_steer_angle",   1.5708);
    wheel_lim_.max_rpm = declare_parameter<double>("limits.max_wheel_rpm",     200.0);
    singularity_speed_ = declare_parameter<double>("limits.singularity_speed", 0.001);

    loop_hz_   = declare_parameter<double>("rate_limit.loop_hz", 20.0);
    rate_lim_.max_d_speed = declare_parameter<double>("rate_limit.max_d_speed_per_step", 0.05);

    const int watchdog_ms   = declare_parameter<int>("fsm.cmd_watchdog_ms", 200);
    const int settle_ms     = declare_parameter<int>("fsm.lock_settle_ms",   100);
    const double stop_eps   = declare_parameter<double>("fsm.stop_speed_eps", 0.005);
    fsm_ = std::make_unique<StateMachine>(FsmConfig{
        std::chrono::milliseconds(watchdog_ms),
        std::chrono::milliseconds(settle_ms),
        stop_eps});

    input_timeout_ms_ = declare_parameter<int>("arbitrator.input_timeout_ms", 1000);
    arbitrator_ = std::make_unique<CommandArbitrator>(
        std::chrono::milliseconds(watchdog_ms),
        std::chrono::milliseconds(input_timeout_ms_));

    diagnostic_cfg_.wheel_feedback_timeout_ms =
        declare_ms_parameter("diagnostics.wheel_feedback_timeout_ms", 500);
    diagnostic_cfg_.steer_feedback_timeout_ms =
        declare_ms_parameter("diagnostics.steer_feedback_timeout_ms", 300);
    diagnostic_cfg_.steer_loopback_window_ms =
        declare_ms_parameter("diagnostics.steer_loopback_window_ms", 50);
    diagnostic_cfg_.expect_tx_loopback =
        declare_parameter<bool>("diagnostics.expect_tx_loopback", true);
    diagnostic_cfg_.min_motion_command_mps =
        declare_parameter<double>("diagnostics.min_motion_command_mps", 0.05);
    diagnostic_cfg_.steer_mismatch_deg =
        declare_parameter<double>("diagnostics.steer_mismatch_deg", 8.0);

    joy_.enable_button   = declare_parameter<int>("joystick.enable_button", 8);
    joy_.ad_enable_button = declare_parameter<int>("joystick.ad_enable_button", 9);
    joy_.emergency_button = declare_parameter<int>("joystick.emergency_button", 7);
    joy_.left_linear_axis  = declare_parameter<int>("joystick.left_linear_axis", 1);
    joy_.left_angular_axis = declare_parameter<int>("joystick.left_angular_axis", 0);
    joy_.left_linear_sign  = declare_parameter<double>("joystick.left_linear_sign", 1.0);
    joy_.left_angular_sign = declare_parameter<double>("joystick.left_angular_sign", 1.0);
    joy_.speed_axis       = declare_parameter<int>("joystick.speed_axis", 7);
    joy_.angular_axis     = declare_parameter<int>("joystick.angular_axis", 2);
    joy_.angular_left_button  = declare_parameter<int>("joystick.angular_left_button", 3);
    joy_.angular_right_button = declare_parameter<int>("joystick.angular_right_button", 1);
    joy_.heartbeat_timeout_s  = declare_parameter<double>("joystick.heartbeat_timeout_s", 0.5);
    joy_.teleop_max_speed     = declare_parameter<double>("joystick.teleop_max_speed", 0.4);
    const double legacy_rotate_speed = declare_parameter<double>("joystick.teleop_rotate_speed", 0.3);
    const double front_rotate_radius = std::hypot(geom_.front_x, geom_.front_y);
    const double rear_rotate_radius = std::hypot(geom_.rear_x, geom_.rear_y);
    const double rotate_radius = std::max(1e-6, std::max(front_rotate_radius, rear_rotate_radius));
    joy_.teleop_rotate_omega  = declare_parameter<double>(
        "joystick.teleop_rotate_omega", legacy_rotate_speed / rotate_radius);
    joy_.teleop_steer_lock    = declare_parameter<double>("joystick.teleop_steer_lock", 1.5708);
    joy_.teleop_deadband      = declare_parameter<double>("joystick.teleop_deadband", 0.1);

    safety_.feedback_stop_enabled = declare_parameter<bool>("safety.feedback_stop_enabled", true);
    safety_.steer_alignment_enabled = declare_parameter<bool>("safety.steer_alignment_enabled", false);
    safety_.steer_alignment_max_error_deg = declare_parameter<double>(
        "safety.steer_alignment_max_error_deg", 15.0);
    safety_.steer_error_stop_enabled = declare_parameter<bool>("safety.steer_error_stop_enabled", false);
    safety_.steer_error_stop_ms = declare_ms_parameter("safety.steer_error_stop_ms", 1000);
    safety_.hold_steering_on_teleop_stop = declare_parameter<bool>(
        "safety.hold_steering_on_teleop_stop", false);
    diagnostic_period_ms_ = declare_ms_parameter("diagnostics.publish_period_ms", 200);
    hardware_id_ = declare_parameter<std::string>("diagnostics.hardware_id", "agv");

    const auto require = [](bool valid, const char* message) {
      if (!valid) throw std::invalid_argument(message);
    };
    require(all_finite({geom_.front_x, geom_.front_y, geom_.rear_x, geom_.rear_y}) &&
            std::hypot(geom_.front_x - geom_.rear_x, geom_.front_y - geom_.rear_y) > 1e-6,
            "chassis wheel positions must be finite and distinct");
    require(finite_positive(wheel_lim_.wheel_radius), "chassis.wheel_radius must be positive");
    require((dirs_.front == 1 || dirs_.front == -1) && (dirs_.rear == 1 || dirs_.rear == -1) &&
            (steer_dirs_.front == 1 || steer_dirs_.front == -1) &&
            (steer_dirs_.rear == 1 || steer_dirs_.rear == -1), "motor/steer directions must be +/-1");
    require(finite_positive(max_linear_speed_) && finite_positive(max_angular_speed_) &&
            finite_positive(wheel_lim_.max_rpm) && finite_positive(singularity_speed_) &&
            finite_in_range(max_steer_angle_, 0.001, 1.5708), "invalid motion limits");
    require(finite_in_range(loop_hz_, 1.0, 200.0) && finite_positive(rate_lim_.max_d_speed),
            "loop_hz must be 1..200 and speed step must be positive");
    require(settle_ms >= 0 && finite_nonnegative(stop_eps), "invalid FSM stop/settle parameters");
    require(bringup_cfg_.status_stale_ms > 0 && bringup_cfg_.sdo_retry_period_ms > 0 &&
            diagnostic_cfg_.wheel_feedback_timeout_ms > 0 &&
            diagnostic_cfg_.steer_feedback_timeout_ms > 0 && diagnostic_period_ms_ > 0,
            "feedback, SDO retry and diagnostic periods must be positive");
    require(finite_positive(diagnostic_cfg_.steer_mismatch_deg) &&
            finite_nonnegative(diagnostic_cfg_.min_motion_command_mps) &&
            finite_in_range(safety_.steer_alignment_max_error_deg, 0.1, 180.0) &&
            safety_.steer_error_stop_ms > 0, "invalid steering safety thresholds");
    require(finite_nonnegative(joy_.heartbeat_timeout_s) && finite_positive(joy_.teleop_max_speed) &&
            finite_positive(joy_.teleop_rotate_omega) && finite_in_range(joy_.teleop_deadband, 0.0, 1.0) &&
            finite_in_range(joy_.teleop_steer_lock, 0.0, max_steer_angle_) &&
            all_finite({joy_.left_linear_sign, joy_.left_angular_sign}), "invalid joystick parameters");
    const std::set<uint32_t> ids{can_ids_.steer_front, can_ids_.steer_rear,
        can_ids_.wheel_front, can_ids_.wheel_rear, can_ids_.wheel_front_status,
        can_ids_.wheel_rear_status, can_ids_.sdo_front, can_ids_.sdo_rear};
    require(ids.size() == 8, "configured motor CAN IDs must be distinct");
    require(can_ids_.front_node_id != can_ids_.rear_node_id, "CANopen node IDs must be distinct");
  }

  // -------- callbacks --------
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!all_finite({msg->linear.x, msg->linear.y, msg->angular.z})) {
      reject_input(2, "cmd_vel contains non-finite values", mode_ == Mode::Auto);
      return;
    }
    invalid_sources_ &= ~2u;
    cmd_vx_    = std::clamp(msg->linear.x,  -max_linear_speed_, max_linear_speed_);
    cmd_vy_    = std::clamp(msg->linear.y,  -max_linear_speed_, max_linear_speed_);
    cmd_omega_ = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
    last_cmd_vel_t_ = now_steady();
    if (mode_ == Mode::Auto) arbitrator_->note_input(CommandSource::CmdVel, last_cmd_vel_t_);
  }

  void on_wheel_command(const msg::WheelCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!all_finite({msg->front_steer_angle, msg->rear_steer_angle,
                     msg->front_wheel_speed, msg->rear_wheel_speed})) {
      reject_input(4, "wheel_command contains non-finite values", mode_ == Mode::Auto);
      return;
    }
    invalid_sources_ &= ~4u;
    wheel_cmd_target_.theta_front = std::clamp(msg->front_steer_angle, -max_steer_angle_, max_steer_angle_);
    wheel_cmd_target_.theta_rear  = std::clamp(msg->rear_steer_angle,  -max_steer_angle_, max_steer_angle_);
    wheel_cmd_target_.v_front = std::clamp(msg->front_wheel_speed, -max_linear_speed_, max_linear_speed_);
    wheel_cmd_target_.v_rear  = std::clamp(msg->rear_wheel_speed,  -max_linear_speed_, max_linear_speed_);
    wheel_cmd_target_.singular = false;
    last_wheel_cmd_t_ = now_steady();
    if (mode_ == Mode::Auto) arbitrator_->note_input(CommandSource::WheelCmd, last_wheel_cmd_t_);
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr j) {
    std::lock_guard<std::mutex> lk(mu_);
    last_joy_t_ = now_steady();

    auto btn = [&](int idx) -> bool {
      return idx >= 0 && idx < static_cast<int>(j->buttons.size()) && j->buttons[idx] != 0;
    };
    auto axis = [&](int idx) -> double {
      return (idx >= 0 && idx < static_cast<int>(j->axes.size())) ? j->axes[idx] : 0.0;
    };

    const bool emergency_now = btn(joy_.emergency_button);
    const bool enable_now    = btn(joy_.enable_button);
    const bool ad_now        = btn(joy_.ad_enable_button);

    bool axes_neutral = true;
    for (int i = 0; i <= 3 && i < static_cast<int>(j->axes.size()); ++i) {
      if (!std::isfinite(j->axes[i]) || std::fabs(j->axes[i]) > 1e-3) axes_neutral = false;
    }
    const auto action = joy_gate_.update(emergency_now, enable_now, ad_now, axes_neutral);
    if (action == JoyEnableAction::Emergency) {
      stop_motion("software_emergency");
    }
    if (!std::all_of(j->axes.begin(), j->axes.end(),
                     [](float value) { return std::isfinite(value); })) {
      reject_input(1, "joy contains non-finite axes", mode_ != Mode::Idle);
      return;
    }
    invalid_sources_ &= ~1u;
    if (action == JoyEnableAction::TeleopRejected) {
      RCLCPP_WARN(get_logger(), "Joy: enable rejected; axes 0-3 not neutral.");
    } else if (action == JoyEnableAction::TeleopEnable || action == JoyEnableAction::AutoEnable) {
      if (fault_latch_.latched() != 0 || shutdown_prepared_ || invalid_sources_ != 0) {
        RCLCPP_WARN(get_logger(), "Enable rejected: clear faults/invalid input before rearming.");
      } else {
        stop_motion("enabled_waiting_input");
        mode_ = action == JoyEnableAction::TeleopEnable ? Mode::Teleop : Mode::Auto;
        release_startup_interlock(mode_ == Mode::Teleop ? "Teleop" : "Auto");
      }
    }

    // Compute teleop target (only used while mode_ == Teleop).
    const double sa = axis(joy_.speed_axis);
    const double aa = axis(joy_.angular_axis);
    const double left_linear = axis(joy_.left_linear_axis) * joy_.left_linear_sign;
    const double left_angular = axis(joy_.left_angular_axis) * joy_.left_angular_sign;
    const bool bx = btn(joy_.angular_left_button);
    const bool bb = btn(joy_.angular_right_button);

    joy_target_ = make_teleop_target(
        TeleopInput{left_linear, left_angular, sa, aa, bx, bb},
        TeleopParams{joy_.teleop_max_speed, joy_.teleop_rotate_omega,
                     joy_.teleop_steer_lock, joy_.teleop_deadband},
        geom_, singularity_speed_, state_.theta_front, state_.theta_rear);
    if (safety_.hold_steering_on_teleop_stop &&
        joy_target_.v_front == 0.0 && joy_target_.v_rear == 0.0) {
      joy_target_.theta_front = state_.theta_front;
      joy_target_.theta_rear = state_.theta_rear;
    }
    if (mode_ == Mode::Teleop) fsm_->on_active_input(last_joy_t_);
  }

  void on_can(const can_msgs::msg::Frame::SharedPtr f) {
    const int64_t now_ms = now_steady_ms();
    if (f->is_error) {
      ++can_error_frame_count_;
      return;
    }
    if (f->is_extended || f->is_rtr || f->dlc > 8) return;
    auto status = decode_wheel_status(
        *f, can_ids_.wheel_front_status, can_ids_.wheel_rear_status);
    if (status.present) {
      if (f->id == can_ids_.wheel_front_status) {
        wheel_front_status_ = status;
        note_wheel_status(wheel_front_bringup_, now_ms);
        ++wheel_front_feedback_count_;
      }
      if (f->id == can_ids_.wheel_rear_status) {
        wheel_rear_status_ = status;
        note_wheel_status(wheel_rear_bringup_, now_ms);
        ++wheel_rear_feedback_count_;
      }
    }
    if (!consume_steer_query_loopback(*f, now_ms)) {
      auto pos = decode_steer_position(
          *f, can_ids_.steer_rear, can_ids_.steer_front);
      if (pos.present) {
        if (f->id == can_ids_.steer_front) {
          steer_front_fb_deg_ = normalize_single_turn_angle_deg(pos.position_deg) *
                                steer_dirs_.front;
          steer_front_feedback_seen_ = true;
          steer_front_feedback_ms_ = now_ms;
          ++steer_front_feedback_count_;
        }
        if (f->id == can_ids_.steer_rear) {
          steer_rear_fb_deg_ = normalize_single_turn_angle_deg(pos.position_deg) *
                               steer_dirs_.rear;
          steer_rear_feedback_seen_ = true;
          steer_rear_feedback_ms_ = now_ms;
          ++steer_rear_feedback_count_;
        }
      }
    }
    auto soc = decode_battery_soc(*f);
    if (soc.present) {
      std_msgs::msg::Float64 m;
      m.data = soc.soc;
      soc_pub_->publish(m);
    }
  }

  // -------- main loop --------
  void tick() {
    if (shutdown_prepared_) {
      publish_diagnostics();
      return;
    }

    // Publish the final frame from a normal executor callback while the ROS
    // context and matched CAN sender subscription are still fully active.
    if (g_shutdown_requested.load()) {
      if (!shutdown_processed_) {
        shutdown_processed_ = true;
        emit_terminator();
      }
      return;
    }

    // Joy heartbeat: if any control mode is active and joy is silent for
    // longer than the timeout, drop to Idle and lock.
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (mode_ != Mode::Idle && joy_.heartbeat_timeout_s > 0.0) {
        const auto now = now_steady();
        const double age = std::chrono::duration<double>(now - last_joy_t_).count();
        if (last_joy_t_.time_since_epoch().count() != 0 &&
            age > joy_.heartbeat_timeout_s) {
          RCLCPP_ERROR(get_logger(),
              "Joy heartbeat lost (%.2fs) — dropping to Idle/Locked.", age);
          stop_motion("joy_heartbeat_timeout");
        }
      }
    }

    update_fault_state();
    WheelTargets target{};
    bool have_input = false;
    Mode mode_now;
    {
      std::lock_guard<std::mutex> lk(mu_);
      mode_now = mode_;
      have_input = arbitrate(target);
    }

    // Hard limits on target.
    target.theta_front = std::clamp(target.theta_front, -max_steer_angle_, max_steer_angle_);
    target.theta_rear  = std::clamp(target.theta_rear,  -max_steer_angle_, max_steer_angle_);
    target.v_front = std::clamp(target.v_front, -max_linear_speed_, max_linear_speed_);
    target.v_rear  = std::clamp(target.v_rear,  -max_linear_speed_, max_linear_speed_);
    const WheelTargets requested_target = target;

    // FSM: if no fresh input or mode is Idle, force ramp to zero speed.
    const FsmState st = fsm_->tick(now_steady(),
                                   std::fabs(state_.v_front),
                                   std::fabs(state_.v_rear));
    if (mode_now == Mode::Idle || !have_input || st != FsmState::Normal) {
      target.theta_front = state_.theta_front;
      target.theta_rear  = state_.theta_rear;
      target.v_front = 0.0;
      target.v_rear  = 0.0;
    }

    // The optional steering gate is off in the compatibility profile.
    const bool align_wait = safety_.steer_alignment_enabled && have_input &&
        (std::fabs(steer_error_deg(target.theta_front, steer_front_fb_deg_)) >
             safety_.steer_alignment_max_error_deg ||
         std::fabs(steer_error_deg(target.theta_rear, steer_rear_fb_deg_)) >
             safety_.steer_alignment_max_error_deg);
    if (align_wait) {
      target.v_front = target.v_rear = 0.0;
      state_.v_front = state_.v_rear = 0.0;
      stop_reason_ = "steering_alignment";
    } else if (mode_now != Mode::Idle) {
      stop_reason_ = have_input && st == FsmState::Normal ? "running" : "command_timeout";
    }
    state_ = step(state_, target, rate_lim_);

    // Emit per-tick CAN traffic.
    emit_can_frames(st == FsmState::Locked, startup_interlock_released_);
    publish_telemetry(requested_target, have_input, mode_now, st);
    publish_diagnostics();
  }

  // Pick the highest-priority active input. Returns true if a target was set.
  // Must be called with mu_ held.
  bool arbitrate(WheelTargets& out) {
    const auto now = now_steady();
    auto fresh = [&](const Clock::time_point& t) {
      if (t.time_since_epoch().count() == 0) return false;
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count();
      return age_ms <= input_timeout_ms_;
    };

    if (mode_ == Mode::Teleop) {
      if (fresh(last_joy_t_)) {
        out = joy_target_;
        return true;
      }
      return false;
    }
    if (mode_ == Mode::Auto) {
      const auto selected = arbitrator_->select(now);
      active_source_ = selected.source;
      if (!selected.fresh_for_motion) return false;
      // Only the owner refreshes motion safety, using its receive time rather
      // than the timer time. Other publishers cannot keep an old command alive.
      fsm_->on_active_input(selected.last_input);
      if (selected.source == CommandSource::WheelCmd) {
        out = wheel_cmd_target_;
        return true;
      }
      if (selected.source == CommandSource::CmdVel) {
        out = solve_ik(cmd_vx_, cmd_vy_, cmd_omega_, geom_, singularity_speed_,
                       state_.theta_front, state_.theta_rear);
        return true;
      }
    }
    return false;
  }

  // -------- CAN emit (per tick) --------
  void emit_can_frames(bool locked, bool startup_interlock_released) {
    const int64_t now_ms = now_steady_ms();
    const bool can_sender_ready = can_sender_ready_for_bringup();

    // Steering: query current position (legacy diagnostic, harmless if ignored).
    note_steer_query(can_ids_.steer_rear, now_ms);
    can_pub_->publish(encode_steer_position_query(can_ids_.steer_rear));
    note_steer_query(can_ids_.steer_front, now_ms);
    can_pub_->publish(encode_steer_position_query(can_ids_.steer_front));

    // Startup is passive: do not enable either motor family or publish any
    // target until the operator has made an explicit post-start enable edge,
    // both steering positions are known, and the real CAN sender is present.
    if (!startup_interlock_released || !can_sender_ready) {
      state_.v_front = state_.v_rear = 0.0;
      return;
    }
    if (fault_latch_.latched() != 0) {
      state_.v_front = state_.v_rear = 0.0;
      if (recovery_requested_) service_transport_recovery(now_ms);
      else if (actuation_ever_started_) publish_zero_wheels();
      return;
    }
    if (!steering_feedback_fresh(now_ms)) {
      state_.v_front = state_.v_rear = 0.0;
      if (actuation_ever_started_) publish_zero_wheels();
      stop_reason_ = "waiting_steering_feedback";
      return;
    }

    // Enable steering once, then wait until the next control tick before
    // issuing the first position target.
    if (!steering_brought_up_) {
      bring_up_steering();
      steering_brought_up_ = true;
      actuation_ever_started_ = true;
      state_.v_front = state_.v_rear = 0.0;
      stop_reason_ = "steering_bringup";
      return;
    }

    // 0x94 is single-turn feedback and wraps at 360 degrees; it must not be
    // used to select between 0 and 360 for the 0xA4 multi-turn command. Keep
    // commanded angles in the calibrated local turn around zero, matching the
    // legacy driver's working convention and the chassis' +/-90 degree range.
    can_pub_->publish(encode_steer_position_cmd(
        can_ids_.steer_rear, state_.theta_rear * steer_dirs_.rear,
        steer_speed_field_));
    can_pub_->publish(encode_steer_position_cmd(
        can_ids_.steer_front, state_.theta_front * steer_dirs_.front,
        steer_speed_field_));

    // Wheel motors: run a retryable CANopen bring-up before allowing velocity.
    const bool front_ready = service_wheel_bringup(
        "front", can_ids_.wheel_front, can_ids_.sdo_front, can_ids_.front_node_id,
        can_ids_.wheel_front_status, wheel_front_status_, wheel_front_bringup_,
        can_sender_ready, now_ms);
    const bool rear_ready = service_wheel_bringup(
        "rear", can_ids_.wheel_rear, can_ids_.sdo_rear, can_ids_.rear_node_id,
        can_ids_.wheel_rear_status, wheel_rear_status_, wheel_rear_bringup_,
        can_sender_ready, now_ms);

    if (front_ready && rear_ready) {
      hardware_monitor_armed_ = true;
      const double vf = locked ? 0.0 : state_.v_front;
      const double vr = locked ? 0.0 : state_.v_rear;
      can_pub_->publish(encode_wheel_velocity(can_ids_.wheel_front, vf, dirs_.front, wheel_lim_));
      can_pub_->publish(encode_wheel_velocity(can_ids_.wheel_rear,  vr, dirs_.rear,  wheel_lim_));
    } else {
      // Reset the limiter while no valid output is possible. Also stop the
      // healthy wheel explicitly; silence must never be used as a stop command.
      state_.v_front = state_.v_rear = 0.0;
      if (front_ready) can_pub_->publish(encode_wheel_velocity(
          can_ids_.wheel_front, 0.0, dirs_.front, wheel_lim_));
      if (rear_ready) can_pub_->publish(encode_wheel_velocity(
          can_ids_.wheel_rear, 0.0, dirs_.rear, wheel_lim_));
      // The not-ready drive already received its zero-target bring-up
      // controlword. Do not overwrite switch-on/enable with shutdown here.
      if (wheel_front_status_.fault || wheel_front_status_.error_code != 0)
        can_pub_->publish(encode_wheel_controlword(can_ids_.wheel_front, 0));
      if (wheel_rear_status_.fault || wheel_rear_status_.error_code != 0)
        can_pub_->publish(encode_wheel_controlword(can_ids_.wheel_rear, 0));
      stop_reason_ = "waiting_wheel_bringup";
    }
  }

  bool service_wheel_bringup(const char* label,
                             uint32_t wheel_id,
                             uint32_t sdo_id,
                             uint8_t node_id,
                             uint32_t status_id,
                             const WheelStatus& status,
                             WheelBringupState& bringup,
                             bool can_sender_ready,
                             int64_t now_ms) {
    const auto decision = update_wheel_bringup(
        bringup, status, bringup_cfg_, can_sender_ready, now_ms);

    if (decision.send_sdo_init) {
      for (auto& f : encode_wheel_sdo_init(sdo_id, node_id)) {
        can_pub_->publish(f);
      }
    }
    if (decision.send_controlword) {
      can_pub_->publish(encode_wheel_controlword(wheel_id, decision.controlword_step));
    }
    if (decision.warn) {
      log_wheel_bringup(label, wheel_id, status_id, status, bringup, decision, now_ms,
                        can_sender_ready);
    }
    return decision.allow_velocity;
  }

  // -------- one-shot bring-up --------
  void bring_up_steering() {
    // Steering motors: 0x88 enable.
    can_pub_->publish(encode_steer_enable(can_ids_.steer_rear));
    can_pub_->publish(encode_steer_enable(can_ids_.steer_front));
    RCLCPP_INFO(get_logger(), "Steering bring-up frames emitted.");
  }

  void release_startup_interlock(const char* requested_mode) {
    if (startup_interlock_released_) return;
    startup_interlock_released_ = true;
    RCLCPP_WARN(
        get_logger(),
        "Startup actuation interlock released by explicit %s enable edge; "
        "hardware bring-up will wait for both steering positions and CAN sender.",
        requested_mode);
  }

  // -------- helpers --------
  using Clock = std::chrono::steady_clock;
  Clock::time_point now_steady() const { return Clock::now(); }
  int64_t now_steady_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now_steady().time_since_epoch()).count();
  }

  bool can_sender_ready_for_bringup() const {
    if (!bringup_cfg_.wait_for_can_sender) return true;
    if (can_sender_node_name_.empty()) {
      return (can_pub_->get_subscription_count() +
              can_pub_->get_intra_process_subscription_count()) > 0;
    }
    const auto subscriptions =
        get_subscriptions_info_by_topic(can_pub_->get_topic_name());
    return std::any_of(
        subscriptions.begin(), subscriptions.end(),
        [this](const rclcpp::TopicEndpointInfo& endpoint) {
          return endpoint.node_name() == can_sender_node_name_;
        });
  }

  enum Fault : uint32_t {
    CanSenderMissing = 1u, FrontWheelTimeout = 2u, RearWheelTimeout = 4u,
    FrontSteerTimeout = 8u, RearSteerTimeout = 16u, FrontWheelFault = 32u,
    RearWheelFault = 64u, SteeringMismatch = 128u, InvalidInput = 256u,
    FrontWheelDisabled = 512u, RearWheelDisabled = 1024u, TransportRecovery = 2048u
  };

  void stop_motion(const char* reason) {
    mode_ = Mode::Idle;
    fsm_->on_lock_request();
    arbitrator_->reset();
    active_source_ = ActiveSource::None;
    state_.v_front = state_.v_rear = 0.0;
    stop_reason_ = reason;
  }

  void reject_input(uint32_t source, const char* reason, bool controlling) {
    invalid_sources_ |= source;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason);
    if (controlling) {
      fault_latch_.update(fault_latch_.active() | InvalidInput);
      stop_motion("invalid_input");
      last_diagnostic_ms_ = -1;
    }
  }

  bool wheel_feedback_fresh(const WheelBringupState& wheel, int64_t now_ms) const {
    return wheel.status_seen && now_ms >= wheel.last_status_ms &&
        now_ms - wheel.last_status_ms <= std::min(
            diagnostic_cfg_.wheel_feedback_timeout_ms, bringup_cfg_.status_stale_ms);
  }

  bool steering_feedback_fresh(int64_t now_ms) const {
    return steer_front_feedback_seen_ && steer_rear_feedback_seen_ &&
        now_ms - steer_front_feedback_ms_ <= diagnostic_cfg_.steer_feedback_timeout_ms &&
        now_ms - steer_rear_feedback_ms_ <= diagnostic_cfg_.steer_feedback_timeout_ms;
  }

  uint32_t current_faults(bool recovery = false) {
    const auto now_ms = now_steady_ms();
    uint32_t flags = invalid_sources_ != 0 ? InvalidInput : 0u;
    if (recovery_requested_) flags |= TransportRecovery;
    if (startup_interlock_released_ || recovery) {
      if (wheel_front_status_.fault || wheel_front_status_.error_code != 0) flags |= FrontWheelFault;
      if (wheel_rear_status_.fault || wheel_rear_status_.error_code != 0) flags |= RearWheelFault;
    }
    if (!hardware_monitor_armed_ && !recovery) return flags;
    if (!can_sender_ready_for_bringup()) flags |= CanSenderMissing;
    const bool front_fresh = wheel_feedback_fresh(wheel_front_bringup_, now_ms);
    const bool rear_fresh = wheel_feedback_fresh(wheel_rear_bringup_, now_ms);
    if (safety_.feedback_stop_enabled || recovery) {
      if (!front_fresh) flags |= FrontWheelTimeout;
      if (!rear_fresh) flags |= RearWheelTimeout;
      if (!steer_front_feedback_seen_ ||
          now_ms - steer_front_feedback_ms_ > diagnostic_cfg_.steer_feedback_timeout_ms)
        flags |= FrontSteerTimeout;
      if (!steer_rear_feedback_seen_ ||
          now_ms - steer_rear_feedback_ms_ > diagnostic_cfg_.steer_feedback_timeout_ms)
        flags |= RearSteerTimeout;
    }
    if (wheel_front_status_.fault || wheel_front_status_.error_code != 0) flags |= FrontWheelFault;
    if (wheel_rear_status_.fault || wheel_rear_status_.error_code != 0) flags |= RearWheelFault;
    if (!recovery) {
      if (front_fresh && (!wheel_front_status_.ready || !wheel_front_status_.switched_on ||
                          !wheel_front_status_.enabled)) flags |= FrontWheelDisabled;
      if (rear_fresh && (!wheel_rear_status_.ready || !wheel_rear_status_.switched_on ||
                         !wheel_rear_status_.enabled)) flags |= RearWheelDisabled;
    }
    const bool mismatch = steering_feedback_fresh(now_ms) &&
        (std::fabs(steer_error_deg(state_.theta_front, steer_front_fb_deg_)) >= diagnostic_cfg_.steer_mismatch_deg ||
         std::fabs(steer_error_deg(state_.theta_rear, steer_rear_fb_deg_)) >= diagnostic_cfg_.steer_mismatch_deg);
    if (safety_.steer_error_stop_enabled && mismatch) {
      if (steer_mismatch_since_ms_ < 0) steer_mismatch_since_ms_ = now_ms;
      if (recovery || now_ms - steer_mismatch_since_ms_ >= safety_.steer_error_stop_ms)
        flags |= SteeringMismatch;
    } else {
      steer_mismatch_since_ms_ = -1;
    }
    return flags;
  }

  static std::string fault_description(uint32_t flags) {
    std::string result;
    const std::pair<uint32_t, const char*> reasons[] = {
        {CanSenderMissing, "can_sender_missing"}, {FrontWheelTimeout, "front_wheel_timeout"},
        {RearWheelTimeout, "rear_wheel_timeout"}, {FrontSteerTimeout, "front_steer_timeout"},
        {RearSteerTimeout, "rear_steer_timeout"}, {FrontWheelFault, "front_wheel_fault"},
        {RearWheelFault, "rear_wheel_fault"}, {SteeringMismatch, "steering_mismatch"},
        {InvalidInput, "invalid_input"}, {FrontWheelDisabled, "front_wheel_disabled"},
        {RearWheelDisabled, "rear_wheel_disabled"}, {TransportRecovery, "transport_recovery"}};
    for (const auto& reason : reasons) {
      if ((flags & reason.first) == 0) continue;
      if (!result.empty()) result += ",";
      result += reason.second;
    }
    return result.empty() ? "none" : result;
  }

  void update_fault_state() {
    const auto previous = fault_latch_.latched();
    // Before the first enable, malformed inputs are rejected/reported but do
    // not energize hardware or latch an otherwise passive startup.
    const auto flags = current_faults();
    if (startup_interlock_released_ || previous != 0) fault_latch_.update(flags);
    if (fault_latch_.latched() != 0) {
      stop_motion("fault_latched");
      if (previous != fault_latch_.latched()) {
        RCLCPP_ERROR(get_logger(), "Motion inhibited: %s; resolve then call agv2/reset_faults and re-enable.",
                     fault_description(fault_latch_.latched()).c_str());
        last_diagnostic_ms_ = -1;
      }
    }
  }

  void reset_faults(std_srvs::srv::Trigger::Response& response) {
    if (shutdown_prepared_ || g_shutdown_requested.load() || joy_gate_.emergency_active()) {
      response.success = false;
      response.message = "cannot reset during shutdown or while software emergency is held";
      return;
    }
    if (mode_ != Mode::Idle) {
      response.success = false;
      response.message = "lock the chassis before resetting faults";
      return;
    }
    // Explicit acknowledgement discards an invalid command session, including
    // one whose publisher has exited. A continuing bad publisher is rejected
    // again, and reset never restores motion permission.
    const uint32_t unresolved = current_faults(true) & ~static_cast<uint32_t>(InvalidInput);
    if (unresolved != 0) {
      response.success = false;
      response.message = "faults still active: " + fault_description(unresolved);
      return;
    }
    fault_latch_.update(0);
    fault_latch_.clear();
    invalid_sources_ = 0;
    cmd_vx_ = cmd_vy_ = cmd_omega_ = 0.0;
    wheel_cmd_target_ = {};
    joy_target_ = {};
    stop_motion("faults_cleared_reenable_required");
    hardware_monitor_armed_ = false;
    startup_interlock_released_ = false;
    steering_brought_up_ = false;
    reset_wheel_bringup(wheel_front_bringup_);
    reset_wheel_bringup(wheel_rear_bringup_);
    response.success = true;
    response.message = "host faults cleared; hardware fault reset was NOT sent; a new enable edge is required";
    RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    publish_diagnostics(true);
  }

  void service_transport_recovery(int64_t now_ms) {
    // Explicit recovery configures communication only. It must not perform the
    // normal 0x07 -> 0x0f enable sequence or publish nonzero targets.
    can_pub_->publish(encode_wheel_controlword(can_ids_.wheel_front, 0));
    can_pub_->publish(encode_wheel_controlword(can_ids_.wheel_rear, 0));
    if (now_ms - recovery_started_ms_ > 5000) {
      recovery_requested_ = false;
      recovery_status_ = "timeout_kept_locked";
      RCLCPP_ERROR(get_logger(), "Transport recovery timed out; chassis remains locked.");
      return;
    }
    if (wheel_feedback_fresh(wheel_front_bringup_, now_ms) &&
        wheel_feedback_fresh(wheel_rear_bringup_, now_ms) && steering_feedback_fresh(now_ms)) {
      recovery_requested_ = false;
      recovery_status_ = "feedback_received_reset_required";
      return;
    }
    const auto retry = [this, now_ms](WheelBringupState& tracking,
                                    const WheelStatus& status, uint32_t id, uint8_t node_id) {
      if (wheel_feedback_fresh(tracking, now_ms) || status.fault || status.error_code != 0) return;
      if (tracking.last_sdo_ms >= 0 && now_ms - tracking.last_sdo_ms < bringup_cfg_.sdo_retry_period_ms) return;
      for (const auto& frame : encode_wheel_sdo_init(id, node_id)) can_pub_->publish(frame);
      tracking.last_sdo_ms = now_ms;
      ++tracking.sdo_retry_count;
    };
    retry(wheel_front_bringup_, wheel_front_status_, can_ids_.sdo_front, can_ids_.front_node_id);
    retry(wheel_rear_bringup_, wheel_rear_status_, can_ids_.sdo_rear, can_ids_.rear_node_id);
  }

  void publish_zero_wheels() {
    const auto now_ms = now_steady_ms();
    const auto zero = [this, now_ms](uint32_t id, int direction, const WheelStatus& status,
                                   const WheelBringupState& tracking) {
      // Never use an enable-operation velocity frame to re-enable a known
      // disabled/faulted drive. The healthy wheel still receives explicit zero.
      if (wheel_feedback_fresh(tracking, now_ms) && status.present && status.ready && status.switched_on && status.enabled &&
          !status.fault && status.error_code == 0) {
        can_pub_->publish(encode_wheel_velocity(id, 0.0, direction, wheel_lim_));
      } else {
        can_pub_->publish(encode_wheel_controlword(id, 0));
      }
    };
    zero(can_ids_.wheel_front, dirs_.front, wheel_front_status_, wheel_front_bringup_);
    zero(can_ids_.wheel_rear, dirs_.rear, wheel_rear_status_, wheel_rear_bringup_);
  }

  bool wait_for_can_delivery() {
    try {
      return can_pub_->wait_for_all_acked(std::chrono::milliseconds(500));
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "DDS delivery confirmation failed: %s", error.what());
      return false;
    }
  }

  void publish_diagnostics(bool force = false) {
    const auto now_ms = now_steady_ms();
    if (!force && last_diagnostic_ms_ >= 0 && now_ms - last_diagnostic_ms_ < diagnostic_period_ms_) return;
    last_diagnostic_ms_ = now_ms;
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name()) + "/control";
    status.hardware_id = hardware_id_;
    status.level = fault_latch_.latched() != 0 ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        (stop_reason_ == "running" || shutdown_prepared_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
                                                       diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = fault_latch_.latched() != 0 ? fault_description(fault_latch_.latched()) : stop_reason_;
    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
    add("active_faults", std::to_string(current_faults()));
    add("latched_faults", std::to_string(fault_latch_.latched()));
    add("stop_reason", shutdown_prepared_ ? "shutdown_prepared" : stop_reason_);
    add("can_sender_ready", can_sender_ready_for_bringup() ? "true" : "false");
    add("startup_interlock_released", startup_interlock_released_ ? "true" : "false");
    add("hardware_monitor_armed", hardware_monitor_armed_ ? "true" : "false");
    add("software_emergency", joy_gate_.emergency_active() ? "true" : "false");
    add("invalid_input_sources", std::to_string(invalid_sources_));
    add("transport_recovery", recovery_status_);
    add("front_wheel_feedback_age_ms", std::to_string(feedback_age_ms(
        wheel_front_bringup_.status_seen, wheel_front_bringup_.last_status_ms, now_ms)));
    add("rear_wheel_feedback_age_ms", std::to_string(feedback_age_ms(
        wheel_rear_bringup_.status_seen, wheel_rear_bringup_.last_status_ms, now_ms)));
    add("front_steer_feedback_age_ms", std::to_string(feedback_age_ms(
        steer_front_feedback_seen_, steer_front_feedback_ms_, now_ms)));
    add("rear_steer_feedback_age_ms", std::to_string(feedback_age_ms(
        steer_rear_feedback_seen_, steer_rear_feedback_ms_, now_ms)));
    add("front_wheel_error_code", std::to_string(wheel_front_status_.error_code));
    add("rear_wheel_error_code", std::to_string(wheel_rear_status_.error_code));
    add("front_sdo_retries", std::to_string(wheel_front_bringup_.sdo_retry_count));
    add("rear_sdo_retries", std::to_string(wheel_rear_bringup_.sdo_retry_count));
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    add("ros_domain_id", domain ? domain : "0");
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  uint32_t declare_can_id_parameter(const char* name, uint32_t default_value) {
    const int value = declare_parameter<int>(name, static_cast<int>(default_value));
    if (value <= 0 || value > 0x7ff) {
      throw std::invalid_argument(std::string(name) + " must be a nonzero standard CAN ID (1..2047)");
    }
    return static_cast<uint32_t>(value);
  }

  uint8_t declare_node_id_parameter(const char* name, uint8_t default_value) {
    const int value = declare_parameter<int>(name, static_cast<int>(default_value));
    if (value < 1 || value > 127) {
      throw std::invalid_argument(std::string(name) + " must be a CANopen node ID (1..127)");
    }
    return static_cast<uint8_t>(value);
  }

  uint16_t declare_uint16_parameter(const char* name,
                                    uint16_t default_value,
                                    uint16_t minimum = 0) {
    const int value = declare_parameter<int>(name, static_cast<int>(default_value));
    if (value < static_cast<int>(minimum) || value > 65535) {
      RCLCPP_WARN(get_logger(),
                  "Invalid 16-bit field for %s (%d); using %u.",
                  name, value, static_cast<unsigned int>(default_value));
      return default_value;
    }
    return static_cast<uint16_t>(value);
  }

  int64_t declare_ms_parameter(const char* name, int default_value) {
    const int value = declare_parameter<int>(name, default_value);
    if (value < 0) {
      RCLCPP_WARN(get_logger(), "Invalid negative duration for %s; using %d ms.",
                  name, default_value);
      return default_value;
    }
    return value;
  }

  bool is_exact_steer_query(const can_msgs::msg::Frame& frame) const {
    if (frame.is_rtr || frame.dlc < 8 || frame.data[0] != 0x94) return false;
    return std::all_of(
        frame.data.begin() + 1, frame.data.end(),
        [](uint8_t value) { return value == 0; });
  }

  void note_steer_query(uint32_t can_id, int64_t now_ms) {
    if (!diagnostic_cfg_.expect_tx_loopback) return;
    if (can_id == can_ids_.steer_front) {
      steer_front_query_pending_ = true;
      steer_front_query_ms_ = now_ms;
    } else if (can_id == can_ids_.steer_rear) {
      steer_rear_query_pending_ = true;
      steer_rear_query_ms_ = now_ms;
    }
  }

  bool consume_steer_query_loopback(const can_msgs::msg::Frame& frame, int64_t now_ms) {
    if (!diagnostic_cfg_.expect_tx_loopback || !is_exact_steer_query(frame)) {
      return false;
    }
    if (frame.id == can_ids_.steer_front &&
        steer_front_query_pending_ &&
        now_ms - steer_front_query_ms_ <= diagnostic_cfg_.steer_loopback_window_ms) {
      steer_front_query_pending_ = false;
      ++steer_front_loopback_count_;
      return true;
    }
    if (frame.id == can_ids_.steer_rear &&
        steer_rear_query_pending_ &&
        now_ms - steer_rear_query_ms_ <= diagnostic_cfg_.steer_loopback_window_ms) {
      steer_rear_query_pending_ = false;
      ++steer_rear_loopback_count_;
      return true;
    }
    return false;
  }

  int64_t feedback_age_ms(bool seen, int64_t feedback_ms, int64_t now_ms) const {
    return seen ? std::max<int64_t>(0, now_ms - feedback_ms) : -1;
  }

  double steer_error_deg(double commanded_rad, double feedback_deg) const {
    const double commanded_deg = commanded_rad * 180.0 / 3.14159265358979323846;
    return std::remainder(commanded_deg - feedback_deg, 360.0);
  }

  uint8_t mode_value(Mode mode) const {
    switch (mode) {
      case Mode::Idle: return msg::ChassisTelemetry::MODE_IDLE;
      case Mode::Teleop: return msg::ChassisTelemetry::MODE_TELEOP;
      case Mode::Auto: return msg::ChassisTelemetry::MODE_AUTO;
    }
    return msg::ChassisTelemetry::MODE_IDLE;
  }

  uint8_t fsm_value(FsmState state) const {
    switch (state) {
      case FsmState::Normal: return msg::ChassisTelemetry::FSM_NORMAL;
      case FsmState::RampDown: return msg::ChassisTelemetry::FSM_RAMP_DOWN;
      case FsmState::Locked: return msg::ChassisTelemetry::FSM_LOCKED;
    }
    return msg::ChassisTelemetry::FSM_LOCKED;
  }

  uint8_t source_value(Mode mode) const {
    if (mode == Mode::Teleop) return msg::ChassisTelemetry::SOURCE_TELEOP;
    if (mode != Mode::Auto) return msg::ChassisTelemetry::SOURCE_NONE;
    switch (active_source_) {
      case ActiveSource::CmdVel: return msg::ChassisTelemetry::SOURCE_CMD_VEL;
      case ActiveSource::WheelCmd: return msg::ChassisTelemetry::SOURCE_WHEEL_COMMAND;
      case ActiveSource::None: return msg::ChassisTelemetry::SOURCE_NONE;
    }
    return msg::ChassisTelemetry::SOURCE_NONE;
  }

  void publish_telemetry(const WheelTargets& requested,
                         bool have_input,
                         Mode mode,
                         FsmState fsm_state) {
    const int64_t now_ms = now_steady_ms();
    msg::ChassisTelemetry out;
    out.header.stamp = now();
    out.header.frame_id = "base_link";
    out.mode = mode_value(mode);
    out.fsm_state = fsm_value(fsm_state);
    out.active_source = source_value(mode);
    out.have_input = have_input;

    out.steer_front_can_id = can_ids_.steer_front;
    out.steer_rear_can_id = can_ids_.steer_rear;
    out.wheel_front_command_can_id = can_ids_.wheel_front;
    out.wheel_rear_command_can_id = can_ids_.wheel_rear;
    out.wheel_front_status_can_id = can_ids_.wheel_front_status;
    out.wheel_rear_status_can_id = can_ids_.wheel_rear_status;

    out.requested_front_steer_rad = requested.theta_front;
    out.requested_rear_steer_rad = requested.theta_rear;
    out.requested_front_speed_mps = requested.v_front;
    out.requested_rear_speed_mps = requested.v_rear;
    out.commanded_front_steer_rad = state_.theta_front;
    out.commanded_rear_steer_rad = state_.theta_rear;
    out.commanded_front_speed_mps = state_.v_front;
    out.commanded_rear_speed_mps = state_.v_rear;

    out.front_wheel_feedback_seen = wheel_front_bringup_.status_seen;
    out.rear_wheel_feedback_seen = wheel_rear_bringup_.status_seen;
    out.front_wheel_feedback_count = wheel_front_feedback_count_;
    out.rear_wheel_feedback_count = wheel_rear_feedback_count_;
    out.front_wheel_feedback_age_ms = feedback_age_ms(
        wheel_front_bringup_.status_seen, wheel_front_bringup_.last_status_ms, now_ms);
    out.rear_wheel_feedback_age_ms = feedback_age_ms(
        wheel_rear_bringup_.status_seen, wheel_rear_bringup_.last_status_ms, now_ms);
    out.front_wheel_status_word = wheel_front_status_.status_word;
    out.rear_wheel_status_word = wheel_rear_status_.status_word;
    out.front_wheel_error_code = wheel_front_status_.error_code;
    out.rear_wheel_error_code = wheel_rear_status_.error_code;
    out.front_wheel_actual_velocity_raw = wheel_front_status_.actual_velocity_raw;
    out.rear_wheel_actual_velocity_raw = wheel_rear_status_.actual_velocity_raw;

    out.front_steer_feedback_seen = steer_front_feedback_seen_;
    out.rear_steer_feedback_seen = steer_rear_feedback_seen_;
    out.front_steer_feedback_count = steer_front_feedback_count_;
    out.rear_steer_feedback_count = steer_rear_feedback_count_;
    out.front_steer_loopback_count = steer_front_loopback_count_;
    out.rear_steer_loopback_count = steer_rear_loopback_count_;
    out.front_steer_feedback_age_ms = feedback_age_ms(
        steer_front_feedback_seen_, steer_front_feedback_ms_, now_ms);
    out.rear_steer_feedback_age_ms = feedback_age_ms(
        steer_rear_feedback_seen_, steer_rear_feedback_ms_, now_ms);
    out.front_steer_feedback_deg = steer_front_fb_deg_;
    out.rear_steer_feedback_deg = steer_rear_fb_deg_;
    out.front_steer_error_deg = steer_error_deg(state_.theta_front, steer_front_fb_deg_);
    out.rear_steer_error_deg = steer_error_deg(state_.theta_rear, steer_rear_fb_deg_);
    out.can_error_frame_count = can_error_frame_count_;

    uint32_t flags = 0;
    if (out.front_wheel_feedback_age_ms < 0 ||
        out.front_wheel_feedback_age_ms > diagnostic_cfg_.wheel_feedback_timeout_ms) {
      flags |= msg::ChassisTelemetry::FAULT_FRONT_WHEEL_FEEDBACK_TIMEOUT;
    }
    if (out.rear_wheel_feedback_age_ms < 0 ||
        out.rear_wheel_feedback_age_ms > diagnostic_cfg_.wheel_feedback_timeout_ms) {
      flags |= msg::ChassisTelemetry::FAULT_REAR_WHEEL_FEEDBACK_TIMEOUT;
    }
    if (wheel_front_status_.fault || wheel_front_status_.error_code != 0) {
      flags |= msg::ChassisTelemetry::FAULT_FRONT_WHEEL_DRIVE;
    }
    if (wheel_rear_status_.fault || wheel_rear_status_.error_code != 0) {
      flags |= msg::ChassisTelemetry::FAULT_REAR_WHEEL_DRIVE;
    }
    if (std::fabs(state_.v_front) >= diagnostic_cfg_.min_motion_command_mps &&
        out.front_wheel_feedback_age_ms >= 0 &&
        out.front_wheel_feedback_age_ms <= diagnostic_cfg_.wheel_feedback_timeout_ms &&
        wheel_front_status_.actual_velocity_raw == 0) {
      flags |= msg::ChassisTelemetry::FAULT_FRONT_WHEEL_NO_MOTION;
    }
    if (std::fabs(state_.v_rear) >= diagnostic_cfg_.min_motion_command_mps &&
        out.rear_wheel_feedback_age_ms >= 0 &&
        out.rear_wheel_feedback_age_ms <= diagnostic_cfg_.wheel_feedback_timeout_ms &&
        wheel_rear_status_.actual_velocity_raw == 0) {
      flags |= msg::ChassisTelemetry::FAULT_REAR_WHEEL_NO_MOTION;
    }
    if (out.front_steer_feedback_age_ms < 0 ||
        out.front_steer_feedback_age_ms > diagnostic_cfg_.steer_feedback_timeout_ms) {
      flags |= msg::ChassisTelemetry::FAULT_FRONT_STEER_FEEDBACK_TIMEOUT;
    } else if (std::fabs(out.front_steer_error_deg) >=
               diagnostic_cfg_.steer_mismatch_deg) {
      flags |= msg::ChassisTelemetry::FAULT_FRONT_STEER_MISMATCH;
    }
    if (out.rear_steer_feedback_age_ms < 0 ||
        out.rear_steer_feedback_age_ms > diagnostic_cfg_.steer_feedback_timeout_ms) {
      flags |= msg::ChassisTelemetry::FAULT_REAR_STEER_FEEDBACK_TIMEOUT;
    } else if (std::fabs(out.rear_steer_error_deg) >=
               diagnostic_cfg_.steer_mismatch_deg) {
      flags |= msg::ChassisTelemetry::FAULT_REAR_STEER_MISMATCH;
    }
    if (can_error_frame_count_ > 0) {
      flags |= msg::ChassisTelemetry::FAULT_CAN_ERROR_FRAME;
    }
    out.diagnostic_flags = flags;
    telemetry_pub_->publish(out);
  }

  void log_wheel_bringup(const char* label,
                         uint32_t wheel_id,
                         uint32_t status_id,
                         const WheelStatus& status,
                         const WheelBringupState& bringup,
                         const WheelBringupDecision& decision,
                         int64_t now_ms,
                         bool can_sender_ready) {
    const long long age_ms = bringup.status_seen
        ? static_cast<long long>(now_ms - bringup.last_status_ms)
        : -1LL;
    RCLCPP_WARN(
        get_logger(),
        "Wheel %s bring-up phase=%s cmd_id=0x%03x status_id=0x%03x "
        "can_sender=%s status_seen=%s status_age_ms=%lld stale=%s "
        "status(R/S/E=%d/%d/%d) sdo_retries=%d",
        label,
        wheel_bringup_phase_name(decision.phase),
        static_cast<unsigned int>(wheel_id),
        static_cast<unsigned int>(status_id),
        can_sender_ready ? "yes" : "no",
        bringup.status_seen ? "yes" : "no",
        age_ms,
        decision.status_stale ? "yes" : "no",
        status.ready ? 1 : 0,
        status.switched_on ? 1 : 0,
        status.enabled ? 1 : 0,
        bringup.sdo_retry_count);
  }

  // -------- ROS interfaces --------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<msg::WheelCommand>::SharedPtr wheel_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr soc_pub_;
  rclcpp::Publisher<msg::ChassisTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lock_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_shutdown_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_faults_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reinitialize_srv_;

  // -------- config --------
  WheelGeometry geom_{};
  WheelLimits wheel_lim_{};
  WheelDirections dirs_{};
  SteerDirections steer_dirs_{};
  CanIds can_ids_{kSteerFrontId, kSteerRearId, kWheelFrontId, kWheelRearId,
                  kWheelFrontStatusId, kWheelRearStatusId, kSdoFrontId,
                  kSdoRearId, 0x01, 0x02};
  uint16_t steer_speed_field_{kSteerSpeedDps};
  WheelBringupConfig bringup_cfg_{true, 1000, 500, 2000};
  std::string can_sender_node_name_{"socket_can_sender"};
  RateLimits rate_lim_{};
  double max_linear_speed_{0.5};
  double max_angular_speed_{0.5};
  double max_steer_angle_{1.5708};
  double singularity_speed_{0.001};
  double loop_hz_{20.0};
  int    input_timeout_ms_{1000};

  struct DiagnosticConfig {
    int64_t wheel_feedback_timeout_ms{500};
    int64_t steer_feedback_timeout_ms{300};
    int64_t steer_loopback_window_ms{50};
    bool expect_tx_loopback{true};
    double min_motion_command_mps{0.05};
    double steer_mismatch_deg{8.0};
  } diagnostic_cfg_;

  struct JoyParams {
    int enable_button, ad_enable_button, emergency_button;
    int left_linear_axis, left_angular_axis;
    int speed_axis, angular_axis;
    int angular_left_button, angular_right_button;
    double left_linear_sign, left_angular_sign;
    double heartbeat_timeout_s;
    double teleop_max_speed;
    double teleop_rotate_omega;
    double teleop_steer_lock;
    double teleop_deadband;
  } joy_{};

  // -------- runtime state --------
  struct SafetyConfig {
    bool feedback_stop_enabled{true};
    bool steer_alignment_enabled{false};
    double steer_alignment_max_error_deg{15.0};
    bool steer_error_stop_enabled{false};
    int64_t steer_error_stop_ms{1000};
    bool hold_steering_on_teleop_stop{false};
  } safety_;
  std::unique_ptr<CommandArbitrator> arbitrator_;
  JoyEnableGate joy_gate_;
  FaultLatch fault_latch_;
  uint32_t invalid_sources_{0};
  bool hardware_monitor_armed_{false};
  bool actuation_ever_started_{false};
  bool recovery_requested_{false};
  int64_t recovery_started_ms_{-1};
  std::string recovery_status_{"not_requested"};
  int64_t steer_mismatch_since_ms_{-1};
  int64_t last_diagnostic_ms_{-1};
  int64_t diagnostic_period_ms_{200};
  std::string hardware_id_{"agv"};
  std::string stop_reason_{"startup_interlock"};
  std::unique_ptr<StateMachine> fsm_;
  Mode mode_{Mode::Idle};
  ActiveSource active_source_{ActiveSource::None};
  std::mutex mu_;

  // Latest cmd_vel / wheel_cmd / joy targets and timestamps.
  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_omega_{0.0};
  WheelTargets wheel_cmd_target_{};
  WheelTargets joy_target_{};
  Clock::time_point last_cmd_vel_t_{};
  Clock::time_point last_wheel_cmd_t_{};
  Clock::time_point last_joy_t_{};

  // Last commanded wheel state (post rate-limit).
  LimiterState state_{0.0, 0.0, 0.0, 0.0};

  // Last decoded feedback.
  WheelStatus wheel_front_status_{};
  WheelStatus wheel_rear_status_{};
  WheelBringupState wheel_front_bringup_{};
  WheelBringupState wheel_rear_bringup_{};
  double steer_front_fb_deg_{0.0};
  double steer_rear_fb_deg_{0.0};
  bool steer_front_feedback_seen_{false};
  bool steer_rear_feedback_seen_{false};
  int64_t steer_front_feedback_ms_{-1};
  int64_t steer_rear_feedback_ms_{-1};
  uint64_t wheel_front_feedback_count_{0};
  uint64_t wheel_rear_feedback_count_{0};
  uint64_t steer_front_feedback_count_{0};
  uint64_t steer_rear_feedback_count_{0};
  uint64_t steer_front_loopback_count_{0};
  uint64_t steer_rear_loopback_count_{0};
  uint64_t can_error_frame_count_{0};
  bool steer_front_query_pending_{false};
  bool steer_rear_query_pending_{false};
  int64_t steer_front_query_ms_{-1};
  int64_t steer_rear_query_ms_{-1};

  bool steering_brought_up_{false};
  bool startup_interlock_released_{false};
  bool shutdown_processed_{false};
  bool shutdown_prepared_{false};
};

}  // namespace agv2_pkg

int main(int argc, char** argv) {
  struct sigaction sa{};
  sa.sa_handler = agv2_pkg::shutdown_signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // Do not let rclcpp invalidate the context before emit_terminator(). The
  // process-level handler above only flips an atomic flag; all ROS work stays
  // in the normal executor thread.
  rclcpp::init(
      argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<agv2_pkg::Agv2Control>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto shutdown_deadline = std::chrono::steady_clock::time_point::max();
  while (rclcpp::ok()) {
    executor.spin_some();
    if (agv2_pkg::g_shutdown_requested.load()) {
      if (shutdown_deadline == std::chrono::steady_clock::time_point::max()) {
        shutdown_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
      } else if (std::chrono::steady_clock::now() >= shutdown_deadline) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
