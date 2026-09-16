// agv2_control_node — single-process driver replacing the agv_pkg command path
// + the steering_adapter intermediate process. See plan file for design notes.

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include "agv2_pkg/can_codec.hpp"
#include "agv2_pkg/fsm.hpp"
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
enum class ActiveSource { None, CmdVel, WheelCmd };

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

    rclcpp::QoS cmd_qos(rclcpp::KeepLast(10));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", cmd_qos,
        std::bind(&Agv2Control::on_cmd_vel, this, std::placeholders::_1));
    wheel_cmd_sub_ = create_subscription<msg::WheelCommand>(
        "wheel_command", cmd_qos,
        std::bind(&Agv2Control::on_wheel_command, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10),
        std::bind(&Agv2Control::on_joy, this, std::placeholders::_1));
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
        "from_can_bus", rclcpp::QoS(1000),
        std::bind(&Agv2Control::on_can, this, std::placeholders::_1));

    can_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS(1000));
    soc_pub_ = create_publisher<std_msgs::msg::Float64>("Battery_SOC_STATE", rclcpp::QoS(10));
    telemetry_pub_ = create_publisher<msg::ChassisTelemetry>(
        "agv2/chassis_telemetry", rclcpp::QoS(100));

    lock_srv_ = create_service<std_srvs::srv::Trigger>(
        "agv2/lock",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          {
            std::lock_guard<std::mutex> lk(mu_);
            fsm_->on_lock_request();
            mode_ = Mode::Idle;
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
            fsm_->on_lock_request();
            mode_ = Mode::Idle;
            state_.v_front = 0.0;
            state_.v_rear = 0.0;
          }

          if (!steering_brought_up_) {
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

          can_pub_->publish(encode_wheel_velocity(
              can_ids_.wheel_front, 0.0, dirs_.front, wheel_lim_));
          can_pub_->publish(encode_wheel_velocity(
              can_ids_.wheel_rear, 0.0, dirs_.rear, wheel_lim_));
          const auto terminator = encode_terminator();
          RCLCPP_INFO(get_logger(),
                      "Preparing terminator id=0x%03x dlc=%u tail=0x%02x.",
                      static_cast<unsigned int>(terminator.id),
                      static_cast<unsigned int>(terminator.dlc),
                      static_cast<unsigned int>(terminator.data[7]));
          can_pub_->publish(terminator);
          const bool acknowledged =
              can_pub_->wait_for_all_acked(std::chrono::milliseconds(500));
          shutdown_prepared_ = true;
          res->success = true;
          res->message = acknowledged
              ? "zero wheel frames and terminator acknowledged by CAN sender"
              : "final frames queued; DDS acknowledgment timed out, verify CAN capture";
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
    if (!steering_brought_up_) {
      RCLCPP_INFO(get_logger(),
                  "Actuation was never brought up; suppressing terminator CAN frame.");
      return;
    }
    can_pub_->publish(encode_terminator());
    const bool acknowledged =
        can_pub_->wait_for_all_acked(std::chrono::milliseconds(500));
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
  }

  // -------- callbacks --------
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    cmd_vx_    = std::clamp(msg->linear.x,  -max_linear_speed_, max_linear_speed_);
    cmd_vy_    = std::clamp(msg->linear.y,  -max_linear_speed_, max_linear_speed_);
    cmd_omega_ = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
    last_cmd_vel_t_ = now_steady();
    if (mode_ == Mode::Auto) fsm_->on_active_input(last_cmd_vel_t_);
  }

  void on_wheel_command(const msg::WheelCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    wheel_cmd_target_.theta_front = std::clamp(msg->front_steer_angle, -max_steer_angle_, max_steer_angle_);
    wheel_cmd_target_.theta_rear  = std::clamp(msg->rear_steer_angle,  -max_steer_angle_, max_steer_angle_);
    wheel_cmd_target_.v_front = std::clamp(msg->front_wheel_speed, -max_linear_speed_, max_linear_speed_);
    wheel_cmd_target_.v_rear  = std::clamp(msg->rear_wheel_speed,  -max_linear_speed_, max_linear_speed_);
    wheel_cmd_target_.singular = false;
    last_wheel_cmd_t_ = now_steady();
    if (mode_ == Mode::Auto) fsm_->on_active_input(last_wheel_cmd_t_);
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

    // Establish a released/pressed baseline first. A button already held while
    // the node starts must not release the startup actuation interlock.
    if (!joy_edges_initialized_) {
      joy_prev_.emergency = emergency_now;
      joy_prev_.enable = enable_now;
      joy_prev_.ad = ad_now;
      joy_edges_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Joy edge baseline established; actuation remains interlocked.");
    } else if (emergency_now && !joy_prev_.emergency) {
      mode_ = Mode::Idle;
      fsm_->on_lock_request();
      RCLCPP_WARN(get_logger(), "Joy: emergency pressed -> Idle/Locked.");
    } else if (enable_now && !joy_prev_.enable) {
      bool axes_neutral = true;
      for (int i = 0; i <= 3 && i < static_cast<int>(j->axes.size()); ++i) {
        if (std::fabs(j->axes[i]) > 1e-3) { axes_neutral = false; break; }
      }
      if (axes_neutral) {
        mode_ = Mode::Teleop;
        release_startup_interlock("Teleop");
        RCLCPP_INFO(get_logger(), "Joy: enable pressed -> Teleop.");
      } else {
        RCLCPP_WARN(get_logger(), "Joy: enable rejected; axes 0-3 not neutral.");
      }
    } else if (ad_now && !joy_prev_.ad) {
      mode_ = Mode::Auto;
      release_startup_interlock("Auto");
      RCLCPP_INFO(get_logger(), "Joy: AD pressed -> Auto.");
    }
    joy_prev_.emergency = emergency_now;
    joy_prev_.enable    = enable_now;
    joy_prev_.ad        = ad_now;

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
    if (mode_ == Mode::Teleop) fsm_->on_active_input(last_joy_t_);
  }

  void on_can(const can_msgs::msg::Frame::SharedPtr f) {
    const int64_t now_ms = now_steady_ms();
    if (f->is_error) {
      ++can_error_frame_count_;
    }
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
    if (shutdown_prepared_) return;

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
          mode_ = Mode::Idle;
          fsm_->on_lock_request();
        }
      }
    }

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

    // Apply command shaping: steering angle passes through, wheel speed slews.
    state_ = step(state_, target, rate_lim_);

    // Emit per-tick CAN traffic.
    emit_can_frames(st == FsmState::Locked, startup_interlock_released_);
    publish_telemetry(requested_target, have_input, mode_now, st);
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
      const bool wc_fresh = fresh(last_wheel_cmd_t_);
      const bool cv_fresh = fresh(last_cmd_vel_t_);

      // Session model: the source that starts a session holds priority until
      // it goes silent for >input_timeout_ms; only then can the other take over.
      if (active_source_ == ActiveSource::WheelCmd && !wc_fresh) {
        active_source_ = ActiveSource::None;
      } else if (active_source_ == ActiveSource::CmdVel && !cv_fresh) {
        active_source_ = ActiveSource::None;
      }
      if (active_source_ == ActiveSource::None) {
        if (wc_fresh && cv_fresh) {
          active_source_ = (last_wheel_cmd_t_ <= last_cmd_vel_t_)
              ? ActiveSource::WheelCmd : ActiveSource::CmdVel;
        } else if (wc_fresh) {
          active_source_ = ActiveSource::WheelCmd;
        } else if (cv_fresh) {
          active_source_ = ActiveSource::CmdVel;
        }
      }

      if (active_source_ == ActiveSource::WheelCmd && wc_fresh) {
        out = wheel_cmd_target_;
        return true;
      }
      if (active_source_ == ActiveSource::CmdVel && cv_fresh) {
        out = solve_ik(cmd_vx_, cmd_vy_, cmd_omega_, geom_,
                       singularity_speed_,
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
    if (!startup_interlock_released ||
        !steer_front_feedback_seen_ || !steer_rear_feedback_seen_ ||
        !can_sender_ready) {
      return;
    }

    // Enable steering once, then wait until the next control tick before
    // issuing the first position target.
    if (!steering_brought_up_) {
      bring_up_steering();
      steering_brought_up_ = true;
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
      const double vf = locked ? 0.0 : state_.v_front;
      const double vr = locked ? 0.0 : state_.v_rear;
      can_pub_->publish(encode_wheel_velocity(can_ids_.wheel_front, vf, dirs_.front, wheel_lim_));
      can_pub_->publish(encode_wheel_velocity(can_ids_.wheel_rear,  vr, dirs_.rear,  wheel_lim_));
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

  uint32_t declare_can_id_parameter(const char* name, uint32_t default_value) {
    const int value = declare_parameter<int>(name, static_cast<int>(default_value));
    if (value < 0) {
      RCLCPP_WARN(get_logger(), "Invalid negative CAN id for %s; using 0x%03x.",
                  name, static_cast<unsigned int>(default_value));
      return default_value;
    }
    return static_cast<uint32_t>(value);
  }

  uint8_t declare_node_id_parameter(const char* name, uint8_t default_value) {
    const int value = declare_parameter<int>(name, static_cast<int>(default_value));
    if (value < 0 || value > 255) {
      RCLCPP_WARN(get_logger(), "Invalid CANopen node id for %s; using 0x%02x.",
                  name, static_cast<unsigned int>(default_value));
      return default_value;
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

  struct JoyPrev { bool emergency{false}, enable{false}, ad{false}; } joy_prev_;

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
  bool joy_edges_initialized_{false};
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
