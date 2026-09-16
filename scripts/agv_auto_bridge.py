#!/usr/bin/env python3
"""Bridge VR teleop Auto mode without modifying agv2_control_node.

Subscribes to /agv/auto_request (std_msgs/Bool) from the VR teleop side.
When enabled, simulates gamepad RT (button 9) to enter Auto and publishes
/joy heartbeats so agv2_control_node stays in Auto while /cmd_vel drives motion.
When disabled, simulates RB (button 7) and calls /agv2/lock.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class AgvAutoBridge(Node):
    def __init__(self) -> None:
        super().__init__("agv_auto_bridge")

        self.declare_parameter("auto_request_topic", "/agv/auto_request")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("auto_button_index", 9)
        self.declare_parameter("emergency_button_index", 7)
        self.declare_parameter("joy_button_count", 12)
        self.declare_parameter("joy_axis_count", 8)
        self.declare_parameter("lock_service_name", "/agv2/lock")
        self.declare_parameter("call_lock_on_disable", True)

        auto_topic = self.get_parameter("auto_request_topic").get_parameter_value().string_value
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self._auto_button = self.get_parameter("auto_button_index").get_parameter_value().integer_value
        self._emergency_button = (
            self.get_parameter("emergency_button_index").get_parameter_value().integer_value
        )
        self._button_count = self.get_parameter("joy_button_count").get_parameter_value().integer_value
        self._axis_count = self.get_parameter("joy_axis_count").get_parameter_value().integer_value
        lock_name = self.get_parameter("lock_service_name").get_parameter_value().string_value
        self._call_lock_on_disable = (
            self.get_parameter("call_lock_on_disable").get_parameter_value().bool_value
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._joy_pub = self.create_publisher(Joy, joy_topic, qos)
        self.create_subscription(Bool, auto_topic, self._on_auto_request, qos)

        self._lock_client = self.create_client(Trigger, lock_name)
        self._auto_enabled = False
        self._pending_neutral_before_auto = False
        self._pending_auto_edge = False
        self._pending_emergency_edge = False

        period_s = 1.0 / max(publish_hz, 1.0)
        self.create_timer(period_s, self._on_timer)

        self.get_logger().info(
            f"Listening on {auto_topic!r}, publishing {joy_topic!r} at {publish_hz:.1f} Hz"
        )

    def _on_auto_request(self, msg: Bool) -> None:
        if msg.data and not self._auto_enabled:
            self._auto_enabled = True
            # The control node requires a post-start rising edge. Send an
            # explicit neutral baseline on the preceding timer tick so Auto
            # still arms even when this bridge is the first /joy publisher.
            self._pending_neutral_before_auto = True
            self._pending_auto_edge = True
            self.get_logger().info("auto_request=true -> arming Auto (RT edge pending)")
        elif not msg.data and self._auto_enabled:
            self._auto_enabled = False
            self._pending_neutral_before_auto = False
            self._pending_auto_edge = False
            self._pending_emergency_edge = True
            self.get_logger().info("auto_request=false -> disarming (emergency/lock pending)")

    def _on_timer(self) -> None:
        if self._pending_emergency_edge:
            self._publish_joy(emergency=True)
            self._pending_emergency_edge = False
            if self._call_lock_on_disable:
                self._call_lock()
            return

        if not self._auto_enabled:
            return

        if self._pending_neutral_before_auto:
            self._publish_joy(heartbeat=True)
            self._pending_neutral_before_auto = False
            return

        if self._pending_auto_edge:
            self._publish_joy(auto_edge=True)
            self._pending_auto_edge = False
            return

        self._publish_joy(heartbeat=True)

    def _make_joy(self) -> Joy:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * self._axis_count
        msg.buttons = [0] * self._button_count
        return msg

    def _publish_joy(self, *, auto_edge: bool = False, emergency: bool = False, heartbeat: bool = False) -> None:
        msg = self._make_joy()
        if auto_edge:
            if 0 <= self._auto_button < len(msg.buttons):
                msg.buttons[self._auto_button] = 1
        elif emergency:
            if 0 <= self._emergency_button < len(msg.buttons):
                msg.buttons[self._emergency_button] = 1
        elif not heartbeat:
            return
        self._joy_pub.publish(msg)

    def _call_lock(self) -> None:
        if not self._lock_client.service_is_ready():
            self.get_logger().warn(
                f"Lock service not ready ({self._lock_client.srv_name}); sent joy emergency only"
            )
            return
        future = self._lock_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_lock_done)

    def _on_lock_done(self, future) -> None:
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"Lock service OK: {response.message}")
            else:
                self.get_logger().warn("Lock service returned failure")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Lock service call failed: {exc}")


def main() -> None:
    rclpy.init()
    node = AgvAutoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
