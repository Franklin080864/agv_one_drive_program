#!/usr/bin/env python3
"""ROS 2 black-box safety regression tests; no SocketCAN device is opened.

Run after building and sourcing this package:
  python3 -m unittest discover -v -s test -p test_node_safety.py

Each test starts a new driver in an isolated namespace. This process emulates
the SocketCAN sender subscription and feeds synthetic CAN feedback, so tests
exercise the actual node callbacks, timers, DDS interfaces and encoded output.
They do not establish the physical response or stopping distance of a robot.
"""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import uuid

# Set before either process creates its ROS context. CI may explicitly choose a
# domain, while local runs default to one away from the usual production domain.
os.environ["ROS_DOMAIN_ID"] = os.environ.get("AGV_TEST_DOMAIN_ID", "93")
os.environ["ROS_LOCALHOST_ONLY"] = "1"

from ament_index_python.packages import get_package_prefix  # noqa: E402
from agv2_pkg.msg import ChassisTelemetry, WheelCommand  # noqa: E402
from can_msgs.msg import Frame  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


class TestNodeSafety(unittest.TestCase):
    def setUp(self):
        self.namespace = "/agv_safety_" + uuid.uuid4().hex[:10]
        self.context = Context()
        rclpy.init(context=self.context, args=[])
        # The driver identifies the real CAN transport by subscriber node name.
        # This is an in-memory substitute; it never forwards frames to hardware.
        self.node = Node("socket_can_sender", namespace=self.namespace,
                         context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.can_pub = self.node.create_publisher(Frame, "from_can_bus", 1000)
        self.joy_pub = self.node.create_publisher(Joy, "joy", 10)
        self.wheel_pub = self.node.create_publisher(WheelCommand, "wheel_command", 10)
        self.twist_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.telemetry = []
        self.diagnostics = {}
        self.can_frames = []
        self.node.create_subscription(
            ChassisTelemetry, "agv2/chassis_telemetry",
            lambda msg: self.telemetry.append((time.monotonic(), msg)), 100)
        self.node.create_subscription(
            DiagnosticArray, "agv2/diagnostics", self._on_diagnostics, 100)
        self.can_sub = None
        self.reset_client = self.node.create_client(Trigger, "agv2/reset_faults")
        self.recovery_client = self.node.create_client(Trigger, "agv2/reinitialize_transport")
        self.shutdown_client = self.node.create_client(Trigger, "agv2/prepare_shutdown")
        self.buttons = [0] * 10
        self.wheel_command = None
        self.twist_command = None
        self.feedback_ids = {0x181, 0x182, 0x141, 0x142}
        self.last_publish = 0.0
        self.process = None
        self.log = tempfile.TemporaryFile(mode="w+b")

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=7.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
        self.log.close()

    def _on_diagnostics(self, msg):
        for status in msg.status:
            self.diagnostics.update({entry.key: entry.value for entry in status.values})

    def _attach_sender(self):
        self.can_sub = self.node.create_subscription(
            Frame, "to_can_bus",
            lambda msg: self.can_frames.append((time.monotonic(), msg)), 1000)

    def _start(self, sender=True):
        if sender:
            self._attach_sender()
        executable = Path(get_package_prefix("agv2_pkg")) / "lib/agv2_pkg/agv2_control_node"
        self.assertTrue(executable.is_file(), str(executable))
        parameters = {
            "diagnostics.expect_tx_loopback": "false",
            "safety.feedback_stop_enabled": "true",
            "safety.steer_alignment_enabled": "false",
            "safety.steer_error_stop_enabled": "false",
            "chassis.wheel_radius": "0.10",
            "rate_limit.loop_hz": "20.0",
            "rate_limit.max_d_speed_per_step": "0.05",
            "fsm.cmd_watchdog_ms": "200",
            "arbitrator.input_timeout_ms": "1000",
            "diagnostics.wheel_feedback_timeout_ms": "500",
            "diagnostics.steer_feedback_timeout_ms": "300",
        }
        command = [str(executable), "--ros-args", "-r", "__ns:=" + self.namespace]
        for key, value in parameters.items():
            command.extend(["-p", key + ":=" + value])
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self._until(lambda: self.latest is not None and
                    self.latest.front_wheel_feedback_seen and
                    self.latest.rear_wheel_feedback_seen and
                    self.latest.front_steer_feedback_seen and
                    self.latest.rear_steer_feedback_seen and
                    self.joy_pub.get_subscription_count() == 1 and
                    self.reset_client.service_is_ready() and
                    bool(self.diagnostics), timeout=10.0,
                    description="driver discovery, feedback and diagnostic readiness")
        # Receiving multiple telemetry cycles also guarantees a neutral Joy
        # baseline before testing a deliberate enable edge.
        self._observe_ticks(3)

    @property
    def latest(self):
        return self.telemetry[-1][1] if self.telemetry else None

    def _pump(self):
        now = time.monotonic()
        if now - self.last_publish >= 0.02:
            self.last_publish = now
            joy = Joy()
            joy.axes = [0.0] * 8
            joy.buttons = list(self.buttons)
            self.joy_pub.publish(joy)
            for can_id in sorted(self.feedback_ids):
                frame = Frame()
                frame.id = can_id
                frame.dlc = 8
                if can_id in (0x181, 0x182):
                    # Ready/switched-on/enabled, no drive error. Nonzero raw
                    # feedback avoids an unrelated passive no-motion warning.
                    frame.data = list(struct.pack("<HHi", 0x0007, 0, 100))
                else:
                    frame.data = [0x94, 0, 0, 0, 0, 0, 0, 0]
                self.can_pub.publish(frame)
            if self.wheel_command is not None:
                self.wheel_pub.publish(self.wheel_command)
            if self.twist_command is not None:
                self.twist_pub.publish(self.twist_command)
        self.executor.spin_once(timeout_sec=0.01)

    def _failure_context(self):
        self.log.flush()
        self.log.seek(0)
        log_tail = self.log.read().decode("utf-8", errors="replace")[-6000:]
        frame_tail = [(hex(frame.id), bytes(frame.data).hex())
                      for _, frame in self.can_frames[-20:]]
        return ("\nlast telemetry: " + str(self.latest) +
                "\ndiagnostics: " + str(self.diagnostics) +
                "\nlast received CAN frames: " + str(frame_tail) +
                "\ndriver log:\n" + log_tail)

    def _until(self, predicate, timeout=5.0, description="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._pump()
            if self.process is not None and self.process.poll() is not None:
                self.fail("driver exited unexpectedly" + self._failure_context())
            if predicate():
                return
        self.fail("timed out waiting for " + description + self._failure_context())

    def _observe_ticks(self, count, check=None):
        observed = len(self.telemetry)
        self._until(lambda: len(self.telemetry) >= observed + count,
                    timeout=max(3.0, count * 0.2), description="telemetry cycles")
        if check is not None:
            for _, message in self.telemetry[observed:]:
                check(message)

    def _fault_bits(self, name):
        value = self.diagnostics.get(name)
        return int(value, 0) if value is not None else 0

    def _set_wheel_speed(self, speed):
        command = WheelCommand()
        command.front_wheel_speed = speed
        command.rear_wheel_speed = speed
        self.wheel_command = command

    def _enable_auto(self):
        self.buttons[9] = 1
        self._until(lambda: self.latest.mode == ChassisTelemetry.MODE_AUTO,
                    description="explicit Auto enable")
        self.buttons[9] = 0

    def _start_moving(self, speed=0.2):
        self._enable_auto()
        self._set_wheel_speed(speed)
        self._until(lambda: self.latest.active_source == ChassisTelemetry.SOURCE_WHEEL_COMMAND
                    and self.latest.commanded_front_speed_mps >= speed - 0.01
                    and self.latest.commanded_rear_speed_mps >= speed - 0.01,
                    description="wheel-command motion")

    def _assert_stopped(self, message):
        self.assertAlmostEqual(message.commanded_front_speed_mps, 0.0, places=6)
        self.assertAlmostEqual(message.commanded_rear_speed_mps, 0.0, places=6)

    def _wheel_frames(self, after=0.0):
        return [(stamp, frame) for stamp, frame in self.can_frames
                if stamp >= after and frame.id in (0x201, 0x202)
                and frame.dlc == 8 and frame.data[0] == 0x0F
                and frame.data[2] == 0x03]

    @staticmethod
    def _encoded_speed(frame):
        raw = struct.unpack("<i", bytes(frame.data[3:7]))[0]
        return abs(raw) / 10.0 * (2.0 * math.pi * 0.10) / 60.0

    def _service(self, client):
        self._until(client.service_is_ready, description="service availability")
        future = client.call_async(Trigger.Request())
        self._until(future.done, timeout=6.0, description="service response")
        result = future.result()
        self.assertIsNotNone(result)
        return result

    def test_held_emergency_blocks_new_auto_enable(self):
        self._start()
        self._start_moving()
        self.buttons[7] = 1
        self._until(lambda: self.latest.mode == ChassisTelemetry.MODE_IDLE and
                    self.latest.commanded_front_speed_mps == 0.0,
                    description="emergency stop")
        self.buttons[9] = 1

        def check(message):
            self._assert_stopped(message)
            self.assertEqual(message.mode, ChassisTelemetry.MODE_IDLE)

        self._observe_ticks(8, check)

    def test_inactive_input_does_not_refresh_selected_watchdog(self):
        self._start()
        self._start_moving()
        self.wheel_command = None
        self.twist_command = Twist()  # Inactive publisher continues at 50 Hz.
        # 200 ms freshness + 200 ms ramp + scheduler margin, below the
        # 1000 ms ownership timeout that hid this defect in the old node.
        self._until(lambda: self.latest.commanded_front_speed_mps == 0.0 and
                    self.latest.commanded_rear_speed_mps == 0.0,
                    timeout=0.85, description="selected-source watchdog stop")
        self._observe_ticks(4, self._assert_stopped)

    def test_single_wheel_feedback_loss_latches_and_requires_healthy_reset(self):
        self._start()
        self._start_moving()
        self.feedback_ids.remove(0x181)
        self._until(lambda: self._fault_bits("latched_faults") != 0 and
                    self.latest.mode == ChassisTelemetry.MODE_IDLE,
                    description="front feedback fault latch")
        self._assert_stopped(self.latest)
        response = self._service(self.reset_client)
        self.assertFalse(response.success, response.message)
        stopped_at = time.monotonic()
        self._observe_ticks(4, self._assert_stopped)
        self.assertTrue(any(frame.id == 0x202 and self._encoded_speed(frame) == 0.0
                            for _, frame in self._wheel_frames(stopped_at)),
                        "healthy rear wheel must receive an explicit zero target")

        self.feedback_ids.add(0x181)
        self._until(lambda: self.latest.front_wheel_feedback_age_ms < 100 and
                    self._fault_bits("active_faults") == 0,
                    description="feedback recovery")
        self.assertNotEqual(self._fault_bits("latched_faults"), 0)
        response = self._service(self.reset_client)
        self.assertTrue(response.success, response.message)
        self._until(lambda: self._fault_bits("latched_faults") == 0,
                    description="fault acknowledgement")
        # Commands are still arriving; reset alone must not grant motion.
        self._observe_ticks(6, self._assert_stopped)
        self._enable_auto()
        self._until(lambda: self.latest.commanded_front_speed_mps > 0.0,
                    description="motion after explicit re-enable")

    def test_nan_input_latches_without_poisoning_output(self):
        self._start()
        self._start_moving()
        self._set_wheel_speed(float("nan"))
        self._until(lambda: self._fault_bits("latched_faults") != 0 and
                    self.latest.mode == ChassisTelemetry.MODE_IDLE,
                    description="invalid-command fault latch")
        self._set_wheel_speed(0.0)

        def check(message):
            self._assert_stopped(message)
            for value in (message.commanded_front_steer_rad,
                          message.commanded_rear_steer_rad,
                          message.commanded_front_speed_mps,
                          message.commanded_rear_speed_mps):
                self.assertTrue(math.isfinite(value))

        self._observe_ticks(8, check)
        self.assertNotEqual(self._fault_bits("latched_faults"), 0)

    def test_transport_recovery_reconfigures_without_enabling_motion(self):
        self._start()
        self._start_moving()
        self.feedback_ids.remove(0x181)
        self._until(lambda: self._fault_bits("latched_faults") != 0 and
                    self.latest.mode == ChassisTelemetry.MODE_IDLE,
                    description="feedback loss before explicit transport recovery")
        request_index = len(self.can_frames)
        response = self._service(self.recovery_client)
        self.assertTrue(response.success, response.message)
        self._until(lambda: any(frame.id == 0x601
                                for _, frame in self.can_frames[request_index:]),
                    description="front wheel communication mapping retry")
        # Use the first recovery SDO as an ordered CAN-stream boundary. Service
        # replies and CAN samples travel on different DDS writers; old zero
        # frames could otherwise arrive after the service reply in this process.
        first_sdo = next(index for index in range(request_index, len(self.can_frames))
                         if self.can_frames[index][1].id == 0x601)
        self._observe_ticks(5, self._assert_stopped)
        recovery_frames = [frame for _, frame in self.can_frames[first_sdo:]
                           if frame.id in (0x201, 0x202)]
        self.assertEqual({frame.id for frame in recovery_frames}, {0x201, 0x202})
        for frame in recovery_frames:
            self.assertEqual(frame.data[0], 0x06,
                             "transport recovery must not switch on or enable drives")
            self.assertEqual(self._encoded_speed(frame), 0.0)
        self.assertNotEqual(self._fault_bits("latched_faults") & 2048, 0)

        self.feedback_ids.add(0x181)
        self._until(lambda: self.diagnostics.get("transport_recovery") ==
                    "feedback_received_reset_required" and
                    self._fault_bits("active_faults") == 0,
                    description="recovered feedback requiring explicit reset")
        self.assertNotEqual(self._fault_bits("latched_faults"), 0)
        response = self._service(self.reset_client)
        self.assertTrue(response.success, response.message)
        self._until(lambda: self._fault_bits("latched_faults") == 0,
                    description="explicit acknowledgement after transport recovery")
        # The old command publisher keeps sending; recovery/reset must leave
        # the robot stationary until an independent new operator enable edge.
        self._observe_ticks(6, self._assert_stopped)
        self.assertEqual(self.latest.mode, ChassisTelemetry.MODE_IDLE)

    def test_nan_publisher_can_exit_before_reset_and_bad_input_blocks_reenable(self):
        self._start()
        self._start_moving()
        self._set_wheel_speed(float("nan"))
        self._until(lambda: self._fault_bits("latched_faults") != 0 and
                    self.latest.mode == ChassisTelemetry.MODE_IDLE,
                    description="invalid command latched before publisher exits")
        self.wheel_command = None
        self._observe_ticks(4, self._assert_stopped)
        # No valid replacement command is sent. A deliberate reset must be
        # sufficient to discard the abandoned invalid command session.
        response = self._service(self.reset_client)
        self.assertTrue(response.success, response.message)
        self._until(lambda: self._fault_bits("latched_faults") == 0 and
                    self.diagnostics.get("invalid_input_sources") == "0",
                    description="reset discards exited invalid publisher session")
        self._observe_ticks(5, self._assert_stopped)
        self.assertEqual(self.latest.mode, ChassisTelemetry.MODE_IDLE)

        self._set_wheel_speed(float("nan"))
        self._until(lambda: int(self.diagnostics.get("invalid_input_sources", "0")) != 0,
                    description="continuing malformed publisher is rejected again")
        self.buttons[9] = 1

        def check(message):
            self._assert_stopped(message)
            self.assertEqual(message.mode, ChassisTelemetry.MODE_IDLE)

        self._observe_ticks(8, check)

    def test_waiting_for_sender_does_not_accumulate_speed_ramp(self):
        self._start(sender=False)
        self._enable_auto()
        self._set_wheel_speed(0.5)
        self._observe_ticks(14, self._assert_stopped)
        self._attach_sender()
        self._until(lambda: all(any(frame.id == can_id and
                                    self._encoded_speed(frame) > 0.0
                                    for _, frame in self._wheel_frames())
                                for can_id in (0x201, 0x202)),
                    description="first motion after CAN sender becomes ready")
        first_by_id = {}
        for _, frame in self._wheel_frames():
            if self._encoded_speed(frame) > 0.0:
                first_by_id.setdefault(frame.id, self._encoded_speed(frame))
        self.assertEqual(set(first_by_id), {0x201, 0x202})
        for speed in first_by_id.values():
            self.assertLessEqual(speed, 0.08,
                                 "first emitted speed must begin at the ramp origin")

    def test_prepare_shutdown_sends_zero_and_terminator(self):
        self._start()
        self._start_moving()
        start = time.monotonic()
        response = self._service(self.shutdown_client)
        self.assertTrue(response.success, response.message + self._failure_context())
        self._until(lambda: any(stamp >= start and frame.id == 0x001 and
                                bytes(frame.data) == bytes([0xFF] * 7 + [0xFD])
                                for stamp, frame in self.can_frames) and
                    all(any(frame.id == can_id and self._encoded_speed(frame) == 0.0
                            for _, frame in self._wheel_frames(start))
                        for can_id in (0x201, 0x202)),
                    description="shutdown zero targets and terminator")

    def test_prepare_shutdown_reports_sender_loss(self):
        self._start()
        self._start_moving()
        self.node.destroy_subscription(self.can_sub)
        self.can_sub = None
        # Wait for the driver's own graph observation, rather than assuming
        # local subscription destruction is instantly visible to its process.
        self._until(lambda: self.diagnostics.get("can_sender_ready") == "false",
                    description="driver observes CAN sender disconnection")
        response = self._service(self.shutdown_client)
        self.assertFalse(response.success, response.message)


if __name__ == "__main__":
    unittest.main(verbosity=2)
