#!/usr/bin/env python3
"""Expose a manual, control-independent service for saving the rosbag snapshot."""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosbag2_interfaces.srv import Snapshot
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ChassisFaultSnapshot(Node):
    """Mark and save the current passive chassis logging buffer."""

    def __init__(self) -> None:
        super().__init__("chassis_fault_snapshot")
        self.declare_parameter("snapshot_service", "/rosbag2_recorder/snapshot")
        self.declare_parameter("output_directory", "")
        self.declare_parameter("event_settle_ms", 200)
        self.declare_parameter("service_timeout_s", 10.0)

        self._snapshot_service = (
            self.get_parameter("snapshot_service").get_parameter_value().string_value
        )
        self._output_directory = (
            self.get_parameter("output_directory").get_parameter_value().string_value
        )
        self._event_settle_s = (
            self.get_parameter("event_settle_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._service_timeout_s = (
            self.get_parameter("service_timeout_s").get_parameter_value().double_value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._event_pub = self.create_publisher(String, "agv2/log_event", 10)
        self._snapshot_client = self.create_client(
            Snapshot,
            self._snapshot_service,
            callback_group=self._callback_group,
        )
        self._save_service = self.create_service(
            Trigger,
            "agv2/save_fault_log",
            self._save,
            callback_group=self._callback_group,
        )
        self._save_lock = threading.Lock()

        self.get_logger().info(
            "Passive fault snapshot service ready: /agv2/save_fault_log -> %s"
            % self._output_directory
        )

    def _save(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if not self._save_lock.acquire(blocking=False):
            response.success = False
            response.message = "a chassis fault snapshot is already in progress"
            return response

        try:
            if not self._snapshot_client.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = (
                    f"snapshot recorder service unavailable: {self._snapshot_service}"
                )
                return response

            event = String()
            event.data = "manual_fault_detected"
            self._event_pub.publish(event)
            time.sleep(max(0.0, self._event_settle_s))

            completed = threading.Event()
            result_holder = {}
            future = self._snapshot_client.call_async(Snapshot.Request())

            def on_done(done_future) -> None:
                try:
                    result_holder["response"] = done_future.result()
                except Exception as exc:  # noqa: BLE001
                    result_holder["exception"] = exc
                completed.set()

            future.add_done_callback(on_done)
            if not completed.wait(timeout=max(0.1, self._service_timeout_s)):
                response.success = False
                response.message = "timed out while saving chassis fault snapshot"
                return response

            if "exception" in result_holder:
                response.success = False
                response.message = f"snapshot failed: {result_holder['exception']}"
                return response

            snapshot_response = result_holder.get("response")
            response.success = bool(
                snapshot_response is not None and snapshot_response.success
            )
            if response.success:
                response.message = f"chassis fault log saved: {self._output_directory}"
                self.get_logger().info(response.message)
            else:
                response.message = "rosbag recorder rejected the snapshot request"
                self.get_logger().error(response.message)
            return response
        finally:
            self._save_lock.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisFaultSnapshot()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
