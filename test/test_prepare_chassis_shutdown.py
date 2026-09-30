"""Offline shutdown contract tests: no ROS daemon, CAN, or hardware commands."""

import argparse
import contextlib
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'prepare_chassis_shutdown.py'
SPEC = importlib.util.spec_from_file_location('prepare_chassis_shutdown', SCRIPT)
CLIENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLIENT)


class ShutdownClientTest(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(
            service='/agv2/prepare_shutdown', timeout=1.0, wait_ready=False)
        self.ros = MagicMock()
        self.ros.ok.return_value = True
        self.node = self.ros.create_node.return_value
        self.client = self.node.create_client.return_value
        self.client.service_is_ready.return_value = True
        self.future = self.client.call_async.return_value
        self.future.done.return_value = True
        self.future.result.return_value = SimpleNamespace(success=True, message='zero sent')
        self.trigger = SimpleNamespace(Request=MagicMock())
        self.now = 0.0
        node_names = {'/from_can_bus': 'socket_can_receiver',
                      '/joy': 'joy_node', '/agv2/chassis_telemetry': 'agv2_control_node'}
        self.node.get_publishers_info_by_topic.side_effect = lambda topic: [
            SimpleNamespace(node_name=node_names[topic], node_namespace='/')]
        self.node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name='socket_can_sender', node_namespace='/')]

    def clock(self):
        self.now += 0.1
        return self.now

    def run_client(self):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            result = CLIENT.run_client(self.args, self.ros, self.trigger, self.clock)
        self.node.destroy_node.assert_called_once()
        return result

    def test_success_requires_affirmative_response(self):
        self.assertEqual(self.run_client(), 0)
        self.client.call_async.assert_called_once()

    def test_transport_success_with_rejected_response_is_failure(self):
        self.future.result.return_value = SimpleNamespace(success=False, message='no CAN sender')
        self.assertEqual(self.run_client(), 1)

    def test_response_timeout_is_failure_and_cancels_future(self):
        self.future.done.return_value = False
        self.assertEqual(self.run_client(), 3)
        self.future.cancel.assert_called()

    def test_service_discovery_timeout_does_not_send_shutdown(self):
        self.client.service_is_ready.return_value = False
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_transport_exception_is_failure(self):
        self.future.result.side_effect = RuntimeError('connection lost')
        self.assertEqual(self.run_client(), 4)

    def test_missing_response_is_failure(self):
        self.future.result.return_value = None
        self.assertEqual(self.run_client(), 4)

    def test_readiness_observes_graph_without_calling_shutdown(self):
        self.args.wait_ready = True
        self.assertEqual(self.run_client(), 0)
        self.client.call_async.assert_not_called()
        self.node.get_subscriptions_info_by_topic.assert_called_with('/to_can_bus')

    def test_readiness_fails_when_can_sender_missing(self):
        self.args.wait_ready = True
        self.node.get_subscriptions_info_by_topic.return_value = []
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_readiness_does_not_mistake_rosbag_for_can_sender(self):
        self.args.wait_ready = True
        self.node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name='rosbag2_recorder', node_namespace='/')]
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_readiness_rejects_unrelated_publisher(self):
        self.args.wait_ready = True
        self.node.get_publishers_info_by_topic.side_effect = lambda topic: [
            SimpleNamespace(node_name='rosbag2_player', node_namespace='/')]
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_readiness_rejects_sender_in_another_namespace(self):
        self.args.wait_ready = True
        self.node.get_subscriptions_info_by_topic.return_value = [
            SimpleNamespace(node_name='socket_can_sender', node_namespace='/another_robot')]
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_ros_shutdown_during_discovery_fails_closed(self):
        self.ros.ok.return_value = False
        self.assertEqual(self.run_client(), 3)
        self.client.call_async.assert_not_called()

    def test_timeout_rejects_unbounded_or_nonpositive_values(self):
        for value in ('nan', 'inf', '-inf', '0', '-1'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                CLIENT.positive_timeout(value)


if __name__ == '__main__':
    unittest.main()
