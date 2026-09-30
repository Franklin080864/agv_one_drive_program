#!/usr/bin/env python3
"""Bounded Trigger client; CLI success means response.success, not transport success.

The readiness mode only inspects the ROS graph. It does not arm the chassis or
claim that a motor, physical E-stop, or CAN delivery has been verified.
"""

import argparse
import math
import os
import sys
import time


def positive_timeout(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError('timeout must be a finite positive number')
    return parsed


def run_client(args, ros, trigger, clock=time.monotonic):
    """Return 0 only on readiness, or an affirmative shutdown response."""
    node = ros.create_node('agv_prepare_shutdown_{}'.format(os.getpid()))
    client = node.create_client(trigger, args.service)
    deadline = clock() + args.timeout
    future = None
    try:
        while ros.ok() and clock() < deadline:
            if client.service_is_ready():
                break
            ros.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - clock())))
        else:
            print('Shutdown service unavailable before timeout: {}'.format(args.service),
                  file=sys.stderr)
            return 3

        if args.wait_ready:
            required_topics = (
                ('publishers', '/from_can_bus', 'socket_can_receiver'),
                ('subscribers', '/to_can_bus', 'socket_can_sender'),
                ('publishers', '/joy', 'joy_node'),
                ('publishers', '/agv2/chassis_telemetry', 'agv2_control_node'),
            )
            missing = []
            while ros.ok() and clock() < deadline:
                missing = []
                for endpoint_kind, topic, expected_node in required_topics:
                    query = (node.get_publishers_info_by_topic
                             if endpoint_kind == 'publishers'
                             else node.get_subscriptions_info_by_topic)
                    if not any(endpoint.node_name == expected_node
                               and endpoint.node_namespace == '/'
                               for endpoint in query(topic)):
                        missing.append('{} {} from /{}'.format(
                            endpoint_kind, topic, expected_node))
                if not missing:
                    print('ROS graph ready: shutdown service, CAN endpoints, Joy, telemetry. '
                          'Keep physical E-stop pressed until hardware checks pass.')
                    return 0
                ros.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - clock())))
            print('ROS graph readiness timed out; missing: {}'.format(', '.join(missing)),
                  file=sys.stderr)
            return 3

        future = client.call_async(trigger.Request())
        ros.spin_until_future_complete(
            node, future, timeout_sec=max(0.0, deadline - clock()))
        if not future.done():
            future.cancel()
            print('Shutdown response timed out; keep tmux and CAN running.', file=sys.stderr)
            return 3
        response = future.result()
        if response is None:
            print('Shutdown service returned no response.', file=sys.stderr)
            return 4
        print('prepare_shutdown: success={} message={}'.format(
            response.success, response.message), flush=True)
        if not response.success:
            print('Controller rejected shutdown; keep tmux and CAN running.', file=sys.stderr)
            return 1
        return 0
    except Exception as exc:  # ROS transport errors must fail closed.
        print('Shutdown/readiness request failed: {}'.format(exc), file=sys.stderr)
        return 4
    finally:
        if future is not None and not future.done():
            future.cancel()
        node.destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service', default='/agv2/prepare_shutdown')
    parser.add_argument('--timeout', type=positive_timeout, default=10.0,
                        help='Total ROS discovery + response budget in seconds')
    parser.add_argument('--wait-ready', action='store_true',
                        help='Only wait for ROS graph endpoints; never call shutdown')
    args = parser.parse_args(argv)
    try:
        import rclpy
        from std_srvs.srv import Trigger
    except ImportError as exc:
        print('ROS environment unavailable: {}. Source setup_agv_env.bash first.'.format(exc),
              file=sys.stderr)
        return 2
    try:
        rclpy.init(args=[])
        return run_client(args, rclpy, Trigger)
    except KeyboardInterrupt:
        print('Request interrupted; keep tmux and CAN running.', file=sys.stderr)
        return 130
    except Exception as exc:
        print('ROS initialization/request failed: {}'.format(exc), file=sys.stderr)
        return 4
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
