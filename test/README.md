# Driver regression tests

The C++ tests exercise protocol encoding, kinematics, input ownership, state
transitions, wheel bring-up and command shaping. `test_node_safety.py` starts the
actual ROS 2 node and injects synthetic messages over DDS. It checks:

- A held software emergency button blocks a new Auto-enable edge.
- An inactive command source cannot sustain an expired active command.
- One wheel losing feedback stops both wheels, latches a reported fault, and
  prevents reset until feedback recovers. Reset alone does not re-enable motion.
- A NaN command latches a fault while emitted command state stays finite.
- An invalid publisher may exit before an explicit reset; reset keeps the robot
  idle and a continuing invalid publisher still blocks a new enable edge.
- Explicit transport recovery retries the lost wheel's communication mapping
  using only shutdown controlwords, then requires reset and a new enable edge.
- Waiting for the CAN sender cannot accumulate an invisible speed ramp.
- Preparing shutdown sends zero speed to both wheels and the terminator frame.
- Preparing shutdown reports failure if the CAN sender has disconnected.

## Run on Ubuntu 22.04 / ROS 2 Humble

Run from the package checkout after installing the dependencies in `package.xml`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agv2_pkg --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select agv2_pkg --ctest-args -R '^test_' --output-on-failure
colcon test-result --verbose
python3 -m unittest discover -v -s test -p test_node_safety.py
```

The Python suite forces localhost-only ROS discovery, uses Domain 93 by default,
and gives every case a unique namespace. Override the test domain with
`AGV_TEST_DOMAIN_ID` if 93 is already in use. It creates an in-memory subscriber
named `socket_can_sender`; **it never opens a CAN device**. Each test runs a fresh
driver process and closes its process and ROS context afterward. Readiness and
state transitions use bounded predicates, not fixed startup sleeps. Failure
messages include the node log and latest diagnostics.

The feedback source emits constant healthy wheel status and zero steering angle.
It does not simulate motor dynamics, CAN bus-off behavior, device firmware,
mechanical braking, physical emergency circuits or actual stopping distance.
These remain real-robot acceptance tests. Steering alignment/error-stop options
are deliberately disabled here so the baseline safety suite does not assume
unvalidated steering dynamics.

GitHub Actions runs the build, C++ regression tests and this suite in a Humble
Jammy container on pushes to `main`/`develop` and pull requests. Existing source
formatting lint remains outside this safety regression gate.
