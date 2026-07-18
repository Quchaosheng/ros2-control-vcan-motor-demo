# Engineering Hardening Design

## Goal

Strengthen the vcan DiffBot demo as a reproducible ROS 2 driver reference without changing its
single-package, end-to-end teaching scope. The work adds continuous integration, complete licensing,
shared runtime configuration, explicit per-motor faults, SocketCAN error policy, standard ROS
diagnostics, and a generic physical-CAN launch path.

## Scope

This change includes:

- a GitHub Actions build and test workflow for ROS 2 Humble on Ubuntu 22.04;
- a root Apache-2.0 license file and a live CI badge;
- launch arguments shared by the hardware plugin and virtual motor;
- configurable left and right CAN node IDs;
- explicit command and feedback loss for one configured motor;
- a tested SocketCAN error classifier and fatal-error safe stop;
- `diagnostic_msgs/DiagnosticArray` output from the hardware plugin;
- a launch switch that omits the virtual motor for a physical `can0` bus;
- physical CAN setup and validation documentation.

This change does not add CANopen, a vendor SDK, a GUI, a custom diagnostics message, automatic bus
recovery, or support for a motor protocol other than the protocol already documented by the demo.

## Architecture

The existing `vcan_diffbot_demo` package remains the only ROS package. `demo.launch.py` is the
runtime configuration boundary: its shared arguments are passed to both Xacro and
`virtual_motor_node`, so a normal launch cannot configure the two endpoints differently by accident.
The virtual motor retains its standalone parameter defaults, but the installed demo launch always
sets the shared parameters explicitly.

Small, header-only policy helpers remain independently testable:

- `can_protocol.hpp` continues to own frame encoding and decoding;
- `hardware_health.hpp` continues to own ACK and feedback freshness state;
- a new `can_error_policy.hpp` classifies SocketCAN error frames without ROS dependencies.

`CanMotorHardware` owns diagnostics because it is the authoritative source for pending ACKs,
feedback freshness, and safe-stop causes. It publishes standard diagnostics directly; a separate
bus monitor would duplicate the protocol state machine and could disagree with the hardware plugin.

## Shared Runtime Configuration

`demo.launch.py` declares these arguments with the existing behavior as defaults:

| Argument | Default | Consumer |
| --- | ---: | --- |
| `can_interface` | `vcan0` | hardware and virtual motor |
| `left_node_id` | `1` | hardware and virtual motor |
| `right_node_id` | `2` | hardware and virtual motor |
| `encoder_counts_per_revolution` | `4096` | hardware and virtual motor |
| `command_watchdog_ms` | `200` | hardware command frames |
| `feedback_timeout_ms` | `500` | hardware health checks |
| `start_virtual_motor` | `true` | launch condition |

The Xacro macro accepts and writes the shared hardware parameters instead of embedding numeric
constants. The virtual motor declares `left_node_id` and `right_node_id`, validates distinct values
from 1 through 127, and derives all command, ACK, and feedback IDs from them. Its encoder resolution
comes from the same launch argument as the hardware plugin.

`virtual_motor.yaml` retains simulator-only defaults such as update rate, feedback rate,
acceleration, and fault injection. Shared values are supplied by launch and are not duplicated in
that file.

## Explicit Fault Injection

Two new integer parameters and launch arguments are added:

- `drop_command_node_id` drops commands for the selected configured node;
- `drop_feedback_node_id` drops feedback for the selected configured node.

The value `0` disables the fault. A nonzero value must equal the configured left or right node ID;
any other value fails node construction with a clear configuration error. Explicit node faults are
combined with the existing every-N faults using logical OR. The every-N controls remain available
for repeatable intermittent loss tests.

The one-sided feedback timeout launch test uses `drop_feedback_node_id:=<right_node_id>` rather than
depending on the iteration order implied by `drop_feedback_every_n:=2`.

## SocketCAN Error Policy

The error classifier returns one of three levels:

- `NONE`: no SocketCAN error bits are present;
- `WARNING`: controller, protocol, ACK, transceiver, arbitration-lost, or generic bus errors;
- `FATAL`: `CAN_ERR_BUSOFF` or `CAN_ERR_TX_TIMEOUT`.

Warnings are recorded in diagnostics and logged without immediately stopping the driver. Existing
ACK and feedback timeouts still stop the hardware if communication does not recover. Fatal errors
immediately set the stop reason, send one disabled-zero command to each motor, publish an ERROR
diagnostic, and return `hardware_interface::return_type::ERROR`.

The policy intentionally does not loop on safe-stop transmission. The two bounded stop frames are
backed by each motor's command watchdog, which remains the final stop mechanism if the bus cannot
carry a new command.

## Diagnostics

The hardware plugin creates a publisher for `/diagnostics` using
`diagnostic_msgs/msg/DiagnosticArray`. Publishing does not require a separate executor because the
plugin only sends messages and does not host subscriptions, services, or timers.

Each array contains three statuses:

1. `vcan_diffbot/can_bus`
2. `vcan_diffbot/left_motor`
3. `vcan_diffbot/right_motor`

The bus status includes:

- `can_interface`;
- `state`: `inactive`, `active`, or `safe_stop`;
- `last_can_error`;
- `stop_reason`;
- `command_watchdog_ms` and `feedback_timeout_ms`.

Each motor status includes:

- `node_id`;
- `feedback_age_ms`;
- `pending_ack_count`;
- `last_ack_status`;
- `wheel_velocity_rad_s`;
- whether a feedback or ACK timeout is active.

Healthy diagnostics are published at most once every 500 ms. A severity change, stop-reason change,
activation, deactivation, or detected fault publishes immediately. Diagnostics use ROS time for the
message stamp and steady time for timeout calculations.

## Continuous Integration And License

`.github/workflows/ci.yml` runs on `ubuntu-22.04` for pushes to `master` and pull requests. It:

1. checks out the repository;
2. installs ROS 2 Humble through the official ROS tooling action;
3. resolves package dependencies with `rosdep`;
4. loads the kernel `vcan` module and creates `vcan0`;
5. builds `vcan_diffbot_demo` with `colcon`;
6. runs the complete package test suite and prints verbose test results.

The workflow badge links to the workflow run page. A root `LICENSE` contains the unmodified Apache
License 2.0 text already declared by `package.xml` and the README badge.

## Physical CAN Path

`start_virtual_motor:=false` prevents `virtual_motor_node` from starting. A generic physical launch
uses:

```bash
ros2 launch vcan_diffbot_demo demo.launch.py \
  can_interface:=can0 \
  start_virtual_motor:=false
```

`docs/hardware-can.md` documents:

- bringing up a SocketCAN interface with an explicit bitrate and restart delay;
- confirming link state and inspecting traffic with `ip -details link` and `candump`;
- verifying the six configured CAN IDs before enabling motion;
- starting with raised wheels and an accessible hardware emergency stop;
- returning the interface to the down state;
- the requirement that the physical motor implements this demo's byte-level protocol.

The repository does not claim electrical validation, real-time bus guarantees, or compatibility
with an arbitrary commercial motor controller.

## Testing

Implementation follows red-green-refactor. New coverage includes:

- unit tests for warning and fatal SocketCAN error classification;
- health tests for pending ACK counts and feedback ages used by diagnostics;
- Xacro tests for every shared hardware parameter;
- virtual motor launch coverage using non-default node IDs;
- explicit one-node command and feedback loss tests;
- a diagnostics launch test covering healthy output and a safe-stop ERROR transition;
- launch-description coverage for `start_virtual_motor:=false` without requiring physical hardware.

The complete WSL verification command remains:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo
colcon test-result --verbose
```

All existing tests must remain green. The final test count may increase; acceptance is zero errors,
zero failures, and zero skipped tests.

## Acceptance Criteria

- A default `vcan0` launch behaves exactly like the current demo.
- Non-default node IDs produce matching command, ACK, and feedback IDs at both endpoints.
- One motor can lose commands or feedback without relying on every-N ordering.
- fatal SocketCAN errors enter the bounded safe-stop path; warnings remain observable.
- `/diagnostics` exposes bus and per-motor health and reports the actual stop reason.
- CI creates `vcan0` and passes the complete test suite on ROS 2 Humble.
- the repository contains a valid Apache-2.0 license file.
- the physical-CAN launch omits the simulator and the documentation states its protocol and safety
  limits clearly.
