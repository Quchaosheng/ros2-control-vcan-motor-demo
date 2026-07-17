# ros2_control + vcan Virtual Motor Demo Design

## Goal

Build a runnable ROS 2 Humble demo in WSL2 that drives a differential robot through
`diff_drive_controller`, transports wheel commands and feedback over `vcan0`, and simulates two
CAN motors with acknowledgements, encoder feedback, watchdog stopping, and fault injection.

## Environment

- Host: Windows with WSL2
- Distribution: Ubuntu 22.04
- ROS: ROS 2 Humble from `/opt/ros/humble`
- CAN interface: Linux SocketCAN `vcan0`, created inside WSL
- Build system: `ament_cmake` and `colcon`
- Dependencies: binary Humble packages for `ros2_control`, `ros2_controllers`, and
  `ros2_socketcan`

The source remains in the shared project directory and is built from WSL. The setup script does
not install or configure ROS itself; it only checks dependencies and creates `vcan0`.

## Scope

The first version includes:

- A two-wheel robot description based on `ros2_control_demos/example_2`.
- A `hardware_interface::SystemInterface` plugin with velocity command interfaces and position and
  velocity state interfaces.
- Direct use of `drivers::socketcan::SocketCanSender` and `SocketCanReceiver` in both endpoints.
- A virtual motor ROS node that models two motors and encoder counts.
- `diff_drive_controller` and `joint_state_broadcaster` configuration.
- Command ACK, encoder feedback, receive timeout detection, command watchdog stopping, and safe
  zero commands during deactivation.
- Configurable command/feedback loss, feedback delay, malformed protocol frames, and SocketCAN
  error frames.
- Protocol unit tests and an end-to-end launch test on `vcan0`.

The first version does not implement CANopen, CiA 402, EDS files, CAN FD, physical motor dynamics,
or a custom GUI.

## Package Structure

Use one ROS package, `vcan_diffbot_demo`, because the plugin, simulator, launch files, and robot
description are one small deployable example.

```text
src/vcan_diffbot_demo/
  CMakeLists.txt
  package.xml
  vcan_diffbot_demo.xml
  include/vcan_diffbot_demo/can_motor_hardware.hpp
  include/vcan_diffbot_demo/can_protocol.hpp
  src/can_motor_hardware.cpp
  src/virtual_motor_node.cpp
  config/controllers.yaml
  config/virtual_motor.yaml
  launch/demo.launch.py
  urdf/diffbot.ros2_control.xacro
  urdf/diffbot.urdf.xacro
  scripts/setup_vcan.sh
  test/test_can_protocol.cpp
  test/test_demo_launch.py
```

## Runtime Architecture

```text
/cmd_vel
    |
diff_drive_controller
    |
left/right wheel velocity commands
    |
CanMotorHardwareInterface::write()
    |
SocketCAN command frames on vcan0
    |
VirtualMotorNode
    |
ACK and feedback frames on vcan0
    |
CanMotorHardwareInterface::read()
    |
wheel position/velocity states
    |
joint_state_broadcaster and diff-drive odometry
```

The hardware plugin opens one sender and one receiver. `write()` sends one command frame for each
wheel. `read()` drains available ACK and feedback frames without blocking the controller loop.
The virtual motor uses its own sender and receiver and runs a fixed-rate update timer.

## CAN Protocol

Classic 11-bit CAN identifiers are used.

| Direction | Left ID | Right ID | Purpose |
| --- | ---: | ---: | --- |
| Hardware to motor | `0x101` | `0x102` | Velocity command |
| Motor to hardware | `0x181` | `0x182` | Encoder feedback |
| Motor to hardware | `0x281` | `0x282` | Command ACK |

All multibyte fields use little-endian encoding. Encoding and decoding are explicit byte
operations, not packed-struct casts.

### Velocity Command, DLC 8

| Byte | Field |
| ---: | --- |
| 0 | Sequence number, wrapping `uint8` |
| 1 | Flags: bit 0 is motor enable |
| 2-3 | Target velocity in milliradians per second, signed `int16` |
| 4-5 | Motor command watchdog in milliseconds, unsigned `uint16` |
| 6-7 | Reserved, must be zero |

Velocity is saturated to the `int16` wire range before encoding. Normal demo limits are set lower
in the controller configuration.

### Encoder Feedback, DLC 8

| Byte | Field |
| ---: | --- |
| 0 | Sequence number of the most recently accepted command |
| 1 | Status: bit 0 enabled, bit 1 watchdog stopped, bit 2 protocol fault |
| 2-3 | Measured velocity in milliradians per second, signed `int16` |
| 4-7 | Encoder count, signed `int32` |

The hardware plugin converts encoder counts to wheel radians using a configurable
`encoder_counts_per_revolution` value.

### ACK, DLC 8

| Byte | Field |
| ---: | --- |
| 0 | Accepted command sequence number |
| 1 | Result: `0` accepted, `1` invalid DLC, `2` invalid reserved field |
| 2-7 | Reserved, zero |

Frames with an unexpected identifier or DLC are ignored and counted. A malformed-frame injection
sends a known motor feedback ID with a short DLC so this path is testable.

## Hardware Interface Behavior

`on_init()` validates exactly two joints, velocity command interfaces, position and velocity state
interfaces, CAN interface name, motor IDs, encoder resolution, and timeout values.

`on_activate()` opens SocketCAN, initializes state to zero, and sends zero velocity commands.
`write()` clamps and encodes both wheel commands. `read()` processes feedback and ACK frames and
updates exported state.

If feedback for either wheel is older than `feedback_timeout_ms`, the plugin reports an error and
sends zero commands to both motors. `on_deactivate()` also sends zero commands before closing the
sockets. Socket exceptions are caught, logged, and converted to lifecycle or read/write errors.

## Virtual Motor Behavior

The simulator maintains target velocity, measured velocity, encoder count, last command time, and
last accepted sequence number for each wheel.

At 100 Hz it approaches target velocity using a configurable acceleration limit, integrates the
encoder count, and publishes feedback at 50 Hz. Every valid command receives an ACK. If no command
arrives before the watchdog duration carried in the command, the target becomes zero and the
watchdog status bit is set.

Fault injection is disabled by default and controlled by deterministic parameters:

- `drop_command_every_n`
- `drop_feedback_every_n`
- `feedback_delay_ms`
- `malformed_feedback_every_n`
- `error_frame_every_n`

Integer intervals are used instead of random probabilities so tests are repeatable. A value of
zero disables each fault.

## Launch And Operation

`scripts/setup_vcan.sh` loads the `vcan` kernel module when available, creates `vcan0` if missing,
and brings it up. It is idempotent.

`demo.launch.py` starts:

- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster` spawner
- `diff_drive_controller` spawner
- `virtual_motor_node`

The default launch enables normal operation with fault injection disabled. Fault parameters remain
overridable from the launch command line or YAML file.

## Testing And Acceptance

Protocol unit tests cover positive and negative velocity values, saturation, little-endian layout,
sequence wrapping, feedback decoding, ACK decoding, and invalid DLC rejection.

The launch test creates or reuses `vcan0`, starts the full stack, publishes a forward `/cmd_vel`,
and verifies:

1. Both wheel velocities become positive in `/joint_states`.
2. Odometry advances in the positive x direction.
3. Publishing zero velocity brings both wheel velocities back to zero.
4. Stopping command publication triggers the motor watchdog and returns both wheels to zero.
5. Enabling deterministic feedback loss causes the hardware feedback timeout to be reported.

Manual acceptance uses `candump vcan0` to show command, ACK, and feedback identifiers while the
robot is driven with `ros2 topic pub`.

## Delivery Boundary

The demo is complete when it builds under ROS 2 Humble in WSL2, the tests pass, one launch command
starts the stack, wheel motion is visible in ROS state and odometry, CAN traffic is visible through
`candump`, and the watchdog stops both motors after command loss.
