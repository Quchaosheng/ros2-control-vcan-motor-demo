# ros2-control-vcan-motor-demo

A ROS 2 Humble differential-drive demo that connects `diff_drive_controller` to two virtual
motors through SocketCAN `vcan0`. It includes a real `hardware_interface::SystemInterface`, ACK and
encoder feedback, watchdog stopping, and deterministic CAN fault injection.

## Architecture

```text
/diffbot_base_controller/cmd_vel
                 |
       diff_drive_controller
                 |
       wheel velocity commands
                 |
       CanMotorHardware::write()
                 |
              vcan0
                 |
         virtual_motor_node
                 |
       ACK + encoder feedback
                 |
        CanMotorHardware::read()
                 |
        /joint_states + /odom
```

Both CAN endpoints use the `ros2_socketcan` C++ sender and receiver APIs directly. There is no ROS
topic bridge between `ros2_control` and SocketCAN.

## Requirements

- WSL2 Ubuntu 22.04
- ROS 2 Humble installed at `/opt/ros/humble`

Install the binary dependencies inside WSL:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros2-socketcan \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-ament-cmake-pytest \
  can-utils
```

## Build

Run from the repository root inside WSL:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
```

## Run

Create `vcan0`. The command is idempotent, but must be run again if the WSL VM restarts:

```bash
bash src/vcan_diffbot_demo/scripts/setup_vcan.sh
```

Start the complete stack:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vcan_diffbot_demo demo.launch.py
```

In another WSL terminal, drive forward with a small turn:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --rate 10 \
  /diffbot_base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
```

Stop the publisher with `Ctrl+C`. The controller command timeout and motor watchdog bring both
wheels back to zero.

Inspect ROS state:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /diffbot_base_controller/odom
```

Inspect the CAN bus:

```bash
candump -L vcan0
```

Normal traffic contains command IDs `101`/`102`, feedback IDs `181`/`182`, and ACK IDs
`281`/`282`.

## CAN Protocol

All frames use classic 11-bit CAN identifiers and little-endian fields.

| Frame | IDs | Bytes |
| --- | --- | --- |
| Velocity command | `0x101`, `0x102` | sequence, flags, velocity in mrad/s, watchdog in ms, reserved |
| Encoder feedback | `0x181`, `0x182` | sequence, status, velocity in mrad/s, signed encoder count |
| ACK | `0x281`, `0x282` | sequence, result, reserved |

Command flags bit 0 enables the motor. Feedback status bits report enabled, watchdog-stopped, and
protocol-fault states. See `can_protocol.hpp` for the exact byte offsets.

## Fault Injection

Faults are disabled by default. Every-N parameters are deterministic so tests are repeatable; zero
disables a fault.

```bash
ros2 launch vcan_diffbot_demo demo.launch.py \
  drop_command_every_n:=5 \
  drop_feedback_every_n:=7 \
  feedback_delay_ms:=50 \
  malformed_feedback_every_n:=11 \
  error_frame_every_n:=13
```

Available controls:

| Argument | Behavior |
| --- | --- |
| `drop_command_every_n` | Drops the Nth motor command and its ACK |
| `drop_feedback_every_n` | Drops the Nth encoder feedback frame |
| `feedback_delay_ms` | Delays feedback without blocking the node timer |
| `malformed_feedback_every_n` | Sends feedback with DLC 7 instead of 8 |
| `error_frame_every_n` | Adds a SocketCAN ERROR frame |
| `spawn_controllers` | Set false for hardware-only diagnostics |

Dropping all feedback demonstrates hardware timeout and automatic stopping:

```bash
ros2 launch vcan_diffbot_demo demo.launch.py \
  drop_feedback_every_n:=1 \
  spawn_controllers:=false
```

## Tests

The suite covers byte-level protocol encoding, motor dynamics, xacro expansion, plugin loading,
raw CAN ACK/feedback/watchdog behavior, malformed and ERROR frames, the full diff-drive loop, and
feedback-timeout stopping.

```bash
bash src/vcan_diffbot_demo/scripts/setup_vcan.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo
colcon test-result --verbose
```
