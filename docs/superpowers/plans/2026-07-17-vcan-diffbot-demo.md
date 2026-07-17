# vcan DiffBot Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a ROS 2 Humble differential-drive demo whose ros2_control hardware plugin exchanges wheel commands, ACKs, and encoder feedback with a virtual motor over WSL2 `vcan0`.

**Architecture:** A single `vcan_diffbot_demo` package owns the SystemInterface plugin, a virtual motor executable, the byte-level CAN protocol, robot/controller configuration, launch files, and tests. Both CAN endpoints use the Humble `ros2_socketcan` C++ API directly; deterministic fault-injection parameters exercise malformed frames, loss, delay, and CAN error frames without introducing a ROS-topic bridge.

**Tech Stack:** ROS 2 Humble, ros2_control, ros2_controllers, ros2_socketcan, C++17, Python launch, ament_cmake, GoogleTest, launch_testing, SocketCAN/vcan, can-utils.

---

## File Map

```text
README.md                                      Build, launch, drive, inspect, and fault-test commands
src/vcan_diffbot_demo/CMakeLists.txt           Targets, installs, exports, and tests
src/vcan_diffbot_demo/package.xml              ROS package dependencies
src/vcan_diffbot_demo/vcan_diffbot_demo.xml    ros2_control plugin registration
src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_protocol.hpp
                                                CAN IDs, frame types, byte encoding, and decoding
src/vcan_diffbot_demo/include/vcan_diffbot_demo/motor_state.hpp
                                                Deterministic motor/watchdog/encoder state model
src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp
                                                SystemInterface declaration and state
src/vcan_diffbot_demo/src/can_motor_hardware.cpp SystemInterface lifecycle and CAN I/O
src/vcan_diffbot_demo/src/virtual_motor_node.cpp  Two virtual motors, timers, CAN I/O, fault injection
src/vcan_diffbot_demo/config/controllers.yaml     Controller manager and DiffDriveController parameters
src/vcan_diffbot_demo/config/virtual_motor.yaml   Simulator defaults and fault injection switches
src/vcan_diffbot_demo/launch/demo.launch.py       Full-stack launch
src/vcan_diffbot_demo/urdf/diffbot.ros2_control.xacro
                                                Hardware plugin and joint interfaces
src/vcan_diffbot_demo/urdf/diffbot.urdf.xacro     Minimal visible differential robot
src/vcan_diffbot_demo/scripts/setup_vcan.sh       Idempotent WSL vcan0 setup
src/vcan_diffbot_demo/test/test_can_protocol.cpp  Wire format unit tests
src/vcan_diffbot_demo/test/test_motor_state.cpp   Dynamics, encoder, and watchdog unit tests
src/vcan_diffbot_demo/test/test_urdf_xacro.py     Robot description validation
src/vcan_diffbot_demo/test/test_demo_launch.py    Full-stack launch and motion test
```

## Task 1: Create The ROS Package And WSL Setup

**Files:**
- Create: `src/vcan_diffbot_demo/CMakeLists.txt`
- Create: `src/vcan_diffbot_demo/package.xml`
- Create: `src/vcan_diffbot_demo/scripts/setup_vcan.sh`
- Create: `README.md`

- [ ] **Step 1: Create the minimal package manifest**

Declare build dependencies for `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, and
`ros2_socketcan`; runtime dependencies for controller manager, controller packages,
`robot_state_publisher`, `xacro`, and launch; test dependencies for GTest, pytest, and launch testing.

```xml
<package format="3">
  <name>vcan_diffbot_demo</name>
  <version>0.1.0</version>
  <description>ros2_control differential drive demo backed by SocketCAN vcan motors.</description>
  <maintainer email="maintainer@example.com">Project Maintainer</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>ros2_socketcan</depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: Create an idempotent `vcan0` setup script**

```bash
#!/usr/bin/env bash
set -euo pipefail

interface="${1:-vcan0}"
sudo modprobe vcan 2>/dev/null || true
if ! ip link show "$interface" >/dev/null 2>&1; then
  sudo ip link add dev "$interface" type vcan
fi
sudo ip link set up "$interface"
ip -details link show "$interface"
```

- [ ] **Step 3: Add the initial CMake shell**

Create an `ament_cmake` project using C++17, warnings, dependency lookup, resource installation, and
an empty testing block that later tasks fill.

```cmake
cmake_minimum_required(VERSION 3.16)
project(vcan_diffbot_demo LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_socketcan REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS scripts/setup_vcan.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
endif()

ament_package()
```

- [ ] **Step 4: Install binary dependencies and verify the empty package configures**

Run:

```bash
wsl.exe -d Ubuntu-22.04 -- bash -lc '
  source /opt/ros/humble/setup.bash
  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-ros2-socketcan ros-humble-xacro \
    ros-humble-robot-state-publisher can-utils
  cd "/mnt/c/Users/qucha/Documents/New project"
  colcon build --packages-select vcan_diffbot_demo
'
```

Expected: package configures and installs without targets.

- [ ] **Step 5: Commit**

```bash
git add README.md src/vcan_diffbot_demo
git commit -m "build: scaffold vcan diffbot demo"
```

## Task 2: Implement The CAN Protocol With Unit Tests

**Files:**
- Create: `src/vcan_diffbot_demo/test/test_can_protocol.cpp`
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_protocol.hpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing protocol tests**

Test exact bytes rather than round trips alone, so an encoder and decoder cannot share the same
endianness bug.

```cpp
#include <gtest/gtest.h>
#include "vcan_diffbot_demo/can_protocol.hpp"

using namespace vcan_diffbot_demo::protocol;

TEST(CanProtocol, EncodesNegativeVelocityLittleEndian)
{
  const auto frame = encode_command(0x2a, true, -1234, 200);
  EXPECT_EQ(frame, (FrameData{0x2a, 0x01, 0x2e, 0xfb, 0xc8, 0x00, 0x00, 0x00}));
}

TEST(CanProtocol, SaturatesVelocity)
{
  EXPECT_EQ(command_velocity_mrad(encode_command(1, true, 40000, 200)), 32767);
  EXPECT_EQ(command_velocity_mrad(encode_command(1, true, -40000, 200)), -32768);
}

TEST(CanProtocol, DecodesFeedback)
{
  const FrameData bytes{7, 3, 0xd2, 0x04, 0x40, 0xe2, 0x01, 0x00};
  const auto feedback = decode_feedback(bytes);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->sequence, 7);
  EXPECT_EQ(feedback->velocity_mrad_s, 1234);
  EXPECT_EQ(feedback->encoder_count, 123456);
}

TEST(CanProtocol, RejectsWrongDlc)
{
  EXPECT_FALSE(decode_feedback(FrameData{}, 7).has_value());
  EXPECT_FALSE(decode_ack(FrameData{}, 6).has_value());
}
```

- [ ] **Step 2: Register and run the test to verify failure**

Add:

```cmake
ament_add_gtest(test_can_protocol test/test_can_protocol.cpp)
target_include_directories(test_can_protocol PRIVATE include)
```

Run:

```bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_can_protocol
```

Expected: FAIL because `can_protocol.hpp` does not exist.

- [ ] **Step 3: Implement the header-only protocol**

Define `FrameData`, the three ID bases, `Command`, `Feedback`, `Ack`, explicit `put_u16`, `put_i16`,
`put_i32`, matching getters, and encode/decode functions. The public surface is:

```cpp
namespace vcan_diffbot_demo::protocol {
using FrameData = std::array<uint8_t, 8>;
constexpr uint32_t command_id(uint8_t node) { return 0x100U + node; }
constexpr uint32_t feedback_id(uint8_t node) { return 0x180U + node; }
constexpr uint32_t ack_id(uint8_t node) { return 0x280U + node; }

struct Command { uint8_t sequence; bool enabled; int16_t velocity_mrad_s; uint16_t watchdog_ms; };
struct Feedback { uint8_t sequence; uint8_t status; int16_t velocity_mrad_s; int32_t encoder_count; };
struct Ack { uint8_t sequence; uint8_t result; };

FrameData encode_command(uint8_t sequence, bool enabled, int32_t velocity_mrad_s,
                         uint16_t watchdog_ms);
std::optional<Command> decode_command(const FrameData & data, std::size_t dlc = 8);
FrameData encode_feedback(const Feedback & feedback);
std::optional<Feedback> decode_feedback(const FrameData & data, std::size_t dlc = 8);
FrameData encode_ack(const Ack & ack);
std::optional<Ack> decode_ack(const FrameData & data, std::size_t dlc = 8);
int16_t command_velocity_mrad(const FrameData & data);
}  // namespace vcan_diffbot_demo::protocol
```

- [ ] **Step 4: Run the protocol tests**

Run:

```bash
colcon build --packages-select vcan_diffbot_demo
colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_can_protocol
colcon test-result --verbose
```

Expected: all protocol tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: add virtual motor CAN protocol"
```

## Task 3: Implement The Deterministic Motor Model With Unit Tests

**Files:**
- Create: `src/vcan_diffbot_demo/test/test_motor_state.cpp`
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/motor_state.hpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing state-model tests**

```cpp
#include <gtest/gtest.h>
#include "vcan_diffbot_demo/motor_state.hpp"

using vcan_diffbot_demo::MotorState;

TEST(MotorState, AppliesAccelerationLimitAndIntegratesEncoder)
{
  MotorState motor(4096, 20.0);
  motor.accept_command(5, 10.0, std::chrono::milliseconds(200));
  motor.update(0.1, std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(motor.velocity_rad_s(), 2.0);
  EXPECT_NEAR(motor.position_rad(), 0.2, 1e-9);
  EXPECT_EQ(motor.last_sequence(), 5);
}

TEST(MotorState, WatchdogStopsTarget)
{
  MotorState motor(4096, 100.0);
  motor.accept_command(1, 5.0, std::chrono::milliseconds(200));
  motor.update(0.1, std::chrono::milliseconds(250));
  EXPECT_TRUE(motor.watchdog_stopped());
  EXPECT_DOUBLE_EQ(motor.target_velocity_rad_s(), 0.0);
}
```

- [ ] **Step 2: Register and run the test to verify failure**

```cmake
ament_add_gtest(test_motor_state test/test_motor_state.cpp)
target_include_directories(test_motor_state PRIVATE include)
```

Expected: FAIL because `motor_state.hpp` does not exist.

- [ ] **Step 3: Implement `MotorState`**

The class stores encoder resolution, acceleration limit, target/measured velocity, integrated
position, elapsed time since command, sequence, enabled state, and watchdog state. `update()` clamps
the velocity delta to `max_acceleration_rad_s2 * dt`, integrates position, and derives encoder count
with `llround(position / (2*pi) * counts_per_revolution)`.

```cpp
class MotorState {
public:
  MotorState(int32_t counts_per_revolution, double max_acceleration_rad_s2);
  void accept_command(uint8_t sequence, double velocity_rad_s,
                      std::chrono::milliseconds watchdog);
  void disable(uint8_t sequence, std::chrono::milliseconds watchdog);
  void update(double dt_seconds, std::chrono::milliseconds since_last_command);
  double target_velocity_rad_s() const;
  double velocity_rad_s() const;
  double position_rad() const;
  int32_t encoder_count() const;
  uint8_t last_sequence() const;
  uint8_t status() const;
  bool watchdog_stopped() const;
};
```

- [ ] **Step 4: Run both unit-test targets**

```bash
colcon build --packages-select vcan_diffbot_demo
colcon test --packages-select vcan_diffbot_demo --ctest-args -R 'test_(can_protocol|motor_state)'
colcon test-result --verbose
```

Expected: both targets pass.

- [ ] **Step 5: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: add deterministic virtual motor model"
```

## Task 4: Add Robot Description And ros2_control Plugin Configuration

**Files:**
- Create: `src/vcan_diffbot_demo/vcan_diffbot_demo.xml`
- Create: `src/vcan_diffbot_demo/urdf/diffbot.ros2_control.xacro`
- Create: `src/vcan_diffbot_demo/urdf/diffbot.urdf.xacro`
- Create: `src/vcan_diffbot_demo/test/test_urdf_xacro.py`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write the failing xacro test**

```python
import os
import subprocess
from ament_index_python.packages import get_package_share_directory


def test_diffbot_xacro_contains_plugin_and_two_wheels():
    share = get_package_share_directory("vcan_diffbot_demo")
    path = os.path.join(share, "urdf", "diffbot.urdf.xacro")
    xml = subprocess.check_output(["xacro", path], text=True)
    assert "vcan_diffbot_demo/CanMotorHardware" in xml
    assert 'joint name="left_wheel_joint"' in xml
    assert 'joint name="right_wheel_joint"' in xml
    assert '<param name="can_interface">vcan0</param>' in xml
```

- [ ] **Step 2: Register and run the test to verify failure**

```cmake
ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py)
```

Expected: FAIL because the xacro files are absent.

- [ ] **Step 3: Create the plugin descriptor and ros2_control xacro**

Register `vcan_diffbot_demo::CanMotorHardware` and configure these hardware parameters:

```xml
<hardware>
  <plugin>vcan_diffbot_demo/CanMotorHardware</plugin>
  <param name="can_interface">vcan0</param>
  <param name="left_node_id">1</param>
  <param name="right_node_id">2</param>
  <param name="encoder_counts_per_revolution">4096</param>
  <param name="command_watchdog_ms">200</param>
  <param name="feedback_timeout_ms">500</param>
</hardware>
```

Each wheel declares velocity command plus position and velocity state interfaces.

- [ ] **Step 4: Create a self-contained DiffBot URDF**

Use `base_link`, two cylindrical wheels, fixed caster geometry, continuous wheel joints, and the
ros2_control macro. Set wheel separation to `0.40 m` and wheel radius to `0.10 m`; these same values
must appear in controller configuration.

- [ ] **Step 5: Run the xacro test**

```bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_urdf_xacro
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: add vcan diffbot robot description"
```

## Task 5: Implement The ros2_control Hardware Plugin

**Files:**
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp`
- Create: `src/vcan_diffbot_demo/src/can_motor_hardware.cpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Extend the xacro test to require all hardware parameters**

Add assertions for node IDs, encoder resolution, watchdog, and feedback timeout, then run it before
the plugin exists. Expected: xacro test remains green; the next build step fails until the plugin
target is added.

- [ ] **Step 2: Declare the SystemInterface**

```cpp
class CanMotorHardware final : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;
};
```

Store two-element command, velocity, position, encoder, sequence, ACK time, and feedback time arrays;
the parsed parameters; and unique pointers to `SocketCanSender` and `SocketCanReceiver`.

- [ ] **Step 3: Implement lifecycle validation and interface export**

Validate exactly two joints and the expected interface order. Parse positive numeric parameters
with clear fatal logs. `on_activate()` opens `vcan0`, installs filters for `0x181`, `0x182`, `0x281`,
`0x282`, initializes timestamps, and sends zero commands. `on_deactivate()` sends zero commands and
releases sockets.

- [ ] **Step 4: Implement nonblocking `read()`**

Call receiver `receive()` with zero timeout in a bounded loop. Catch `SocketCanTimeout` to finish
the drain normally. Decode feedback by ID and update:

```cpp
velocity_rad_s = static_cast<double>(feedback.velocity_mrad_s) / 1000.0;
position_rad = static_cast<double>(feedback.encoder_count) * 2.0 * std::numbers::pi /
               encoder_counts_per_revolution;
```

Since C++17 has no `std::numbers`, define a local `constexpr double kTwoPi`. Reject invalid DLC and
log it at throttled warning level. When either wheel exceeds `feedback_timeout_ms`, send zeros once
and return `hardware_interface::return_type::ERROR`.

- [ ] **Step 5: Implement `write()`**

Convert rad/s to millirad/s with `llround`, let the protocol encoder saturate, increment the wheel's
sequence, and send one command frame per wheel with a 1 ms send timeout. Catch send exceptions,
attempt the two zero commands, and return `ERROR`.

- [ ] **Step 6: Build and inspect plugin registration**

Add the shared library target, dependencies, includes, install/export rules, and:

```cmake
pluginlib_export_plugin_description_file(hardware_interface vcan_diffbot_demo.xml)
```

Run:

```bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
ros2 pkg plugins --attrib=plugin hardware_interface | grep vcan_diffbot_demo
```

Expected: `vcan_diffbot_demo/CanMotorHardware` is listed.

- [ ] **Step 7: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: add SocketCAN ros2_control hardware"
```

## Task 6: Implement The Virtual Motor Node And Controllers

**Files:**
- Create: `src/vcan_diffbot_demo/src/virtual_motor_node.cpp`
- Create: `src/vcan_diffbot_demo/config/virtual_motor.yaml`
- Create: `src/vcan_diffbot_demo/config/controllers.yaml`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Create default configuration**

```yaml
virtual_motor:
  ros__parameters:
    can_interface: vcan0
    encoder_counts_per_revolution: 4096
    max_acceleration_rad_s2: 20.0
    update_rate_hz: 100.0
    feedback_rate_hz: 50.0
    drop_command_every_n: 0
    drop_feedback_every_n: 0
    feedback_delay_ms: 0
    malformed_feedback_every_n: 0
    error_frame_every_n: 0
```

Controller manager runs at 50 Hz. `diffbot_base_controller` uses closed-loop odometry,
`wheel_separation: 0.40`, `wheel_radius: 0.10`, `cmd_vel_timeout: 0.5`, and conservative linear and
angular limits.

- [ ] **Step 2: Implement CAN receive and ACK handling**

The node owns two `MotorState` objects. A receive thread filters command IDs, decodes frames,
increments deterministic counters, optionally drops the command, accepts or disables the motor,
and immediately sends:

```cpp
protocol::encode_ack({command.sequence, result})
```

Protect motor state with one mutex because the receive thread and ROS timer share the two small
objects.

- [ ] **Step 3: Implement update and feedback timers**

At 100 Hz, compute elapsed steady-clock time, update both motors, and queue feedback frames. At
50 Hz, send or delay feedback according to parameters. A delayed frame is stored with a steady
deadline and flushed by the update timer.

- [ ] **Step 4: Implement deterministic faults**

For every-N counters:

- command loss skips command application and ACK;
- feedback loss skips the frame;
- malformed feedback sends only 7 bytes under a normal feedback ID;
- error frame uses `CanId(id, 0, FrameType::ERROR, StandardFrame)`;
- delay queues a valid frame rather than sleeping a ROS callback.

- [ ] **Step 5: Build and run the node against `vcan0`**

```bash
./src/vcan_diffbot_demo/scripts/setup_vcan.sh
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
ros2 run vcan_diffbot_demo virtual_motor_node --ros-args \
  --params-file src/vcan_diffbot_demo/config/virtual_motor.yaml
```

Expected: node starts and waits for command IDs without throwing.

- [ ] **Step 6: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: add configurable virtual CAN motors"
```

## Task 7: Add Full-Stack Launch And A Failing Integration Test

**Files:**
- Create: `src/vcan_diffbot_demo/launch/demo.launch.py`
- Create: `src/vcan_diffbot_demo/test/test_demo_launch.py`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write the launch test first**

Launch `demo.launch.py`, wait for controllers, publish `geometry_msgs/msg/TwistStamped` to
`/diffbot_base_controller/cmd_vel`, subscribe to `/joint_states` and
`/diffbot_base_controller/odom`, and assert both wheel velocities become positive and odometry x
increases. Then stop publishing and assert wheel velocities return near zero after the watchdog.

Core assertions:

```python
self.assertGreater(left_velocity, 0.1)
self.assertGreater(right_velocity, 0.1)
self.assertGreater(odom.pose.pose.position.x, 0.01)
self.assertLess(abs(stopped_left_velocity), 0.05)
self.assertLess(abs(stopped_right_velocity), 0.05)
```

- [ ] **Step 2: Register and run the launch test to verify failure**

```cmake
add_launch_test(test/test_demo_launch.py TIMEOUT 90)
```

Expected: FAIL because `demo.launch.py` does not exist.

- [ ] **Step 3: Implement `demo.launch.py`**

Declare launch arguments for `can_interface` and all fault switches. Generate robot description via
xacro, start `virtual_motor_node`, `robot_state_publisher`, and `ros2_control_node`; spawn
`joint_state_broadcaster`, then spawn `diffbot_base_controller` after the broadcaster exits.

Use the Humble controller topic without remapping ambiguity:

```text
/diffbot_base_controller/cmd_vel
```

- [ ] **Step 4: Run the complete launch test**

```bash
./src/vcan_diffbot_demo/scripts/setup_vcan.sh
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_demo_launch
colcon test-result --verbose
```

Expected: controller activation, positive wheel motion, positive x odometry, and watchdog stop all
pass.

- [ ] **Step 5: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "test: add full vcan diffbot launch coverage"
```

## Task 8: Document Operation And Verify Fault Paths

**Files:**
- Modify: `README.md`
- Modify: `src/vcan_diffbot_demo/test/test_demo_launch.py`

- [ ] **Step 1: Add concise operating instructions**

Document exact WSL commands for dependency installation, `vcan0` setup, build, launch, driving,
CAN inspection, stopping, and deterministic faults.

```bash
ros2 launch vcan_diffbot_demo demo.launch.py
ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"
candump vcan0
```

- [ ] **Step 2: Add a deterministic feedback-timeout integration case**

Launch with `drop_feedback_every_n:=1`, verify controller/hardware logs contain a feedback-timeout
message, and verify command IDs eventually carry zero velocity. Keep this as a second launch test so
normal motion and expected-error behavior have separate process expectations.

- [ ] **Step 3: Run all package tests**

```bash
./src/vcan_diffbot_demo/scripts/setup_vcan.sh
colcon build --packages-select vcan_diffbot_demo --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --event-handlers console_direct+
colcon test-result --verbose
```

Expected: protocol, motor model, xacro, normal launch, and timeout launch tests all pass.

- [ ] **Step 4: Perform a manual smoke test**

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vcan_diffbot_demo demo.launch.py
```

Terminal 2:

```bash
candump -L vcan0
```

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 3 ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
ros2 topic echo --once /joint_states
ros2 topic echo --once /diffbot_base_controller/odom
```

Expected: `candump` shows `101`, `102`, `181`, `182`, `281`, and `282`; joint velocities are
nonzero while driving and return to zero after publication stops; odometry advances.

- [ ] **Step 5: Inspect the final diff and commit**

```bash
git diff --check
git status --short
git add README.md src/vcan_diffbot_demo
git commit -m "docs: add vcan diffbot demo usage"
```

## Task 9: Final Verification

**Files:**
- No planned source changes

- [ ] **Step 1: Verify a clean build from a removed build cache**

Resolve the workspace path and remove only its generated `build`, `install`, and `log` directories,
then rebuild:

```bash
workspace="/mnt/c/Users/qucha/Documents/New project"
cd "$workspace"
rm -rf -- "$workspace/build" "$workspace/install" "$workspace/log"
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo
colcon test-result --verbose
```

Expected: zero test failures.

- [ ] **Step 2: Verify repository state and recent commits**

```bash
git diff --check
git status --short --branch
git log --oneline -10
```

Expected: clean working tree and task-sized commits for scaffold, protocol, model, description,
hardware, simulator, integration tests, and documentation.
