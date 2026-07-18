# Engineering Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add CI, complete licensing, shared motor configuration, explicit per-node faults, fatal CAN error handling, ROS diagnostics, and a documented physical SocketCAN path to the vcan DiffBot demo.

**Architecture:** Keep the existing single ROS package. Make launch arguments the shared runtime configuration boundary, keep protocol and CAN-error policy in small header-only helpers, and publish standard diagnostics from the hardware plugin that owns the authoritative ACK and feedback state.

**Tech Stack:** ROS 2 Humble, ros2_control, diagnostic_msgs, ros2_socketcan, C++17, Python launch, launch_testing, GoogleTest, SocketCAN/vcan, GitHub Actions

---

## File Map

- Create `.github/workflows/ci.yml`: Ubuntu 22.04 and ROS 2 Humble build/test workflow.
- Create `LICENSE`: Apache License 2.0 text.
- Create `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_error_policy.hpp`: pure SocketCAN error classification.
- Create `src/vcan_diffbot_demo/test/test_can_error_policy.cpp`: policy unit tests.
- Create `src/vcan_diffbot_demo/test/test_can_busoff_launch.py`: fatal error safe-stop integration test.
- Create `src/vcan_diffbot_demo/test/test_diagnostics_launch.py`: healthy-to-fault diagnostics test.
- Create `docs/hardware-can.md`: generic physical SocketCAN setup and safety checklist.
- Modify `src/vcan_diffbot_demo/launch/demo.launch.py`: shared parameters, explicit faults, and simulator condition.
- Modify `src/vcan_diffbot_demo/urdf/diffbot.urdf.xacro`: shared Xacro arguments.
- Modify `src/vcan_diffbot_demo/urdf/diffbot.ros2_control.xacro`: consume shared hardware values.
- Modify `src/vcan_diffbot_demo/config/virtual_motor.yaml`: retain simulator-only settings.
- Modify `src/vcan_diffbot_demo/src/virtual_motor_node.cpp`: configurable IDs and explicit node faults.
- Modify `src/vcan_diffbot_demo/include/vcan_diffbot_demo/hardware_health.hpp`: diagnostic read accessors.
- Modify `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp`: diagnostic publisher state and stop reason.
- Modify `src/vcan_diffbot_demo/src/can_motor_hardware.cpp`: error policy, diagnostics construction, and publication.
- Modify `src/vcan_diffbot_demo/CMakeLists.txt`: diagnostic dependency and tests.
- Modify `src/vcan_diffbot_demo/package.xml`: diagnostic dependency.
- Modify existing unit and launch tests for shared configuration and explicit faults.
- Modify `README.md` and `docs/learning-guide.zh-CN.md`: CI, diagnostics, and physical CAN usage.

### Task 1: Repository Baseline, CI, And License

**Files:**
- Create: `LICENSE`
- Create: `.github/workflows/ci.yml`
- Modify: `README.md:3-6`

- [ ] **Step 1: Add the Apache-2.0 license file**

Copy the canonical license installed by Ubuntu:

```bash
cp /usr/share/common-licenses/Apache-2.0 LICENSE
grep -q "Apache License" LICENSE
grep -q "END OF TERMS AND CONDITIONS" LICENSE
```

Expected: both checks exit 0.

- [ ] **Step 2: Add the Humble CI workflow**

Create `.github/workflows/ci.yml` with this behavior:

```yaml
name: ROS 2 Humble CI

on:
  push:
    branches: [master]
  pull_request:

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble
      - name: Install dependencies
        shell: bash
        run: |
          sudo apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
      - name: Configure vcan0
        shell: bash
        run: |
          sudo modprobe vcan
          sudo ip link add dev vcan0 type vcan
          sudo ip link set up vcan0
      - name: Build
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          colcon build --packages-select vcan_diffbot_demo
      - name: Test
        shell: bash
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          colcon test --packages-select vcan_diffbot_demo --event-handlers console_direct+
          colcon test-result --verbose
```

- [ ] **Step 3: Validate YAML locally**

Run:

```bash
python3 -c "import pathlib, yaml; yaml.safe_load(pathlib.Path('.github/workflows/ci.yml').read_text())"
```

Expected: exit 0 and no output.

- [ ] **Step 4: Add the live workflow badge**

Add this badge beside the existing badges:

```markdown
[![ROS 2 Humble CI](https://github.com/Quchaosheng/ros2-control-vcan-motor-demo/actions/workflows/ci.yml/badge.svg)](https://github.com/Quchaosheng/ros2-control-vcan-motor-demo/actions/workflows/ci.yml)
```

- [ ] **Step 5: Commit**

```bash
git add LICENSE .github/workflows/ci.yml README.md
git commit -m "ci: test ROS 2 Humble on vcan"
```

### Task 2: Shared Motor Configuration

**Files:**
- Modify: `src/vcan_diffbot_demo/test/test_urdf_xacro.py`
- Modify: `src/vcan_diffbot_demo/test/test_virtual_motor_node.py`
- Modify: `src/vcan_diffbot_demo/urdf/diffbot.urdf.xacro`
- Modify: `src/vcan_diffbot_demo/urdf/diffbot.ros2_control.xacro`
- Modify: `src/vcan_diffbot_demo/launch/demo.launch.py`
- Modify: `src/vcan_diffbot_demo/config/virtual_motor.yaml`
- Modify: `src/vcan_diffbot_demo/src/virtual_motor_node.cpp`

- [ ] **Step 1: Write failing Xacro override coverage**

Expand the Xacro with non-default values and assert every hardware parameter:

```python
xml = subprocess.check_output(
    [
        "xacro",
        path,
        "can_interface:=vcan9",
        "left_node_id:=17",
        "right_node_id:=23",
        "encoder_counts_per_revolution:=8192",
        "command_watchdog_ms:=350",
        "feedback_timeout_ms:=900",
    ],
    text=True,
)
assert '<param name="left_node_id">17</param>' in xml
assert '<param name="right_node_id">23</param>' in xml
assert '<param name="encoder_counts_per_revolution">8192</param>' in xml
assert '<param name="command_watchdog_ms">350</param>' in xml
assert '<param name="feedback_timeout_ms">900</param>' in xml
```

- [ ] **Step 2: Write failing non-default virtual motor ID coverage**

Launch `virtual_motor_node` with `left_node_id=17` and `right_node_id=23`. Send the left command to
`0x111` and require ACK `0x291` and feedback `0x191`; reject observations on the old left IDs.

```python
parameters=[{"can_interface": can_interface, "left_node_id": 17, "right_node_id": 23}]
command_id = 0x100 + 17
feedback_id = 0x180 + 17
ack_id = 0x280 + 17
```

- [ ] **Step 3: Run the focused tests and verify RED**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R 'test_urdf_xacro|test_virtual_motor_node'
colcon test-result --verbose
```

Expected: Xacro rejects unknown arguments or retains old values, and the motor does not answer on
the non-default IDs.

- [ ] **Step 4: Add shared Xacro arguments**

Declare these defaults in `diffbot.urdf.xacro` and pass them to the ros2_control macro:

```xml
<xacro:arg name="left_node_id" default="1"/>
<xacro:arg name="right_node_id" default="2"/>
<xacro:arg name="encoder_counts_per_revolution" default="4096"/>
<xacro:arg name="command_watchdog_ms" default="200"/>
<xacro:arg name="feedback_timeout_ms" default="500"/>
```

Change the macro signature to accept those values and replace its five embedded constants with the
corresponding macro parameters.

- [ ] **Step 5: Add node ID parameters to the virtual motor**

Declare and validate both IDs, store them in `std::array<uint8_t, 2> node_ids_`, and replace every
`index + 1U`, `command_id(1U)`, and `command_id(2U)` with `node_ids_[index]`-based lookup.

```cpp
const auto left_node = declare_parameter<int64_t>("left_node_id", 1);
const auto right_node = declare_parameter<int64_t>("right_node_id", 2);
if (left_node < 1 || left_node > 127 || right_node < 1 || right_node > 127 ||
  left_node == right_node)
{
  throw std::invalid_argument("motor node IDs must be distinct values from 1 to 127");
}
node_ids_ = {static_cast<uint8_t>(left_node), static_cast<uint8_t>(right_node)};
```

- [ ] **Step 6: Make launch the shared runtime boundary**

Declare the five shared arguments, pass them to the Xacro command, and pass typed values to the
virtual motor parameter dictionary. Remove `encoder_counts_per_revolution` from
`virtual_motor.yaml`; it remains a C++ standalone default and is always explicit in the demo launch.

- [ ] **Step 7: Run focused and full tests and verify GREEN**

Run the command from Step 3, then:

```bash
colcon test --packages-select vcan_diffbot_demo
colcon test-result --verbose
```

Expected: all tests pass with zero errors, failures, and skipped tests.

- [ ] **Step 8: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: share motor configuration across CAN endpoints"
```

### Task 3: Explicit Per-Node Fault Injection

**Files:**
- Modify: `src/vcan_diffbot_demo/test/test_feedback_timeout_launch.py`
- Modify: `src/vcan_diffbot_demo/test/test_ack_timeout_launch.py`
- Modify: `src/vcan_diffbot_demo/test/test_virtual_motor_node.py`
- Modify: `src/vcan_diffbot_demo/launch/demo.launch.py`
- Modify: `src/vcan_diffbot_demo/config/virtual_motor.yaml`
- Modify: `src/vcan_diffbot_demo/src/virtual_motor_node.cpp`

- [ ] **Step 1: Convert timeout tests to explicit node faults**

Use these launch arguments:

```python
"drop_feedback_node_id": "2"  # feedback timeout test
"drop_command_node_id": "1"   # ACK timeout test
```

Remove the corresponding every-N fault from each test while preserving the existing safe-stop
assertions.

- [ ] **Step 2: Verify RED**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R 'test_feedback_timeout_launch|test_ack_timeout_launch'
colcon test-result --verbose
```

Expected: launch fails because the new launch arguments are not declared.

- [ ] **Step 3: Implement validated fault selectors**

Add parameters and members:

```cpp
int64_t drop_command_node_id_{0};
int64_t drop_feedback_node_id_{0};
```

Validate each value with a helper that accepts `0` or one of `node_ids_`. Drop a frame when either
the explicit selector matches or the existing every-N predicate matches:

```cpp
if (drop_command_node_id_ == node_ids_[index] ||
  every_n(drop_command_every_n_, command_count_))
{
  continue;
}
```

Apply the same rule to feedback. Add launch arguments and typed virtual motor parameters. Keep both
YAML defaults at `0`.

- [ ] **Step 4: Verify GREEN and regression coverage**

Run the focused command from Step 2 and the complete package suite.

Expected: both timeout tests exercise one named motor and all package tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: inject CAN loss by motor node"
```

### Task 4: SocketCAN Error Classification And Fatal Stop

**Files:**
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_error_policy.hpp`
- Create: `src/vcan_diffbot_demo/test/test_can_error_policy.cpp`
- Create: `src/vcan_diffbot_demo/test/test_can_busoff_launch.py`
- Modify: `src/vcan_diffbot_demo/src/can_motor_hardware.cpp`
- Modify: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing policy unit tests**

```cpp
EXPECT_EQ(classify_can_error(0U), CanErrorSeverity::NONE);
EXPECT_EQ(classify_can_error(CAN_ERR_CRTL), CanErrorSeverity::WARNING);
EXPECT_EQ(classify_can_error(CAN_ERR_ACK | CAN_ERR_PROT), CanErrorSeverity::WARNING);
EXPECT_EQ(classify_can_error(CAN_ERR_BUSOFF), CanErrorSeverity::FATAL);
EXPECT_EQ(classify_can_error(CAN_ERR_TX_TIMEOUT), CanErrorSeverity::FATAL);
EXPECT_EQ(classify_can_error(CAN_ERR_BUSOFF | CAN_ERR_CRTL), CanErrorSeverity::FATAL);
```

Also test `describe_can_error()` returns stable comma-separated names such as `bus_off` and
`controller` for diagnostics.

- [ ] **Step 2: Register the test and verify RED**

Add `test_can_error_policy` to CMake, then build.

Expected: compilation fails because `can_error_policy.hpp` does not exist.

- [ ] **Step 3: Implement the pure policy helper**

Define:

```cpp
enum class CanErrorSeverity {NONE, WARNING, FATAL};

inline CanErrorSeverity classify_can_error(const uint32_t mask)
{
  if ((mask & (CAN_ERR_BUSOFF | CAN_ERR_TX_TIMEOUT)) != 0U) {
    return CanErrorSeverity::FATAL;
  }
  return mask == 0U ? CanErrorSeverity::NONE : CanErrorSeverity::WARNING;
}
```

`describe_can_error()` must inspect known Linux CAN error bits in a fixed order and return
`"unknown"` when the mask is nonzero but no known bit matches.

- [ ] **Step 4: Verify the policy test GREEN**

```bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_can_error_policy
colcon test-result --verbose
```

- [ ] **Step 5: Write a failing BUS-OFF launch test**

Launch the demo with controllers disabled, bind a raw CAN observer, and send:

```python
CAN_ERR_FLAG = 0x20000000
CAN_ERR_BUSOFF = 0x00000040
frame = struct.pack(CAN_FRAME_FORMAT, CAN_ERR_FLAG | CAN_ERR_BUSOFF, 8, bytes(8))
can_socket.send(frame)
```

Require the hardware log `Fatal SocketCAN error: bus_off` and two bounded disabled-zero commands,
one on each configured command ID.

- [ ] **Step 6: Verify the launch test RED**

Expected: the hardware only warns and continues, so the fatal log and safe-stop assertions fail.

- [ ] **Step 7: Route fatal errors into safe stop**

Store `last_can_error_` and `stop_reason_` in the hardware class. On a warning, update the error text
and continue. On a fatal classification, set `stop_reason_ = "can_bus_off"` or
`"can_tx_timeout"`, send the bounded safe stop, mark the stop episode, and return ERROR.

- [ ] **Step 8: Run focused and full tests, then commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: stop hardware on fatal CAN errors"
```

### Task 5: Standard ROS Diagnostics

**Files:**
- Create: `src/vcan_diffbot_demo/test/test_diagnostics_launch.py`
- Modify: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/hardware_health.hpp`
- Modify: `src/vcan_diffbot_demo/test/test_hardware_health.cpp`
- Modify: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp`
- Modify: `src/vcan_diffbot_demo/src/can_motor_hardware.cpp`
- Modify: `src/vcan_diffbot_demo/test/test_hardware_plugin.cpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`
- Modify: `src/vcan_diffbot_demo/package.xml`

- [ ] **Step 1: Write failing health accessor tests**

Add assertions for diagnostic state:

```cpp
health.command_sent(0, 7, now);
EXPECT_EQ(health.pending_ack_count(0), 1U);
EXPECT_EQ(health.last_ack_status(0), AckStatus::NONE);
EXPECT_EQ(health.ack_received(0, 7, 0), AckStatus::ACCEPTED);
EXPECT_EQ(health.pending_ack_count(0), 0U);
EXPECT_EQ(health.last_ack_status(0), AckStatus::ACCEPTED);
EXPECT_EQ(health.feedback_age(0, now + 75ms), 75ms);
```

- [ ] **Step 2: Verify RED**

Expected: compilation fails because the accessors and `AckStatus::NONE` do not exist.

- [ ] **Step 3: Add read-only diagnostic health accessors**

Initialize `last_ack_status_` to `NONE` in `reset()`, update it in `ack_received()`, and add bounds-safe
`pending_ack_count()`, `last_ack_status()`, and `feedback_age()` methods. Invalid indexes return zero,
`UNEXPECTED`, and zero milliseconds respectively.

- [ ] **Step 4: Verify the health tests GREEN**

Run `test_hardware_health` through CTest and confirm zero failures.

- [ ] **Step 5: Write the failing diagnostics launch test**

Launch with `drop_feedback_node_id:=2` and controllers disabled. Subscribe to
`diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics`. Require:

```python
names = {status.name for status in message.status}
self.assertEqual(
    names,
    {"vcan_diffbot/can_bus", "vcan_diffbot/left_motor", "vcan_diffbot/right_motor"},
)
```

Then wait for the bus status level `DiagnosticStatus.ERROR`, value `state=safe_stop`, and value
`stop_reason=feedback_timeout_node_2`. Require the right motor status to report a feedback timeout.

- [ ] **Step 6: Verify diagnostics RED**

Expected: no publisher exists and the test times out.

- [ ] **Step 7: Add dependencies and publisher state**

Add `<depend>diagnostic_msgs</depend>`, `find_package(diagnostic_msgs REQUIRED)`, and the dependency
to the hardware target and diagnostics launch test. Add a diagnostics-only `rclcpp::Node`, a
`DiagnosticArray` publisher, last-publication steady time, last level, `last_can_error_`, and
`stop_reason_` to `CanMotorHardware`.

Because `on_init()` now creates a node, add a GoogleTest fixture in `test_hardware_plugin.cpp` that
calls `rclcpp::init()` once in `SetUpTestSuite()` and `rclcpp::shutdown()` in
`TearDownTestSuite()`. Convert the two tests that call `on_init()` or `on_activate()` to `TEST_F`.

- [ ] **Step 8: Build and publish the three statuses**

Create the publisher during `on_init()`. Build messages with `diagnostic_msgs::msg::KeyValue` and
publish when 500 ms elapsed or `force=true`. Publish immediately on activation, deactivation,
warning/fatal CAN errors, ACK rejection, ACK timeout, and feedback timeout. Use steady time for ages
and `diagnostics_node_->now()` for the header stamp.

Use these exact level rules:

```text
bus: OK when active and healthy, WARN for nonfatal CAN errors, ERROR for safe stop
motor: ERROR for ACK or feedback timeout, WARN for rejected/unexpected ACK, otherwise OK
```

- [ ] **Step 9: Verify GREEN and full regression coverage**

Run the diagnostics test, health test, and then the complete package suite. Confirm the diagnostic
test observes an initial healthy message and the later safe-stop message.

- [ ] **Step 10: Commit**

```bash
git add src/vcan_diffbot_demo
git commit -m "feat: publish CAN motor diagnostics"
```

### Task 6: Generic Physical SocketCAN Path

**Files:**
- Modify: `src/vcan_diffbot_demo/launch/demo.launch.py`
- Create: `docs/hardware-can.md`
- Modify: `README.md`

- [ ] **Step 1: Add the conditional simulator launch**

Declare `start_virtual_motor` with default `true` and add:

```python
condition=IfCondition(LaunchConfiguration("start_virtual_motor"))
```

to `virtual_motor_node`. Do not condition the hardware, robot state publisher, or controllers.

- [ ] **Step 2: Check launch arguments**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vcan_diffbot_demo
source install/setup.bash
ros2 launch vcan_diffbot_demo demo.launch.py --show-args
```

Expected: `start_virtual_motor`, both node IDs, shared encoder/watchdog/timeout values, and all fault
arguments appear with their defaults.

- [ ] **Step 3: Write the physical CAN guide**

Document exact commands:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
candump -tz can0
ros2 launch vcan_diffbot_demo demo.launch.py can_interface:=can0 start_virtual_motor:=false
sudo ip link set can0 down
```

State that bitrate is hardware-specific, the physical device must implement the six documented CAN
IDs and byte layouts, wheels must initially be raised, and a physical emergency stop must be within
reach.

- [ ] **Step 4: Link the guide from README**

Add a short `Physical CAN` section after Quick Start with the launch command and a link to
`docs/hardware-can.md`. Preserve the existing Demo scope warning.

- [ ] **Step 5: Build and verify default launch remains unchanged**

Run the full package suite. Then run the normal demo for five seconds and confirm
`Virtual motors listening on` and `CAN motor hardware active on` both appear.

- [ ] **Step 6: Commit**

```bash
git add src/vcan_diffbot_demo/launch/demo.launch.py docs/hardware-can.md README.md
git commit -m "docs: add physical SocketCAN launch path"
```

### Task 7: Documentation, Verification, And Delivery

**Files:**
- Modify: `README.md`
- Modify: `docs/learning-guide.zh-CN.md`

- [ ] **Step 1: Update operating documentation**

Document the shared launch arguments, explicit node faults, `/diagnostics` command, CAN error
severity behavior, CI status, and physical CAN link. Replace the learning guide's one-sided
`drop_feedback_every_n:=2` example with `drop_feedback_node_id:=2` while retaining every-N examples
for intermittent loss.

- [ ] **Step 2: Validate documentation contracts**

```bash
rg -n "drop_feedback_node_id|drop_command_node_id|/diagnostics|start_virtual_motor|hardware-can.md" README.md docs/learning-guide.zh-CN.md
rg -n "Apache-2.0" LICENSE src/vcan_diffbot_demo/package.xml README.md
```

Expected: every new user-facing control is documented and the license is consistent.

- [ ] **Step 3: Run the final cache-clean build and complete suite**

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache --packages-select vcan_diffbot_demo
source install/setup.bash
colcon test --packages-select vcan_diffbot_demo --event-handlers console_direct+
colcon test-result --verbose
```

Expected: one package builds, every test passes, and the summary reports zero errors, failures, and
skipped tests.

- [ ] **Step 4: Inspect repository state**

```bash
git diff --check
git status --short
git log --oneline --decorate -10
```

Expected: no whitespace errors and only the intended documentation changes remain before the final
commit.

- [ ] **Step 5: Commit final documentation**

```bash
git add README.md docs/learning-guide.zh-CN.md
git commit -m "docs: explain diagnostics and hardware CAN"
```

- [ ] **Step 6: Push and open the pull request**

```bash
git push -u origin feat/engineering-hardening
```

Create a ready-for-review PR targeting `master` with the test summary, CI scope, diagnostic topic,
fault behavior, and physical CAN limitations. Do not merge until the remote GitHub Actions run
passes.
