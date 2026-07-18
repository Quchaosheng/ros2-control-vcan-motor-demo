# Code Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix all six verified review findings and add regression coverage for each behavior.

**Architecture:** Keep the CAN protocol unchanged. Add a small header-only hardware health tracker for ACK and feedback-fault state, use ros2_socketcan's native filter API, and share one Python helper that owns a process-specific virtual CAN interface for launch tests.

**Tech Stack:** ROS 2 Humble, ros2_control, ros2_socketcan, C++17, GTest, pytest, launch_testing, SocketCAN.

---

### Task 1: Hardware Health State

**Files:**
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/hardware_health.hpp`
- Create: `src/vcan_diffbot_demo/test/test_hardware_health.cpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing ACK and feedback-state tests**

Add GTests that require these behaviors:

```cpp
HardwareHealth health;
health.reset(now);
health.command_sent(0, 7, now);
EXPECT_EQ(health.ack_received(0, 7, 0), AckStatus::ACCEPTED);
EXPECT_FALSE(health.ack_timed_out(now + 250ms, 200ms));

health.command_sent(0, 8, now);
EXPECT_EQ(health.ack_received(0, 8, 2), AckStatus::REJECTED);
EXPECT_TRUE(health.ack_fault());

health.reset(now);
health.command_sent(0, 9, now);
EXPECT_TRUE(health.ack_timed_out(now + 201ms, 200ms));

health.reset(now);
health.feedback_received(0, now + 100ms);
EXPECT_TRUE(health.feedback_timed_out(now + 501ms, 500ms));
health.mark_stop_sent();
health.feedback_received(0, now + 550ms);
EXPECT_TRUE(health.stop_sent());
health.feedback_received(1, now + 550ms);
health.recover_if_healthy(now + 550ms, 500ms);
EXPECT_FALSE(health.stop_sent());
```

- [ ] **Step 2: Run only the new target and verify RED**

Run `colcon test --packages-select vcan_diffbot_demo --ctest-args -R test_hardware_health`.
Expected: build/test failure because `hardware_health.hpp` and the target do not exist.

- [ ] **Step 3: Implement the minimal tracker**

Implement two-element pending ACK queues, one ignored safe-stop sequence per motor, feedback
timestamps, a latched ACK fault, and a stop-sent episode flag. A successful ACK must match the
oldest pending command; a matching ignored safe-stop ACK is discarded; other sequences and
nonzero results latch an ACK fault.

- [ ] **Step 4: Build and run the target to verify GREEN**

Run the new GTest plus existing protocol and motor-state tests. Expected: all selected tests pass.

### Task 2: Lifecycle Safety And Native CAN Filters

**Files:**
- Create: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_filters.hpp`
- Create: `src/vcan_diffbot_demo/test/test_can_filters.cpp`
- Modify: `src/vcan_diffbot_demo/include/vcan_diffbot_demo/can_motor_hardware.hpp`
- Modify: `src/vcan_diffbot_demo/src/can_motor_hardware.cpp`
- Modify: `src/vcan_diffbot_demo/src/virtual_motor_node.cpp`
- Modify: `src/vcan_diffbot_demo/test/test_hardware_plugin.cpp`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing lifecycle and filter tests**

Require exact standard-frame filters for `0x181/0x182/0x281/0x282` in the hardware and
`0x101/0x102` in the simulator. Extend the hardware test to initialize a `CanMotorHardware`, set
exported commands nonzero, activate against a nonexistent interface, and verify the command values
were cleared and activation returned `ERROR`. Calling deactivation without a usable sender must
also return `ERROR`.

- [ ] **Step 2: Verify RED**

Run `test_can_filters` and `test_hardware_plugin`. Expected: missing filter helpers and lifecycle
assertion failures.

- [ ] **Step 3: Implement lifecycle safety**

Change `send_safe_stop()` to return `bool`, clear `commands_` at activation and deactivation,
propagate stop failure, and route normal command sends through `HardwareHealth::command_sent()`.
Route ACKs and feedback through the tracker. `read()` must stop once for either feedback or ACK
fault and must return `ERROR` while the fault remains.

- [ ] **Step 4: Install filters**

Build `SocketCanReceiver::CanFilterList` with `CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG`, call
`SetCanFilters()` immediately after receiver construction, and enable `CAN_ERR_MASK` only on the
hardware receiver.

- [ ] **Step 5: Verify GREEN**

Build and run all C++ GTests. Expected: lifecycle, filter, protocol, model, and plugin tests pass.

### Task 3: Isolated Launch-Test CAN Interfaces

**Files:**
- Create: `src/vcan_diffbot_demo/test/vcan_test_utils.py`
- Create: `src/vcan_diffbot_demo/test/test_vcan_test_utils.py`
- Modify: `src/vcan_diffbot_demo/test/test_virtual_motor_node.py`
- Modify: `src/vcan_diffbot_demo/test/test_demo_launch.py`
- Modify: `src/vcan_diffbot_demo/test/test_feedback_timeout_launch.py`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`

- [ ] **Step 1: Write failing utility tests**

Test that generated names start with `vcan`, fit Linux's 15-character interface limit, vary by
process suffix, and that privileged commands use direct `ip` as root or `sudo -n ip` otherwise.

- [ ] **Step 2: Verify RED**

Run `test_vcan_test_utils`. Expected: import failure because the utility does not exist.

- [ ] **Step 3: Implement interface ownership**

Create a process-specific interface, reuse it only if already present, register cleanup with
`atexit`, and delete only interfaces created by that helper. Replace the three duplicated
`ensure_vcan()` functions and pass the returned interface through node parameters or
`can_interface` launch arguments.

- [ ] **Step 4: Verify GREEN**

Run the utility test and the normal virtual-motor/full-stack launch tests. Expected: each test logs
and uses its own interface and passes without touching `vcan0`.

### Task 4: ACK And One-Sided Feedback Regression Tests

**Files:**
- Create: `src/vcan_diffbot_demo/test/test_ack_timeout_launch.py`
- Modify: `src/vcan_diffbot_demo/test/test_feedback_timeout_launch.py`
- Modify: `src/vcan_diffbot_demo/CMakeLists.txt`
- Modify: `README.md`

- [ ] **Step 1: Write failing launch tests**

Launch with `drop_command_every_n:=1` and assert an `ACK timeout` is reported followed by disabled
zero command frames. Change the feedback-timeout test to `drop_feedback_every_n:=2`, observe a
one-sided timeout, then count disabled frames for one second and require no more than the initial
fault stop plus lifecycle shutdown stop.

- [ ] **Step 2: Verify RED against the pre-fix behavior**

Run the two launch targets. Expected: no ACK timeout in the old implementation and repeated stop
traffic during one-sided feedback loss.

- [ ] **Step 3: Complete integration behavior and documentation**

Make only adjustments required by the failing launch tests. Update README to state that ACK loss or
rejection faults the hardware and that tests allocate private virtual CAN interfaces.

- [ ] **Step 4: Verify GREEN**

Run both fault launch tests. Expected: ACK loss is detected, one-sided feedback loss sends one stop
episode, and both tests pass.

### Task 5: Final Verification And Delivery

**Files:**
- No additional source files planned.

- [ ] **Step 1: Run a cold build**

Copy the worktree source into a new `/tmp/vcan-review-fixes-*` workspace, source ROS 2 Humble, and
run `colcon build --packages-select vcan_diffbot_demo`.

- [ ] **Step 2: Run the complete suite**

Run `colcon test --packages-select vcan_diffbot_demo` and `colcon test-result`. Expected: zero
errors, failures, and skipped tests.

- [ ] **Step 3: Inspect repository state**

Run `git diff --check`, inspect the complete diff, and confirm no generated build output is tracked.

- [ ] **Step 4: Commit and push**

Commit the implementation, push `feat/fix-code-review-findings`, and create a PR targeting
`master` with the six fixes and exact verification count.
