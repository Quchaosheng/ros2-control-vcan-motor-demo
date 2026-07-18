# ros2_control vcan Code Review Fixes Design

## Goal

Fix the six verified review findings without changing the existing CAN frame layout, ROS topics,
or default launch command.

## Approach

Keep the current single-package architecture and add only the state needed to make failures
observable. The hardware plugin will treat safe-stop delivery and command acknowledgement as part
of hardware health. Tests will use an interface unique to each launch test so an existing demo
cannot inject matching frames.

## Hardware Lifecycle Safety

`send_safe_stop()` will return success only when disabled zero commands are sent to both motors.
Activation and deactivation will propagate failure instead of reporting lifecycle success. Both
paths will clear the exported command buffer before sending the stop, preventing a later `write()`
from replaying a command retained from a previous activation.

## Feedback Timeout State

The hardware will track whether it is currently in a feedback-timeout episode. A single healthy
motor frame will not clear that state. The episode ends only after both motors have received fresh
feedback, so a one-sided outage emits one stop attempt instead of one attempt per control cycle.

## ACK Health

Each successful command send records the sequence and send time for that motor. A matching
successful ACK clears the pending command. A rejected ACK, a mismatched ACK for the current pending
command, or a pending ACK older than a configurable timeout makes `read()` return `ERROR` after one
safe-stop attempt. The default ACK timeout will be derived from the existing command watchdog, so
the wire protocol and URDF configuration remain compatible.

Safe-stop commands are not added to the pending-ACK tracker. Their delivery result is the sender
result itself; requiring a later ACK during shutdown would make lifecycle completion depend on a
future controller loop.

## CAN Filtering

On activation, the hardware receiver will install exact standard-frame filters for both feedback
IDs and both ACK IDs, plus the SocketCAN error mask. The virtual motor receiver will install exact
filters for the two command IDs. This prevents unrelated traffic from consuming the bounded drain
loop.

## Test Isolation

Each launch test will create a deterministic, process-specific virtual CAN interface and pass it to
the node or launch file. Setup will first reuse an existing interface, invoke `sudo -n` only when
creation or activation needs privilege, and fail promptly with a useful error instead of waiting
for a password prompt. Tests will delete only the interface they created.

## Verification

Regression coverage will include:

- activation and deactivation failure propagation;
- command-buffer clearing across activation;
- one-sided feedback timeout emits one stop episode;
- matching, rejected, mismatched, and missing ACK behavior;
- receiver filter configuration;
- unique CAN interface propagation through all launch tests.

Acceptance requires a cold ROS 2 Humble build, all package tests passing, `git diff --check`, and a
manual fault-path smoke run on WSL2.
