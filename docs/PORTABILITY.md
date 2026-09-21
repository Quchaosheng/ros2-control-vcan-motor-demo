# Portability

The CAN hardware path is Linux-specific because it uses SocketCAN and Linux
CAN error-frame headers. The protocol-policy helpers are intentionally kept
header-only and can be compiled independently on other hosts when a small
CAN constant shim is supplied by the test harness.

- Linux: supported runtime target for SocketCAN and vcan.
- WSL2: supported for policy tests and vcan demonstrations when the kernel
  exposes the required virtual CAN interface.
- Windows and macOS: supported for documentation and host-side policy tests;
  native CAN hardware access is out of scope.

Keep hardware-dependent launch tests in the Linux CI job and run pure policy
tests separately so portability failures are visible without weakening the
runtime safety boundary.

## ros2_socketcan version boundary

`ros2_socketcan` 1.4.0 inserted an `enable_loopback` argument between `enable_fd`
and `default_id` in `SocketCanSender`, defaulting it to `false`, and it now sets
`CAN_RAW_LOOPBACK=0` on the sender socket.

A `vcan` interface has no physical medium. Local loopback is therefore the only
mechanism that delivers a frame to the other processes bound to that interface.
On `vcan`, a sender that leaves loopback disabled is inaudible: commands are
handed to the kernel, yet `virtual_motor_node` and the hardware interface never
observe each other's frames, so every ACK, feedback, `/diagnostics`, and
`/joint_states` expectation times out while all pure policy tests keep passing.

Both senders are therefore created through
`vcan_diffbot_demo::make_loopback_sender` (`include/vcan_diffbot_demo/can_sender.hpp`),
which requests loopback on releases that expose the argument and falls back to
the previous constructor otherwise. Add new sockets through that helper rather
than constructing `SocketCanSender` directly.

On an ARM64 board with ROS 2 Humble installed, build natively with:

```bash
bash scripts/build_on_arm64.sh
```
