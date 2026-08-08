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
