# CAN Capture, Audit, And vcan Replay

Use this workflow to preserve a physical SocketCAN smoke capture and to replay
the same application frames on an isolated `vcan` interface. It validates the
demo's software protocol contract; it is not a substitute for ECU HIL,
electrical fault validation, or a motor-safety assessment.

## Capture

Start `candump` before launching the stack and keep the raw log unchanged:

```bash
mkdir -p artifacts/can
candump -L -t a can0 | tee artifacts/can/physical-smoke.log
```

Record the adapter, bitrate, interface state, launch arguments, and test
boundary beside the log. Do not infer command-to-motor success from a matching
ACK alone.

## Audit

The audit tool accepts normal `candump` hash notation and bracket notation. It
checks the IDs, DLC, reserved bytes, ACK result range, and feedback status
bits, then reports the first matching ACK and feedback for each observed
command sequence.

```bash
python3 src/vcan_diffbot_demo/scripts/can_trace_audit.py \
  --input artifacts/can/physical-smoke.log \
  --output artifacts/can/physical-smoke.manifest.json \
  --replay-output artifacts/can/physical-smoke.replay.log \
  --strict
```

`command_to_ack` and `command_to_first_feedback` are differences between
captured packet timestamps. They do not measure controller execution,
mechanical motion, or end-to-end task latency. Unmatched packets can occur at
the beginning or end of a capture; preserve them in the manifest rather than
silently discarding them.

## Replay On vcan Only

The generated replay log is a `canplayer`-compatible normalization of the
capture. Replay it only on an isolated virtual interface, never on a live
physical CAN network:

```bash
sudo modprobe vcan
sudo ip link add dev vcan9 type vcan
sudo ip link set up vcan9
canplayer -I artifacts/can/physical-smoke.replay.log can0=vcan9
```

The source interface name in the capture is explicitly mapped to `vcan9`.
Delete the temporary interface after the regression run:

```bash
sudo ip link delete vcan9
```
