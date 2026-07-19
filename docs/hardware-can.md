# Physical SocketCAN / 物理 SocketCAN

This path starts the same `ros2_control` hardware interface and controllers used by
the virtual demo, but does not start `virtual_motor_node`. It is a generic
SocketCAN/HIL launch path, not a motor-controller integration.

该启动方式使用与虚拟 Demo 相同的 `ros2_control` 硬件接口和控制器，但不启动
`virtual_motor_node`。它是通用的 SocketCAN/HIL 路径，不是针对某个电机控制器的集成。

## Bring-up / 启动

Build and source the workspace first. Configure the physical adapter before
launching ROS 2:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

### Terminal A: monitor the bus / 终端 A：监视总线

Run the following command in Terminal A and leave it running in the background
while the stack is launched from Terminal B:

```bash
candump -tz can0
```

### Terminal B: launch the control stack / 终端 B：启动控制栈

In Terminal B, source the workspace and run:

```bash
ros2 launch vcan_diffbot_demo demo.launch.py can_interface:=can0 start_virtual_motor:=false
```

After stopping the ROS launch and `candump`, bring the interface down:

```bash
sudo ip link set can0 down
```

The bitrate is adapter- and bus-specific. Replace `500000` only with the bitrate
used by the physical CAN network; every node on the bus must agree.

比特率取决于适配器和实际 CAN 总线。只有在确认所有节点使用相同比特率时才替换
`500000`。

## Compatibility and safety / 兼容性与安全

The physical controller must implement this demo's six CAN IDs and byte layouts:
`0x101`, `0x102`, `0x181`, `0x182`, `0x281`, and `0x282`. It must also follow the
command, feedback, ACK, DLC, and byte-order rules implemented in this repository.
This repository does not claim electrical validation or compatibility with
arbitrary commercial controllers.

实体控制器必须实现本 Demo 的六个 CAN ID 和字节布局：`0x101`、`0x102`、`0x181`、
`0x182`、`0x281` 和 `0x282`。它还必须遵守本仓库实现的命令、反馈、ACK、DLC 和
字节序规则。本仓库不声明已完成电气验证，也不声明与任意商用控制器兼容。

Start with the wheels lifted off the ground. Keep the physical emergency stop
accessible. Verify expected traffic with `candump` before commanding motion.

首次启动时请使车轮离地，并保证实体紧急停止装置随时可用。在发送运动命令前，先用
`candump` 确认总线流量符合预期。

## Troubleshooting / 排障

### Interface missing / 接口不存在

Confirm that the adapter driver is loaded and that the interface name is correct
with `ip link show`. USB-CAN devices may expose a name other than `can0`.

确认适配器驱动已加载，并使用 `ip link show` 确认接口名称正确。USB-CAN 设备可能
使用 `can0` 以外的名称。

### No ACK / 没有 ACK

Check the CAN bitrate, wiring, termination, controller power, and whether the
controller receives `0x101` and `0x102`. Confirm it replies on `0x281` and
`0x282` with this demo's ACK byte layout.

检查 CAN 比特率、布线、终端电阻、控制器供电，以及控制器是否收到 `0x101` 和
`0x102`。确认其使用本 Demo 的 ACK 字节布局在 `0x281` 和 `0x282` 上回复。

### Feedback timeout / 反馈超时

Confirm that the controller periodically transmits feedback on `0x181` and
`0x182` using the expected DLC and byte order. Match its transmission period to
the configured `feedback_timeout_ms`.

确认控制器按照预期 DLC 和字节序，周期性在 `0x181` 和 `0x182` 上发送反馈。其发送
周期必须与配置的 `feedback_timeout_ms` 相匹配。

### Bus-off / 总线关闭

Bring the interface down, fix the physical bus fault or bitrate mismatch, then
bring it up again. `restart-ms 100` enables automatic restart after a bus-off,
but it does not correct wiring, termination, or configuration faults.

先关闭接口，修复物理总线故障或比特率不匹配，再重新启用接口。`restart-ms 100` 会在
bus-off 后自动重启，但不能修复布线、终端电阻或配置错误。
