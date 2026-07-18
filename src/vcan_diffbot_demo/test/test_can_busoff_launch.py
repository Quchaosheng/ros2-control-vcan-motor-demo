import os
import socket
import struct
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from vcan_test_utils import create_test_vcan


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_ERR_FLAG = 0x20000000
CAN_ERR_CRTL = 0x00000004
CAN_ERR_BUSOFF = 0x00000040
COMMAND_IDS = (0x101, 0x102)


@pytest.mark.launch_test
def generate_test_description():
    can_interface = create_test_vcan()
    launch_file = os.path.join(
        get_package_share_directory("vcan_diffbot_demo"), "launch", "demo.launch.py"
    )
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "can_interface": can_interface,
            "spawn_controllers": "false",
        }.items(),
    )
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()]), {
        "can_interface": can_interface
    }


class TestCanBusOff(unittest.TestCase):
    def test_bus_off_stops_both_motors_once(self, proc_output, can_interface):
        proc_output.assertWaitFor(
            f"CAN motor hardware active on {can_interface}", timeout=10.0
        )

        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.bind((can_interface,))
        can_socket.settimeout(0.2)
        can_socket.send(
            struct.pack(CAN_FRAME_FORMAT, CAN_ERR_FLAG | CAN_ERR_CRTL, 8, bytes(8))
        )
        proc_output.assertWaitFor("SocketCAN warning: controller", timeout=5.0)

        disabled_after_warning = 0
        warning_deadline = time.monotonic() + 0.4
        while time.monotonic() < warning_deadline:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id not in COMMAND_IDS or dlc != 8:
                continue
            _, flags, velocity = struct.unpack("<BBh", data[:4])
            disabled_after_warning += flags == 0 and velocity == 0
        self.assertEqual(disabled_after_warning, 0)

        can_socket.send(
            struct.pack(CAN_FRAME_FORMAT, CAN_ERR_FLAG | CAN_ERR_BUSOFF, 8, bytes(8))
        )

        proc_output.assertWaitFor("Fatal SocketCAN error: bus_off", timeout=5.0)

        disabled_commands = {command_id: 0 for command_id in COMMAND_IDS}
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id not in disabled_commands or dlc != 8:
                continue
            _, flags, velocity = struct.unpack("<BBh", data[:4])
            if flags == 0 and velocity == 0:
                disabled_commands[can_id] += 1

        self.assertEqual(set(disabled_commands), set(COMMAND_IDS))
        self.assertTrue(all(count >= 1 for count in disabled_commands.values()))
        self.assertTrue(all(count <= 2 for count in disabled_commands.values()))
