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
            "drop_feedback_node_id": "2",
            "spawn_controllers": "false",
        }.items(),
    )
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()]), {
        "can_interface": can_interface
    }


class TestFeedbackTimeout(unittest.TestCase):
    def test_timeout_sends_disabled_zero_commands(self, proc_output, can_interface):
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.bind((can_interface,))
        can_socket.settimeout(0.2)

        proc_output.assertWaitFor("Feedback timeout for motor node", timeout=10.0)

        disabled_commands = 0
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id not in (0x101, 0x102) or dlc != 8:
                continue
            sequence, flags, velocity = struct.unpack("<BBh", data[:4])
            if sequence > 5 and flags == 0 and velocity == 0:
                disabled_commands += 1

        self.assertGreaterEqual(disabled_commands, 2)
        self.assertLessEqual(disabled_commands, 4)
