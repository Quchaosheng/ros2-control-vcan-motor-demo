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
            "drop_command_node_id": "1",
            "spawn_controllers": "false",
        }.items(),
    )
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()]), {
        "can_interface": can_interface
    }


class TestAckTimeout(unittest.TestCase):
    def test_missing_ack_faults_and_stops_hardware(self, proc_output, can_interface):
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.bind((can_interface,))
        can_socket.settimeout(0.2)

        proc_output.assertWaitFor("ACK timeout for motor node 1", timeout=10.0)

        safe_stop_received = False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not safe_stop_received:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id not in (0x101, 0x102) or dlc != 8:
                continue
            sequence, flags, velocity = struct.unpack("<BBh", data[:4])
            safe_stop_received = sequence > 2 and flags == 0 and velocity == 0

        self.assertTrue(safe_stop_received)
