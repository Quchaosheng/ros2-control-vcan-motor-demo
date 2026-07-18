import os
import socket
import struct
import subprocess
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


CAN_FRAME_FORMAT = "=IB3x8s"


def ensure_vcan():
    subprocess.run(["sudo", "modprobe", "vcan"], check=False)
    if subprocess.run(["ip", "link", "show", "vcan0"], check=False).returncode != 0:
        subprocess.run(
            ["sudo", "ip", "link", "add", "dev", "vcan0", "type", "vcan"],
            check=True,
        )
    subprocess.run(["sudo", "ip", "link", "set", "up", "vcan0"], check=True)


@pytest.mark.launch_test
def generate_test_description():
    ensure_vcan()
    launch_file = os.path.join(
        get_package_share_directory("vcan_diffbot_demo"), "launch", "demo.launch.py"
    )
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "drop_feedback_every_n": "1",
            "spawn_controllers": "false",
        }.items(),
    )
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestFeedbackTimeout(unittest.TestCase):
    def test_timeout_sends_disabled_zero_commands(self, proc_output):
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.bind(("vcan0",))
        can_socket.settimeout(0.2)

        proc_output.assertWaitFor("Feedback timeout for motor node", timeout=10.0)

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
            safe_stop_received = sequence > 5 and flags == 0 and velocity == 0

        self.assertTrue(safe_stop_received)
