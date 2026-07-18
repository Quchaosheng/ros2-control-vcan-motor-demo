import socket
import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from vcan_test_utils import create_test_vcan


CAN_FRAME_FORMAT = "=IB3x8s"
CAN_ERR_FLAG = 0x20000000
CAN_ERR_MASK = 0x1FFFFFFF


@pytest.mark.launch_test
def generate_test_description():
    can_interface = create_test_vcan()
    motor = launch_ros.actions.Node(
        package="vcan_diffbot_demo",
        executable="virtual_motor_node",
        name="virtual_motor",
        output="screen",
        parameters=[
            {
                "can_interface": can_interface,
                "encoder_counts_per_revolution": 4096,
                "max_acceleration_rad_s2": 20.0,
                "update_rate_hz": 100.0,
                "feedback_rate_hz": 50.0,
                "feedback_delay_ms": 5,
                "malformed_feedback_every_n": 3,
                "error_frame_every_n": 4,
            }
        ],
    )
    return launch.LaunchDescription([motor, launch_testing.actions.ReadyToTest()]), {
        "can_interface": can_interface
    }


class TestVirtualMotorNode(unittest.TestCase):
    def test_ack_feedback_and_watchdog(self, proc_output, can_interface):
        proc_output.assertWaitFor(
            f"Virtual motors listening on {can_interface}", timeout=5.0
        )

        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.setsockopt(
            socket.SOL_CAN_RAW,
            socket.CAN_RAW_ERR_FILTER,
            struct.pack("=I", CAN_ERR_MASK),
        )
        can_socket.bind((can_interface,))
        can_socket.settimeout(0.2)

        command = struct.pack("<BBhH2x", 9, 1, 1500, 200)
        can_socket.send(struct.pack(CAN_FRAME_FORMAT, 0x101, 8, command))

        ack_received = False
        positive_feedback_received = False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not (
            ack_received and positive_feedback_received
        ):
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id == 0x281 and dlc == 8:
                ack_received = data[0] == 9 and data[1] == 0
            if can_id == 0x181 and dlc == 8 and data[0] == 9:
                positive_feedback_received = struct.unpack("<h", data[2:4])[0] > 0

        self.assertTrue(ack_received)
        self.assertTrue(positive_feedback_received)

        watchdog_stopped = False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not watchdog_stopped:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id == 0x181 and dlc == 8 and data[0] == 9:
                velocity = struct.unpack("<h", data[2:4])[0]
                watchdog_stopped = bool(data[1] & 0x02) and abs(velocity) < 50

        self.assertTrue(watchdog_stopped)

        error_frame_received = False
        malformed_feedback_received = False
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not (
            error_frame_received and malformed_feedback_received
        ):
            try:
                can_id, dlc, _ = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            error_frame_received = error_frame_received or bool(can_id & CAN_ERR_FLAG)
            malformed_feedback_received = malformed_feedback_received or (
                can_id in (0x181, 0x182) and dlc == 7
            )

        self.assertTrue(error_frame_received)
        self.assertTrue(malformed_feedback_received)
