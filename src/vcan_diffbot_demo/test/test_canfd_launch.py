"""CAN-FD profile regression against a vcan interface.

Requires a vcan interface, which needs CAP_NET_ADMIN. When the interface cannot
be created (no passwordless sudo, no vcan module) this test is skipped rather
than reported as a pass, so the FD protocol coverage that does not need a bus is
carried by test_canfd_protocol / test_can_transport / test_canfd_exchange.
"""

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


LEFT_NODE_ID = 31
RIGHT_NODE_ID = 37
LEFT_COMMAND_ID = 0x100 + LEFT_NODE_ID
LEFT_ACK_ID = 0x280 + LEFT_NODE_ID
LEFT_FEEDBACK_ID = 0x180 + LEFT_NODE_ID

CAN_RAW_FD_FRAMES = 5
CANFD_MTU = 72
CANFD_FRAME_FORMAT = "=IBB2x64s"


def fd_vcan_available():
    try:
        create_test_vcan()
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(
    not fd_vcan_available(),
    reason="vcan interface unavailable (needs CAP_NET_ADMIN); FD protocol logic is covered by the unit tests",
)


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
                "transport": "canfd",
                "left_node_id": LEFT_NODE_ID,
                "right_node_id": RIGHT_NODE_ID,
                "encoder_counts_per_revolution": 4096,
                "max_acceleration_rad_s2": 20.0,
                "update_rate_hz": 100.0,
                "feedback_rate_hz": 50.0,
            }
        ],
    )
    return launch.LaunchDescription([motor, launch_testing.actions.ReadyToTest()]), {
        "can_interface": can_interface
    }


class TestCanFdTransport(unittest.TestCase):
    def open_fd_socket(self, can_interface):
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.addCleanup(can_socket.close)
        can_socket.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        can_socket.bind((can_interface,))
        can_socket.settimeout(0.2)
        return can_socket

    def test_fd_command_ack_and_feedback(self, proc_output, can_interface):
        proc_output.assertWaitFor(
            f"Virtual motors listening on {can_interface}", timeout=5.0
        )
        can_socket = self.open_fd_socket(can_interface)

        # FD command layout v1: byte 1 version, byte 13 type 0x01, id 0x31 at
        # bytes 8-11, and the widened command id at bytes 4-7.
        command_id = 0x00010031
        payload = bytearray(20)
        payload[0] = command_id & 0xFF
        payload[1] = 1
        payload[2] = 1
        struct.pack_into("<I", payload, 4, command_id)
        struct.pack_into("<I", payload, 8, LEFT_NODE_ID)
        payload[13] = 0x01
        struct.pack_into("<h", payload, 14, 1500)
        struct.pack_into("<H", payload, 16, 200)

        can_socket.send(
            struct.pack(CANFD_FRAME_FORMAT, LEFT_COMMAND_ID, 20, 0, bytes(payload))
        )

        ack_seen = False
        feedback_seen = False
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not (ack_seen and feedback_seen):
            try:
                frame = can_socket.recv(CANFD_MTU)
            except socket.timeout:
                continue
            if len(frame) < 72:
                continue
            can_id, length, _flags, data = struct.unpack(CANFD_FRAME_FORMAT, frame)
            if can_id == LEFT_ACK_ID and length == 16:
                if data[1] == 1 and data[13] == 0x03 and struct.unpack_from("<I", data, 4)[0] == command_id:
                    ack_seen = True
            if can_id == LEFT_FEEDBACK_ID and length == 28:
                if data[1] == 1 and data[13] == 0x02:
                    feedback_seen = True

        self.assertTrue(ack_seen, "no CAN-FD ACK")
        self.assertTrue(feedback_seen, "no CAN-FD feedback")

    def test_malformed_fd_frame_is_rejected(self, proc_output, can_interface):
        proc_output.assertWaitFor(
            f"Virtual motors listening on {can_interface}", timeout=5.0
        )
        can_socket = self.open_fd_socket(can_interface)

        command_id = 0x00020031
        payload = bytearray(20)
        payload[0] = command_id & 0xFF
        payload[1] = 1
        payload[2] = 1
        struct.pack_into("<I", payload, 4, command_id)
        struct.pack_into("<I", payload, 8, LEFT_NODE_ID)
        payload[13] = 0x01
        # Dirty reserved byte: the decoder must refuse this frame.
        payload[19] = 0x7F

        can_socket.send(
            struct.pack(CANFD_FRAME_FORMAT, LEFT_COMMAND_ID, 20, 0, bytes(payload))
        )

        rejected = False
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not rejected:
            try:
                frame = can_socket.recv(CANFD_MTU)
            except socket.timeout:
                continue
            if len(frame) < 72:
                continue
            can_id, length, _flags, data = struct.unpack(CANFD_FRAME_FORMAT, frame)
            if can_id == LEFT_ACK_ID and length == 16 and data[14] != 0:
                rejected = True

        self.assertTrue(rejected, "malformed FD frame was not rejected with a non-zero result")
