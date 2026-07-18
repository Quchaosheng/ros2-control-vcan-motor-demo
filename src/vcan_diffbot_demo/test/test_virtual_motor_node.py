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
LEFT_NODE_ID = 17
RIGHT_NODE_ID = 23
LEFT_COMMAND_ID = 0x100 + LEFT_NODE_ID
LEFT_FEEDBACK_ID = 0x180 + LEFT_NODE_ID
LEFT_ACK_ID = 0x280 + LEFT_NODE_ID
RIGHT_COMMAND_ID = 0x100 + RIGHT_NODE_ID
RIGHT_FEEDBACK_ID = 0x180 + RIGHT_NODE_ID
RIGHT_ACK_ID = 0x280 + RIGHT_NODE_ID
FEEDBACK_IDS = (LEFT_FEEDBACK_ID, RIGHT_FEEDBACK_ID)
RESPONSE_IDS = (LEFT_FEEDBACK_ID, LEFT_ACK_ID, RIGHT_FEEDBACK_ID, RIGHT_ACK_ID)
LEGACY_RESPONSE_IDS = (0x181, 0x182, 0x281, 0x282)
ALL_ACK_IDS = (LEFT_ACK_ID, RIGHT_ACK_ID, 0x281, 0x282)
ALL_FEEDBACK_IDS = (LEFT_FEEDBACK_ID, RIGHT_FEEDBACK_ID, 0x181, 0x182)


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
                "left_node_id": LEFT_NODE_ID,
                "right_node_id": RIGHT_NODE_ID,
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

        left_command = struct.pack("<BBhH2x", 9, 1, 1500, 200)
        right_command = struct.pack("<BBhH2x", 10, 1, 1200, 200)
        can_socket.send(
            struct.pack(CAN_FRAME_FORMAT, LEFT_COMMAND_ID, 8, left_command)
        )
        can_socket.send(
            struct.pack(CAN_FRAME_FORMAT, RIGHT_COMMAND_ID, 8, right_command)
        )

        ack_sequences = set()
        positive_feedback_sequences = set()
        response_frames = 0
        legacy_response_ids = set()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id in LEGACY_RESPONSE_IDS:
                legacy_response_ids.add(can_id)
            if can_id in RESPONSE_IDS:
                response_frames += 1
            if can_id == LEFT_ACK_ID and dlc == 8:
                if data[0] == 9 and data[1] == 0:
                    ack_sequences.add(data[0])
            if can_id == RIGHT_ACK_ID and dlc == 8:
                if data[0] == 10 and data[1] == 0:
                    ack_sequences.add(data[0])
            if can_id == LEFT_FEEDBACK_ID and dlc == 8 and data[0] == 9:
                if struct.unpack("<h", data[2:4])[0] > 0:
                    positive_feedback_sequences.add(data[0])
            if can_id == RIGHT_FEEDBACK_ID and dlc == 8 and data[0] == 10:
                if struct.unpack("<h", data[2:4])[0] > 0:
                    positive_feedback_sequences.add(data[0])

        self.assertEqual(ack_sequences, {9, 10})
        self.assertEqual(positive_feedback_sequences, {9, 10})
        self.assertGreaterEqual(response_frames, 4)

        legacy_command_sequences = {201, 202}
        can_socket.send(
            struct.pack(
                CAN_FRAME_FORMAT,
                0x101,
                8,
                struct.pack("<BBhH2x", 201, 1, 1001, 200),
            )
        )
        can_socket.send(
            struct.pack(
                CAN_FRAME_FORMAT,
                0x102,
                8,
                struct.pack("<BBhH2x", 202, 1, 1201, 200),
            )
        )

        legacy_ack_sequences = set()
        legacy_feedback_sequences = set()
        configured_feedback_frames = 0
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id in LEGACY_RESPONSE_IDS:
                legacy_response_ids.add(can_id)
            if can_id in FEEDBACK_IDS and dlc == 8:
                configured_feedback_frames += 1
            if can_id in ALL_ACK_IDS and dlc == 8 and data[0] in legacy_command_sequences:
                legacy_ack_sequences.add(data[0])
            if (
                can_id in ALL_FEEDBACK_IDS
                and dlc == 8
                and data[0] in legacy_command_sequences
            ):
                legacy_feedback_sequences.add(data[0])

        self.assertGreater(configured_feedback_frames, 0)
        self.assertEqual(legacy_ack_sequences, set())
        self.assertEqual(legacy_feedback_sequences, set())

        watchdog_stopped = False
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not watchdog_stopped:
            try:
                can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, can_socket.recv(16))
            except socket.timeout:
                continue
            if can_id in LEGACY_RESPONSE_IDS:
                legacy_response_ids.add(can_id)
            if can_id == LEFT_FEEDBACK_ID and dlc == 8 and data[0] == 9:
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
            if can_id in LEGACY_RESPONSE_IDS:
                legacy_response_ids.add(can_id)
            error_frame_received = error_frame_received or bool(can_id & CAN_ERR_FLAG)
            malformed_feedback_received = malformed_feedback_received or (
                can_id in FEEDBACK_IDS and dlc == 7
            )

        self.assertTrue(error_frame_received)
        self.assertTrue(malformed_feedback_received)
        self.assertEqual(legacy_response_ids, set())
