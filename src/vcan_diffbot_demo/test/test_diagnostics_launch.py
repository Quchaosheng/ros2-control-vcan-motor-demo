import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy

from vcan_test_utils import create_test_vcan


STATUS_NAMES = {
    "vcan_diffbot/can_bus",
    "vcan_diffbot/left_motor",
    "vcan_diffbot/right_motor",
}
MOTOR_KEYS = {
    "node_id",
    "feedback_age_ms",
    "pending_ack_count",
    "last_ack_status",
    "wheel_velocity_rad_s",
    "ack_timeout",
    "feedback_timeout",
}


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
            "feedback_timeout_ms": "3000",
            "spawn_controllers": "false",
        }.items(),
    )
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestDiagnostics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_vcan_diffbot_diagnostics")
        cls.messages = []
        cls.subscription = cls.node.create_subscription(
            DiagnosticArray, "/diagnostics", cls.messages.append, 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def statuses(message):
        return {status.name: status for status in message.status}

    @staticmethod
    def values(status):
        return {item.key: item.value for item in status.values}

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            for message in self.messages:
                if predicate(message):
                    return message
        self.fail("Timed out waiting for matching /diagnostics message")

    def test_reports_healthy_then_feedback_timeout(self):
        def healthy(message):
            statuses = self.statuses(message)
            if set(statuses) != STATUS_NAMES:
                return False
            bus = statuses["vcan_diffbot/can_bus"]
            return bus.level == DiagnosticStatus.OK and self.values(bus).get("state") == "active"

        healthy_message = self.wait_for(healthy, 10.0)
        self.assertEqual(set(self.statuses(healthy_message)), STATUS_NAMES)

        def feedback_timeout(message):
            statuses = self.statuses(message)
            if set(statuses) != STATUS_NAMES:
                return False
            bus = statuses["vcan_diffbot/can_bus"]
            left = statuses["vcan_diffbot/left_motor"]
            right = statuses["vcan_diffbot/right_motor"]
            bus_values = self.values(bus)
            right_values = self.values(right)
            return (
                bus.level == DiagnosticStatus.ERROR
                and bus_values.get("state") == "safe_stop"
                and bus_values.get("stop_reason") == "feedback_timeout_node_2"
                and left.level == DiagnosticStatus.ERROR
                and left.message == "hardware safe stop"
                and right.level == DiagnosticStatus.ERROR
                and right_values.get("feedback_timeout") == "true"
            )

        fault_message = self.wait_for(feedback_timeout, 10.0)
        statuses = self.statuses(fault_message)
        left_values = self.values(statuses["vcan_diffbot/left_motor"])
        right_values = self.values(statuses["vcan_diffbot/right_motor"])
        self.assertEqual(set(left_values), MOTOR_KEYS)
        self.assertEqual(set(right_values), MOTOR_KEYS)
        self.assertEqual(right_values["node_id"], "2")
