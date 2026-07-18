import os
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import JointState


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
    demo = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file))
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestDemoLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_demo_launch")
        cls.latest_joint_state = None
        cls.latest_odom = None
        cls.command_publisher = cls.node.create_publisher(
            TwistStamped, "/diffbot_base_controller/cmd_vel", 10
        )
        cls.node.create_subscription(
            JointState,
            "/joint_states",
            lambda message: setattr(cls, "latest_joint_state", message),
            10,
        )
        cls.node.create_subscription(
            Odometry,
            "/diffbot_base_controller/odom",
            lambda message: setattr(cls, "latest_odom", message),
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def spin_until(cls, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def wheel_velocities(cls):
        if cls.latest_joint_state is None:
            return None
        velocities = dict(
            zip(cls.latest_joint_state.name, cls.latest_joint_state.velocity)
        )
        if "left_wheel_joint" not in velocities or "right_wheel_joint" not in velocities:
            return None
        return velocities["left_wheel_joint"], velocities["right_wheel_joint"]

    def test_forward_motion_and_stop(self):
        ready = self.spin_until(
            lambda: self.command_publisher.get_subscription_count() > 0
            and self.wheel_velocities() is not None,
            15.0,
        )
        self.assertTrue(ready)

        command = TwistStamped()
        command.twist.linear.x = 0.3
        drive_deadline = time.monotonic() + 2.0
        while time.monotonic() < drive_deadline:
            command.header.stamp = self.node.get_clock().now().to_msg()
            self.command_publisher.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        velocities = self.wheel_velocities()
        self.assertIsNotNone(velocities)
        self.assertGreater(velocities[0], 0.1)
        self.assertGreater(velocities[1], 0.1)
        self.assertIsNotNone(self.latest_odom)
        self.assertGreater(self.latest_odom.pose.pose.position.x, 0.01)

        stopped = self.spin_until(
            lambda: self.wheel_velocities() is not None
            and abs(self.wheel_velocities()[0]) < 0.05
            and abs(self.wheel_velocities()[1]) < 0.05,
            3.0,
        )
        self.assertTrue(stopped)
