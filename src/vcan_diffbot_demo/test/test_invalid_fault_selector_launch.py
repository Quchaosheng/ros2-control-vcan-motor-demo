import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from vcan_test_utils import create_test_vcan


@pytest.mark.launch_test
def generate_test_description():
    motor = launch_ros.actions.Node(
        package="vcan_diffbot_demo",
        executable="virtual_motor_node",
        name="virtual_motor",
        output="screen",
        parameters=[
            {
                "can_interface": create_test_vcan(),
                "left_node_id": 1,
                "right_node_id": 2,
                "drop_feedback_node_id": 99,
            }
        ],
    )
    return launch.LaunchDescription([motor, launch_testing.actions.ReadyToTest()])


class TestInvalidFaultSelector(unittest.TestCase):
    def test_invalid_feedback_selector_is_rejected(self, proc_output):
        proc_output.assertWaitFor(
            "drop_feedback_node_id must be 0 or one of configured motor node IDs 1, 2",
            timeout=5.0,
        )
