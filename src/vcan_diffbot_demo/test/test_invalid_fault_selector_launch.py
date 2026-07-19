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
    can_interface = create_test_vcan()
    valid_motor = launch_ros.actions.Node(
        package="vcan_diffbot_demo",
        executable="virtual_motor_node",
        name="valid_virtual_motor",
        output="screen",
        parameters=[
            {
                "can_interface": can_interface,
                "left_node_id": 17,
                "right_node_id": 23,
                "drop_feedback_node_id": 23,
            }
        ],
    )
    invalid_motor = launch_ros.actions.Node(
        package="vcan_diffbot_demo",
        executable="virtual_motor_node",
        name="invalid_virtual_motor",
        output="screen",
        parameters=[
            {
                "can_interface": can_interface,
                "left_node_id": 17,
                "right_node_id": 23,
                "drop_feedback_node_id": 99,
            }
        ],
    )
    return launch.LaunchDescription(
        [valid_motor, invalid_motor, launch_testing.actions.ReadyToTest()]
    ), {
        "can_interface": can_interface,
        "valid_motor": valid_motor,
        "invalid_motor": invalid_motor,
    }


class TestInvalidFaultSelector(unittest.TestCase):
    def test_custom_selector_is_accepted_and_invalid_selector_exits(
        self, proc_info, proc_output, can_interface, valid_motor, invalid_motor
    ):
        proc_output.assertWaitFor(
            f"Virtual motors listening on {can_interface}",
            process=valid_motor,
            timeout=5.0,
        )
        proc_output.assertWaitFor(
            "drop_feedback_node_id must be 0 or one of configured motor node IDs 17, 23",
            process=invalid_motor,
            timeout=5.0,
        )
        proc_info.assertWaitForShutdown(process=invalid_motor, timeout=5.0)


@launch_testing.post_shutdown_test()
class TestInvalidFaultSelectorAfterShutdown(unittest.TestCase):
    def test_invalid_selector_exits_unsuccessfully(self, proc_info, invalid_motor):
        self.assertNotEqual(proc_info[invalid_motor].returncode, 0)
