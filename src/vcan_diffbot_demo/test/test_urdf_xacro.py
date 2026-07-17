import os
import subprocess

from ament_index_python.packages import get_package_share_directory


def test_diffbot_xacro_contains_can_hardware_and_two_wheels():
    share = get_package_share_directory("vcan_diffbot_demo")
    path = os.path.join(share, "urdf", "diffbot.urdf.xacro")
    xml = subprocess.check_output(["xacro", path], text=True)

    assert "vcan_diffbot_demo/CanMotorHardware" in xml
    assert 'joint name="left_wheel_joint"' in xml
    assert 'joint name="right_wheel_joint"' in xml
    assert '<param name="can_interface">vcan0</param>' in xml
    assert '<param name="left_node_id">1</param>' in xml
    assert '<param name="right_node_id">2</param>' in xml
    assert '<param name="encoder_counts_per_revolution">4096</param>' in xml
    assert '<param name="command_watchdog_ms">200</param>' in xml
    assert '<param name="feedback_timeout_ms">500</param>' in xml
