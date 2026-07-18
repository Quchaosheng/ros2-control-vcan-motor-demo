from vcan_test_utils import interface_name, privileged_command


def test_interface_name_is_unique_and_fits_linux_limit():
    assert interface_name(1234) == "vcan1234"
    assert interface_name(1234) != interface_name(5678)
    assert len(interface_name(123456789012345)) <= 15


def test_privileged_command_is_direct_for_root():
    assert privileged_command(["ip", "link"], uid=0) == ["ip", "link"]


def test_privileged_command_uses_noninteractive_sudo_for_user():
    assert privileged_command(["ip", "link"], uid=1000) == [
        "sudo",
        "-n",
        "ip",
        "link",
    ]
