import atexit
import os
import subprocess


def interface_name(pid=None):
    digits = str(abs(os.getpid() if pid is None else int(pid)))[-11:]
    return f"vcan{digits}"


def privileged_command(command, uid=None):
    effective_uid = os.geteuid() if uid is None else uid
    return list(command) if effective_uid == 0 else ["sudo", "-n", *command]


def create_test_vcan():
    name = interface_name()
    subprocess.run(
        privileged_command(["modprobe", "vcan"]),
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if subprocess.run(
        ["ip", "link", "show", name],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0:
        return name

    subprocess.run(
        privileged_command(["ip", "link", "add", "dev", name, "type", "vcan"]),
        check=True,
    )
    subprocess.run(
        privileged_command(["ip", "link", "set", "up", name]),
        check=True,
    )

    def cleanup():
        subprocess.run(
            privileged_command(["ip", "link", "delete", name]),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    atexit.register(cleanup)
    return name
