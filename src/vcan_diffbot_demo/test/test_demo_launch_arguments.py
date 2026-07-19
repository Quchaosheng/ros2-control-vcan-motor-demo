import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "demo.launch.py"


def load_launch_description():
    spec = importlib.util.spec_from_file_location("demo_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def action_is_enabled(action, start_virtual_motor):
    context = LaunchContext()
    context.launch_configurations["start_virtual_motor"] = start_virtual_motor
    context.launch_configurations["spawn_controllers"] = "true"
    return action.condition is None or action.condition.evaluate(context)


def substitution_text(substitutions):
    return perform_substitutions(LaunchContext(), substitutions)


def test_virtual_motor_can_be_disabled_without_disabling_the_control_stack():
    description = load_launch_description()
    arguments = [
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == "start_virtual_motor"
    ]
    assert len(arguments) == 1
    assert substitution_text(arguments[0].default_value) == "true"

    nodes = [action for action in description.entities if isinstance(action, Node)]
    virtual_motor = next(
        node for node in nodes if substitution_text(node.node_name) == "virtual_motor"
    )
    controller_manager = next(
        node
        for node in nodes
        if substitution_text(node.node_executable) == "ros2_control_node"
    )
    robot_state_publisher = next(
        node
        for node in nodes
        if substitution_text(node.node_executable) == "robot_state_publisher"
    )
    controller_spawners = [
        node
        for node in nodes
        if substitution_text(node.node_executable) == "spawner"
    ]
    assert len(controller_spawners) == 2

    assert action_is_enabled(virtual_motor, "true")
    assert not action_is_enabled(virtual_motor, "false")
    for action in [controller_manager, robot_state_publisher, *controller_spawners]:
        assert action_is_enabled(action, "true")
        assert action_is_enabled(action, "false")
