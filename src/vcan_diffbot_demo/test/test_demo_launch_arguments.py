import importlib.util
from pathlib import Path

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
except ModuleNotFoundError:
    from launch_test_stubs import install

    install()
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


def action_is_enabled(action, start_virtual_motor, spawn_controllers="true"):
    context = LaunchContext()
    context.launch_configurations["start_virtual_motor"] = start_virtual_motor
    context.launch_configurations["spawn_controllers"] = spawn_controllers
    return action.condition is None or action.condition.evaluate(context)


def substitution_text(substitutions):
    if isinstance(substitutions, str):
        return substitutions
    if not isinstance(substitutions, (list, tuple)):
        substitutions = [substitutions]
    return perform_substitutions(LaunchContext(), substitutions)


def walk_entities(entities, seen=None):
    if seen is None:
        seen = set()
    for entity in entities:
        if id(entity) in seen:
            continue
        seen.add(id(entity))
        yield entity
        get_sub_entities = getattr(entity, "get_sub_entities", None)
        children = list(get_sub_entities()) if get_sub_entities else []
        # Humble's RegisterEventHandler does not expose on-exit actions via
        # get_sub_entities(), so traverse the event-handler relationships too.
        for attribute in ("event_handler", "on_exit", "target_action"):
            related = getattr(entity, attribute, None)
            if related is None:
                continue
            if isinstance(related, (list, tuple)):
                children.extend(related)
            else:
                children.append(related)
        yield from walk_entities(children, seen)


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

    nodes = [
        action
        for action in walk_entities(description.entities)
        if isinstance(action, Node)
    ]
    virtual_motor = next(
        node for node in nodes if substitution_text(node.node_executable) == "virtual_motor_node"
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
    for action in [controller_manager, robot_state_publisher]:
        assert action.condition is None
        assert action_is_enabled(action, "true")
        assert action_is_enabled(action, "false")
    for action in controller_spawners:
        condition_description = action.condition.describe()
        assert "spawn_controllers" in condition_description
        assert "start_virtual_motor" not in condition_description
        assert action_is_enabled(action, "true", "true")
        assert action_is_enabled(action, "false", "true")
        assert not action_is_enabled(action, "true", "false")
        assert not action_is_enabled(action, "false", "false")
