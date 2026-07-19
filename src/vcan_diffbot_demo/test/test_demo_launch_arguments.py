import ast
from pathlib import Path


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "demo.launch.py"


def test_virtual_motor_can_be_disabled_without_affecting_other_nodes():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    argument_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "start_virtual_motor"
    ]
    assert len(argument_calls) == 1
    default_value = next(
        keyword.value
        for keyword in argument_calls[0].keywords
        if keyword.arg == "default_value"
    )
    assert isinstance(default_value, ast.Constant)
    assert default_value.value == "true"

    virtual_motor = next(
        node
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "virtual_motor"
            for target in node.targets
        )
    )
    node_call = virtual_motor.value
    condition = next(
        keyword.value for keyword in node_call.keywords if keyword.arg == "condition"
    )
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == "IfCondition"
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "start_virtual_motor"
