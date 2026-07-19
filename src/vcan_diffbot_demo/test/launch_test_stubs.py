import sys
import types


class LaunchContext:
    def __init__(self):
        self.launch_configurations = {}


class Entity:
    def get_sub_entities(self):
        return []


class TextSubstitution:
    def __init__(self, text):
        self.text = text

    def perform(self, _context):
        return self.text


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.launch_configurations[self.name]


class IfCondition:
    def __init__(self, predicate):
        self.predicate = predicate

    def evaluate(self, context):
        return self.predicate.perform(context).lower() in ("1", "true", "yes")

    def describe(self):
        return "IfCondition(LaunchConfiguration('{}'))".format(self.predicate.name)


class DeclareLaunchArgument(Entity):
    def __init__(self, name, default_value=None):
        self.name = name
        self.default_value = [TextSubstitution(default_value)]


class Node(Entity):
    def __init__(self, package, executable, name=None, condition=None, **_kwargs):
        self.node_package = [TextSubstitution(package)]
        self.node_executable = [TextSubstitution(executable)]
        self.node_name = [TextSubstitution(name)]
        self.condition = condition


class OnProcessExit(Entity):
    def __init__(self, target_action, on_exit):
        self.target_action = target_action
        self.on_exit = on_exit

    def get_sub_entities(self):
        return [self.target_action, *self.on_exit]


class RegisterEventHandler(Entity):
    def __init__(self, event_handler):
        self.event_handler = event_handler

    def get_sub_entities(self):
        return [self.event_handler]


class ParameterValue:
    def __init__(self, value, **_kwargs):
        self.value = value


class ValueSubstitution(TextSubstitution):
    def __init__(self, value=None, **kwargs):
        super().__init__(value or kwargs.get("name", ""))


def perform_substitutions(context, substitutions):
    return "".join(substitution.perform(context) for substitution in substitutions)


def install():
    launch = types.ModuleType("launch")
    launch.LaunchContext = LaunchContext
    launch.LaunchDescription = lambda entities: types.SimpleNamespace(entities=entities)
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = DeclareLaunchArgument
    actions.RegisterEventHandler = RegisterEventHandler
    conditions = types.ModuleType("launch.conditions")
    conditions.IfCondition = IfCondition
    event_handlers = types.ModuleType("launch.event_handlers")
    event_handlers.OnProcessExit = OnProcessExit
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.Command = ValueSubstitution
    substitutions.FindExecutable = ValueSubstitution
    substitutions.LaunchConfiguration = LaunchConfiguration
    substitutions.PathJoinSubstitution = ValueSubstitution
    utilities = types.ModuleType("launch.utilities")
    utilities.perform_substitutions = perform_substitutions
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = Node
    parameter_descriptions = types.ModuleType("launch_ros.parameter_descriptions")
    parameter_descriptions.ParameterValue = ParameterValue
    launch_ros_substitutions = types.ModuleType("launch_ros.substitutions")
    launch_ros_substitutions.FindPackageShare = ValueSubstitution

    sys.modules.update(
        {
            "launch": launch,
            "launch.actions": actions,
            "launch.conditions": conditions,
            "launch.event_handlers": event_handlers,
            "launch.substitutions": substitutions,
            "launch.utilities": utilities,
            "launch_ros": launch_ros,
            "launch_ros.actions": launch_ros_actions,
            "launch_ros.parameter_descriptions": parameter_descriptions,
            "launch_ros.substitutions": launch_ros_substitutions,
        }
    )
