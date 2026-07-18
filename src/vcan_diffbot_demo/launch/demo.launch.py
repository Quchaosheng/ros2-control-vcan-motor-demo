from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    left_node_id = LaunchConfiguration("left_node_id")
    right_node_id = LaunchConfiguration("right_node_id")
    encoder_counts_per_revolution = LaunchConfiguration(
        "encoder_counts_per_revolution"
    )
    command_watchdog_ms = LaunchConfiguration("command_watchdog_ms")
    feedback_timeout_ms = LaunchConfiguration("feedback_timeout_ms")
    drop_command_every_n = LaunchConfiguration("drop_command_every_n")
    drop_feedback_every_n = LaunchConfiguration("drop_feedback_every_n")
    feedback_delay_ms = LaunchConfiguration("feedback_delay_ms")
    malformed_feedback_every_n = LaunchConfiguration("malformed_feedback_every_n")
    error_frame_every_n = LaunchConfiguration("error_frame_every_n")
    spawn_controllers = LaunchConfiguration("spawn_controllers")

    package_share = FindPackageShare("vcan_diffbot_demo")
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    ' "',
                    PathJoinSubstitution(
                        [package_share, "urdf", "diffbot.urdf.xacro"]
                    ),
                    '" can_interface:=',
                    can_interface,
                    " left_node_id:=",
                    left_node_id,
                    " right_node_id:=",
                    right_node_id,
                    " encoder_counts_per_revolution:=",
                    encoder_counts_per_revolution,
                    " command_watchdog_ms:=",
                    command_watchdog_ms,
                    " feedback_timeout_ms:=",
                    feedback_timeout_ms,
                ]
            ),
            value_type=str,
        )
    }
    controllers = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
    )
    motor_parameters = PathJoinSubstitution(
        [package_share, "config", "virtual_motor.yaml"]
    )

    virtual_motor = Node(
        package="vcan_diffbot_demo",
        executable="virtual_motor_node",
        name="virtual_motor",
        output="screen",
        parameters=[
            motor_parameters,
            {
                "can_interface": can_interface,
                "left_node_id": ParameterValue(left_node_id, value_type=int),
                "right_node_id": ParameterValue(right_node_id, value_type=int),
                "encoder_counts_per_revolution": ParameterValue(
                    encoder_counts_per_revolution, value_type=int
                ),
                "drop_command_every_n": ParameterValue(
                    drop_command_every_n, value_type=int
                ),
                "drop_feedback_every_n": ParameterValue(
                    drop_feedback_every_n, value_type=int
                ),
                "feedback_delay_ms": ParameterValue(feedback_delay_ms, value_type=int),
                "malformed_feedback_every_n": ParameterValue(
                    malformed_feedback_every_n, value_type=int
                ),
                "error_frame_every_n": ParameterValue(
                    error_frame_every_n, value_type=int
                ),
            },
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
        condition=IfCondition(spawn_controllers),
    )
    diffbot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
        condition=IfCondition(spawn_controllers),
    )
    start_diffbot_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diffbot_controller],
        )
    )

    arguments = [
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("left_node_id", default_value="1"),
        DeclareLaunchArgument("right_node_id", default_value="2"),
        DeclareLaunchArgument(
            "encoder_counts_per_revolution", default_value="4096"
        ),
        DeclareLaunchArgument("command_watchdog_ms", default_value="200"),
        DeclareLaunchArgument("feedback_timeout_ms", default_value="500"),
        DeclareLaunchArgument("drop_command_every_n", default_value="0"),
        DeclareLaunchArgument("drop_feedback_every_n", default_value="0"),
        DeclareLaunchArgument("feedback_delay_ms", default_value="0"),
        DeclareLaunchArgument("malformed_feedback_every_n", default_value="0"),
        DeclareLaunchArgument("error_frame_every_n", default_value="0"),
        DeclareLaunchArgument("spawn_controllers", default_value="true"),
    ]
    return LaunchDescription(
        arguments
        + [
            virtual_motor,
            robot_state_publisher,
            control_node,
            joint_state_broadcaster,
            start_diffbot_after_broadcaster,
        ]
    )
