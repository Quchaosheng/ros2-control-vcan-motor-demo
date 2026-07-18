from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    drop_command_every_n = LaunchConfiguration("drop_command_every_n")
    drop_feedback_every_n = LaunchConfiguration("drop_feedback_every_n")
    feedback_delay_ms = LaunchConfiguration("feedback_delay_ms")
    malformed_feedback_every_n = LaunchConfiguration("malformed_feedback_every_n")
    error_frame_every_n = LaunchConfiguration("error_frame_every_n")

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
    )
    start_diffbot_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diffbot_controller],
        )
    )

    arguments = [
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("drop_command_every_n", default_value="0"),
        DeclareLaunchArgument("drop_feedback_every_n", default_value="0"),
        DeclareLaunchArgument("feedback_delay_ms", default_value="0"),
        DeclareLaunchArgument("malformed_feedback_every_n", default_value="0"),
        DeclareLaunchArgument("error_frame_every_n", default_value="0"),
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
