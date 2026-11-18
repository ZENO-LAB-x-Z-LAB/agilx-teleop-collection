from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from piper_nero_teleop.teleop_launch_common import (
    build_teleop_arm_entities,
    common_launch_arguments,
    parse_launch_value,
)


def _build_launch(context, *args, **kwargs):
    del args, kwargs
    role = str(parse_launch_value(context, "role")).strip().lower()

    entities = []
    entities.extend(
        build_teleop_arm_entities(
            context,
            role=role,
            teleop_ns="master_left",
            robot_ns="slave_left",
            leader_can_port=str(parse_launch_value(context, "leader_left_can_port")),
            follower_can_port=str(parse_launch_value(context, "follower_left_can_port")),
        )
    )
    entities.extend(
        build_teleop_arm_entities(
            context,
            role=role,
            teleop_ns="master_right",
            robot_ns="slave_right",
            leader_can_port=str(parse_launch_value(context, "leader_right_can_port")),
            follower_can_port=str(parse_launch_value(context, "follower_right_can_port")),
        )
    )
    return entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("role", default_value="leader", choices=["leader", "follower"]),
            DeclareLaunchArgument("leader_left_can_port", default_value="can_ml"),
            DeclareLaunchArgument("leader_right_can_port", default_value="can_mr"),
            DeclareLaunchArgument("follower_left_can_port", default_value="can_sl"),
            DeclareLaunchArgument("follower_right_can_port", default_value="can_sr"),
            *common_launch_arguments(enable_gripper_haptic_default="false"),
            OpaqueFunction(function=_build_launch),
        ]
    )
