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
    teleop_ns = str(parse_launch_value(context, "teleop_ns")).strip()
    robot_ns = str(parse_launch_value(context, "robot_ns")).strip()
    leader_can_port = str(parse_launch_value(context, "leader_can_port")).strip()
    follower_can_port = str(parse_launch_value(context, "follower_can_port")).strip()

    return build_teleop_arm_entities(
        context,
        role=role,
        teleop_ns=teleop_ns,
        robot_ns=robot_ns,
        leader_can_port=leader_can_port,
        follower_can_port=follower_can_port,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("role", default_value="leader", choices=["leader", "follower"]),
            DeclareLaunchArgument("leader_can_port", default_value="can_mr"),
            DeclareLaunchArgument("follower_can_port", default_value="can_sr"),
            DeclareLaunchArgument("teleop_ns", default_value="master_right"),
            DeclareLaunchArgument("robot_ns", default_value="slave_right"),
            *common_launch_arguments(enable_gripper_haptic_default="true"),
            OpaqueFunction(function=_build_launch),
        ]
    )
