from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import re
import shlex
import yaml


def parse_launch_value(context, arg_name: str):
    raw_value = LaunchConfiguration(arg_name).perform(context)
    try:
        return yaml.safe_load(raw_value)
    except Exception:
        return raw_value


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _arg(name: str, default_value: str, choices=None):
    kwargs = {"default_value": default_value}
    if choices is not None:
        kwargs["choices"] = choices
    return DeclareLaunchArgument(name, **kwargs)


def common_launch_arguments(*, enable_gripper_haptic_default: str = "true"):
    return [
        _arg("log_level", "info"),
        _arg("enable_cmd_mux", "false"),
        _arg("cmd_mux_default_source", "teleop"),
        _arg("control_sched_policy", "off", choices=["off", "nice", "rr", "fifo"]),
        _arg("control_sched_nice", "0"),
        _arg("control_sched_rt_priority", "20"),
        _arg("control_sched_cpu_list", ""),
        _arg("auto_enable", "true"),
        _arg("enable_timeout", "15.0"),
        _arg("firmware_profile", "auto", choices=["auto", "default", "v111", "v112"]),
        _arg("speed_percent", "100"),
        _arg("publish_rate", "200.0"),
        _arg("control_rate", "200.0"),
        _arg("master_slave_enable", "true"),
        _arg("robot_feedback_timeout_sec", "0.2"),
        _arg("mit_enable_pos", "true"),
        _arg("mit_enable_tor", "true"),
        _arg("mit_enable_gravity", "true"),
        _arg("gravity_mix_mode", "additive"),
        _arg("mit_kp", "[0.02, 0.10, 0.15, 0.10, 0.10, 0.10, 0.08]"),
        _arg("mit_kd", "[0.02, 0.03, 0.01, 0.02, 0.02, 0.02, 0.02]"),
        _arg("mit_torque_scale", "[1.0, 0.12, 0.2, 0.1, 0.2, 0.20, 0.20]"),
        _arg(
            "mit_torque_feedback_sign",
            "[-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]",
        ),
        _arg("mit_max_torque_abs", "18.0"),
        _arg("follower_control_mode", "js", choices=["js", "mit_gravity"]),
        _arg("follower_mit_enable_gravity", "false"),
        _arg("follower_mit_kp", "[8.0, 8.0, 8.0, 8.0, 6.0, 5.0, 3.0]"),
        _arg("follower_mit_kd", "[0.8, 0.8, 0.8, 0.7, 0.5, 0.4, 0.3]"),
        _arg(
            "follower_mit_max_torque_abs",
            "[18.0, 18.0, 18.0, 14.0, 8.0, 8.0, 8.0]",
        ),
        _arg("follower_gravity_torque_scale", "[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"),
        _arg("follower_gravity_debug_enable", "false"),
        _arg("follower_gravity_debug_interval_sec", "1.0"),
        _arg("enforce_joint_limits", "true"),
        _arg("gravity_clip_to_limits", "true"),
        _arg("gravity_unwrap_joint_positions", "true"),
        _arg("gravity_max_joint_step", "0.0"),
        _arg("gravity_max_torque_delta_warn", "2.0"),
        _arg("gravity_joint_scale", "[1.0, 0.96, 0.97, 0.95, 1.0, 1.0, 1.0]"),
        _arg("leader_shutdown_cleanup_enable", "true"),
        _arg("leader_shutdown_ramp_sec", "0.6"),
        _arg("leader_shutdown_ramp_steps", "12"),
        _arg(
            "leader_shutdown_final_mode",
            "zero_force",
            choices=["zero_force", "leader", "normal", "disable", "none"],
        ),
        _arg("leader_shutdown_disable_on_failure", "true"),
        _arg("enable_gripper", "true"),
        _arg("enable_gripper_haptic", enable_gripper_haptic_default),
        _arg("gripper_startup_recovery_enable", "true"),
        _arg("gripper_mode_override", "width", choices=["width"]),
        _arg("gripper_range", "0.1"),
        _arg("gripper_reverse", "true"),
        _arg("gripper_val_multiple", "1.0"),
        _arg("gripper_follow_force_default", "3.0"),
        _arg("gripper_max_step", "0.0"),
        _arg("gripper_cmd_limit_enable", "true"),
        _arg("gripper_cmd_min_raw", "0"),
        _arg("gripper_cmd_max_raw", "100000"),
        _arg("gripper_cmd_deadband_raw", "200"),
        _arg("gripper_filter_enable", "true"),
        _arg("gripper_filter_alpha_position", "0.7"),
        _arg("gripper_filter_alpha_effort", "0.1"),
        _arg("gripper_haptic_effort_sign", "0.0"),
        _arg("gripper_haptic_effort_bias", "0.0"),
        _arg("gripper_haptic_effort_alpha", "0.2"),
        _arg("gripper_haptic_release_when_opening", "true"),
        _arg("gripper_haptic_opening_direction_eps", "0.0001"),
        _arg("gripper_haptic_force_max_n", "0.8"),
        _arg("gripper_haptic_constant_force_n", "0.0"),
        _arg("gripper_haptic_constant_directional", "false"),
        _arg("gripper_haptic_spring_align_enable", "false"),
        _arg("gripper_haptic_spring_hold_sec", "0.5"),
        _arg("gripper_haptic_spring_k_n_per_m", "80.0"),
        _arg("gripper_haptic_spring_min_force_n", "0.4"),
        _arg("gripper_haptic_mismatch_pulse_enable", "false"),
        _arg("gripper_haptic_mismatch_threshold_m", "0.006"),
        _arg("gripper_haptic_mismatch_reset_threshold_m", "0.003"),
        _arg("gripper_haptic_mismatch_pulse_sec", "0.25"),
        _arg("gripper_haptic_mismatch_force_threshold_n", "0.0"),
        _arg("gripper_haptic_mismatch_force_confirm_sec", "0.0"),
        _arg("gripper_haptic_mismatch_force_sign", "0.0"),
        _arg("gripper_haptic_mismatch_closing_only", "false"),
        _arg("gripper_haptic_mismatch_retrigger_sec", "0.0"),
        _arg("gripper_haptic_effort_deadband", "0.15"),
        _arg("gripper_haptic_effort_max", "0.6"),
        _arg("gripper_haptic_cmd_max", "500"),
        _arg("gripper_haptic_cmd_enable_threshold", "80"),
        _arg("gripper_haptic_opening_relief", "0.1"),
        _arg("gripper_haptic_opening_sign", "0.0"),
        _arg("gripper_haptic_force_ramp_up_n_per_sec", "4.0"),
        _arg("gripper_haptic_force_ramp_down_n_per_sec", "20.0"),
        _arg("gripper_v2_geom_scale", "1.0"),
        _arg("gripper_v2_geom_offset", "0.0"),
        _arg("leader_gripper_min_trusted_angle_position", "1.0"),
        _arg("gripper_recover_seed_master_open", "true"),
        _arg("gripper_recover_seed_angle_deg", "71.2"),
        _arg("gripper_recover_seed_width", "0.1"),
        _arg("gripper_recover_seed_force", "1.0"),
        _arg("gripper_recover_seed_min_width", "0.09"),
        _arg("gripper_v2_velocity_eps", "0.002"),
        _arg("gripper_v2_confirm_time_sec", "0.02"),
        _arg("gripper_v2_close_error_on", "0.003"),
        _arg("gripper_v2_close_error_off", "0.0015"),
        _arg("gripper_v2_open_error_on", "0.0035"),
        _arg("gripper_v2_open_error_off", "0.00175"),
        _arg("gripper_v2_k_close", "400.0"),
        _arg("gripper_v2_d_close", "4.0"),
        _arg("gripper_v2_fmax_close", "2.0"),
        _arg("gripper_v2_k_open", "220.0"),
        _arg("gripper_v2_d_open", "2.0"),
        _arg("gripper_v2_fmax_open", "1.2"),
        _arg("gripper_v2_force_low_close", "0.3"),
        _arg("gripper_v2_force_high_close", "1.2"),
        _arg("gripper_v2_force_low_open", "0.2"),
        _arg("gripper_v2_force_high_open", "0.8"),
        _arg("gripper_v2_lambda_close", "0.5"),
        _arg("gripper_v2_lambda_open", "0.2"),
    ]


def build_ctrl_params(context, teleop_ns: str, robot_ns: str):
    return {
        "teleop_ns": teleop_ns,
        "robot_ns": robot_ns,
        "auto_enable": parse_launch_value(context, "auto_enable"),
        "enable_timeout": parse_launch_value(context, "enable_timeout"),
        "firmware_profile": parse_launch_value(context, "firmware_profile"),
        "speed_percent": parse_launch_value(context, "speed_percent"),
        "publish_rate": parse_launch_value(context, "publish_rate"),
        "control_rate": parse_launch_value(context, "control_rate"),
        "master_slave_enable": parse_launch_value(context, "master_slave_enable"),
        "robot_feedback_timeout_sec": parse_launch_value(
            context, "robot_feedback_timeout_sec"
        ),
        "mit_enable_pos": parse_launch_value(context, "mit_enable_pos"),
        "mit_enable_tor": parse_launch_value(context, "mit_enable_tor"),
        "mit_enable_gravity": parse_launch_value(context, "mit_enable_gravity"),
        "gravity_mix_mode": parse_launch_value(context, "gravity_mix_mode"),
        "mit_kp": parse_launch_value(context, "mit_kp"),
        "mit_kd": parse_launch_value(context, "mit_kd"),
        "mit_torque_scale": parse_launch_value(context, "mit_torque_scale"),
        "mit_torque_feedback_sign": parse_launch_value(context, "mit_torque_feedback_sign"),
        "mit_max_torque_abs": parse_launch_value(context, "mit_max_torque_abs"),
        "follower_control_mode": parse_launch_value(context, "follower_control_mode"),
        "follower_mit_enable_gravity": parse_launch_value(
            context, "follower_mit_enable_gravity"
        ),
        "follower_mit_kp": parse_launch_value(context, "follower_mit_kp"),
        "follower_mit_kd": parse_launch_value(context, "follower_mit_kd"),
        "follower_mit_max_torque_abs": parse_launch_value(
            context, "follower_mit_max_torque_abs"
        ),
        "follower_gravity_torque_scale": parse_launch_value(
            context, "follower_gravity_torque_scale"
        ),
        "follower_gravity_debug_enable": parse_launch_value(
            context, "follower_gravity_debug_enable"
        ),
        "follower_gravity_debug_interval_sec": parse_launch_value(
            context, "follower_gravity_debug_interval_sec"
        ),
        "enforce_joint_limits": parse_launch_value(context, "enforce_joint_limits"),
        "unwrap_joint_positions": parse_launch_value(context, "gravity_unwrap_joint_positions"),
        "leader_shutdown_cleanup_enable": parse_launch_value(
            context, "leader_shutdown_cleanup_enable"
        ),
        "leader_shutdown_ramp_sec": parse_launch_value(
            context, "leader_shutdown_ramp_sec"
        ),
        "leader_shutdown_ramp_steps": parse_launch_value(
            context, "leader_shutdown_ramp_steps"
        ),
        "leader_shutdown_final_mode": parse_launch_value(
            context, "leader_shutdown_final_mode"
        ),
        "leader_shutdown_disable_on_failure": parse_launch_value(
            context, "leader_shutdown_disable_on_failure"
        ),
        "enable_gripper": parse_launch_value(context, "enable_gripper"),
        "enable_gripper_haptic": parse_launch_value(context, "enable_gripper_haptic"),
        "gripper_startup_recovery_enable": parse_launch_value(
            context, "gripper_startup_recovery_enable"
        ),
        "gripper_mode_override": parse_launch_value(context, "gripper_mode_override"),
        "gripper_range": parse_launch_value(context, "gripper_range"),
        "gripper_reverse": parse_launch_value(context, "gripper_reverse"),
        "gripper_val_multiple": parse_launch_value(context, "gripper_val_multiple"),
        "gripper_follow_force_default": parse_launch_value(
            context, "gripper_follow_force_default"
        ),
        "gripper_max_step": parse_launch_value(context, "gripper_max_step"),
        "gripper_cmd_limit_enable": parse_launch_value(
            context, "gripper_cmd_limit_enable"
        ),
        "gripper_cmd_min_raw": parse_launch_value(context, "gripper_cmd_min_raw"),
        "gripper_cmd_max_raw": parse_launch_value(context, "gripper_cmd_max_raw"),
        "gripper_cmd_deadband_raw": parse_launch_value(
            context, "gripper_cmd_deadband_raw"
        ),
        "gripper_filter_enable": parse_launch_value(context, "gripper_filter_enable"),
        "gripper_filter_alpha_position": parse_launch_value(
            context, "gripper_filter_alpha_position"
        ),
        "gripper_filter_alpha_effort": parse_launch_value(
            context, "gripper_filter_alpha_effort"
        ),
        "gripper_haptic_effort_sign": parse_launch_value(
            context, "gripper_haptic_effort_sign"
        ),
        "gripper_haptic_effort_bias": parse_launch_value(
            context, "gripper_haptic_effort_bias"
        ),
        "gripper_haptic_effort_alpha": parse_launch_value(
            context, "gripper_haptic_effort_alpha"
        ),
        "gripper_haptic_release_when_opening": parse_launch_value(
            context, "gripper_haptic_release_when_opening"
        ),
        "gripper_haptic_opening_direction_eps": parse_launch_value(
            context, "gripper_haptic_opening_direction_eps"
        ),
        "gripper_haptic_force_max_n": parse_launch_value(
            context, "gripper_haptic_force_max_n"
        ),
        "gripper_haptic_constant_force_n": parse_launch_value(
            context, "gripper_haptic_constant_force_n"
        ),
        "gripper_haptic_constant_directional": parse_launch_value(
            context, "gripper_haptic_constant_directional"
        ),
        "gripper_haptic_spring_align_enable": parse_launch_value(
            context, "gripper_haptic_spring_align_enable"
        ),
        "gripper_haptic_spring_hold_sec": parse_launch_value(
            context, "gripper_haptic_spring_hold_sec"
        ),
        "gripper_haptic_spring_k_n_per_m": parse_launch_value(
            context, "gripper_haptic_spring_k_n_per_m"
        ),
        "gripper_haptic_spring_min_force_n": parse_launch_value(
            context, "gripper_haptic_spring_min_force_n"
        ),
        "gripper_haptic_mismatch_pulse_enable": parse_launch_value(
            context, "gripper_haptic_mismatch_pulse_enable"
        ),
        "gripper_haptic_mismatch_threshold_m": parse_launch_value(
            context, "gripper_haptic_mismatch_threshold_m"
        ),
        "gripper_haptic_mismatch_reset_threshold_m": parse_launch_value(
            context, "gripper_haptic_mismatch_reset_threshold_m"
        ),
        "gripper_haptic_mismatch_pulse_sec": parse_launch_value(
            context, "gripper_haptic_mismatch_pulse_sec"
        ),
        "gripper_haptic_mismatch_force_threshold_n": parse_launch_value(
            context, "gripper_haptic_mismatch_force_threshold_n"
        ),
        "gripper_haptic_mismatch_force_confirm_sec": parse_launch_value(
            context, "gripper_haptic_mismatch_force_confirm_sec"
        ),
        "gripper_haptic_mismatch_force_sign": parse_launch_value(
            context, "gripper_haptic_mismatch_force_sign"
        ),
        "gripper_haptic_mismatch_closing_only": parse_launch_value(
            context, "gripper_haptic_mismatch_closing_only"
        ),
        "gripper_haptic_mismatch_retrigger_sec": parse_launch_value(
            context, "gripper_haptic_mismatch_retrigger_sec"
        ),
        "gripper_haptic_effort_deadband": parse_launch_value(
            context, "gripper_haptic_effort_deadband"
        ),
        "gripper_haptic_effort_max": parse_launch_value(
            context, "gripper_haptic_effort_max"
        ),
        "gripper_haptic_cmd_max": parse_launch_value(context, "gripper_haptic_cmd_max"),
        "gripper_haptic_cmd_enable_threshold": parse_launch_value(
            context, "gripper_haptic_cmd_enable_threshold"
        ),
        "gripper_haptic_opening_relief": parse_launch_value(
            context, "gripper_haptic_opening_relief"
        ),
        "gripper_haptic_opening_sign": parse_launch_value(
            context, "gripper_haptic_opening_sign"
        ),
        "gripper_haptic_force_ramp_up_n_per_sec": parse_launch_value(
            context, "gripper_haptic_force_ramp_up_n_per_sec"
        ),
        "gripper_haptic_force_ramp_down_n_per_sec": parse_launch_value(
            context, "gripper_haptic_force_ramp_down_n_per_sec"
        ),
        "gripper_v2_geom_scale": parse_launch_value(context, "gripper_v2_geom_scale"),
        "gripper_v2_geom_offset": parse_launch_value(context, "gripper_v2_geom_offset"),
        "leader_gripper_min_trusted_angle_position": parse_launch_value(
            context, "leader_gripper_min_trusted_angle_position"
        ),
        "gripper_recover_seed_master_open": parse_launch_value(
            context, "gripper_recover_seed_master_open"
        ),
        "gripper_recover_seed_angle_deg": parse_launch_value(
            context, "gripper_recover_seed_angle_deg"
        ),
        "gripper_recover_seed_width": parse_launch_value(
            context, "gripper_recover_seed_width"
        ),
        "gripper_recover_seed_force": parse_launch_value(
            context, "gripper_recover_seed_force"
        ),
        "gripper_recover_seed_min_width": parse_launch_value(
            context, "gripper_recover_seed_min_width"
        ),
        "gripper_v2_velocity_eps": parse_launch_value(context, "gripper_v2_velocity_eps"),
        "gripper_v2_confirm_time_sec": parse_launch_value(
            context, "gripper_v2_confirm_time_sec"
        ),
        "gripper_v2_close_error_on": parse_launch_value(
            context, "gripper_v2_close_error_on"
        ),
        "gripper_v2_close_error_off": parse_launch_value(
            context, "gripper_v2_close_error_off"
        ),
        "gripper_v2_open_error_on": parse_launch_value(
            context, "gripper_v2_open_error_on"
        ),
        "gripper_v2_open_error_off": parse_launch_value(
            context, "gripper_v2_open_error_off"
        ),
        "gripper_v2_k_close": parse_launch_value(context, "gripper_v2_k_close"),
        "gripper_v2_d_close": parse_launch_value(context, "gripper_v2_d_close"),
        "gripper_v2_fmax_close": parse_launch_value(context, "gripper_v2_fmax_close"),
        "gripper_v2_k_open": parse_launch_value(context, "gripper_v2_k_open"),
        "gripper_v2_d_open": parse_launch_value(context, "gripper_v2_d_open"),
        "gripper_v2_fmax_open": parse_launch_value(context, "gripper_v2_fmax_open"),
        "gripper_v2_force_low_close": parse_launch_value(
            context, "gripper_v2_force_low_close"
        ),
        "gripper_v2_force_high_close": parse_launch_value(
            context, "gripper_v2_force_high_close"
        ),
        "gripper_v2_force_low_open": parse_launch_value(
            context, "gripper_v2_force_low_open"
        ),
        "gripper_v2_force_high_open": parse_launch_value(
            context, "gripper_v2_force_high_open"
        ),
        "gripper_v2_lambda_close": parse_launch_value(context, "gripper_v2_lambda_close"),
        "gripper_v2_lambda_open": parse_launch_value(context, "gripper_v2_lambda_open"),
    }


def build_gravity_params(context):
    return {
        "clip_to_limits": parse_launch_value(context, "gravity_clip_to_limits"),
        "unwrap_joint_positions": parse_launch_value(
            context, "gravity_unwrap_joint_positions"
        ),
        "max_joint_step": parse_launch_value(context, "gravity_max_joint_step"),
        "max_torque_delta_warn": parse_launch_value(
            context, "gravity_max_torque_delta_warn"
        ),
        "gravity_joint_scale": parse_launch_value(context, "gravity_joint_scale"),
    }


def _str_launch_value(context, arg_name: str) -> str:
    value = parse_launch_value(context, arg_name)
    if value is None:
        return ""
    return str(value).strip()


def _parse_int_launch_value(
    context,
    arg_name: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    raw = _str_launch_value(context, arg_name)
    try:
        value = int(raw)
    except ValueError as exc:
        raise RuntimeError(f"{arg_name} must be an integer; got {raw!r}") from exc
    if value < minimum or value > maximum:
        raise RuntimeError(
            f"{arg_name} must be between {minimum} and {maximum}; got {value}"
        )
    return value


def control_sched_prefix(context) -> str | None:
    policy = _str_launch_value(context, "control_sched_policy").lower()
    if policy in {"", "off"}:
        return None

    parts: list[str] = []
    cpu_list = _str_launch_value(context, "control_sched_cpu_list")
    if cpu_list:
        if re.fullmatch(r"[0-9,-]+", cpu_list) is None:
            raise RuntimeError(
                "control_sched_cpu_list must look like '0-3' or '0,2,4'; "
                f"got {cpu_list!r}"
            )
        parts.extend(["taskset", "-c", cpu_list])

    if policy == "nice":
        nice_value = _parse_int_launch_value(
            context, "control_sched_nice", minimum=-20, maximum=19
        )
        parts.extend(["nice", "-n", str(nice_value)])
    elif policy in {"rr", "fifo"}:
        priority = _parse_int_launch_value(
            context, "control_sched_rt_priority", minimum=1, maximum=99
        )
        parts.extend(["chrt", "-r" if policy == "rr" else "-f", str(priority)])
    else:
        raise RuntimeError(f"unsupported control_sched_policy: {policy!r}")

    return " ".join(shlex.quote(part) for part in parts)


def _node_kwargs_with_prefix(control_prefix: str | None) -> dict:
    if not control_prefix:
        return {}
    return {"prefix": control_prefix}


def _build_cmd_mux_node(
    *,
    teleop_ns: str,
    robot_ns: str,
    cmd_mux_default_source: str,
    log_level: str,
    control_prefix: str | None = None,
):
    return Node(
        package="piper_nero_teleop",
        executable="joint_cmd_mux_node",
        namespace=robot_ns,
        name="joint_cmd_mux_node",
        parameters=[
            {
                "default_source": cmd_mux_default_source,
                "teleop_joint_topic": f"/{teleop_ns}/joint_states_single",
                "teleop_gripper_topic": f"/{teleop_ns}/gripper_intent",
                "policy_topic": f"/{robot_ns}/vla_joint_cmd",
                "output_topic": f"/{robot_ns}/joint_cmd_mux/output",
                "select_topic": f"/{robot_ns}/joint_cmd_mux/select",
                "selected_topic": f"/{robot_ns}/joint_cmd_mux/selected",
                "global_selected_topic": "/mux/selected",
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        **_node_kwargs_with_prefix(control_prefix),
    )


def _follower_remappings(*, teleop_ns: str, robot_ns: str, enable_cmd_mux: bool):
    if not enable_cmd_mux:
        return []
    return [
        ("joint_pos_cmd", "joint_cmd_mux/output"),
        (
            f"/{teleop_ns}/joint_states_single",
            f"/{robot_ns}/joint_cmd_mux/disabled_joint_states_single",
        ),
        (
            f"/{teleop_ns}/gripper_intent",
            f"/{robot_ns}/joint_cmd_mux/disabled_gripper_intent",
        ),
    ]


def build_teleop_arm_entities(
    context,
    *,
    role: str,
    teleop_ns: str,
    robot_ns: str,
    leader_can_port: str,
    follower_can_port: str,
):
    role = role.strip().lower()
    log_level = str(parse_launch_value(context, "log_level")).strip()
    enable_cmd_mux = as_bool(parse_launch_value(context, "enable_cmd_mux"))
    cmd_mux_default_source = str(
        parse_launch_value(context, "cmd_mux_default_source")
    ).strip()
    common_ctrl_params = build_ctrl_params(context, teleop_ns, robot_ns)
    gravity_params = build_gravity_params(context)
    control_prefix = control_sched_prefix(context)

    entities = []
    if role == "follower":
        follower_params = dict(common_ctrl_params)
        follower_params.update(
            {
                "role": "follower",
                "can_port": follower_can_port.strip(),
                "master_slave_enable": False,
                "mit_enable_pos": False,
                "mit_enable_tor": False,
                "mit_enable_gravity": False,
            }
        )
        if enable_cmd_mux:
            entities.append(
                _build_cmd_mux_node(
                    teleop_ns=teleop_ns,
                    robot_ns=robot_ns,
                    cmd_mux_default_source=cmd_mux_default_source,
                    log_level=log_level,
                    control_prefix=control_prefix,
                )
            )
        entities.append(
            Node(
                package="piper_nero_teleop",
                executable="nero_ctrl_node",
                namespace=robot_ns,
                name="nero_ctrl_node",
                parameters=[follower_params],
                remappings=_follower_remappings(
                    teleop_ns=teleop_ns,
                    robot_ns=robot_ns,
                    enable_cmd_mux=enable_cmd_mux,
                ),
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
                **_node_kwargs_with_prefix(control_prefix),
            )
        )
        if as_bool(parse_launch_value(context, "follower_mit_enable_gravity")):
            entities.append(
                Node(
                    package="piper_nero_teleop",
                    executable="nero_gravity_node",
                    namespace=robot_ns,
                    name="nero_gravity_node",
                    parameters=[gravity_params],
                    arguments=["--ros-args", "--log-level", log_level],
                    output="screen",
                    **_node_kwargs_with_prefix(control_prefix),
                )
            )
        return entities

    if role != "leader":
        raise RuntimeError("role must be one of: leader, follower")

    leader_params = dict(common_ctrl_params)
    leader_params.update(
        {
            "role": "leader",
            "can_port": leader_can_port.strip(),
        }
    )
    entities.append(
        Node(
            package="piper_nero_teleop",
            executable="nero_ctrl_node",
            namespace=teleop_ns,
            name="nero_ctrl_node",
            parameters=[leader_params],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
            **_node_kwargs_with_prefix(control_prefix),
        )
    )
    if as_bool(parse_launch_value(context, "mit_enable_gravity")) and not as_bool(
        parse_launch_value(context, "follower_mit_enable_gravity")
    ):
        entities.append(
            Node(
                package="piper_nero_teleop",
                executable="nero_gravity_node",
                namespace=robot_ns,
                name="nero_gravity_node",
                parameters=[gravity_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
                **_node_kwargs_with_prefix(control_prefix),
            )
        )
    return entities
