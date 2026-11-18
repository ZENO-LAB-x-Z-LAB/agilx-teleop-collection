#!/usr/bin/env python3
import math
import threading
import time
from typing import List, Optional, Tuple

from pyAgxArm import AgxArmFactory, NeroFW, create_agx_arm_config

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Empty, Int32

    from agx_arm_msgs.msg import AgxArmStatus, GripperStatus
except ModuleNotFoundError as exc:
    if exc.name not in {
        "rclpy",
        "builtin_interfaces",
        "sensor_msgs",
        "std_msgs",
        "agx_arm_msgs",
    }:
        raise

    class _MissingROS2:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 Python packages are required for NeroCtrlNode runtime")

    class _MissingRclpy:
        def __getattr__(self, name):
            raise RuntimeError("ROS2 Python packages are required for NeroCtrlNode runtime")

    class _OfflineQoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    class _OfflinePolicy:
        KEEP_LAST = "keep_last"
        RELIABLE = "reliable"
        TRANSIENT_LOCAL = "transient_local"

    rclpy = _MissingRclpy()
    Time = JointState = Bool = Empty = Int32 = AgxArmStatus = GripperStatus = _MissingROS2
    Node = object
    QoSProfile = _OfflineQoSProfile
    HistoryPolicy = ReliabilityPolicy = DurabilityPolicy = _OfflinePolicy

from agx_arm_ctrl.effector import AgxGripperWrapper, GripperStatus as DriverGripperStatus
from piper_nero_teleop.gripper_haptic import (
    HAPTIC_ACTIVE,
    HAPTIC_FREE_FOLLOW,
    HAPTIC_MISMATCH_PULSE,
    HAPTIC_OPENING_RELEASE,
    HAPTIC_SPRING_ALIGN,
    HAPTIC_SPRING_HOLD,
    HAPTIC_STALE,
    CueHapticConfig,
    CueHapticResult,
    CueHapticController,
    constant_resistance_result,
    mismatch_force_ready,
    mismatch_pulse_result,
    spring_align_result,
    spring_hold_result,
)
from piper_nero_teleop.gripper_width import (
    follower_to_master_opening,
    master_to_follower_opening,
    require_formal_width_from_status,
)

HOT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
LATCHED_HOT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
GRIPPER_JOINT_NAME = "gripper"
GRIPPER_INTENT_TOPIC = "gripper_intent"


class GripperWidthModeError(RuntimeError):
    pass


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_vector(values, length: int, default: float) -> List[float]:
    if not isinstance(values, (list, tuple)) or len(values) == 0:
        return [default] * length
    if len(values) == 1:
        return [float(values[0])] * length
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {len(values)}")
    return [float(value) for value in values]


def _select_nero_firmware_profile(software_version: str):
    try:
        major_text, minor_text, *_ = str(software_version).strip().split(".")
        version = (int(major_text), int(minor_text))
    except (TypeError, ValueError):
        return NeroFW.DEFAULT

    if version >= (1, 12):
        return NeroFW.V112
    if version >= (1, 11):
        return NeroFW.V111
    return NeroFW.DEFAULT


def _normalize_nero_firmware_profile(profile: str):
    normalized = str(profile or "auto").strip().lower()
    if normalized in {"", "auto", "detect"}:
        return None
    if normalized in {"default", "v110", "1.10", "1.09"}:
        return NeroFW.DEFAULT
    if normalized in {"v111", "1.11"}:
        return NeroFW.V111
    if normalized in {"v112", "1.12", "1.13", "new", "latest"}:
        return NeroFW.V112
    raise ValueError(
        "firmware_profile must be one of: auto, default, v111, v112"
    )


class MasterGripperAdapter:
    """Normalize Nero master feedback into a direct logical width.

    On this Nero master gripper, valid runtime feedback often reports
    `mode=angle` while the numeric value is stroke-like millimeters. Keep this
    path 1:1 with the old working behavior: 99.8 means about 99.8 mm.
    """

    def __init__(
        self,
        gripper_range: float,
    ):
        self.gripper_range = max(1e-6, float(gripper_range))
        self._last_logical_opening: Optional[float] = None

    def reset(self):
        self._last_logical_opening = None

    @property
    def has_history(self) -> bool:
        return self._last_logical_opening is not None

    @property
    def last_opening(self) -> Optional[float]:
        return self._last_logical_opening

    def opening_from_status(self, status: Optional[DriverGripperStatus]) -> float:
        if status is None:
            return 0.0
        if status.mode != "angle":
            raw_width = float(status.width if not math.isnan(status.width) else status.position)
            if not math.isfinite(raw_width):
                return self._last_or_default()
            return self._direct_update(raw_width)
        raw_angle = float(status.position)
        if not math.isfinite(raw_angle):
            return self._last_or_default()
        return self._opening_from_stroke_like_angle(raw_angle)

    def _last_or_default(self) -> float:
        if self._last_logical_opening is not None:
            return float(self._last_logical_opening)
        return 0.0

    def _direct_update(self, candidate_opening: float) -> float:
        candidate = _clamp(
            float(candidate_opening),
            0.0,
            self.gripper_range,
        )
        self._last_logical_opening = candidate
        return candidate

    def _opening_from_stroke_like_angle(self, raw_angle: float) -> float:
        raw_value = float(raw_angle)
        stroke_width = raw_value * 1e-3
        return self._direct_update(stroke_width)


class NeroCtrlNode(Node):
    def __init__(self):
        super().__init__("nero_ctrl_node")

        self._shutdown_event = threading.Event()
        self._declare_parameters()
        self._load_parameters()
        self._init_runtime_state()
        self._init_arm()
        self._setup_ros_interfaces()

        self._publisher_thread = threading.Thread(
            target=self._publisher_loop,
            name=f"{self.role}_publisher",
            daemon=True,
        )
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name=f"{self.role}_control",
            daemon=True,
        )
        self._publisher_thread.start()
        self._control_thread.start()

    def _declare_parameters(self):
        self.declare_parameter("role", "leader")
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("teleop_ns", "master_right")
        self.declare_parameter("robot_ns", "slave_right")
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("firmware_profile", "auto")
        self.declare_parameter("speed_percent", 100)
        self.declare_parameter("enable_timeout", 15.0)
        self.declare_parameter("publish_rate", 200.0)
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("filter_enable", True)
        self.declare_parameter("filter_alpha_pos", 0.7)
        self.declare_parameter("filter_alpha_vel", 0.5)
        self.declare_parameter("filter_alpha_effort", 0.5)

        self.declare_parameter("master_slave_enable", True)
        self.declare_parameter("mit_enable_pos", True)
        self.declare_parameter("mit_enable_tor", True)
        self.declare_parameter("mit_enable_gravity", True)
        self.declare_parameter("gravity_mix_mode", "additive")
        self.declare_parameter("mit_kp", [0.02, 0.10, 0.15, 0.10, 0.10, 0.10, 0.08])
        self.declare_parameter("mit_kd", [0.02, 0.03, 0.01, 0.02, 0.02, 0.02, 0.02])
        self.declare_parameter("mit_torque_scale", [1.0, 0.12, 0.20, 0.10, 0.20, 0.20, 0.20])
        self.declare_parameter("mit_torque_feedback_sign", [-1.0] * 7)
        self.declare_parameter("mit_max_torque_abs", 18.0)
        self.declare_parameter("follower_control_mode", "js")
        self.declare_parameter("follower_mit_enable_gravity", False)
        self.declare_parameter("follower_mit_kp", [8.0, 8.0, 8.0, 8.0, 6.0, 5.0, 3.0])
        self.declare_parameter("follower_mit_kd", [0.8, 0.8, 0.8, 0.7, 0.5, 0.4, 0.3])
        self.declare_parameter("follower_mit_max_torque_abs", [18.0, 18.0, 18.0, 14.0, 8.0, 8.0, 8.0])
        self.declare_parameter("follower_gravity_torque_scale", [1.0] * 7)
        self.declare_parameter("follower_gravity_debug_enable", False)
        self.declare_parameter("follower_gravity_debug_interval_sec", 1.0)
        self.declare_parameter("enforce_joint_limits", True)
        self.declare_parameter("unwrap_joint_positions", True)
        self.declare_parameter("max_joint_step", 0.0)
        self.declare_parameter("robot_feedback_timeout_sec", 0.2)
        self.declare_parameter("robot_feedback_restore_ramp_sec", 0.5)
        self.declare_parameter("hold_position_when_feedback_inactive", True)
        self.declare_parameter("leader_shutdown_cleanup_enable", True)
        self.declare_parameter("leader_shutdown_ramp_sec", 0.6)
        self.declare_parameter("leader_shutdown_ramp_steps", 12)
        self.declare_parameter("leader_shutdown_final_mode", "zero_force")
        self.declare_parameter("leader_shutdown_disable_on_failure", True)

        self.declare_parameter("enable_gripper", True)
        self.declare_parameter("enable_gripper_haptic", False)
        self.declare_parameter("gripper_startup_recovery_enable", True)
        self.declare_parameter("gripper_mode_override", "width")
        self.declare_parameter("gripper_range", 0.1)
        self.declare_parameter("gripper_reverse", True)
        self.declare_parameter("gripper_val_multiple", 1.0)
        self.declare_parameter("gripper_follow_force_default", 3.0)
        self.declare_parameter("gripper_max_step", 0.0)
        self.declare_parameter("gripper_cmd_limit_enable", True)
        self.declare_parameter("gripper_cmd_min_raw", 0)
        self.declare_parameter("gripper_cmd_max_raw", 100000)
        self.declare_parameter("gripper_cmd_deadband_raw", 200)
        self.declare_parameter("gripper_filter_enable", True)
        self.declare_parameter("gripper_filter_alpha_position", 0.7)
        self.declare_parameter("gripper_filter_alpha_effort", 0.1)
        self.declare_parameter("gripper_haptic_effort_sign", 0.0)
        self.declare_parameter("gripper_haptic_effort_bias", 0.0)
        self.declare_parameter("gripper_haptic_effort_alpha", 0.2)
        self.declare_parameter("gripper_haptic_release_when_opening", True)
        self.declare_parameter("gripper_haptic_opening_direction_eps", 0.0001)
        self.declare_parameter("gripper_haptic_force_max_n", 0.8)
        self.declare_parameter("gripper_haptic_constant_force_n", 0.0)
        self.declare_parameter("gripper_haptic_constant_directional", False)
        self.declare_parameter("gripper_haptic_spring_align_enable", False)
        self.declare_parameter("gripper_haptic_spring_hold_sec", 0.5)
        self.declare_parameter("gripper_haptic_spring_k_n_per_m", 80.0)
        self.declare_parameter("gripper_haptic_spring_min_force_n", 0.4)
        self.declare_parameter("gripper_haptic_mismatch_pulse_enable", False)
        self.declare_parameter("gripper_haptic_mismatch_threshold_m", 0.006)
        self.declare_parameter("gripper_haptic_mismatch_reset_threshold_m", 0.003)
        self.declare_parameter("gripper_haptic_mismatch_pulse_sec", 0.25)
        self.declare_parameter("gripper_haptic_mismatch_force_threshold_n", 0.0)
        self.declare_parameter("gripper_haptic_mismatch_force_confirm_sec", 0.0)
        self.declare_parameter("gripper_haptic_mismatch_force_sign", 0.0)
        self.declare_parameter("gripper_haptic_mismatch_closing_only", False)
        self.declare_parameter("gripper_haptic_mismatch_retrigger_sec", 0.0)
        self.declare_parameter("gripper_haptic_effort_deadband", 0.15)
        self.declare_parameter("gripper_haptic_effort_max", 0.6)
        self.declare_parameter("gripper_haptic_cmd_max", 500)
        self.declare_parameter("gripper_haptic_cmd_enable_threshold", 80)
        self.declare_parameter("gripper_haptic_opening_relief", 0.1)
        self.declare_parameter("gripper_haptic_opening_sign", 0.0)
        self.declare_parameter("gripper_haptic_force_ramp_up_n_per_sec", 4.0)
        self.declare_parameter("gripper_haptic_force_ramp_down_n_per_sec", 20.0)
        self.declare_parameter("gripper_v2_geom_scale", 1.0)
        self.declare_parameter("gripper_v2_geom_offset", 0.0)
        self.declare_parameter("gripper_v2_velocity_eps", 0.002)
        self.declare_parameter("gripper_v2_confirm_time_sec", 0.02)
        self.declare_parameter("gripper_v2_close_error_on", 0.003)
        self.declare_parameter("gripper_v2_close_error_off", 0.0015)
        self.declare_parameter("gripper_v2_open_error_on", 0.0035)
        self.declare_parameter("gripper_v2_open_error_off", 0.00175)
        self.declare_parameter("gripper_v2_k_close", 400.0)
        self.declare_parameter("gripper_v2_d_close", 4.0)
        self.declare_parameter("gripper_v2_fmax_close", 2.0)
        self.declare_parameter("gripper_v2_k_open", 220.0)
        self.declare_parameter("gripper_v2_d_open", 2.0)
        self.declare_parameter("gripper_v2_fmax_open", 1.2)
        self.declare_parameter("gripper_v2_force_low_close", 0.3)
        self.declare_parameter("gripper_v2_force_high_close", 1.2)
        self.declare_parameter("gripper_v2_force_low_open", 0.2)
        self.declare_parameter("gripper_v2_force_high_open", 0.8)
        self.declare_parameter("gripper_v2_lambda_close", 0.5)
        self.declare_parameter("gripper_v2_lambda_open", 0.2)
        self.declare_parameter("gripper_command_deadband", 0.0)
        self.declare_parameter("leader_gripper_min_trusted_angle_position", 1.0)
        self.declare_parameter("leader_gripper_stale_hold_angle_position", 99.0)
        self.declare_parameter("gripper_recover_seed_master_open", True)
        self.declare_parameter("runtime_reseed_enabled", False)
        self.declare_parameter("gripper_recover_seed_angle_deg", 71.2)
        self.declare_parameter("gripper_recover_seed_width", 0.1)
        self.declare_parameter("gripper_recover_seed_force", 1.0)
        self.declare_parameter("gripper_recover_seed_min_width", 0.09)
        self.declare_parameter("leader_gripper_lock_force", 3.0)

    def _load_parameters(self):
        self.role = str(self.get_parameter("role").value).strip().lower()
        if self.role not in ("leader", "follower"):
            raise ValueError("role must be one of: leader, follower")

        self.can_port = str(self.get_parameter("can_port").value)
        self.teleop_ns = str(self.get_parameter("teleop_ns").value)
        self.robot_ns = str(self.get_parameter("robot_ns").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.firmware_profile = str(self.get_parameter("firmware_profile").value)
        self.speed_percent = int(self.get_parameter("speed_percent").value)
        self.enable_timeout = float(self.get_parameter("enable_timeout").value)
        self.publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.control_rate = max(1.0, float(self.get_parameter("control_rate").value))
        self.filter_enable = bool(self.get_parameter("filter_enable").value)
        self.filter_alpha_pos = _clamp(
            float(self.get_parameter("filter_alpha_pos").value), 0.0, 1.0
        )
        self.filter_alpha_vel = _clamp(
            float(self.get_parameter("filter_alpha_vel").value), 0.0, 1.0
        )
        self.filter_alpha_effort = _clamp(
            float(self.get_parameter("filter_alpha_effort").value), 0.0, 1.0
        )

        self.master_slave_enable = bool(self.get_parameter("master_slave_enable").value)
        self.mit_enable_pos = bool(self.get_parameter("mit_enable_pos").value)
        self.mit_enable_tor = bool(self.get_parameter("mit_enable_tor").value)
        self.mit_enable_gravity = bool(self.get_parameter("mit_enable_gravity").value)
        self.gravity_mix_mode = str(self.get_parameter("gravity_mix_mode").value).strip().lower()
        if self.gravity_mix_mode not in ("additive", "legacy_diff", "gravity_only"):
            raise ValueError("gravity_mix_mode must be additive, legacy_diff or gravity_only")
        self.mit_kp = _normalize_vector(self.get_parameter("mit_kp").value, 7, 0.1)
        self.mit_kd = _normalize_vector(self.get_parameter("mit_kd").value, 7, 0.02)
        self.mit_torque_scale = _normalize_vector(
            self.get_parameter("mit_torque_scale").value, 7, 1.0
        )
        self.mit_torque_feedback_sign = _normalize_vector(
            self.get_parameter("mit_torque_feedback_sign").value, 7, -1.0
        )
        self.mit_max_torque_abs = _normalize_vector(
            self.get_parameter("mit_max_torque_abs").value, 7, 18.0
        )
        self.follower_control_mode = str(
            self.get_parameter("follower_control_mode").value
        ).strip().lower()
        if self.follower_control_mode not in ("js", "mit_gravity"):
            raise ValueError("follower_control_mode must be js or mit_gravity")
        self.follower_mit_enable_gravity = bool(
            self.get_parameter("follower_mit_enable_gravity").value
        )
        self.follower_mit_kp = _normalize_vector(
            self.get_parameter("follower_mit_kp").value, 7, 8.0
        )
        self.follower_mit_kd = _normalize_vector(
            self.get_parameter("follower_mit_kd").value, 7, 0.8
        )
        self.follower_mit_max_torque_abs = _normalize_vector(
            self.get_parameter("follower_mit_max_torque_abs").value, 7, 8.0
        )
        self.follower_gravity_torque_scale = _normalize_vector(
            self.get_parameter("follower_gravity_torque_scale").value, 7, 1.0
        )
        self.follower_gravity_debug_enable = bool(
            self.get_parameter("follower_gravity_debug_enable").value
        )
        self.follower_gravity_debug_interval_sec = max(
            0.1, float(self.get_parameter("follower_gravity_debug_interval_sec").value)
        )
        self.enforce_joint_limits = bool(self.get_parameter("enforce_joint_limits").value)
        self.unwrap_joint_positions = bool(self.get_parameter("unwrap_joint_positions").value)
        self.max_joint_step = max(0.0, float(self.get_parameter("max_joint_step").value))
        self.robot_feedback_timeout_sec = max(
            0.0, float(self.get_parameter("robot_feedback_timeout_sec").value)
        )
        self.robot_feedback_restore_ramp_sec = max(
            0.0, float(self.get_parameter("robot_feedback_restore_ramp_sec").value)
        )
        self.hold_position_when_feedback_inactive = bool(
            self.get_parameter("hold_position_when_feedback_inactive").value
        )
        self.leader_shutdown_cleanup_enable = bool(
            self.get_parameter("leader_shutdown_cleanup_enable").value
        )
        self.leader_shutdown_ramp_sec = max(
            0.0, float(self.get_parameter("leader_shutdown_ramp_sec").value)
        )
        self.leader_shutdown_ramp_steps = max(
            1, int(self.get_parameter("leader_shutdown_ramp_steps").value)
        )
        self.leader_shutdown_final_mode = str(
            self.get_parameter("leader_shutdown_final_mode").value
        ).strip().lower()
        if self.leader_shutdown_final_mode not in (
            "zero_force",
            "leader",
            "normal",
            "disable",
            "none",
        ):
            raise ValueError(
                "leader_shutdown_final_mode must be zero_force, leader, normal, "
                "disable or none"
            )
        self.leader_shutdown_disable_on_failure = bool(
            self.get_parameter("leader_shutdown_disable_on_failure").value
        )

        self.enable_gripper = bool(self.get_parameter("enable_gripper").value)
        self.enable_gripper_haptic = bool(self.get_parameter("enable_gripper_haptic").value)
        self.gripper_startup_recovery_enable = bool(
            self.get_parameter("gripper_startup_recovery_enable").value
        )
        self.gripper_mode_override = str(
            self.get_parameter("gripper_mode_override").value
        ).strip().lower()
        if self.gripper_mode_override != "width":
            raise ValueError(
                "gripper_mode_override must be width. "
                "auto/angle are disabled for teleop because they mix gripper units."
            )
        if self.role != "leader" and self.enable_gripper_haptic:
            self.get_logger().warn(
                "Ignoring enable_gripper_haptic on follower; follower always runs pure "
                "gripper position-follow mode."
            )
            self.enable_gripper_haptic = False
        self.gripper_range = max(1e-6, float(self.get_parameter("gripper_range").value))
        self.gripper_reverse = bool(self.get_parameter("gripper_reverse").value)
        self.gripper_val_multiple = max(
            1e-6, float(self.get_parameter("gripper_val_multiple").value)
        )
        self.gripper_follow_force_default = _clamp(
            float(self.get_parameter("gripper_follow_force_default").value),
            0.5,
            3.0,
        )
        self.gripper_max_step = max(
            0.0, float(self.get_parameter("gripper_max_step").value)
        )
        self.gripper_cmd_limit_enable = bool(
            self.get_parameter("gripper_cmd_limit_enable").value
        )
        self.gripper_cmd_min_raw = max(
            0, int(self.get_parameter("gripper_cmd_min_raw").value)
        )
        self.gripper_cmd_max_raw = max(
            self.gripper_cmd_min_raw,
            int(self.get_parameter("gripper_cmd_max_raw").value),
        )
        self.gripper_cmd_deadband_raw = max(
            0, int(self.get_parameter("gripper_cmd_deadband_raw").value)
        )
        self.gripper_filter_enable = bool(
            self.get_parameter("gripper_filter_enable").value
        )
        self.gripper_filter_alpha_position = _clamp(
            float(self.get_parameter("gripper_filter_alpha_position").value), 0.0, 1.0
        )
        self.gripper_filter_alpha_effort = _clamp(
            float(self.get_parameter("gripper_filter_alpha_effort").value), 0.0, 1.0
        )
        self._leader_gripper_adapter = MasterGripperAdapter(
            gripper_range=self.gripper_range,
        )
        gripper_haptic_effort_sign = float(
            self.get_parameter("gripper_haptic_effort_sign").value
        )
        if abs(gripper_haptic_effort_sign) < 1e-9:
            self.gripper_haptic_effort_sign = -1.0 if self.gripper_reverse else 1.0
        else:
            self.gripper_haptic_effort_sign = 1.0 if gripper_haptic_effort_sign >= 0.0 else -1.0
        self.gripper_haptic_effort_bias = float(
            self.get_parameter("gripper_haptic_effort_bias").value
        )
        self.gripper_haptic_effort_alpha = _clamp(
            float(self.get_parameter("gripper_haptic_effort_alpha").value), 0.0, 1.0
        )
        self.gripper_haptic_release_when_opening = bool(
            self.get_parameter("gripper_haptic_release_when_opening").value
        )
        self.gripper_haptic_force_max_n = _clamp(
            float(self.get_parameter("gripper_haptic_force_max_n").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_haptic_constant_force_n = _clamp(
            float(self.get_parameter("gripper_haptic_constant_force_n").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_haptic_constant_directional = bool(
            self.get_parameter("gripper_haptic_constant_directional").value
        )
        self.gripper_haptic_spring_align_enable = bool(
            self.get_parameter("gripper_haptic_spring_align_enable").value
        )
        self.gripper_haptic_spring_hold_sec = max(
            0.0, float(self.get_parameter("gripper_haptic_spring_hold_sec").value)
        )
        self.gripper_haptic_spring_k_n_per_m = max(
            0.0, float(self.get_parameter("gripper_haptic_spring_k_n_per_m").value)
        )
        self.gripper_haptic_spring_min_force_n = _clamp(
            float(self.get_parameter("gripper_haptic_spring_min_force_n").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_haptic_mismatch_pulse_enable = bool(
            self.get_parameter("gripper_haptic_mismatch_pulse_enable").value
        )
        self.gripper_haptic_mismatch_threshold_m = max(
            0.0, float(self.get_parameter("gripper_haptic_mismatch_threshold_m").value)
        )
        self.gripper_haptic_mismatch_reset_threshold_m = max(
            0.0,
            float(self.get_parameter("gripper_haptic_mismatch_reset_threshold_m").value),
        )
        if (
            self.gripper_haptic_mismatch_reset_threshold_m
            > self.gripper_haptic_mismatch_threshold_m
        ):
            self.gripper_haptic_mismatch_reset_threshold_m = (
                0.5 * self.gripper_haptic_mismatch_threshold_m
            )
        self.gripper_haptic_mismatch_pulse_sec = max(
            0.0, float(self.get_parameter("gripper_haptic_mismatch_pulse_sec").value)
        )
        self.gripper_haptic_mismatch_force_threshold_n = max(
            0.0,
            float(
                self.get_parameter("gripper_haptic_mismatch_force_threshold_n").value
            ),
        )
        self.gripper_haptic_mismatch_force_confirm_sec = max(
            0.0,
            float(
                self.get_parameter("gripper_haptic_mismatch_force_confirm_sec").value
            ),
        )
        gripper_haptic_mismatch_force_sign = float(
            self.get_parameter("gripper_haptic_mismatch_force_sign").value
        )
        if abs(gripper_haptic_mismatch_force_sign) < 1e-9:
            self.gripper_haptic_mismatch_force_sign = 0.0
        else:
            self.gripper_haptic_mismatch_force_sign = (
                1.0 if gripper_haptic_mismatch_force_sign > 0.0 else -1.0
            )
        self.gripper_haptic_mismatch_closing_only = bool(
            self.get_parameter("gripper_haptic_mismatch_closing_only").value
        )
        self.gripper_haptic_mismatch_retrigger_sec = max(
            0.0,
            float(self.get_parameter("gripper_haptic_mismatch_retrigger_sec").value),
        )
        self.gripper_haptic_opening_direction_eps = max(
            0.0, float(self.get_parameter("gripper_haptic_opening_direction_eps").value)
        )
        self.gripper_haptic_effort_deadband = max(
            0.0, float(self.get_parameter("gripper_haptic_effort_deadband").value)
        )
        self.gripper_haptic_effort_max = max(
            1e-6, float(self.get_parameter("gripper_haptic_effort_max").value)
        )
        self.gripper_haptic_cmd_max = max(
            0, int(self.get_parameter("gripper_haptic_cmd_max").value)
        )
        self.gripper_haptic_cmd_enable_threshold = max(
            0, int(self.get_parameter("gripper_haptic_cmd_enable_threshold").value)
        )
        self.gripper_haptic_opening_relief = _clamp(
            float(self.get_parameter("gripper_haptic_opening_relief").value), 0.0, 1.0
        )
        self.gripper_haptic_force_ramp_up_n_per_sec = max(
            0.0,
            float(self.get_parameter("gripper_haptic_force_ramp_up_n_per_sec").value),
        )
        self.gripper_haptic_force_ramp_down_n_per_sec = max(
            0.0,
            float(self.get_parameter("gripper_haptic_force_ramp_down_n_per_sec").value),
        )
        gripper_haptic_opening_sign = float(
            self.get_parameter("gripper_haptic_opening_sign").value
        )
        if abs(gripper_haptic_opening_sign) < 1e-9:
            self.gripper_haptic_opening_sign = 1.0
        else:
            self.gripper_haptic_opening_sign = (
                1.0 if gripper_haptic_opening_sign >= 0.0 else -1.0
            )
        self.gripper_v2_geom_scale = float(self.get_parameter("gripper_v2_geom_scale").value)
        self.gripper_v2_geom_offset = float(
            self.get_parameter("gripper_v2_geom_offset").value
        )
        self.gripper_v2_velocity_eps = max(
            0.0, float(self.get_parameter("gripper_v2_velocity_eps").value)
        )
        self.gripper_v2_confirm_time_sec = max(
            0.0, float(self.get_parameter("gripper_v2_confirm_time_sec").value)
        )
        self.gripper_v2_close_error_on = max(
            0.0, float(self.get_parameter("gripper_v2_close_error_on").value)
        )
        self.gripper_v2_close_error_off = max(
            0.0, float(self.get_parameter("gripper_v2_close_error_off").value)
        )
        self.gripper_v2_open_error_on = max(
            0.0, float(self.get_parameter("gripper_v2_open_error_on").value)
        )
        self.gripper_v2_open_error_off = max(
            0.0, float(self.get_parameter("gripper_v2_open_error_off").value)
        )
        self.gripper_v2_k_close = max(
            0.0, float(self.get_parameter("gripper_v2_k_close").value)
        )
        self.gripper_v2_d_close = max(
            0.0, float(self.get_parameter("gripper_v2_d_close").value)
        )
        self.gripper_v2_fmax_close = _clamp(
            float(self.get_parameter("gripper_v2_fmax_close").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_v2_k_open = max(
            0.0, float(self.get_parameter("gripper_v2_k_open").value)
        )
        self.gripper_v2_d_open = max(
            0.0, float(self.get_parameter("gripper_v2_d_open").value)
        )
        self.gripper_v2_fmax_open = _clamp(
            float(self.get_parameter("gripper_v2_fmax_open").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_v2_force_low_close = float(
            self.get_parameter("gripper_v2_force_low_close").value
        )
        self.gripper_v2_force_high_close = float(
            self.get_parameter("gripper_v2_force_high_close").value
        )
        self.gripper_v2_force_low_open = float(
            self.get_parameter("gripper_v2_force_low_open").value
        )
        self.gripper_v2_force_high_open = float(
            self.get_parameter("gripper_v2_force_high_open").value
        )
        self.gripper_v2_lambda_close = max(
            0.0, float(self.get_parameter("gripper_v2_lambda_close").value)
        )
        self.gripper_v2_lambda_open = max(
            0.0, float(self.get_parameter("gripper_v2_lambda_open").value)
        )
        self.gripper_command_deadband = max(
            0.0, float(self.get_parameter("gripper_command_deadband").value)
        )
        self.leader_gripper_min_trusted_angle_position = max(
            0.0,
            float(
                self.get_parameter("leader_gripper_min_trusted_angle_position").value
            ),
        )
        self.leader_gripper_stale_hold_angle_position = max(
            0.0,
            float(
                self.get_parameter("leader_gripper_stale_hold_angle_position").value
            ),
        )
        self.gripper_recover_seed_master_open = bool(
            self.get_parameter("gripper_recover_seed_master_open").value
        )
        self.runtime_reseed_enabled = bool(
            self.get_parameter("runtime_reseed_enabled").value
        )
        self.gripper_recover_seed_angle_deg = float(
            self.get_parameter("gripper_recover_seed_angle_deg").value
        )
        self.gripper_recover_seed_width = _clamp(
            float(self.get_parameter("gripper_recover_seed_width").value),
            0.0,
            self.gripper_range,
        )
        self.gripper_recover_seed_force = _clamp(
            float(self.get_parameter("gripper_recover_seed_force").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self.gripper_recover_seed_min_width = _clamp(
            float(self.get_parameter("gripper_recover_seed_min_width").value),
            0.0,
            self.gripper_range,
        )
        self.leader_gripper_lock_force = _clamp(
            float(self.get_parameter("leader_gripper_lock_force").value),
            0.0,
            AgxGripperWrapper.FORCE_MAX,
        )
        self._gripper_haptic_config = CueHapticConfig(
            enabled=self.enable_gripper_haptic,
            effort_sign=self.gripper_haptic_effort_sign,
            effort_bias=self.gripper_haptic_effort_bias,
            effort_alpha=self.gripper_haptic_effort_alpha,
            effort_deadband=self.gripper_haptic_effort_deadband,
            effort_max=self.gripper_haptic_effort_max,
            cmd_max=self.gripper_haptic_cmd_max,
            cmd_enable_threshold=self.gripper_haptic_cmd_enable_threshold,
            release_when_opening=self.gripper_haptic_release_when_opening,
            opening_relief=self.gripper_haptic_opening_relief,
            opening_sign=self.gripper_haptic_opening_sign,
            opening_direction_eps=self.gripper_haptic_opening_direction_eps,
            force_max_n=self.gripper_haptic_force_max_n,
            force_ramp_up_n_per_sec=self.gripper_haptic_force_ramp_up_n_per_sec,
            force_ramp_down_n_per_sec=self.gripper_haptic_force_ramp_down_n_per_sec,
        )
        if self.enable_gripper_haptic:
            if self.gripper_haptic_spring_align_enable:
                self.get_logger().warn(
                    "enable_gripper_haptic=true with spring-align mode: leader "
                    "stays passive until abs(follower-leader opening) exceeds "
                    f"{self.gripper_haptic_mismatch_threshold_m:.4f}m and abs(follower "
                    "gripper force) exceeds "
                    f"{self.gripper_haptic_mismatch_force_threshold_n:.3f}N for "
                    f"{self.gripper_haptic_mismatch_force_confirm_sec:.3f}s; then "
                    "leader first holds the trigger-time opening target for "
                    f"{self.gripper_haptic_spring_hold_sec:.3f}s at max force, then "
                    "leader gripper target follows follower opening until alignment "
                    f"recovers below {self.gripper_haptic_mismatch_reset_threshold_m:.4f}m."
                )
            elif self.gripper_haptic_constant_force_n > 0.0:
                self.get_logger().warn(
                    "enable_gripper_haptic=true with constant leader gripper "
                    f"resistance={self.gripper_haptic_constant_force_n:.3f}N. "
                    "Leader keeps an aligned self-loop opening target and only "
                    "changes the command force field. "
                    f"directional={self.gripper_haptic_constant_directional}."
                )
            elif self.gripper_haptic_mismatch_pulse_enable:
                self.get_logger().warn(
                    "enable_gripper_haptic=true with mismatch pulse mode: leader "
                    "remains pure/passive except when follower/master gripper opening "
                    f"error exceeds {self.gripper_haptic_mismatch_threshold_m:.4f}m; "
                    "follower gripper force exceeds "
                    f"{self.gripper_haptic_mismatch_force_threshold_n:.3f}N for "
                    f"{self.gripper_haptic_mismatch_force_confirm_sec:.3f}s; "
                    f"pulse={self.gripper_haptic_mismatch_pulse_sec:.3f}s, "
                    f"force={self.gripper_haptic_force_max_n:.3f}N, "
                    f"closing_only={self.gripper_haptic_mismatch_closing_only}, "
                    f"force_sign={self.gripper_haptic_mismatch_force_sign:+.0f}, "
                    f"retrigger={self.gripper_haptic_mismatch_retrigger_sec:.3f}s."
                )
            else:
                self.get_logger().warn(
                    "enable_gripper_haptic=true now activates cue-only haptic mode: "
                    "leader keeps a self-loop opening target while follower force only "
                    "modulates local leader force/enable."
                )

    def _init_runtime_state(self):
        self.arm_joint_names = [f"joint{i}" for i in range(1, 8)]
        self.arm_joint_count = len(self.arm_joint_names)
        self.arm_joint_lower_limits = [-3.14] * self.arm_joint_count
        self.arm_joint_upper_limits = [3.14] * self.arm_joint_count
        self.enable_flag = False
        self.control_ready = False
        self._control_ready_logged = False

        self._state_lock = threading.Lock()
        self._gripper_teaching_param_lock = threading.Lock()
        self._gripper_teaching_friction: Optional[int] = None
        self._gripper_teaching_range_per: Optional[int] = None
        self._gripper_teaching_max_range: Optional[float] = None
        self._current_joint_positions = [0.0] * self.arm_joint_count
        self._current_joint_velocities = [0.0] * self.arm_joint_count
        self._current_joint_efforts = [0.0] * self.arm_joint_count
        self._current_gripper_opening = 0.0
        self._current_gripper_force = 0.0
        self._current_gripper_status: Optional[DriverGripperStatus] = None
        self._filtered_joint_positions = [0.0] * self.arm_joint_count
        self._filtered_joint_velocities = [0.0] * self.arm_joint_count
        self._filtered_joint_efforts = [0.0] * self.arm_joint_count
        self._filtered_gripper_opening = 0.0
        self._filtered_gripper_force = 0.0
        self._gripper_width_valid = False
        self._filter_initialized = False
        self._latest_joint_stamp = 0.0

        self._command_lock = threading.Lock()
        self._target_joint_positions = [0.0] * self.arm_joint_count
        self._target_gripper_opening = 0.0
        self._target_gripper_force = self.gripper_follow_force_default
        self._joint_command_received_at = 0.0
        self._gripper_command_received_at = 0.0

        self._follower_feedback_lock = threading.Lock()
        self._follower_positions = [0.0] * self.arm_joint_count
        self._follower_efforts = [0.0] * self.arm_joint_count
        self._follower_gripper_opening = 0.0
        self._follower_gripper_effort = 0.0
        self._follower_gripper_force = 0.0
        self._filtered_follower_gripper_force = 0.0
        self._follower_gripper_force_filter_initialized = False
        self._follower_received_at = 0.0
        self._follower_gripper_force_received_at = 0.0
        self._follower_gravity_efforts = [0.0] * self.arm_joint_count
        self._follower_gravity_received_at = 0.0
        self._follower_feedback_count = 0
        self._follower_feedback_max_gap_sec = 0.0
        self._robot_feedback_active = False
        self._robot_feedback_restore_started_at = time.monotonic()
        self._robot_feedback_stale_started_at = 0.0
        self._robot_feedback_stale_count = 0
        self._robot_feedback_last_stale_age_sec = 0.0
        self._last_follower_target: Optional[List[float]] = None
        self._last_follower_gripper_target = 0.0
        self._last_follower_gripper_cmd_at = 0.0
        self._follower_gripper_bootstrap_until = 0.0
        self._last_leader_gripper_opening = 0.0
        self._leader_gripper_intent_opening = 0.0
        self._leader_gripper_intent_published_at = 0.0
        self._leader_haptic_engaged = False
        self._leader_haptic_force = 0.0
        self._leader_haptic_state = HAPTIC_FREE_FOLLOW
        self._leader_haptic_cmd_raw = 0
        self._leader_haptic_filtered_effort = 0.0
        self._leader_haptic_opening_detected = False
        self._leader_gripper_lock_enabled = False
        self._leader_gripper_lock_opening = 0.0
        self._leader_directional_resistance_active = False
        self._leader_mismatch_pulse_active_until = 0.0
        self._leader_mismatch_pulse_armed = True
        self._leader_mismatch_pulse_error = 0.0
        self._leader_mismatch_force_ready_since = 0.0
        self._leader_spring_align_active = False
        self._leader_spring_hold_until = 0.0
        self._leader_spring_hold_opening = 0.0
        self._gripper_haptic_controller = CueHapticController(
            self._gripper_haptic_config
        )
        self._leader_gripper_drive_enabled = False
        self._release_retry_count = 0
        self._leader_stale_near_zero_count = 0
        self._runtime_reseed_in_progress = False
        self._runtime_reseed_last_attempt = 0.0
        self._follower_gripper_warmup_done = False
        self._leader_shutdown_cleanup_done = False
        self._last_leader_mit_positions: Optional[List[float]] = None
        self._last_leader_mit_kp: Optional[List[float]] = None
        self._last_leader_mit_kd: Optional[List[float]] = None
        self._last_leader_mit_torques: Optional[List[float]] = None
        self._last_follower_gravity_debug_log = 0.0

    def _configure_arm_control_mode(self, role_motion_mode: str):
        self.agx_arm.set_speed_percent(self.speed_percent)
        self._apply_runtime_role_mode(role_motion_mode)
        self.agx_arm.set_motion_mode(role_motion_mode)
        self.agx_arm.set_auto_set_motion_mode_enabled(False)
        self.get_logger().info(
            f"Pinned Nero motion mode to {role_motion_mode} and disabled per-command auto mode switching"
        )

    def _apply_runtime_role_mode(self, role_motion_mode: str):
        if (
            self.firmware_version == NeroFW.V112
            and self.role == "leader"
            and role_motion_mode == "mit"
        ):
            # Enabled leader-side MIT needs normal controlled feedback and
            # torque commands. Nero V112 leader/zero-force mode is useful for
            # passive drag or shutdown release, but it can suppress the normal
            # joint feedback path and make an enabled master feel locked.
            self.agx_arm.set_follower_mode()
            self.get_logger().warn(
                "Using Nero V112 follower-compatible mode for enabled leader MIT control"
            )
            return

        self._apply_dedicated_role_mode()

    def _init_arm(self):
        forced_firmware_version = _normalize_nero_firmware_profile(self.firmware_profile)
        config_kwargs = {"robot": "nero", "comm": "can", "channel": self.can_port}
        if forced_firmware_version is not None:
            config_kwargs["firmeware_version"] = forced_firmware_version

        config = create_agx_arm_config(**config_kwargs)
        self.arm_joint_names = list(config["joint_limits"].keys())
        self.arm_joint_count = len(self.arm_joint_names)
        self.arm_joint_lower_limits = [
            float(config["joint_limits"][name][0]) for name in self.arm_joint_names
        ]
        self.arm_joint_upper_limits = [
            float(config["joint_limits"][name][1]) for name in self.arm_joint_names
        ]

        self.agx_arm = AgxArmFactory.create_arm(config)
        self.agx_arm.connect()

        firmware_version = (
            forced_firmware_version
            if forced_firmware_version is not None
            else NeroFW.DEFAULT
        )
        if forced_firmware_version is None:
            firmware = self._get_firmware_with_retry(poke_enable=False)
            if firmware is None and self.auto_enable:
                firmware = self._get_firmware_with_retry(poke_enable=True)
            if firmware is None:
                self.get_logger().warn(
                    "Failed to get Nero firmware version, continuing with default firmware profile"
                )
            else:
                current_version = firmware["software_version"]
                self.get_logger().info(f"firmware version: {current_version}")
                firmware_version = _select_nero_firmware_profile(current_version)
        else:
            self.get_logger().info(
                f"Using forced Nero firmware profile: {forced_firmware_version}"
            )

        if forced_firmware_version is None and firmware_version != NeroFW.DEFAULT:
            self.agx_arm.disconnect()
            config = create_agx_arm_config(
                robot="nero",
                comm="can",
                channel=self.can_port,
                firmeware_version=firmware_version,
            )
            self.agx_arm = AgxArmFactory.create_arm(config)
            self.agx_arm.connect()
            self.arm_joint_names = list(config["joint_limits"].keys())
            self.arm_joint_lower_limits = [
                float(config["joint_limits"][name][0]) for name in self.arm_joint_names
            ]
            self.arm_joint_upper_limits = [
                float(config["joint_limits"][name][1]) for name in self.arm_joint_names
            ]

        self.firmware_version = firmware_version
        role_motion_mode = (
            "mit"
            if self.role == "leader" or self.follower_control_mode == "mit_gravity"
            else "js"
        )
        if self.auto_enable:
            self._prepare_arm_for_auto_enable()
            if self._enable_arm(True, self.enable_timeout):
                self._configure_arm_control_mode(role_motion_mode)
            elif self.firmware_version == NeroFW.V112:
                self.get_logger().warn(
                    "Initial auto-enable failed; preparing Nero V112 controlled mode and retrying"
                )
                self._prepare_arm_for_auto_enable()
                if self._enable_arm(True, self.enable_timeout):
                    self._configure_arm_control_mode(role_motion_mode)
                    self.get_logger().info(
                        "Auto-enable succeeded after Nero V112 role-mode retry"
                    )
                else:
                    self.get_logger().error("Failed to auto-enable the arm")
            else:
                self.get_logger().error("Failed to auto-enable the arm")
        else:
            self._configure_arm_control_mode(role_motion_mode)

        self.gripper: Optional[AgxGripperWrapper] = None
        if self.enable_gripper:
            self.gripper = AgxGripperWrapper(self.agx_arm)
            if self.gripper.initialize():
                self.get_logger().info("AgxGripper initialized successfully")
                if self.gripper_startup_recovery_enable:
                    self._recover_gripper_state_once("startup")
                else:
                    self.get_logger().info(
                        "Startup gripper recovery disabled; using live gripper feedback."
                    )
                self._log_gripper_diagnostics()
                if self.role == "leader" and not self.enable_gripper_haptic:
                    self.get_logger().info(
                        "Skipping leader gripper mode seed in passive sensor mode; "
                        "leader gripper commands are disabled."
                    )
                elif self.role == "follower":
                    self.get_logger().info(
                        "Skipping follower gripper mode seed in follower-only startup; "
                        "preserving the externally enabled gripper state."
                    )
                else:
                    self._seed_gripper_mode_once()
                if self.role == "follower":
                    self._warmup_follower_gripper_once()
            else:
                self.get_logger().error("Failed to initialize AgxGripper")
                self.gripper = None

    def _apply_dedicated_role_mode(self):
        if self.firmware_version == NeroFW.V112:
            if self.role == "leader":
                self.agx_arm.set_leader_mode()
                self.get_logger().info("Applied Nero V112 leader mode")
            else:
                self.agx_arm.set_follower_mode()
                self.get_logger().info("Applied Nero V112 follower mode")
            return

        self.agx_arm.set_normal_mode()
        self.get_logger().info(f"Applied dedicated Nero normal mode for role={self.role}")

    def _prepare_arm_for_auto_enable(self):
        if self.firmware_version != NeroFW.V112:
            return

        if self.role == "leader":
            # A previous leader shutdown leaves Nero V112 in leader/zero-force mode.
            # Motor enable is verified through joint enable bits, which only become
            # reliable after returning to the controlled follower/J mode first.
            try:
                self.agx_arm.set_follower_mode()
                self.agx_arm.set_motion_mode("j")
                self.get_logger().warn(
                    "Prepared Nero V112 leader for motor enable using temporary follower/J mode"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to prepare Nero V112 leader for enable: {exc}",
                    throttle_duration_sec=1.0,
                )
            return

        self._apply_dedicated_role_mode()

    def _get_firmware_with_retry(self, *, poke_enable: bool = False) -> Optional[dict]:
        """Probe Nero firmware more robustly during cold bring-up.

        Single-shot reads are flaky right after CAN reactivation or when two
        arms are brought up in parallel. Mirror the pyAgxArm demo strategy:
        keep nudging the controller alive and retry for a short window before
        giving up.
        """
        deadline = time.monotonic() + max(float(self.enable_timeout), 5.0)
        last_enable_attempt = 0.0
        while time.monotonic() < deadline and not self._shutdown_event.is_set():
            firmware = self.agx_arm.get_firmware(timeout=0.5, min_interval=0.0)
            if firmware is not None:
                return firmware

            now = time.monotonic()
            if poke_enable and self.auto_enable and (now - last_enable_attempt) >= 0.5:
                try:
                    self.agx_arm.enable()
                except Exception:
                    pass
                last_enable_attempt = now

            time.sleep(0.1)
        return None

    def _setup_ros_interfaces(self):
        self.joint_state_pub = self.create_publisher(JointState, "joint_states_single", HOT_QOS)
        self.arm_status_pub = self.create_publisher(AgxArmStatus, "arm_status", HOT_QOS)
        self.gripper_status_pub = self.create_publisher(GripperStatus, "gripper_status", HOT_QOS)
        self.gripper_intent_pub = self.create_publisher(JointState, GRIPPER_INTENT_TOPIC, HOT_QOS)
        self.gripper_haptic_debug_pub = self.create_publisher(
            JointState, "gripper_haptic_debug", HOT_QOS
        )
        self.gripper_teaching_friction_pub = self.create_publisher(
            Int32, "gripper/teaching_friction", LATCHED_HOT_QOS
        )
        self.create_subscription(
            Empty,
            "gripper/query_teaching_param",
            self._gripper_teaching_param_query_callback,
            HOT_QOS,
        )
        self._publish_cached_gripper_teaching_friction()

        if self.role == "leader":
            self.create_subscription(
                JointState,
                f"/{self.robot_ns}/joint_states_single",
                self._follower_joint_state_callback,
                HOT_QOS,
            )
            self.create_subscription(
                JointState,
                f"/{self.robot_ns}/joint_states_compensated",
                self._follower_gravity_callback,
                HOT_QOS,
            )
            self.create_subscription(
                GripperStatus,
                f"/{self.robot_ns}/gripper_status",
                self._follower_gripper_status_callback,
                HOT_QOS,
            )
            self.create_subscription(
                Bool,
                "gripper_lock",
                self._leader_gripper_lock_callback,
                LATCHED_HOT_QOS,
            )
            self.get_logger().info(f"Leader pose source: /{self.teleop_ns}/joint_states_single")
            self.get_logger().info(
                f"Follower torque source: /{self.robot_ns}/joint_states_single"
            )
            self.get_logger().info(
                f"Follower gripper force source: /{self.robot_ns}/gripper_status"
            )
            self.get_logger().info(
                f"Gravity source: /{self.robot_ns}/joint_states_compensated"
            )
            self.get_logger().info(
                "Dedicated ROS1-style Nero leader control enabled "
                f"(publish_rate={self.publish_rate:.1f}Hz, control_rate={self.control_rate:.1f}Hz)"
            )
            self.get_logger().info(
                f"Leader gripper lock topic: /{self.teleop_ns}/gripper_lock "
                "(Bool, true=lock current opening, false=release)"
            )
            if self.enable_gripper and not self.enable_gripper_haptic:
                self.get_logger().info(
                    "Leader gripper passive sensor mode active; leader gripper commands are disabled"
                )
            elif self.enable_gripper and self.enable_gripper_haptic:
                self.get_logger().info(
                    "Leader cue-only gripper haptic active; leader keeps its own "
                    "opening target and only adds local resistance from follower force."
                )
        else:
            self.create_subscription(
                JointState,
                f"/{self.teleop_ns}/joint_states_single",
                self._leader_joint_state_command_callback,
                HOT_QOS,
            )
            self.create_subscription(
                JointState,
                f"/{self.teleop_ns}/{GRIPPER_INTENT_TOPIC}",
                self._leader_gripper_intent_callback,
                HOT_QOS,
            )
            self.create_subscription(
                JointState,
                "joint_pos_cmd",
                self._joint_command_callback,
                HOT_QOS,
            )
            if self.follower_mit_enable_gravity:
                self.create_subscription(
                    JointState,
                    f"/{self.robot_ns}/joint_states_compensated",
                    self._follower_gravity_callback,
                    HOT_QOS,
                )
            self.get_logger().info(
                "Dedicated ROS1-style Nero follower control enabled "
                f"(publish_rate={self.publish_rate:.1f}Hz, control_rate={self.control_rate:.1f}Hz)"
            )
            self.get_logger().info(
                f"Follower position source: /{self.teleop_ns}/joint_states_single"
            )
            self.get_logger().info(
                f"Follower gripper intent source: /{self.teleop_ns}/{GRIPPER_INTENT_TOPIC}"
            )
            if self.follower_mit_enable_gravity:
                self.get_logger().info(
                    f"Follower gravity source: /{self.robot_ns}/joint_states_compensated"
                )

    def _log_gripper_diagnostics(self):
        if self.gripper is None:
            return
        try:
            status = self.gripper.get_status()
            teaching = self.gripper.get_teaching_param(timeout=0.2, min_interval=0.0)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read gripper diagnostics: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        if status is not None:
            self.get_logger().info(
                "Gripper status at startup: "
                f"mode={status.mode} "
                f"position={float(status.position):.3f} "
                f"force={float(status.force):.3f} "
                f"enable={bool(status.driver_enable_status)}"
            )
        if teaching is not None:
            self._cache_gripper_teaching_param(teaching)
            self.get_logger().info(
                "Gripper teaching param at startup: "
                f"range_per={int(teaching.teaching_range_per)} "
                f"max_range={float(teaching.max_range_config):.3f} "
                f"friction={int(teaching.teaching_friction)}"
            )
        self.get_logger().info(f"Gripper mode override policy: {self.gripper_mode_override}")

    def _cache_gripper_teaching_param(self, teaching) -> None:
        """Keep the last read-only firmware teaching settings for ROS consumers."""
        with self._gripper_teaching_param_lock:
            self._gripper_teaching_friction = int(teaching.teaching_friction)
            self._gripper_teaching_range_per = int(teaching.teaching_range_per)
            self._gripper_teaching_max_range = float(teaching.max_range_config)

    def _publish_cached_gripper_teaching_friction(self) -> bool:
        with self._gripper_teaching_param_lock:
            friction = self._gripper_teaching_friction
        if friction is None or not hasattr(self, "gripper_teaching_friction_pub"):
            return False
        self.gripper_teaching_friction_pub.publish(Int32(data=int(friction)))
        return True

    def _gripper_teaching_param_query_callback(self, _msg: Empty) -> None:
        """Handle an explicitly requested, read-only SDK teaching-parameter query."""
        if self.gripper is None:
            self.get_logger().warn(
                "Ignoring gripper teaching-parameter query: gripper is unavailable",
                throttle_duration_sec=1.0,
            )
            return
        try:
            teaching = self.gripper.get_teaching_param(timeout=0.5, min_interval=0.0)
        except Exception as exc:
            self.get_logger().warn(
                f"Read-only gripper teaching-parameter query failed: {exc}",
                throttle_duration_sec=1.0,
            )
            return
        if teaching is None:
            self.get_logger().warn(
                "Read-only gripper teaching-parameter query returned no response",
                throttle_duration_sec=1.0,
            )
            return

        self._cache_gripper_teaching_param(teaching)
        self._publish_cached_gripper_teaching_friction()
        self.get_logger().info(
            "Gripper teaching parameter refreshed (read-only): "
            f"range_per={int(teaching.teaching_range_per)} "
            f"max_range={float(teaching.max_range_config):.3f} "
            f"friction={int(teaching.teaching_friction)}"
        )

    def _warmup_follower_gripper_once(self):
        if self.role != "follower" or self.gripper is None or self._follower_gripper_warmup_done:
            return
        try:
            status = self.gripper.get_status()
            if status is None:
                return
            opening = self._require_width_opening(status, "local follower gripper")
            if opening is None:
                self.get_logger().warn(
                    "Skipping follower gripper warmup until width feedback is available",
                    throttle_duration_sec=1.0,
                )
                return
            command_position, command_mode = self._gripper_command_from_opening(opening, status)
            if self.gripper.command_position(
                position=command_position,
                force=self.gripper_follow_force_default,
                mode=command_mode,
                enabled=True,
            ):
                self._follower_gripper_warmup_done = True
                self.get_logger().info(
                    "Follower gripper warmup completed with one no-op command"
                )
        except GripperWidthModeError:
            raise
        except Exception as exc:
            self.get_logger().warn(
                f"Follower gripper warmup failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _leader_gripper_status_needs_open_seed(
        self, status: Optional[DriverGripperStatus]
    ) -> bool:
        if (
            self.role != "leader"
            or self.gripper is None
            or not self.enable_gripper
            or not self.gripper_recover_seed_master_open
            or status is None
        ):
            return False
        mode = str(getattr(status, "mode", "")).strip().lower()
        if mode != "angle":
            return False
        raw_position = float(getattr(status, "position", math.nan))
        return (
            math.isfinite(raw_position)
            and raw_position <= self.leader_gripper_min_trusted_angle_position
            and not bool(getattr(status, "driver_enable_status", False))
        )

    def _send_repeated_gripper_position(
        self,
        *,
        position: float,
        force: float,
        mode: str,
        enabled: bool,
        repeats: int = 12,
        interval_sec: float = 0.05,
    ) -> bool:
        if self.gripper is None:
            return False
        ok = False
        for _ in range(max(1, int(repeats))):
            ok = bool(
                self.gripper.command_position(
                    position=position,
                    force=force,
                    mode=mode,
                    enabled=enabled,
                )
            ) or ok
            time.sleep(interval_sec)
        return ok

    def _seed_leader_master_open_if_needed(
        self, reason: str, fatal_on_failure: bool = True
    ) -> bool:
        if self.role != "leader" or self.gripper is None or not self.enable_gripper:
            return False

        try:
            status = self.gripper.get_status()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read leader gripper before open seed during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )
            return False

        if not self._leader_gripper_status_needs_open_seed(status):
            return False

        raw_position = float(getattr(status, "position", math.nan))
        self._leader_gripper_adapter.reset()
        self.get_logger().warn(
            "Leader gripper feedback is stale near zero during "
            f"{reason} (mode={status.mode}, position={raw_position:.6f}); "
            "seeding angle then width-open before publishing gripper intent."
        )
        angle_ok = self._send_repeated_gripper_position(
            position=self.gripper_recover_seed_angle_deg,
            force=self.gripper_recover_seed_force,
            mode="angle",
            enabled=True,
        )
        status_after_angle = self.gripper.get_status()
        if status_after_angle is not None:
            self.get_logger().info(
                "Leader gripper after angle seed: "
                f"mode={status_after_angle.mode} "
                f"position={float(status_after_angle.position):.6f} "
                f"force={float(status_after_angle.force):.3f} "
                f"enable={bool(status_after_angle.driver_enable_status)}"
            )

        width_ok = self._send_repeated_gripper_position(
            position=self.gripper_recover_seed_width,
            force=self.gripper_recover_seed_force,
            mode="width",
            enabled=True,
        )
        status_after_width = self.gripper.get_status()
        seeded_width = None
        if status_after_width is not None:
            seeded_width = self._leader_gripper_adapter.opening_from_status(
                status_after_width
            )
            self.get_logger().info(
                "Leader gripper after width-open seed: "
                f"mode={status_after_width.mode} "
                f"position={float(status_after_width.position):.6f} "
                f"seeded_width={seeded_width:.6f} "
                f"force={float(status_after_width.force):.3f} "
                f"enable={bool(status_after_width.driver_enable_status)}"
            )

        if seeded_width is None or seeded_width < self.gripper_recover_seed_min_width:
            if fatal_on_failure:
                self._fatal_gripper_width_error(
                    "Leader gripper open seed failed to recover stroke-like feedback: "
                    f"angle_ok={angle_ok}, width_ok={width_ok}, seeded_width={seeded_width}. "
                    "Refusing to publish stale near-zero gripper intent."
                )
            self.get_logger().warn(
                "Runtime leader gripper re-seed failed to recover stroke-like feedback: "
                f"angle_ok={angle_ok}, width_ok={width_ok}, seeded_width={seeded_width}. "
                "Stale-hold guard will remain active until next successful reading.",
                throttle_duration_sec=2.0,
            )
            return False

        command_position, command_mode = self._gripper_command_from_opening(
            seeded_width,
            status_after_width,
        )
        self.gripper.command_position(
            position=command_position,
            force=0.0,
            mode=command_mode,
            enabled=False,
            clear_error=True,
        )
        time.sleep(0.03)
        try:
            self.gripper.disable()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to disable leader gripper after open seed: {exc}",
                throttle_duration_sec=1.0,
            )
        self._current_gripper_opening = float(seeded_width)
        return True

    def _maybe_runtime_reseed_leader(self):
        """If leader feedback has been stale-near-zero for sustained time
        while haptic is enabled AND runtime_reseed_enabled is opted in,
        spawn a background re-seed to recover firmware feedback without
        needing a node restart or power-cycle.

        Disabled by default because the seed physically moves the master
        and can disrupt smooth teleop; the existing stale-hold guard in
        _local_gripper_opening already masks the common firmware quirk
        at max-open. Opt in via runtime_reseed_enabled:=true if you need
        automatic recovery from deep firmware-stuck states.
        """
        if not self.runtime_reseed_enabled:
            return
        if self.role != "leader" or self.gripper is None:
            return
        if not self.enable_gripper_haptic:
            return
        if self._runtime_reseed_in_progress:
            return
        if self._leader_stale_near_zero_count < 400:
            return
        now = time.monotonic()
        if now - self._runtime_reseed_last_attempt < 2.0:
            return
        self._runtime_reseed_last_attempt = now
        self._runtime_reseed_in_progress = True
        self.get_logger().warn(
            "Leader gripper feedback stuck at stale near-zero for >2s; "
            "spawning runtime re-seed to recover firmware feedback.",
            throttle_duration_sec=2.0,
        )
        thread = threading.Thread(
            target=self._runtime_reseed_worker,
            name="leader-gripper-reseed",
            daemon=True,
        )
        thread.start()

    def _runtime_reseed_worker(self):
        try:
            self._seed_leader_master_open_if_needed(
                "runtime stale-feedback recovery",
                fatal_on_failure=False,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Runtime leader gripper re-seed worker failed: {exc}",
                throttle_duration_sec=2.0,
            )
        finally:
            self._leader_stale_near_zero_count = 0
            self._runtime_reseed_in_progress = False

    def _seed_gripper_mode_once(self):
        if self.gripper is None or not self.enable_gripper:
            return
        try:
            status = self.gripper.get_status()
            opening = self._local_gripper_opening(status, "local gripper")
            if opening is None:
                if self._seed_leader_master_open_if_needed("mode seed"):
                    status = self.gripper.get_status()
                    opening = self._local_gripper_opening(status, "local gripper")
                if opening is None:
                    self.get_logger().warn(
                        "Skipping gripper mode seed because feedback is not trusted; "
                        "not sending fallback zero-width command.",
                        throttle_duration_sec=1.0,
                    )
                    return
            command_position, command_mode = self._gripper_command_from_opening(opening, status)
            seed_force = _clamp(self.gripper_follow_force_default, 0.5, 1.0)
            self.gripper.command_position(
                position=command_position,
                force=seed_force,
                mode=command_mode,
                enabled=True,
            )
            time.sleep(0.05)
            self.gripper.command_position(
                position=command_position,
                force=0.0,
                mode=command_mode,
                enabled=False,
            )
            self.get_logger().info(
                "Seeded gripper mode with enabled->disabled no-op command: "
                f"mode={command_mode} force={seed_force:.3f}"
            )
        except GripperWidthModeError:
            raise
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to seed gripper mode override: {exc}",
                throttle_duration_sec=1.0,
            )

    def _recover_gripper_state_once(self, reason: str):
        if self.gripper is None or not self.enable_gripper:
            return

        status: Optional[DriverGripperStatus] = None
        try:
            status = self.gripper.get_status()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read gripper status during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )

        try:
            if status is not None:
                current_opening = self._local_gripper_opening(status, "local gripper")
                if current_opening is None:
                    current_opening = float(self._current_gripper_opening)
            else:
                _, _, current_opening, _, _, _, _ = self._snapshot_local_state()
            command_position, command_mode = self._gripper_command_from_opening(
                current_opening,
                status,
            )

            self.gripper.command_position(
                position=command_position,
                force=0.0,
                mode=command_mode,
                enabled=False,
                clear_error=True,
            )
            time.sleep(0.03)
        except GripperWidthModeError:
            raise
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to send gripper clear-error release during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )

        try:
            self.gripper.disable()
            time.sleep(0.03)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to disable gripper during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )

        try:
            self.gripper.reset()
            time.sleep(0.03)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to reset gripper during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )

        try:
            self.gripper.disable()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed final gripper disable during {reason}: {exc}",
                throttle_duration_sec=1.0,
            )

        if reason == "startup":
            try:
                ok = self.gripper.set_teaching_param(
                    teaching_range_per=100,
                    max_range_config=self.gripper_range,
                    teaching_friction=1,
                    timeout=1.0,
                )
                self.get_logger().info(
                    "Startup gripper teaching restore: "
                    f"ok={ok} range={self.gripper_range:.3f}"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to restore gripper teaching param during {reason}: {exc}",
                    throttle_duration_sec=1.0,
                )
            self._seed_leader_master_open_if_needed("startup post-teaching")

        if self.role == "follower":
            time.sleep(0.5)
            for attempt in range(1, 6):
                try:
                    status = self.gripper.get_status()
                    opening = self._local_gripper_opening(
                        status, "local follower gripper startup enable")
                    if opening is None:
                        opening = 0.0
                    command_position, command_mode = self._gripper_command_from_opening(
                        opening, status)
                    ok = self.gripper.command_position(
                        position=command_position,
                        force=self.gripper_follow_force_default,
                        mode=command_mode,
                        enabled=True,
                    )
                    time.sleep(0.5)
                    status = self.gripper.get_status()
                    enabled = bool(
                        status is not None and status.driver_enable_status)
                    self.get_logger().info(
                        f"Follower-only gripper startup enable attempt={attempt} "
                        f"ok={ok} width={float(opening):.6f} enabled={enabled}"
                    )
                    if enabled:
                        self._follower_gripper_bootstrap_until = time.monotonic() + 10.0
                        break
                except Exception as exc:
                    self.get_logger().warn(
                        f"Failed follower-only gripper startup enable: {exc}",
                        throttle_duration_sec=1.0,
                    )
                    time.sleep(0.5)

    def _float_to_ros_time(self, timestamp: float) -> Time:
        ros_time = Time()
        ros_time.sec = int(timestamp)
        ros_time.nanosec = int((timestamp - ros_time.sec) * 1e9)
        return ros_time

    def _enable_arm(self, enable: bool = True, timeout: float = 5.0) -> bool:
        start_time = time.time()
        action_name = "enable" if enable else "disable"
        while not (self.agx_arm.enable() if enable else self.agx_arm.disable()):
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Timeout waiting for arm to {action_name}")
                self._log_enable_debug(action_name)
                return False
            time.sleep(0.01)

        joints_status = self.agx_arm.get_joint_enable_status(255)
        all_joints_in_target_status = joints_status if enable else not joints_status
        if all_joints_in_target_status:
            self.enable_flag = enable
            self.get_logger().info(f"All joints {action_name}d successfully")
        else:
            self.get_logger().warn(f"Not all joints are {action_name}d")
        return True

    def _log_enable_debug(self, action_name: str):
        try:
            arm_status = self.agx_arm.get_arm_status()
            status_msg = getattr(arm_status, "msg", None)
            ctrl_mode = getattr(status_msg, "ctrl_mode", None)
            mode_feedback = getattr(status_msg, "mode_feedback", None)
            arm_status_code = getattr(status_msg, "arm_status", None)
        except Exception as exc:
            ctrl_mode = mode_feedback = arm_status_code = f"read_error:{exc}"

        try:
            joint_enable = list(self.agx_arm.get_joints_enable_status_list())
        except Exception as exc:
            joint_enable = [f"read_error:{exc}"]

        self.get_logger().error(
            f"Arm {action_name} debug: role={self.role} firmware={self.firmware_version} "
            f"ctrl_mode={ctrl_mode} mode_feedback={mode_feedback} "
            f"arm_status={arm_status_code} joint_enable={joint_enable}"
        )

    def _feedback_age_sec(self, now: float, received_at: float) -> float:
        if received_at <= 0.0:
            return float("inf")
        return max(0.0, now - received_at)

    def _feedback_diag_value_ms(self, value_sec: float) -> str:
        if not math.isfinite(value_sec):
            return "inf"
        return f"{value_sec * 1000.0:.1f}"

    def _message_is_fresh(self, received_at: float) -> bool:
        if received_at <= 0.0:
            return False
        if self.robot_feedback_timeout_sec <= 0.0:
            return True
        return (time.monotonic() - received_at) <= self.robot_feedback_timeout_sec

    def _robot_feedback_scale(self) -> float:
        now = time.monotonic()
        with self._follower_feedback_lock:
            follower_received_at = self._follower_received_at
            feedback_count = self._follower_feedback_count
            max_callback_gap_sec = self._follower_feedback_max_gap_sec
        age_sec = self._feedback_age_sec(now, follower_received_at)
        active = (
            follower_received_at > 0.0
            and (
                self.robot_feedback_timeout_sec <= 0.0
                or age_sec <= self.robot_feedback_timeout_sec
            )
        )
        if active != self._robot_feedback_active:
            self._robot_feedback_active = active
            if active:
                self._robot_feedback_restore_started_at = now
                stale_duration_sec = (
                    now - self._robot_feedback_stale_started_at
                    if self._robot_feedback_stale_started_at > 0.0
                    else 0.0
                )
                self.get_logger().warn(
                    "Follower feedback active; ramping leader torque feedback over "
                    f"{self.robot_feedback_restore_ramp_sec:.3f}s; "
                    f"stale_duration_ms={self._feedback_diag_value_ms(stale_duration_sec)} "
                    f"last_stale_age_ms={self._feedback_diag_value_ms(self._robot_feedback_last_stale_age_sec)} "
                    f"threshold_ms={self._feedback_diag_value_ms(self.robot_feedback_timeout_sec)} "
                    f"max_callback_gap_ms={self._feedback_diag_value_ms(max_callback_gap_sec)} "
                    f"feedback_count={feedback_count} "
                    f"stale_count={self._robot_feedback_stale_count}"
                )
                self._robot_feedback_stale_started_at = 0.0
                with self._follower_feedback_lock:
                    self._follower_feedback_max_gap_sec = 0.0
            else:
                self._robot_feedback_stale_started_at = now
                self._robot_feedback_stale_count += 1
                self._robot_feedback_last_stale_age_sec = age_sec
                self.get_logger().warn(
                    "Follower feedback stale; gating leader torque feedback and "
                    f"{'holding' if self.hold_position_when_feedback_inactive else 'continuing'} "
                    "follower target updates; "
                    f"age_ms={self._feedback_diag_value_ms(age_sec)} "
                    f"threshold_ms={self._feedback_diag_value_ms(self.robot_feedback_timeout_sec)} "
                    f"max_callback_gap_ms={self._feedback_diag_value_ms(max_callback_gap_sec)} "
                    f"feedback_count={feedback_count} "
                    f"stale_count={self._robot_feedback_stale_count}"
                )

        if not active:
            return 0.0
        if self.robot_feedback_restore_ramp_sec <= 0.0:
            return 1.0
        return _clamp(
            (now - self._robot_feedback_restore_started_at)
            / self.robot_feedback_restore_ramp_sec,
            0.0,
            1.0,
        )

    def _normalize_between(self, value: float, lower_endpoint: float, upper_endpoint: float) -> float:
        span = upper_endpoint - lower_endpoint
        if abs(span) < 1e-6:
            return 0.0
        return _clamp((value - lower_endpoint) / span, 0.0, 1.0)

    def _denormalize_between(
        self,
        ratio: float,
        lower_endpoint: float,
        upper_endpoint: float,
    ) -> float:
        return lower_endpoint + _clamp(ratio, 0.0, 1.0) * (upper_endpoint - lower_endpoint)

    def _opening_from_gripper_status(self, status: Optional[DriverGripperStatus]) -> float:
        if status is None:
            return 0.0

        opening = self._local_gripper_opening(status, "local gripper")
        if opening is None:
            return 0.0
        return float(opening)

    def _local_gripper_opening(
        self,
        status: Optional[DriverGripperStatus],
        source: str,
    ) -> Optional[float]:
        if status is None:
            return None
        if self.role == "leader" and self._runtime_reseed_in_progress:
            last_opening = self._leader_gripper_adapter.last_opening
            if last_opening is not None:
                return float(last_opening)
            return None
        mode = str(getattr(status, "mode", "")).strip().lower()
        if self.role == "leader" and mode == "angle":
            raw_position = float(getattr(status, "position", math.nan))
            if (
                math.isfinite(raw_position)
                and raw_position <= self.leader_gripper_min_trusted_angle_position
                and not bool(getattr(status, "driver_enable_status", False))
            ):
                last_opening = self._leader_gripper_adapter.last_opening
                if last_opening is None:
                    self._leader_gripper_adapter.reset()
                    self._leader_stale_near_zero_count += 1
                    self.get_logger().warn(
                        "Leader gripper angle feedback is near zero while driver is disabled; "
                        "treating it as untrusted startup feedback and suppressing gripper intent "
                        "until it changes or the gripper is commanded.",
                        throttle_duration_sec=2.0,
                    )
                    return None
                if last_opening >= self.gripper_recover_seed_min_width:
                    self._leader_stale_near_zero_count += 1
                    self.get_logger().warn(
                        "Leader gripper feedback jumped to stale near-zero while the "
                        f"last trusted opening was {last_opening:.6f}m; holding the "
                        "last trusted gripper intent instead of commanding follower closed. "
                        "Runtime master gripper commands are disabled in passive sensor mode.",
                        throttle_duration_sec=1.0,
                    )
                    return float(last_opening)
            self._leader_stale_near_zero_count = 0
            return self._leader_gripper_adapter.opening_from_status(status)
        self._leader_stale_near_zero_count = 0
        return self._require_width_opening(status, source)

    def _gripper_command_from_opening(
        self,
        opening: float,
        status: Optional[DriverGripperStatus],
    ) -> Tuple[float, str]:
        opening = _clamp(float(opening), 0.0, self.gripper_range)

        width = _clamp(opening, 0.0, self.gripper_range)
        return float(width), "width"

    def _leader_gripper_command_from_opening(
        self,
        opening: float,
        status: Optional[DriverGripperStatus],
    ) -> Tuple[float, str]:
        del status
        # Haptic lock-align uses the proven width command path. Runtime master
        # feedback remains the old 1:1 stroke-like opening path.
        return self._gripper_command_from_opening(opening, None)

    def _master_to_follower_gripper_opening(self, master_opening: float) -> float:
        return master_to_follower_opening(
            master_opening,
            self.gripper_v2_geom_scale,
            self.gripper_v2_geom_offset,
            self.gripper_range,
        )

    def _follower_to_master_gripper_opening(self, follower_opening: float) -> float:
        return follower_to_master_opening(
            follower_opening,
            self.gripper_v2_geom_scale,
            self.gripper_v2_geom_offset,
            self.gripper_range,
        )

    def _snapshot_local_state(self):
        with self._state_lock:
            return (
                list(self._current_joint_positions),
                list(self._current_joint_efforts),
                float(self._current_gripper_opening),
                float(self._current_gripper_force),
                self._current_gripper_status,
                list(self._filtered_joint_positions),
                float(self._filtered_gripper_opening),
            )

    def _update_filtered_state(
        self,
        positions: List[float],
        velocities: List[float],
        efforts: List[float],
        gripper_opening: float,
        gripper_force: float,
    ) -> Tuple[List[float], List[float], List[float], float, float]:
        if not self._filter_initialized:
            self._filtered_joint_positions = list(positions)
            self._filtered_joint_velocities = list(velocities)
            self._filtered_joint_efforts = list(efforts)
            self._filtered_gripper_opening = float(gripper_opening)
            self._filtered_gripper_force = float(gripper_force)
            self._filter_initialized = True
        else:
            if self.filter_enable:
                for index in range(self.arm_joint_count):
                    self._filtered_joint_positions[index] = (
                        self.filter_alpha_pos * positions[index]
                        + (1.0 - self.filter_alpha_pos) * self._filtered_joint_positions[index]
                    )
                    self._filtered_joint_velocities[index] = (
                        self.filter_alpha_vel * velocities[index]
                        + (1.0 - self.filter_alpha_vel) * self._filtered_joint_velocities[index]
                    )
                    self._filtered_joint_efforts[index] = (
                        self.filter_alpha_effort * efforts[index]
                        + (1.0 - self.filter_alpha_effort) * self._filtered_joint_efforts[index]
                    )
            else:
                self._filtered_joint_positions = list(positions)
                self._filtered_joint_velocities = list(velocities)
                self._filtered_joint_efforts = list(efforts)

            # Formal gripper runtime is raw-width direct pass-through. Keep the
            # published / commanded gripper coordinate equal to the current
            # measured width and only filter the force signal.
            self._filtered_gripper_opening = float(gripper_opening)
            if self.gripper_filter_enable:
                self._filtered_gripper_force = (
                    self.gripper_filter_alpha_effort * gripper_force
                    + (1.0 - self.gripper_filter_alpha_effort) * self._filtered_gripper_force
                )
            else:
                self._filtered_gripper_force = float(gripper_force)

        return (
            list(self._filtered_joint_positions),
            list(self._filtered_joint_velocities),
            list(self._filtered_joint_efforts),
            float(self._filtered_gripper_opening),
            float(self._filtered_gripper_force),
        )

    def _joint_command_callback(self, msg: JointState):
        positions = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return
            positions.append(float(msg.position[index]))

        gripper_opening = 0.0
        gripper_force = self.gripper_follow_force_default
        if GRIPPER_JOINT_NAME in msg.name:
            index = msg.name.index(GRIPPER_JOINT_NAME)
            if index < len(msg.position):
                gripper_opening = float(msg.position[index])
            if index < len(msg.effort):
                gripper_force = float(msg.effort[index])

        with self._command_lock:
            self._target_joint_positions = positions
            self._joint_command_received_at = time.monotonic()
            if GRIPPER_JOINT_NAME in msg.name:
                self._target_gripper_opening = gripper_opening
                self._target_gripper_force = gripper_force
                self._gripper_command_received_at = self._joint_command_received_at

    def _leader_joint_state_command_callback(self, msg: JointState):
        positions = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return
            positions.append(float(msg.position[index]))

        with self._command_lock:
            self._target_joint_positions = positions
            self._joint_command_received_at = time.monotonic()

    def _leader_gripper_intent_callback(self, msg: JointState):
        if GRIPPER_JOINT_NAME not in msg.name:
            return
        index = msg.name.index(GRIPPER_JOINT_NAME)
        if index >= len(msg.position):
            return
        leader_gripper_opening = float(msg.position[index])
        gripper_opening = self._master_to_follower_gripper_opening(
            leader_gripper_opening
        )
        gripper_force = self.gripper_follow_force_default
        if index < len(msg.effort):
            gripper_force = float(msg.effort[index])

        with self._command_lock:
            self._target_gripper_opening = gripper_opening
            self._target_gripper_force = gripper_force
            self._gripper_command_received_at = time.monotonic()

    def _follower_joint_state_callback(self, msg: JointState):
        positions = []
        efforts = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return
            positions.append(float(msg.position[index]))
            efforts.append(float(msg.effort[index]) if index < len(msg.effort) else 0.0)

        gripper_opening = 0.0
        gripper_effort = 0.0
        if GRIPPER_JOINT_NAME in msg.name:
            index = msg.name.index(GRIPPER_JOINT_NAME)
            if index < len(msg.position):
                gripper_opening = float(msg.position[index])
            if index < len(msg.effort):
                gripper_effort = float(msg.effort[index])

        now = time.monotonic()
        with self._follower_feedback_lock:
            if self._follower_received_at > 0.0:
                self._follower_feedback_max_gap_sec = max(
                    self._follower_feedback_max_gap_sec,
                    now - self._follower_received_at,
                )
            self._follower_positions = positions
            self._follower_efforts = efforts
            self._follower_gripper_opening = gripper_opening
            self._follower_gripper_effort = gripper_effort
            self._follower_received_at = now
            self._follower_feedback_count += 1

    def _follower_gravity_callback(self, msg: JointState):
        efforts = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return
            index = msg.name.index(joint_name)
            efforts.append(float(msg.effort[index]) if index < len(msg.effort) else 0.0)
        with self._follower_feedback_lock:
            self._follower_gravity_efforts = efforts
            self._follower_gravity_received_at = time.monotonic()

    def _follower_gripper_status_callback(self, msg: GripperStatus):
        mode = str(msg.mode).strip().lower()
        if mode != "width":
            self._fatal_gripper_width_error(
                f"remote follower gripper feedback must be width mode, got mode={mode!r}. "
                "Refusing to mix gripper units."
            )
        force = float(msg.force)
        with self._follower_feedback_lock:
            if not self._follower_gripper_force_filter_initialized:
                self._filtered_follower_gripper_force = force
                self._follower_gripper_force_filter_initialized = True
            elif self.gripper_filter_enable:
                self._filtered_follower_gripper_force = (
                    self.gripper_filter_alpha_effort * force
                    + (1.0 - self.gripper_filter_alpha_effort)
                    * self._filtered_follower_gripper_force
                )
            else:
                self._filtered_follower_gripper_force = force
            self._follower_gripper_force = self._filtered_follower_gripper_force
            self._follower_gripper_force_received_at = time.monotonic()

    def _leader_gripper_lock_callback(self, msg: Bool):
        if self.role != "leader":
            return
        requested_lock = bool(msg.data)
        (
            _positions,
            _efforts,
            current_opening,
            _current_force,
            current_status,
            _filtered_positions,
            _filtered_opening,
        ) = self._snapshot_local_state()
        current_opening = _clamp(float(current_opening), 0.0, self.gripper_range)

        if requested_lock:
            if self.gripper is None or not self.enable_gripper:
                self.get_logger().warn(
                    "Ignoring gripper lock request because leader gripper is disabled",
                    throttle_duration_sec=1.0,
                )
                return
            if self._local_gripper_opening(current_status, "local leader gripper") is None:
                self.get_logger().warn(
                    "Ignoring gripper lock request until leader gripper width feedback is valid",
                    throttle_duration_sec=1.0,
                )
                return
            with self._state_lock:
                already_locked = bool(self._leader_gripper_lock_enabled)
                self._leader_gripper_lock_enabled = True
                self._leader_gripper_lock_opening = current_opening
            if not already_locked:
                self.get_logger().warn(
                    f"Leader gripper LOCKED at {current_opening:.5f} m"
                )
            return

        with self._state_lock:
            was_locked = bool(self._leader_gripper_lock_enabled)
            self._leader_gripper_lock_enabled = False
            self._leader_gripper_lock_opening = current_opening
        if was_locked:
            self._release_leader_gripper_drive(current_opening, current_status)
            self.get_logger().warn("Leader gripper UNLOCKED")

    def _fatal_gripper_width_error(self, message: str):
        self.get_logger().error(message)
        self._shutdown_event.set()
        try:
            rclpy.try_shutdown(context=self.context)
        except Exception:
            pass
        raise GripperWidthModeError(message)

    def _require_width_opening(
        self,
        status: Optional[DriverGripperStatus],
        source: str,
    ) -> Optional[float]:
        try:
            return require_formal_width_from_status(status, self.gripper_range, source)
        except ValueError as exc:
            self._fatal_gripper_width_error(str(exc))

    def _unwrap_target(self, current: List[float], target: List[float]) -> List[float]:
        if not self.unwrap_joint_positions:
            return list(target)
        unwrapped = []
        for current_value, target_value in zip(current, target):
            candidate = target_value
            while candidate - current_value > math.pi:
                candidate -= 2.0 * math.pi
            while candidate - current_value < -math.pi:
                candidate += 2.0 * math.pi
            unwrapped.append(candidate)
        return unwrapped

    def _apply_joint_limits(self, positions: List[float]) -> List[float]:
        if not self.enforce_joint_limits:
            return list(positions)
        return [
            _clamp(value, lower, upper)
            for value, lower, upper in zip(
                positions, self.arm_joint_lower_limits, self.arm_joint_upper_limits
            )
        ]

    def _limit_joint_step(self, current: List[float], target: List[float]) -> List[float]:
        if self.max_joint_step <= 0.0:
            return list(target)
        limited = []
        for current_value, target_value in zip(current, target):
            delta = _clamp(target_value - current_value, -self.max_joint_step, self.max_joint_step)
            limited.append(current_value + delta)
        return limited

    def _limit_gripper_step(self, current_opening: float, target_opening: float) -> float:
        if self.gripper_max_step <= 0.0:
            return float(target_opening)
        delta = _clamp(
            float(target_opening) - float(current_opening),
            -self.gripper_max_step,
            self.gripper_max_step,
        )
        return float(current_opening) + delta

    def _sanitize_gripper_raw_cmd(self, raw_cmd: int) -> int:
        raw_cmd = abs(int(raw_cmd))
        if self.gripper_cmd_limit_enable:
            raw_cmd = max(self.gripper_cmd_min_raw, min(self.gripper_cmd_max_raw, raw_cmd))
        if raw_cmd < self.gripper_cmd_deadband_raw:
            return 0
        return raw_cmd

    def _leader_feedforward(
        self,
        follower_efforts: List[float],
        gravity_efforts: List[float],
        feedback_scale: float,
    ) -> List[float]:
        torques = []
        for index in range(self.arm_joint_count):
            feedback_effort = follower_efforts[index]
            if self.mit_enable_gravity and self.gravity_mix_mode == "additive":
                # The follower's motor torque already contains its static gravity load.
                # In additive mode gravity is injected separately from Pinocchio, so
                # subtract it here to avoid a self-exciting loop when the follower is
                # merely chasing the leader with no external contact.
                feedback_effort -= gravity_efforts[index]
            torque_cmd = feedback_effort * self.mit_torque_feedback_sign[index] * feedback_scale
            if not self.mit_enable_tor:
                torque_cmd = 0.0
            gravity = gravity_efforts[index] if self.mit_enable_gravity else 0.0
            if self.gravity_mix_mode == "legacy_diff":
                total = -(torque_cmd - gravity) * self.mit_torque_scale[index] + gravity
            elif self.gravity_mix_mode == "gravity_only":
                total = gravity
            else:
                total = torque_cmd * self.mit_torque_scale[index] + gravity
            torques.append(
                _clamp(total, -self.mit_max_torque_abs[index], self.mit_max_torque_abs[index])
            )
        return torques

    def _follower_gravity_feedforward(self) -> List[float]:
        if not self.follower_mit_enable_gravity:
            return [0.0] * self.arm_joint_count

        with self._follower_feedback_lock:
            gravity_efforts = list(self._follower_gravity_efforts)
            gravity_received_at = float(self._follower_gravity_received_at)

        if not self._message_is_fresh(gravity_received_at):
            return [0.0] * self.arm_joint_count

        torques = []
        for index in range(self.arm_joint_count):
            total = gravity_efforts[index] * self.follower_gravity_torque_scale[index]
            torques.append(
                _clamp(
                    total,
                    -self.follower_mit_max_torque_abs[index],
                    self.follower_mit_max_torque_abs[index],
                )
            )
        return torques

    def _send_follower_mit_hold(self, positions: List[float]):
        torques = self._follower_gravity_feedforward()
        for index in range(self.arm_joint_count):
            self.agx_arm.move_mit(
                joint_index=index + 1,
                p_des=positions[index],
                v_des=0.0,
                kp=self.follower_mit_kp[index],
                kd=self.follower_mit_kd[index],
                t_ff=torques[index],
            )
        self._maybe_log_follower_gravity_debug(positions, torques)

    def _maybe_log_follower_gravity_debug(
        self,
        positions: List[float],
        torques: List[float],
    ):
        if not self.follower_gravity_debug_enable:
            return
        now = time.monotonic()
        if now - self._last_follower_gravity_debug_log < self.follower_gravity_debug_interval_sec:
            return
        self._last_follower_gravity_debug_log = now

        with self._follower_feedback_lock:
            gravity_efforts = list(self._follower_gravity_efforts)
            gravity_received_at = float(self._follower_gravity_received_at)
        with self._state_lock:
            current_positions = list(self._current_joint_positions)

        gravity_age = now - gravity_received_at if gravity_received_at > 0.0 else math.inf
        try:
            arm_status = self.agx_arm.get_arm_status()
            ctrl_mode = getattr(arm_status.msg, "ctrl_mode", None) if arm_status else None
            mode_feedback = getattr(arm_status.msg, "mode_feedback", None) if arm_status else None
            arm_status_code = getattr(arm_status.msg, "arm_status", None) if arm_status else None
        except Exception as exc:
            ctrl_mode = mode_feedback = arm_status_code = f"read_error:{exc}"

        self.get_logger().warn(
            "Follower MIT gravity debug: "
            f"fresh={self._message_is_fresh(gravity_received_at)} "
            f"gravity_age={gravity_age:.3f}s "
            f"ctrl_mode={ctrl_mode} mode_feedback={mode_feedback} arm_status={arm_status_code} "
            f"j3 target={positions[2]:+.3f} current={current_positions[2]:+.3f} "
            f"raw_g={gravity_efforts[2]:+.3f} scale={self.follower_gravity_torque_scale[2]:.3f} "
            f"t_ff={torques[2]:+.3f} kp={self.follower_mit_kp[2]:.3f} kd={self.follower_mit_kd[2]:.3f}; "
            f"j6 target={positions[5]:+.3f} current={current_positions[5]:+.3f} "
            f"raw_g={gravity_efforts[5]:+.3f} scale={self.follower_gravity_torque_scale[5]:.3f} "
            f"t_ff={torques[5]:+.3f} kp={self.follower_mit_kp[5]:.3f} kd={self.follower_mit_kd[5]:.3f}",
        )

    def _remember_leader_mit_command(
        self,
        positions: List[float],
        kp_values: List[float],
        kd_values: List[float],
        torques: List[float],
    ):
        with self._state_lock:
            self._last_leader_mit_positions = list(positions)
            self._last_leader_mit_kp = list(kp_values)
            self._last_leader_mit_kd = list(kd_values)
            self._last_leader_mit_torques = list(torques)

    def _shutdown_joint_positions(self) -> List[float]:
        try:
            joint_states = self.agx_arm.get_joint_angles()
            if joint_states is not None and len(joint_states.msg) >= self.arm_joint_count:
                return [float(value) for value in joint_states.msg[: self.arm_joint_count]]
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to read live leader joints during shutdown: {exc}",
                throttle_duration_sec=1.0,
            )

        with self._state_lock:
            if len(self._current_joint_positions) >= self.arm_joint_count:
                return list(self._current_joint_positions[: self.arm_joint_count])
            if self._last_leader_mit_positions is not None:
                return list(self._last_leader_mit_positions[: self.arm_joint_count])
        return [0.0] * self.arm_joint_count

    def _snapshot_last_leader_mit_command(
        self,
        positions: List[float],
    ) -> Tuple[List[float], List[float], List[float], List[float]]:
        with self._state_lock:
            last_positions = self._last_leader_mit_positions
            last_kp = self._last_leader_mit_kp
            last_kd = self._last_leader_mit_kd
            last_torques = self._last_leader_mit_torques

        def _valid_or(values: Optional[List[float]], fallback: List[float]) -> List[float]:
            if values is None or len(values) < self.arm_joint_count:
                return list(fallback)
            return [float(value) for value in values[: self.arm_joint_count]]

        return (
            _valid_or(last_positions, positions),
            _valid_or(last_kp, [0.0] * self.arm_joint_count),
            _valid_or(last_kd, [0.0] * self.arm_joint_count),
            _valid_or(last_torques, [0.0] * self.arm_joint_count),
        )

    def _send_leader_mit_frame(
        self,
        positions: List[float],
        kp_values: List[float],
        kd_values: List[float],
        torques: List[float],
    ):
        for index in range(self.arm_joint_count):
            self.agx_arm.move_mit(
                joint_index=index + 1,
                p_des=positions[index],
                v_des=0.0,
                kp=kp_values[index],
                kd=kd_values[index],
                t_ff=torques[index],
            )

    def _leader_shutdown_cleanup(self):
        if (
            self.role != "leader"
            or not self.leader_shutdown_cleanup_enable
            or self._leader_shutdown_cleanup_done
        ):
            return
        self._leader_shutdown_cleanup_done = True
        if not hasattr(self, "agx_arm") or self.agx_arm is None:
            return

        try:
            positions = self._shutdown_joint_positions()
            (
                last_positions,
                start_kp,
                start_kd,
                start_torques,
            ) = self._snapshot_last_leader_mit_command(positions)
            # Keep the target near the current arm pose while ramping force to zero.
            if len(positions) < self.arm_joint_count:
                positions = last_positions

            self.get_logger().info(
                "Leader shutdown cleanup: ramping MIT kp/kd/t_ff to zero "
                f"over {self.leader_shutdown_ramp_sec:.3f}s"
            )
            try:
                self.agx_arm.set_motion_mode("mit")
                self.agx_arm.set_auto_set_motion_mode_enabled(False)
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to pin MIT mode for shutdown cleanup: {exc}",
                    throttle_duration_sec=1.0,
                )

            steps = max(1, int(self.leader_shutdown_ramp_steps))
            sleep_sec = self.leader_shutdown_ramp_sec / steps if steps > 0 else 0.0
            for step in range(1, steps + 1):
                scale = 1.0 - float(step) / float(steps)
                kp_values = [float(value) * scale for value in start_kp]
                kd_values = [float(value) * scale for value in start_kd]
                torques = [float(value) * scale for value in start_torques]
                self._send_leader_mit_frame(positions, kp_values, kd_values, torques)
                if sleep_sec > 0.0 and step < steps:
                    time.sleep(sleep_sec)

            self._send_leader_mit_frame(
                positions,
                [0.0] * self.arm_joint_count,
                [0.0] * self.arm_joint_count,
                [0.0] * self.arm_joint_count,
            )
            self._apply_leader_shutdown_final_mode()
        except Exception as exc:
            self.get_logger().error(f"Leader shutdown cleanup failed: {exc}")
            if self.leader_shutdown_disable_on_failure:
                self._disable_arm_after_shutdown_cleanup_failure()

    def _apply_leader_shutdown_final_mode(self):
        mode = self.leader_shutdown_final_mode
        if mode in ("none", ""):
            self.get_logger().info("Leader shutdown cleanup: leaving final mode unchanged")
            return
        if mode in ("zero_force", "leader"):
            self.agx_arm.set_motion_mode("j")
            self.agx_arm.set_leader_mode()
            self.get_logger().info("Leader shutdown cleanup: restored zero-force leader mode")
            return
        if mode == "normal":
            if getattr(self, "firmware_version", NeroFW.DEFAULT) == NeroFW.V112:
                self.agx_arm.set_follower_mode()
            else:
                self.agx_arm.set_normal_mode()
            self.agx_arm.set_motion_mode("j")
            self.get_logger().info("Leader shutdown cleanup: restored normal J mode")
            return
        if mode == "disable":
            self._enable_arm(False, min(self.enable_timeout, 2.0))
            self.get_logger().info("Leader shutdown cleanup: disabled leader arm")
            return
        raise ValueError(f"Unsupported leader shutdown final mode: {mode}")

    def _disable_arm_after_shutdown_cleanup_failure(self):
        try:
            self._enable_arm(False, min(self.enable_timeout, 2.0))
            self.get_logger().warn(
                "Leader shutdown cleanup fallback disabled the arm after failure"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Leader shutdown cleanup fallback disable failed: {exc}"
            )

    def _compute_leader_joint_command(
        self,
        leader_positions: List[float],
    ) -> Tuple[List[float], List[float], List[float]]:
        target_positions = []
        kp_values = []
        kd_values = []
        softened = []
        for index, position in enumerate(leader_positions):
            lower = self.arm_joint_lower_limits[index]
            upper = self.arm_joint_upper_limits[index]
            clamped = _clamp(position, lower, upper)
            outside = position < lower or position > upper
            target_positions.append(clamped)
            if not self.mit_enable_pos:
                kp_values.append(0.0)
                kd_values.append(self.mit_kd[index])
            elif outside:
                kp_values.append(0.0)
                kd_values.append(self.mit_kd[index])
                softened.append(str(index + 1))
            else:
                kp_values.append(self.mit_kp[index])
                kd_values.append(self.mit_kd[index])
        if softened:
            self.get_logger().warn(
                "Leader joints outside MIT limits; softened impedance on joints "
                + ", ".join(softened),
                throttle_duration_sec=2.0,
            )
        return target_positions, kp_values, kd_values

    def _reset_leader_spring_haptic_state(self):
        self._leader_spring_align_active = False
        self._leader_spring_hold_until = 0.0
        self._leader_spring_hold_opening = 0.0

    def _compute_gripper_haptic_command(
        self,
        *,
        leader_opening: float,
        leader_opening_delta: float,
        follower_gripper_opening: float,
        follower_opening_received_at: float,
        follower_gripper_force: float,
        follower_force_received_at: float,
    ) -> CueHapticResult:
        if self.gripper is None:
            self._leader_directional_resistance_active = False
            self._leader_mismatch_pulse_active_until = 0.0
            self._leader_mismatch_pulse_armed = True
            self._leader_mismatch_pulse_error = 0.0
            self._leader_mismatch_force_ready_since = 0.0
            self._reset_leader_spring_haptic_state()
            return CueHapticResult(
                force=0.0,
                enabled=False,
                cmd_raw=0,
                state=HAPTIC_FREE_FOLLOW,
                filtered_effort=0.0,
                opening_detected=False,
            )
        if self.gripper_haptic_spring_align_enable:
            self._leader_directional_resistance_active = False
            self._leader_mismatch_pulse_active_until = 0.0
            self._leader_mismatch_pulse_armed = True
            now = time.monotonic()
            fresh_follower_opening = self._message_is_fresh(follower_opening_received_at)
            force_ready, force_ready_since = mismatch_force_ready(
                follower_force=follower_gripper_force,
                fresh_follower_force=self._message_is_fresh(
                    follower_force_received_at
                ),
                force_threshold_n=self.gripper_haptic_mismatch_force_threshold_n,
                confirm_sec=self.gripper_haptic_mismatch_force_confirm_sec,
                time_sec=now,
                force_ready_since_sec=self._leader_mismatch_force_ready_since,
                force_sign=self.gripper_haptic_mismatch_force_sign,
            )
            self._leader_mismatch_force_ready_since = force_ready_since

            if not fresh_follower_opening or not all(
                math.isfinite(float(value))
                for value in (leader_opening, follower_gripper_opening)
            ):
                self._reset_leader_spring_haptic_state()
                return CueHapticResult(
                    force=0.0,
                    enabled=False,
                    cmd_raw=0,
                    state=HAPTIC_STALE,
                    filtered_effort=0.0,
                    opening_detected=False,
                )

            abs_error = abs(float(follower_gripper_opening) - float(leader_opening))
            sequence_active = (
                self._leader_spring_hold_until > 0.0
                or self._leader_spring_align_active
            )
            if sequence_active and now < self._leader_spring_hold_until:
                if not force_ready:
                    self._reset_leader_spring_haptic_state()
                    self._leader_mismatch_pulse_error = abs_error
                    return CueHapticResult(
                        force=0.0,
                        enabled=False,
                        cmd_raw=0,
                        state=HAPTIC_FREE_FOLLOW,
                        filtered_effort=abs_error,
                        opening_detected=False,
                    )
                result = spring_hold_result(
                    leader_opening=leader_opening,
                    follower_opening=follower_gripper_opening,
                    hold_opening=self._leader_spring_hold_opening,
                    fresh_follower_opening=fresh_follower_opening,
                    force_n=self.gripper_haptic_force_max_n,
                    force_max_n=self.gripper_haptic_force_max_n,
                    cmd_max=self.gripper_haptic_cmd_max,
                )
                if not result.enabled:
                    self._reset_leader_spring_haptic_state()
                if math.isfinite(float(result.filtered_effort)):
                    self._leader_mismatch_pulse_error = float(result.filtered_effort)
                return result

            if sequence_active:
                result = spring_align_result(
                    leader_opening=leader_opening,
                    follower_opening=follower_gripper_opening,
                    fresh_follower_opening=fresh_follower_opening,
                    force_ready=force_ready,
                    threshold_m=self.gripper_haptic_mismatch_threshold_m,
                    reset_threshold_m=self.gripper_haptic_mismatch_reset_threshold_m,
                    spring_k_n_per_m=self.gripper_haptic_spring_k_n_per_m,
                    min_force_n=self.gripper_haptic_spring_min_force_n,
                    force_max_n=self.gripper_haptic_force_max_n,
                    cmd_max=self.gripper_haptic_cmd_max,
                    active=True,
                )
                self._leader_spring_align_active = (
                    result.enabled and result.state == HAPTIC_SPRING_ALIGN
                )
                if not self._leader_spring_align_active:
                    self._leader_spring_hold_until = 0.0
                    self._leader_spring_hold_opening = 0.0
                if math.isfinite(float(result.filtered_effort)):
                    self._leader_mismatch_pulse_error = float(result.filtered_effort)
                return result

            if (
                self.gripper_haptic_mismatch_threshold_m > 0.0
                and abs_error >= self.gripper_haptic_mismatch_threshold_m
                and force_ready
            ):
                self._leader_spring_hold_opening = float(leader_opening)
                if self.gripper_haptic_spring_hold_sec > 0.0:
                    self._leader_spring_hold_until = (
                        now + self.gripper_haptic_spring_hold_sec
                    )
                    result = spring_hold_result(
                        leader_opening=leader_opening,
                        follower_opening=follower_gripper_opening,
                        hold_opening=self._leader_spring_hold_opening,
                        fresh_follower_opening=fresh_follower_opening,
                        force_n=self.gripper_haptic_force_max_n,
                        force_max_n=self.gripper_haptic_force_max_n,
                        cmd_max=self.gripper_haptic_cmd_max,
                    )
                else:
                    self._leader_spring_hold_until = now
                    self._leader_spring_align_active = True
                    result = spring_align_result(
                        leader_opening=leader_opening,
                        follower_opening=follower_gripper_opening,
                        fresh_follower_opening=fresh_follower_opening,
                        force_ready=True,
                        threshold_m=self.gripper_haptic_mismatch_threshold_m,
                        reset_threshold_m=self.gripper_haptic_mismatch_reset_threshold_m,
                        spring_k_n_per_m=self.gripper_haptic_spring_k_n_per_m,
                        min_force_n=self.gripper_haptic_spring_min_force_n,
                        force_max_n=self.gripper_haptic_force_max_n,
                        cmd_max=self.gripper_haptic_cmd_max,
                        active=True,
                    )
                if math.isfinite(float(result.filtered_effort)):
                    self._leader_mismatch_pulse_error = float(result.filtered_effort)
                return result

            self._reset_leader_spring_haptic_state()
            result = spring_align_result(
                leader_opening=leader_opening,
                follower_opening=follower_gripper_opening,
                fresh_follower_opening=fresh_follower_opening,
                force_ready=False,
                threshold_m=self.gripper_haptic_mismatch_threshold_m,
                reset_threshold_m=self.gripper_haptic_mismatch_reset_threshold_m,
                spring_k_n_per_m=self.gripper_haptic_spring_k_n_per_m,
                min_force_n=self.gripper_haptic_spring_min_force_n,
                force_max_n=self.gripper_haptic_force_max_n,
                cmd_max=self.gripper_haptic_cmd_max,
                active=False,
            )
            if math.isfinite(float(result.filtered_effort)):
                self._leader_mismatch_pulse_error = float(result.filtered_effort)
            return result
        if self.gripper_haptic_mismatch_pulse_enable:
            self._leader_directional_resistance_active = False
            self._reset_leader_spring_haptic_state()
            now = time.monotonic()
            force_ready, force_ready_since = mismatch_force_ready(
                follower_force=follower_gripper_force,
                fresh_follower_force=self._message_is_fresh(
                    follower_force_received_at
                ),
                force_threshold_n=self.gripper_haptic_mismatch_force_threshold_n,
                confirm_sec=self.gripper_haptic_mismatch_force_confirm_sec,
                time_sec=now,
                force_ready_since_sec=self._leader_mismatch_force_ready_since,
                force_sign=self.gripper_haptic_mismatch_force_sign,
            )
            self._leader_mismatch_force_ready_since = force_ready_since
            result, active_until, armed, abs_error = mismatch_pulse_result(
                leader_opening=leader_opening,
                follower_opening=follower_gripper_opening,
                fresh_follower_opening=self._message_is_fresh(follower_opening_received_at),
                follower_force=follower_gripper_force,
                fresh_follower_force=force_ready,
                force_threshold_n=self.gripper_haptic_mismatch_force_threshold_n,
                threshold_m=self.gripper_haptic_mismatch_threshold_m,
                reset_threshold_m=self.gripper_haptic_mismatch_reset_threshold_m,
                pulse_sec=self.gripper_haptic_mismatch_pulse_sec,
                force_n=self.gripper_haptic_force_max_n,
                force_max_n=self.gripper_haptic_force_max_n,
                cmd_max=self.gripper_haptic_cmd_max,
                time_sec=now,
                active_until_sec=self._leader_mismatch_pulse_active_until,
                armed=self._leader_mismatch_pulse_armed,
                closing_only=self.gripper_haptic_mismatch_closing_only,
                retrigger_sec=self.gripper_haptic_mismatch_retrigger_sec,
            )
            self._leader_mismatch_pulse_active_until = active_until
            self._leader_mismatch_pulse_armed = armed
            if math.isfinite(float(abs_error)):
                self._leader_mismatch_pulse_error = float(abs_error)
            return result
        if self.gripper_haptic_constant_force_n > 0.0:
            self._leader_mismatch_pulse_active_until = 0.0
            self._leader_mismatch_pulse_armed = True
            self._leader_mismatch_force_ready_since = 0.0
            self._reset_leader_spring_haptic_state()
            directional_active = None
            if self.gripper_haptic_constant_directional:
                if not math.isfinite(float(leader_opening_delta)):
                    self._leader_directional_resistance_active = False
                elif (
                    float(leader_opening_delta)
                    < -self.gripper_haptic_opening_direction_eps
                ):
                    self._leader_directional_resistance_active = True
                elif (
                    float(leader_opening_delta)
                    > self.gripper_haptic_opening_direction_eps
                ):
                    self._leader_directional_resistance_active = False
                directional_active = self._leader_directional_resistance_active
            return constant_resistance_result(
                force_n=self.gripper_haptic_constant_force_n,
                force_max_n=self.gripper_haptic_force_max_n,
                cmd_max=self.gripper_haptic_cmd_max,
                leader_opening_delta=leader_opening_delta,
                directional=self.gripper_haptic_constant_directional,
                opening_direction_eps=self.gripper_haptic_opening_direction_eps,
                directional_active=directional_active,
            )
        self._leader_directional_resistance_active = False
        self._leader_mismatch_pulse_active_until = 0.0
        self._leader_mismatch_pulse_armed = True
        self._leader_mismatch_force_ready_since = 0.0
        self._reset_leader_spring_haptic_state()
        return self._gripper_haptic_controller.update(
            leader_opening=leader_opening,
            leader_opening_delta=leader_opening_delta,
            follower_force=follower_gripper_force,
            time_sec=time.monotonic(),
            fresh_follower_force=self._message_is_fresh(follower_force_received_at),
        )

    def _snapshot_leader_haptic_lane(self) -> Tuple[bool, float, str, int, float, bool]:
        with self._state_lock:
            return (
                bool(self._leader_haptic_engaged),
                float(self._leader_haptic_force),
                str(self._leader_haptic_state),
                int(self._leader_haptic_cmd_raw),
                float(self._leader_haptic_filtered_effort),
                bool(self._leader_haptic_opening_detected),
            )

    def _update_leader_haptic_lane(
        self,
        *,
        engaged: bool,
        force: float,
        state: str,
        cmd_raw: int = 0,
        filtered_effort: float = 0.0,
        opening_detected: bool = False,
    ):
        with self._state_lock:
            self._leader_haptic_engaged = bool(engaged)
            self._leader_haptic_force = float(force)
            self._leader_haptic_state = str(state)
            self._leader_haptic_cmd_raw = int(cmd_raw)
            self._leader_haptic_filtered_effort = float(filtered_effort)
            self._leader_haptic_opening_detected = bool(opening_detected)

    def _snapshot_leader_gripper_lock(self) -> Tuple[bool, float]:
        with self._state_lock:
            return (
                bool(self._leader_gripper_lock_enabled),
                float(self._leader_gripper_lock_opening),
            )

    def _release_leader_gripper_drive(
        self,
        leader_opening: float,
        leader_gripper_status: Optional[DriverGripperStatus],
    ):
        """Actively return the leader gripper to passive sensor mode.

        Retries each control tick until the firmware reports
        driver_enable_status=False. Escalates to reset()+disable() after
        ~500ms of sustained failure to evict a latched motor without
        power-cycling the arm.
        """
        if self.gripper is None:
            return
        driver_actually_enabled = False
        if leader_gripper_status is not None:
            driver_actually_enabled = bool(
                getattr(leader_gripper_status, "driver_enable_status", False)
            )
        if not self._leader_gripper_drive_enabled and not driver_actually_enabled:
            self._release_retry_count = 0
            return
        command_position, command_mode = self._leader_gripper_command_from_opening(
            leader_opening,
            leader_gripper_status,
        )
        try:
            self.gripper.command_position(
                position=command_position,
                force=0.0,
                mode=command_mode,
                enabled=False,
            )
            if command_mode != "width":
                self.gripper.command_position(
                    position=_clamp(float(leader_opening), 0.0, self.gripper_range),
                    force=0.0,
                    mode="width",
                    enabled=False,
                )
            self.gripper.disable()
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=1.0)
        except Exception as exc:
            self.get_logger().warn(
                f"Leader gripper release failed: {exc}",
                throttle_duration_sec=1.0,
            )
        if not driver_actually_enabled:
            self._leader_gripper_drive_enabled = False
            self._release_retry_count = 0
            return
        self._release_retry_count += 1
        RETRY_ESCALATE_TICKS = 100
        if self._release_retry_count == RETRY_ESCALATE_TICKS:
            self.get_logger().warn(
                f"Leader gripper driver still enabled after "
                f"{RETRY_ESCALATE_TICKS * 5}ms of release attempts; "
                "escalating to reset()+disable()",
                throttle_duration_sec=2.0,
            )
            try:
                self.gripper.reset()
                self.gripper.disable()
            except Exception as exc:
                self.get_logger().warn(
                    f"Leader gripper escalated release failed: {exc}",
                    throttle_duration_sec=1.0,
                )

    def _command_leader_gripper_self_loop(
        self,
        leader_opening: float,
        leader_gripper_status: Optional[DriverGripperStatus],
        force: float,
    ) -> bool:
        return self._command_leader_gripper_position(
            leader_opening,
            leader_gripper_status,
            force,
        )

    def _command_leader_gripper_position(
        self,
        target_opening: float,
        leader_gripper_status: Optional[DriverGripperStatus],
        force: float,
    ) -> bool:
        if self.gripper is None:
            return False
        command_position, command_mode = self._leader_gripper_command_from_opening(
            target_opening,
            leader_gripper_status,
        )
        try:
            self.gripper.command_position(
                position=command_position,
                force=force,
                mode=command_mode,
                enabled=True,
            )
            self._leader_gripper_drive_enabled = True
            return True
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=1.0)
            return False

    def _hold_leader_gripper_self_loop_when_haptic_inactive(self) -> bool:
        return (
            self.gripper_haptic_constant_force_n > 0.0
            and self.gripper_haptic_constant_directional
        )

    def _shutdown_gripper_drive(self):
        """Best-effort gripper release on process shutdown.

        Nero keeps the last gripper command latched in hardware. If we only
        stop the ROS node and disconnect the arm, the gripper can remain in the
        last enabled/force-applied state and feel "locked" after Ctrl+C.
        """
        self._recover_gripper_state_once("shutdown")

    def _publish_joint_state(
        self,
        stamp: float,
        positions: List[float],
        velocities: List[float],
        efforts: List[float],
        gripper_opening: float,
        gripper_force: float,
    ):
        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(stamp)
        msg.name = list(self.arm_joint_names) + [GRIPPER_JOINT_NAME]
        msg.position = list(positions) + [gripper_opening]
        msg.velocity = list(velocities) + [0.0]
        msg.effort = list(efforts) + [gripper_force]
        self.joint_state_pub.publish(msg)

    def _publish_arm_status(self, arm_status):
        if arm_status is None:
            return
        msg = AgxArmStatus()
        msg.ctrl_mode = arm_status.msg.ctrl_mode
        msg.arm_status = arm_status.msg.arm_status
        msg.mode_feedback = arm_status.msg.mode_feedback
        msg.teach_status = arm_status.msg.teach_status
        msg.motion_status = arm_status.msg.motion_status
        msg.trajectory_num = arm_status.msg.trajectory_num
        msg.err_status = int(arm_status.msg.err_code)
        err = arm_status.msg.err_status
        for index in range(self.arm_joint_count):
            msg.joint_angle_limit.append(getattr(err, f"joint_{index + 1}_angle_limit"))
            msg.communication_status_joint.append(
                getattr(err, f"communication_status_joint_{index + 1}")
            )
        self.arm_status_pub.publish(msg)

    def _publish_gripper_status(self, status: Optional[DriverGripperStatus]):
        if status is None:
            return
        msg = GripperStatus()
        msg.header.stamp = self._float_to_ros_time(status.timestamp)
        msg.position = self._published_gripper_status_position(status)
        msg.mode = status.mode
        msg.width = status.width
        msg.force = status.force
        msg.voltage_too_low = status.voltage_too_low
        msg.motor_overheating = status.motor_overheating
        msg.driver_overcurrent = status.driver_overcurrent
        msg.driver_overheating = status.driver_overheating
        msg.sensor_status = status.sensor_status
        msg.driver_error_status = status.driver_error_status
        msg.driver_enable_status = status.driver_enable_status
        msg.homing_status = status.homing_status
        self.gripper_status_pub.publish(msg)

    def _published_gripper_status_position(
        self, status: Optional[DriverGripperStatus]
    ) -> float:
        if status is None:
            return 0.0

        raw_position = float(getattr(status, "position", 0.0))
        if self.role != "leader":
            return raw_position

        mode = str(getattr(status, "mode", "")).strip().lower()
        if mode != "angle":
            return raw_position

        if not math.isfinite(raw_position):
            return raw_position

        if bool(getattr(status, "driver_enable_status", False)):
            return raw_position

        if raw_position > self.leader_gripper_min_trusted_angle_position:
            return raw_position

        last_opening = self._leader_gripper_adapter.last_opening
        if (
            last_opening is not None
            and last_opening >= self.gripper_recover_seed_min_width
        ):
            return float(self.leader_gripper_stale_hold_angle_position)

        return raw_position

    def _publish_gripper_intent(self, stamp: float, gripper_opening: float, width_valid: bool):
        if self.role != "leader" or not width_valid:
            return
        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(stamp)
        msg.name = [GRIPPER_JOINT_NAME]
        msg.position = [float(gripper_opening)]
        msg.velocity = [0.0]
        msg.effort = [float(self.gripper_follow_force_default)]
        self.gripper_intent_pub.publish(msg)
        with self._state_lock:
            self._leader_gripper_intent_opening = float(gripper_opening)
            self._leader_gripper_intent_published_at = time.monotonic()

    def _publish_gripper_haptic_debug(self, stamp: float):
        if self.role != "leader":
            return
        engaged, force, state, cmd_raw, filtered_effort, opening_detected = (
            self._snapshot_leader_haptic_lane()
        )
        state_id = {
            HAPTIC_FREE_FOLLOW: 0,
            HAPTIC_ACTIVE: 1,
            HAPTIC_OPENING_RELEASE: 2,
            HAPTIC_STALE: 3,
            HAPTIC_MISMATCH_PULSE: 4,
            HAPTIC_SPRING_ALIGN: 5,
            HAPTIC_SPRING_HOLD: 6,
        }.get(state, -1)

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(stamp)
        msg.name = [
            "engaged",
            "force_cmd",
            "cmd_raw",
            "filtered_effort",
            "opening_detected",
            "state_id",
        ]
        msg.position = [
            1.0 if engaged else 0.0,
            float(force),
            float(cmd_raw),
            float(filtered_effort),
            1.0 if opening_detected else 0.0,
            float(state_id),
        ]
        self.gripper_haptic_debug_pub.publish(msg)

    def _publisher_step(self):
        joint_states = self.agx_arm.get_joint_angles()
        if (joint_states is None or joint_states.hz <= 0) and self.role == "leader":
            get_leader_joint_angles = getattr(self.agx_arm, "get_leader_joint_angles", None)
            if callable(get_leader_joint_angles):
                joint_states = get_leader_joint_angles()
        if joint_states is None or joint_states.hz <= 0:
            return

        velocities = []
        efforts = []
        for joint_index in range(1, self.arm_joint_count + 1):
            motor_state = self.agx_arm.get_motor_states(joint_index)
            if motor_state is None:
                if self.role == "leader":
                    velocities.append(0.0)
                    efforts.append(0.0)
                    continue
                return
            velocities.append(float(motor_state.msg.velocity))
            efforts.append(float(motor_state.msg.torque))

        gripper_status = self.gripper.get_status() if self.gripper is not None else None
        formal_width = self._local_gripper_opening(gripper_status, "local gripper")
        gripper_width_valid = formal_width is not None
        if gripper_width_valid:
            gripper_opening = float(formal_width)
        else:
            with self._state_lock:
                gripper_opening = float(self._current_gripper_opening)
        gripper_force = float(gripper_status.force) if gripper_status is not None else 0.0

        positions = [float(value) for value in joint_states.msg]
        stamp = float(joint_states.timestamp)
        (
            filtered_positions,
            filtered_velocities,
            filtered_efforts,
            filtered_gripper_opening,
            filtered_gripper_force,
        ) = self._update_filtered_state(
            positions,
            velocities,
            efforts,
            gripper_opening,
            gripper_force,
        )

        with self._state_lock:
            self._current_joint_positions = list(positions)
            self._current_joint_velocities = list(velocities)
            self._current_joint_efforts = list(efforts)
            self._current_gripper_opening = gripper_opening
            self._current_gripper_force = gripper_force
            self._current_gripper_status = gripper_status
            self._filtered_joint_positions = list(filtered_positions)
            self._filtered_joint_velocities = list(filtered_velocities)
            self._filtered_joint_efforts = list(filtered_efforts)
            self._filtered_gripper_opening = filtered_gripper_opening
            self._filtered_gripper_force = filtered_gripper_force
            self._gripper_width_valid = bool(gripper_width_valid)
            self._latest_joint_stamp = stamp

        if not self.control_ready:
            if self.role == "follower":
                with self._command_lock:
                    if self._joint_command_received_at <= 0.0:
                        self._target_joint_positions = list(positions)
                    if self._gripper_command_received_at <= 0.0:
                        self._target_gripper_opening = gripper_opening
                        self._target_gripper_force = self.gripper_follow_force_default
            self.control_ready = True
            if not self._control_ready_logged:
                self.get_logger().info("Arm feedback is ready, dedicated control is now enabled")
                self._control_ready_logged = True

        # Keep arm telemetry filtered. Formal gripper runtime is raw width
        # direct pass-through: publish and follow the current measured width.
        self._publish_joint_state(
            stamp,
            filtered_positions,
            filtered_velocities,
            filtered_efforts,
            gripper_opening,
            filtered_gripper_force,
        )
        self._publish_gripper_intent(stamp, gripper_opening, gripper_width_valid)
        self._publish_gripper_haptic_debug(stamp)
        self._publish_arm_status(self.agx_arm.get_arm_status())
        self._publish_gripper_status(gripper_status)
        self._maybe_runtime_reseed_leader()

    def _publisher_loop(self):
        period = 1.0 / self.publish_rate
        next_tick = time.monotonic()
        while not self._shutdown_event.is_set():
            try:
                if self.agx_arm.is_ok():
                    self._publisher_step()
            except Exception as exc:
                if self._shutdown_event.is_set():
                    break
                self.get_logger().warn(f"Publisher loop error: {exc}", throttle_duration_sec=1.0)
            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    def _check_can_control(self) -> bool:
        return self.enable_flag and self.control_ready and self.agx_arm.is_ok()

    def _follower_control_step(self):
        self.get_logger().info(
            "Follower control step active",
            throttle_duration_sec=5.0,
        )
        if not self._check_can_control():
            self.get_logger().info(
                f"Follower control gate closed enable={self.enable_flag} "
                f"ready={self.control_ready} arm_ok={self.agx_arm.is_ok()}",
                throttle_duration_sec=1.0,
            )
            return
        with self._command_lock:
            positions = list(self._target_joint_positions)
            gripper_opening = float(self._target_gripper_opening)
            gripper_force = float(self._target_gripper_force)
            joint_command_received_at = float(self._joint_command_received_at)
            gripper_command_received_at = float(self._gripper_command_received_at)
        # Before an upstream command arrives, hold the startup pose captured
        # when follower feedback first became ready. Re-sampling live feedback
        # here makes the hold target follow payload-induced sag.
        if gripper_command_received_at <= 0.0:
            _, _, gripper_opening, _, _, _, _ = self._snapshot_local_state()
            gripper_force = self.gripper_follow_force_default
        (
            current_positions,
            _current_efforts,
            current_gripper_opening,
            _current_gripper_force,
            current_gripper_status,
            _filtered_positions,
            _filtered_gripper_opening,
        ) = self._snapshot_local_state()
        positions = self._unwrap_target(current_positions, positions)
        positions = self._apply_joint_limits(positions)
        positions = self._limit_joint_step(current_positions, positions)
        gripper_opening = self._limit_gripper_step(current_gripper_opening, gripper_opening)
        gripper_raw_cmd = self._sanitize_gripper_raw_cmd(
            round(float(gripper_opening) * 1e6 * self.gripper_val_multiple)
        )
        gripper_opening = _clamp(
            float(gripper_raw_cmd) / max(1.0, 1e6 * self.gripper_val_multiple),
            0.0,
            self.gripper_range,
        )

        # When only the gripper is moving, avoid flooding the follower CAN bus
        # with redundant arm JS frames. This shortens the visible gripper lag.
        arm_target_changed = self._last_follower_target is None or any(
            abs(target - last) > 1e-5
            for target, last in zip(positions, self._last_follower_target or [])
        )
        arm_not_at_target = any(
            abs(target - current) > 1e-3
            for target, current in zip(positions, current_positions)
        )
        if self.gripper is not None:
            now = time.monotonic()
            current_gripper_valid = (
                self._require_width_opening(
                    current_gripper_status,
                    "local follower gripper",
                )
                is not None
            )
            bootstrap = (
                not current_gripper_valid
                and now < self._follower_gripper_bootstrap_until
            )
            if not current_gripper_valid and not bootstrap:
                self.get_logger().warn(
                    "Follower gripper width feedback unavailable; skipping gripper command "
                    "until width mode reports a valid width.",
                    throttle_duration_sec=1.0,
                )
            else:
                command_position, command_mode = self._gripper_command_from_opening(
                    gripper_opening, current_gripper_status
                )
                gripper_target_changed = (
                    abs(gripper_opening - self._last_follower_gripper_target)
                    > max(self.gripper_command_deadband, 1e-4)
                )
                gripper_not_at_target = (
                    abs(gripper_opening - current_gripper_opening) > 1e-3
                )
                gripper_heartbeat_due = (
                    now - self._last_follower_gripper_cmd_at >= 0.2
                )
                try:
                    if (gripper_target_changed or gripper_not_at_target
                            or gripper_heartbeat_due or bootstrap):
                        if bootstrap:
                            self.get_logger().info(
                                "Follower gripper bootstrap heartbeat",
                                throttle_duration_sec=1.0,
                            )
                        self.gripper.move_position(
                            position=command_position,
                            force=_clamp(gripper_force, 0.5, 3.0),
                            mode=command_mode,
                        )
                        self._last_follower_gripper_target = gripper_opening
                        self._last_follower_gripper_cmd_at = now
                except ValueError as exc:
                    self.get_logger().warn(str(exc), throttle_duration_sec=1.0)

        if self.follower_control_mode == "mit_gravity":
            self._send_follower_mit_hold(positions)
            self._last_follower_target = list(positions)
        elif arm_target_changed or arm_not_at_target:
            self.agx_arm.move_js(positions)
            self._last_follower_target = list(positions)

    def _leader_control_step(self):
        if not self._check_can_control():
            return

        (
            leader_positions,
            _leader_efforts,
            leader_gripper_opening,
            _leader_gripper_force,
            leader_gripper_status,
            filtered_leader_positions,
            _filtered_leader_gripper_opening,
        ) = self._snapshot_local_state()
        if not leader_positions:
            return

        with self._follower_feedback_lock:
            follower_positions = list(self._follower_positions)
            follower_efforts = list(self._follower_efforts)
            follower_gripper_opening = float(self._follower_gripper_opening)
            follower_gripper_force = float(self._follower_gripper_force)
            follower_received_at = float(self._follower_received_at)
            follower_gripper_force_received_at = float(
                self._follower_gripper_force_received_at
            )
            gravity_efforts = list(self._follower_gravity_efforts)
            gravity_received_at = float(self._follower_gravity_received_at)

        feedback_scale = self._robot_feedback_scale()
        if not self._message_is_fresh(gravity_received_at):
            gravity_efforts = [0.0] * self.arm_joint_count
        if not self._message_is_fresh(follower_received_at):
            follower_efforts = [0.0] * self.arm_joint_count
            follower_positions = list(leader_positions)
        if not self._message_is_fresh(follower_gripper_force_received_at):
            follower_gripper_force = 0.0
        follower_gripper_opening_in_master = self._follower_to_master_gripper_opening(
            follower_gripper_opening
        )

        leader_target, kp_values, kd_values = self._compute_leader_joint_command(leader_positions)
        leader_torques = self._leader_feedforward(
            follower_efforts,
            gravity_efforts,
            feedback_scale,
        )
        self._remember_leader_mit_command(
            leader_target,
            kp_values,
            kd_values,
            leader_torques,
        )
        for index in range(self.arm_joint_count):
            self.agx_arm.move_mit(
                joint_index=index + 1,
                p_des=leader_target[index],
                v_des=0.0,
                kp=kp_values[index],
                kd=kd_values[index],
                t_ff=leader_torques[index],
            )

        if self.gripper is not None and self.enable_gripper:
            leader_gripper_valid = (
                self._local_gripper_opening(
                    leader_gripper_status,
                    "local leader gripper",
                )
                is not None
            )
            if not leader_gripper_valid:
                with self._state_lock:
                    self._leader_gripper_lock_enabled = False
                self._leader_directional_resistance_active = False
                self._reset_leader_spring_haptic_state()
                self._update_leader_haptic_lane(
                    engaged=False,
                    force=0.0,
                    state=HAPTIC_STALE,
                    cmd_raw=0,
                    filtered_effort=0.0,
                    opening_detected=False,
                )
                self._release_leader_gripper_drive(
                    leader_gripper_opening,
                    leader_gripper_status,
                )
                self._last_leader_gripper_opening = leader_gripper_opening
                return
            gripper_locked, gripper_lock_opening = self._snapshot_leader_gripper_lock()
            if gripper_locked:
                self._leader_directional_resistance_active = False
                self._reset_leader_spring_haptic_state()
                self._update_leader_haptic_lane(
                    engaged=False,
                    force=0.0,
                    state=HAPTIC_FREE_FOLLOW,
                    cmd_raw=0,
                    filtered_effort=0.0,
                    opening_detected=False,
                )
                self._command_leader_gripper_position(
                    gripper_lock_opening,
                    leader_gripper_status,
                    force=self.leader_gripper_lock_force,
                )
                self._last_leader_gripper_opening = leader_gripper_opening
                return
            if not self.enable_gripper_haptic:
                self._leader_directional_resistance_active = False
                self._reset_leader_spring_haptic_state()
                self._update_leader_haptic_lane(
                    engaged=False,
                    force=0.0,
                    state=HAPTIC_FREE_FOLLOW,
                    cmd_raw=0,
                    filtered_effort=0.0,
                    opening_detected=False,
                )
                self._release_leader_gripper_drive(
                    leader_gripper_opening,
                    leader_gripper_status,
                )
                self._last_leader_gripper_opening = leader_gripper_opening
                return
            leader_opening_delta = (
                leader_gripper_opening - self._last_leader_gripper_opening
            )
            haptic_result = self._compute_gripper_haptic_command(
                leader_opening=leader_gripper_opening,
                leader_opening_delta=leader_opening_delta,
                follower_gripper_opening=follower_gripper_opening_in_master,
                follower_opening_received_at=follower_received_at,
                follower_gripper_force=follower_gripper_force,
                follower_force_received_at=follower_gripper_force_received_at,
            )
            if not haptic_result.enabled:
                self._update_leader_haptic_lane(
                    engaged=False,
                    force=0.0,
                    state=haptic_result.state,
                    cmd_raw=haptic_result.cmd_raw,
                    filtered_effort=haptic_result.filtered_effort,
                    opening_detected=haptic_result.opening_detected,
                )
                if (
                    self._hold_leader_gripper_self_loop_when_haptic_inactive()
                    and haptic_result.state != HAPTIC_STALE
                ):
                    self._command_leader_gripper_self_loop(
                        leader_gripper_opening,
                        leader_gripper_status,
                        force=0.0,
                    )
                else:
                    self._release_leader_gripper_drive(
                        leader_gripper_opening,
                        leader_gripper_status,
                    )
                self._last_leader_gripper_opening = leader_gripper_opening
                return
            self._update_leader_haptic_lane(
                engaged=True,
                force=haptic_result.force,
                state=haptic_result.state,
                cmd_raw=haptic_result.cmd_raw,
                filtered_effort=haptic_result.filtered_effort,
                opening_detected=haptic_result.opening_detected,
            )
            leader_haptic_target = (
                float(haptic_result.target_opening)
                if haptic_result.target_opening is not None
                and math.isfinite(float(haptic_result.target_opening))
                else leader_gripper_opening
            )
            self._command_leader_gripper_position(
                leader_haptic_target,
                leader_gripper_status,
                force=haptic_result.force,
            )
        self._last_leader_gripper_opening = leader_gripper_opening

    def _control_loop(self):
        period = 1.0 / self.control_rate
        next_tick = time.monotonic()
        while not self._shutdown_event.is_set():
            try:
                if self.role == "leader":
                    self._leader_control_step()
                else:
                    self._follower_control_step()
            except Exception as exc:
                if self._shutdown_event.is_set():
                    break
                self.get_logger().warn(f"Control loop error: {exc}", throttle_duration_sec=1.0)
            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    def destroy_node(self):
        try:
            self._shutdown_event.set()
            if hasattr(self, "_publisher_thread") and self._publisher_thread.is_alive():
                self._publisher_thread.join(timeout=1.0)
            if hasattr(self, "_control_thread") and self._control_thread.is_alive():
                self._control_thread.join(timeout=1.0)
            if hasattr(self, "agx_arm") and self.agx_arm is not None:
                self._leader_shutdown_cleanup()
            if hasattr(self, "gripper") and self.gripper is not None:
                self._shutdown_gripper_drive()
            if hasattr(self, "agx_arm") and self.agx_arm is not None:
                self.agx_arm.disconnect()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NeroCtrlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
