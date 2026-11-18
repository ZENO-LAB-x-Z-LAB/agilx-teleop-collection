#!/usr/bin/env python3
import time
from typing import List, Optional, Tuple

import rclpy
from agx_arm_msgs.msg import GripperStatus, MoveMITMsg
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


def _normalize_vector(values, length: int, default: float) -> List[float]:
    if not isinstance(values, (list, tuple)) or len(values) == 0:
        return [default] * length
    if len(values) == 1:
        return [float(values[0])] * length
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {len(values)}")
    return [float(value) for value in values]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LeaderTeleopNode(Node):
    def __init__(self):
        super().__init__("leader_teleop_node")

        arm_joint_names = [f"joint{i}" for i in range(1, 8)]
        self.declare_parameter("teleop_ns", "teleop_arm")
        self.declare_parameter("robot_ns", "robot_arm")
        self.declare_parameter("joint_names", arm_joint_names)
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("prefer_leader_joint_angles", False)
        self.declare_parameter("joint_lower_limits", [-3.14] * 7)
        self.declare_parameter("joint_upper_limits", [3.14] * 7)
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("enable_pos", True)
        self.declare_parameter("enable_tor", True)
        self.declare_parameter("enable_gravity", True)
        self.declare_parameter("gravity_mix_mode", "additive")
        self.declare_parameter("kp", [0.02, 0.1, 0.15, 0.1, 0.1, 0.1, 0.08])
        self.declare_parameter("kd", [0.02, 0.03, 0.01, 0.02, 0.02, 0.02, 0.02])
        self.declare_parameter("kp_follow", [8.0] * 7)
        self.declare_parameter("torque_scale", [0.5, 0.15, 0.6, 0.5, 0.15, 0.6, 0.2])
        self.declare_parameter("torque_feedback_sign", [-1.0] * 7)
        self.declare_parameter("max_torque_abs", [6.0, 6.0, 6.0, 4.0, 4.0, 3.0, 2.0])
        self.declare_parameter("clip_to_limits", True)
        self.declare_parameter("unwrap_joint_positions", True)
        self.declare_parameter("max_joint_step", 0.0)
        self.declare_parameter("robot_feedback_timeout_sec", 0.2)
        self.declare_parameter("robot_feedback_restore_ramp_sec", 0.5)
        self.declare_parameter("hold_position_when_feedback_inactive", True)
        self.declare_parameter("gripper_follow_force_default", 3.0)
        self.declare_parameter("gripper_haptic_effort_sign", -1.0)
        self.declare_parameter("gripper_haptic_effort_deadband", 0.15)
        self.declare_parameter("gripper_haptic_effort_bias", 0.0)
        self.declare_parameter("gripper_haptic_effort_alpha", 0.2)
        self.declare_parameter("gripper_haptic_effort_max", 3.0)
        self.declare_parameter("gripper_haptic_release_when_opening", True)
        self.declare_parameter("gripper_haptic_opening_relief", 0.15)
        self.declare_parameter("gripper_haptic_opening_sign", 1.0)
        self.declare_parameter("gripper_haptic_opening_direction_eps", 1e-4)

        self.teleop_ns = str(self.get_parameter("teleop_ns").value)
        self.robot_ns = str(self.get_parameter("robot_ns").value)
        self.arm_joint_names = [str(value) for value in self.get_parameter("joint_names").value]
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.arm_joint_count = len(self.arm_joint_names)
        self.prefer_leader_joint_angles = bool(
            self.get_parameter("prefer_leader_joint_angles").value
        )

        self._refresh_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.follow_enabled = True
        self.leader_joint_state: Optional[JointState] = None
        self.follower_joint_state: Optional[JointState] = None
        self.follower_joint_states_compensated: Optional[JointState] = None
        self.follower_gripper_status: Optional[GripperStatus] = None
        self.gravity_torque_compat = [0.0] * self.arm_joint_count

        self.leader_joint_state_received_at = 0.0
        self.follower_joint_state_received_at = 0.0
        self.follower_joint_states_compensated_received_at = 0.0
        self.follower_gripper_status_received_at = 0.0
        self.gravity_torque_compat_received_at = 0.0
        self.robot_feedback_active = False
        self.robot_feedback_restore_started_at = 0.0
        self.last_follower_target: Optional[List[float]] = None
        self.gripper_haptic_effort_filtered = 0.0
        self.last_leader_gripper_width = 0.0

        self.create_subscription(
            JointState,
            "feedback/joint_states",
            self._leader_joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            f"/{self.robot_ns}/feedback/joint_states",
            self._follower_joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            f"/{self.robot_ns}/feedback/joint_states_compensated",
            self._follower_joint_states_compensated_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f"/{self.robot_ns}/gravity/torque",
            self._gravity_compat_callback,
            10,
        )
        self.create_subscription(
            GripperStatus,
            f"/{self.robot_ns}/feedback/gripper_status",
            self._follower_gripper_status_callback,
            10,
        )
        self.create_subscription(Bool, "follow_enabled", self._follow_enabled_callback, 10)

        self.leader_mit_pub = self.create_publisher(MoveMITMsg, "control/move_mit", 10)
        self.leader_gripper_pub = self.create_publisher(JointState, "control/joint_states", 10)
        self.follower_joint_pub = self.create_publisher(
            JointState,
            f"/{self.robot_ns}/control/joint_states",
            10,
        )

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / max(control_rate_hz, 1.0), self._control_loop)

        self.get_logger().info(f"Leader pose source: /{self.teleop_ns}/feedback/joint_states")
        self.get_logger().info(f"Follower torque source: /{self.robot_ns}/feedback/joint_states")
        self.get_logger().info(
            f"Gravity source: /{self.robot_ns}/feedback/joint_states_compensated"
        )
        self.get_logger().info(
            "Follower feedback gate: "
            f"timeout={self.robot_feedback_timeout_sec:.3f}s "
            f"restore_ramp={self.robot_feedback_restore_ramp_sec:.3f}s "
            f"hold_position_when_feedback_inactive={self.hold_position_when_feedback_inactive}"
        )
        if self.prefer_leader_joint_angles:
            self.get_logger().warn(
                "prefer_leader_joint_angles is ignored in ROS1-style bilateral MIT mode."
            )
        self.get_logger().info(
            "ROS1-style bilateral MIT teleop enabled; leader pose comes from feedback/joint_states."
        )

    def _refresh_parameters(self):
        self.joint_lower_limits = _normalize_vector(
            self.get_parameter("joint_lower_limits").value,
            self.arm_joint_count,
            -3.14,
        )
        self.joint_upper_limits = _normalize_vector(
            self.get_parameter("joint_upper_limits").value,
            self.arm_joint_count,
            3.14,
        )
        self.enable_pos = bool(self.get_parameter("enable_pos").value)
        self.enable_tor = bool(self.get_parameter("enable_tor").value)
        self.enable_gravity = bool(self.get_parameter("enable_gravity").value)
        self.gravity_mix_mode = str(self.get_parameter("gravity_mix_mode").value)
        self.kp = _normalize_vector(self.get_parameter("kp").value, self.arm_joint_count, 0.1)
        self.kd = _normalize_vector(self.get_parameter("kd").value, self.arm_joint_count, 0.02)
        self.kp_follow = _normalize_vector(
            self.get_parameter("kp_follow").value,
            self.arm_joint_count,
            8.0,
        )
        self.torque_scale = _normalize_vector(
            self.get_parameter("torque_scale").value,
            self.arm_joint_count,
            1.0,
        )
        self.torque_feedback_sign = _normalize_vector(
            self.get_parameter("torque_feedback_sign").value,
            self.arm_joint_count,
            -1.0,
        )
        self.max_torque_abs = _normalize_vector(
            self.get_parameter("max_torque_abs").value,
            self.arm_joint_count,
            6.0,
        )
        self.clip_to_limits = bool(self.get_parameter("clip_to_limits").value)
        self.unwrap_joint_positions = bool(self.get_parameter("unwrap_joint_positions").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.robot_feedback_timeout_sec = max(
            0.0,
            float(self.get_parameter("robot_feedback_timeout_sec").value),
        )
        self.robot_feedback_restore_ramp_sec = max(
            0.0,
            float(self.get_parameter("robot_feedback_restore_ramp_sec").value),
        )
        self.hold_position_when_feedback_inactive = bool(
            self.get_parameter("hold_position_when_feedback_inactive").value
        )
        self.gripper_follow_force_default = _clamp(
            float(self.get_parameter("gripper_follow_force_default").value),
            0.5,
            3.0,
        )
        self.gripper_haptic_effort_sign = (
            1.0
            if float(self.get_parameter("gripper_haptic_effort_sign").value) >= 0.0
            else -1.0
        )
        self.gripper_haptic_effort_deadband = max(
            0.0,
            float(self.get_parameter("gripper_haptic_effort_deadband").value),
        )
        self.gripper_haptic_effort_bias = float(
            self.get_parameter("gripper_haptic_effort_bias").value
        )
        self.gripper_haptic_effort_alpha = _clamp(
            float(self.get_parameter("gripper_haptic_effort_alpha").value),
            0.0,
            1.0,
        )
        self.gripper_haptic_effort_max = _clamp(
            float(self.get_parameter("gripper_haptic_effort_max").value),
            0.5,
            3.0,
        )
        self.gripper_haptic_release_when_opening = bool(
            self.get_parameter("gripper_haptic_release_when_opening").value
        )
        self.gripper_haptic_opening_relief = _clamp(
            float(self.get_parameter("gripper_haptic_opening_relief").value),
            0.0,
            1.0,
        )
        self.gripper_haptic_opening_sign = (
            1.0
            if float(self.get_parameter("gripper_haptic_opening_sign").value) >= 0.0
            else -1.0
        )
        self.gripper_haptic_opening_direction_eps = max(
            0.0,
            float(self.get_parameter("gripper_haptic_opening_direction_eps").value),
        )
        if self.gravity_mix_mode not in ("additive", "legacy_diff", "gravity_only"):
            raise ValueError(
                "gravity_mix_mode must be one of additive, legacy_diff, gravity_only"
            )

    def _on_set_parameters(self, parameters):
        staged = {parameter.name: parameter.value for parameter in parameters}
        try:
            joint_lower_limits = _normalize_vector(
                staged.get("joint_lower_limits", self.joint_lower_limits),
                self.arm_joint_count,
                -3.14,
            )
            joint_upper_limits = _normalize_vector(
                staged.get("joint_upper_limits", self.joint_upper_limits),
                self.arm_joint_count,
                3.14,
            )
            kp = _normalize_vector(staged.get("kp", self.kp), self.arm_joint_count, 0.1)
            kd = _normalize_vector(staged.get("kd", self.kd), self.arm_joint_count, 0.02)
            kp_follow = _normalize_vector(
                staged.get("kp_follow", self.kp_follow),
                self.arm_joint_count,
                8.0,
            )
            torque_scale = _normalize_vector(
                staged.get("torque_scale", self.torque_scale),
                self.arm_joint_count,
                1.0,
            )
            torque_feedback_sign = _normalize_vector(
                staged.get("torque_feedback_sign", self.torque_feedback_sign),
                self.arm_joint_count,
                -1.0,
            )
            max_torque_abs = _normalize_vector(
                staged.get("max_torque_abs", self.max_torque_abs),
                self.arm_joint_count,
                6.0,
            )
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if "joint_names" in staged:
            return SetParametersResult(
                successful=False,
                reason="joint_names cannot be changed at runtime",
            )
        if "gripper_joint_name" in staged:
            return SetParametersResult(
                successful=False,
                reason="gripper_joint_name cannot be changed at runtime",
            )
        gravity_mix_mode = str(staged.get("gravity_mix_mode", self.gravity_mix_mode))
        if gravity_mix_mode not in ("additive", "legacy_diff", "gravity_only"):
            return SetParametersResult(
                successful=False,
                reason="gravity_mix_mode must be one of additive, legacy_diff, gravity_only",
            )

        self.joint_lower_limits = joint_lower_limits
        self.joint_upper_limits = joint_upper_limits
        self.enable_pos = bool(staged.get("enable_pos", self.enable_pos))
        self.enable_tor = bool(staged.get("enable_tor", self.enable_tor))
        self.enable_gravity = bool(staged.get("enable_gravity", self.enable_gravity))
        self.gravity_mix_mode = gravity_mix_mode
        self.kp = kp
        self.kd = kd
        self.kp_follow = kp_follow
        self.torque_scale = torque_scale
        self.torque_feedback_sign = torque_feedback_sign
        self.max_torque_abs = max_torque_abs
        self.clip_to_limits = bool(staged.get("clip_to_limits", self.clip_to_limits))
        self.unwrap_joint_positions = bool(
            staged.get("unwrap_joint_positions", self.unwrap_joint_positions)
        )
        self.max_joint_step = float(staged.get("max_joint_step", self.max_joint_step))
        self.robot_feedback_timeout_sec = max(
            0.0,
            float(staged.get("robot_feedback_timeout_sec", self.robot_feedback_timeout_sec)),
        )
        self.robot_feedback_restore_ramp_sec = max(
            0.0,
            float(
                staged.get(
                    "robot_feedback_restore_ramp_sec",
                    self.robot_feedback_restore_ramp_sec,
                )
            ),
        )
        self.hold_position_when_feedback_inactive = bool(
            staged.get(
                "hold_position_when_feedback_inactive",
                self.hold_position_when_feedback_inactive,
            )
        )
        self.gripper_follow_force_default = _clamp(
            float(staged.get("gripper_follow_force_default", self.gripper_follow_force_default)),
            0.5,
            3.0,
        )
        self.gripper_haptic_effort_sign = (
            1.0
            if float(
                staged.get("gripper_haptic_effort_sign", self.gripper_haptic_effort_sign)
            )
            >= 0.0
            else -1.0
        )
        self.gripper_haptic_effort_deadband = max(
            0.0,
            float(
                staged.get(
                    "gripper_haptic_effort_deadband",
                    self.gripper_haptic_effort_deadband,
                )
            ),
        )
        self.gripper_haptic_effort_bias = float(
            staged.get("gripper_haptic_effort_bias", self.gripper_haptic_effort_bias)
        )
        self.gripper_haptic_effort_alpha = _clamp(
            float(
                staged.get("gripper_haptic_effort_alpha", self.gripper_haptic_effort_alpha)
            ),
            0.0,
            1.0,
        )
        self.gripper_haptic_effort_max = _clamp(
            float(staged.get("gripper_haptic_effort_max", self.gripper_haptic_effort_max)),
            0.5,
            3.0,
        )
        self.gripper_haptic_release_when_opening = bool(
            staged.get(
                "gripper_haptic_release_when_opening",
                self.gripper_haptic_release_when_opening,
            )
        )
        self.gripper_haptic_opening_relief = _clamp(
            float(
                staged.get(
                    "gripper_haptic_opening_relief",
                    self.gripper_haptic_opening_relief,
                )
            ),
            0.0,
            1.0,
        )
        self.gripper_haptic_opening_sign = (
            1.0
            if float(
                staged.get("gripper_haptic_opening_sign", self.gripper_haptic_opening_sign)
            )
            >= 0.0
            else -1.0
        )
        self.gripper_haptic_opening_direction_eps = max(
            0.0,
            float(
                staged.get(
                    "gripper_haptic_opening_direction_eps",
                    self.gripper_haptic_opening_direction_eps,
                )
            ),
        )
        return SetParametersResult(successful=True)

    def _leader_joint_state_callback(self, msg: JointState):
        self.leader_joint_state = msg
        self.leader_joint_state_received_at = time.monotonic()

    def _follower_joint_state_callback(self, msg: JointState):
        self.follower_joint_state = msg
        self.follower_joint_state_received_at = time.monotonic()

    def _follower_joint_states_compensated_callback(self, msg: JointState):
        self.follower_joint_states_compensated = msg
        self.follower_joint_states_compensated_received_at = time.monotonic()

    def _gravity_compat_callback(self, msg: Float64MultiArray):
        if len(msg.data) != self.arm_joint_count:
            return
        self.gravity_torque_compat = [float(value) for value in msg.data]
        self.gravity_torque_compat_received_at = time.monotonic()

    def _follower_gripper_status_callback(self, msg: GripperStatus):
        self.follower_gripper_status = msg
        self.follower_gripper_status_received_at = time.monotonic()

    def _follow_enabled_callback(self, msg: Bool):
        new_value = bool(msg.data)
        if self.follow_enabled != new_value:
            self.follow_enabled = new_value
            self.get_logger().info(f"follow_enabled set to {self.follow_enabled}")

    def _extract_joint_positions(self, msg: Optional[JointState]) -> Optional[List[float]]:
        if msg is None:
            return None
        positions = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return None
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return None
            positions.append(float(msg.position[index]))
        return positions

    def _extract_joint_efforts(self, msg: Optional[JointState]) -> Optional[List[float]]:
        if msg is None:
            return None
        efforts = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return None
            index = msg.name.index(joint_name)
            efforts.append(float(msg.effort[index]) if index < len(msg.effort) else 0.0)
        return efforts

    def _extract_gripper_width(self, msg: Optional[JointState]) -> float:
        if msg is None:
            return 0.0
        if self.gripper_joint_name in msg.name:
            index = msg.name.index(self.gripper_joint_name)
            if index < len(msg.position):
                return abs(float(msg.position[index]))
        if "gripper_joint1" in msg.name:
            index = msg.name.index("gripper_joint1")
            if index < len(msg.position):
                return abs(float(msg.position[index]) * 2.0)
        if "gripper_joint2" in msg.name:
            index = msg.name.index("gripper_joint2")
            if index < len(msg.position):
                return abs(float(msg.position[index]) * 2.0)
        return 0.0

    def _unwrap_target(self, current: List[float], target: List[float]) -> List[float]:
        if not self.unwrap_joint_positions:
            return target
        unwrapped = []
        for current_value, target_value in zip(current, target):
            candidate = target_value
            while candidate - current_value > 3.141592653589793:
                candidate -= 6.283185307179586
            while candidate - current_value < -3.141592653589793:
                candidate += 6.283185307179586
            unwrapped.append(candidate)
        return unwrapped

    def _apply_limits(self, positions: List[float]) -> List[float]:
        if not self.clip_to_limits:
            return positions
        return [
            _clamp(value, lower, upper)
            for value, lower, upper in zip(
                positions, self.joint_lower_limits, self.joint_upper_limits
            )
        ]

    def _limit_step(self, current: List[float], target: List[float]) -> List[float]:
        if self.max_joint_step <= 0.0:
            return target
        limited = []
        for current_value, target_value in zip(current, target):
            delta = target_value - current_value
            delta = _clamp(delta, -self.max_joint_step, self.max_joint_step)
            limited.append(current_value + delta)
        return limited

    def _message_is_fresh(self, received_at: float) -> bool:
        if received_at <= 0.0:
            return False
        if self.robot_feedback_timeout_sec <= 0.0:
            return True
        return (time.monotonic() - received_at) <= self.robot_feedback_timeout_sec

    def _robot_feedback_scale(self) -> float:
        active = self._message_is_fresh(self.follower_joint_state_received_at)
        now = time.monotonic()
        if active != self.robot_feedback_active:
            self.robot_feedback_active = active
            if active:
                self.robot_feedback_restore_started_at = now
                self.get_logger().info(
                    "Follower feedback active; ramping bilateral torque over "
                    f"{self.robot_feedback_restore_ramp_sec:.3f}s"
                )
            else:
                self.get_logger().warn(
                    "Follower feedback stale; gating bilateral torque and "
                    f"{'holding' if self.hold_position_when_feedback_inactive else 'continuing'} "
                    "follower target updates"
                )

        if not active:
            return 0.0
        if self.robot_feedback_restore_ramp_sec <= 0.0:
            return 1.0
        return _clamp(
            (now - self.robot_feedback_restore_started_at)
            / self.robot_feedback_restore_ramp_sec,
            0.0,
            1.0,
        )

    def _gravity_efforts(self) -> List[float]:
        if self.enable_gravity and self._message_is_fresh(
            self.follower_joint_states_compensated_received_at
        ):
            efforts = self._extract_joint_efforts(self.follower_joint_states_compensated)
            if efforts is not None:
                return efforts
        if self.enable_gravity and self._message_is_fresh(self.gravity_torque_compat_received_at):
            return list(self.gravity_torque_compat)
        return [0.0] * self.arm_joint_count

    def _compute_leader_feedforward(
        self,
        follower_efforts: Optional[List[float]],
        feedback_scale: float,
    ) -> List[float]:
        gravity_torques = self._gravity_efforts()
        source_efforts = follower_efforts or [0.0] * self.arm_joint_count

        torques = []
        for index in range(self.arm_joint_count):
            torque_cmd = (
                source_efforts[index]
                * self.torque_feedback_sign[index]
                * feedback_scale
            )
            gravity = gravity_torques[index] if self.enable_gravity else 0.0
            if not self.enable_tor:
                torque_cmd = 0.0

            if self.gravity_mix_mode == "legacy_diff":
                total = -(torque_cmd - gravity) * self.torque_scale[index] + gravity
            elif self.gravity_mix_mode == "gravity_only":
                total = gravity
            else:
                total = torque_cmd * self.torque_scale[index] + gravity
            torques.append(_clamp(total, -self.max_torque_abs[index], self.max_torque_abs[index]))
        return torques

    def _compute_gripper_haptic_force(self, leader_width: float, feedback_scale: float) -> float:
        if (
            self.follower_gripper_status is None
            or not self._message_is_fresh(self.follower_gripper_status_received_at)
        ):
            self.gripper_haptic_effort_filtered = 0.0
            return 0.5

        raw_effort = float(self.follower_gripper_status.force)
        signed_effort = raw_effort * self.gripper_haptic_effort_sign
        active_effort = max(0.0, signed_effort - self.gripper_haptic_effort_bias)
        active_effort *= feedback_scale

        alpha = self.gripper_haptic_effort_alpha
        if alpha > 0.0:
            self.gripper_haptic_effort_filtered = (
                (1.0 - alpha) * self.gripper_haptic_effort_filtered + alpha * active_effort
            )
        else:
            self.gripper_haptic_effort_filtered = active_effort
        active_effort = self.gripper_haptic_effort_filtered

        if active_effort <= self.gripper_haptic_effort_deadband:
            force = 0.5
        else:
            span = max(
                1e-6,
                self.gripper_haptic_effort_max - self.gripper_haptic_effort_deadband,
            )
            normalized = min(
                1.0,
                (active_effort - self.gripper_haptic_effort_deadband) / span,
            )
            force = 0.5 + normalized * (self.gripper_haptic_effort_max - 0.5)

        opening_metric = (
            leader_width - self.last_leader_gripper_width
        ) * self.gripper_haptic_opening_sign
        opening = opening_metric > self.gripper_haptic_opening_direction_eps
        if opening and self.gripper_haptic_release_when_opening:
            force = 0.5 + (force - 0.5) * self.gripper_haptic_opening_relief
        return _clamp(force, 0.5, self.gripper_haptic_effort_max)

    def _publish_leader_gripper(self, leader_width: float, feedback_scale: float):
        msg = JointState()
        msg.name = [self.gripper_joint_name]
        msg.position = [leader_width]
        msg.velocity = [0.0]
        msg.effort = [self._compute_gripper_haptic_force(leader_width, feedback_scale)]
        self.leader_gripper_pub.publish(msg)

    def _publish_follower_joint_target(self, target_positions: List[float], leader_width: float):
        msg = JointState()
        msg.name = list(self.arm_joint_names) + [self.gripper_joint_name]
        msg.position = list(target_positions) + [leader_width]
        msg.velocity = [0.0] * (self.arm_joint_count + 1)
        msg.effort = [0.0] * self.arm_joint_count + [self.gripper_follow_force_default]
        self.follower_joint_pub.publish(msg)

    def _publish_leader_mit(
        self,
        target_positions: List[float],
        kp_gains: List[float],
        kd_gains: List[float],
        torques: List[float],
    ):
        msg = MoveMITMsg()
        msg.joint_index = list(range(1, self.arm_joint_count + 1))
        msg.p_des = list(target_positions)
        msg.v_des = [0.0] * self.arm_joint_count
        msg.kp = list(kp_gains)
        msg.kd = list(kd_gains)
        msg.torque = list(torques)
        self.leader_mit_pub.publish(msg)

    def _compute_follower_target(
        self,
        leader_positions: List[float],
        feedback_scale: float,
    ) -> List[float]:
        if (
            feedback_scale <= 0.0
            and self.hold_position_when_feedback_inactive
            and self.last_follower_target is not None
        ):
            return self.last_follower_target[:]

        follower_positions = self._extract_joint_positions(self.follower_joint_state)
        if follower_positions is None:
            target_positions = self._apply_limits(leader_positions)
            self.last_follower_target = target_positions[:]
            return target_positions

        target_positions = self._unwrap_target(follower_positions, leader_positions)
        target_positions = self._apply_limits(target_positions)
        target_positions = self._limit_step(follower_positions, target_positions)
        self.last_follower_target = target_positions[:]
        return target_positions

    def _compute_leader_mit_command(
        self,
        leader_positions: List[float],
    ) -> Tuple[List[float], List[float], List[float]]:
        target_positions = []
        kp_gains = []
        kd_gains = []
        softened_joint_indexes = []

        for index, position in enumerate(leader_positions):
            lower = self.joint_lower_limits[index]
            upper = self.joint_upper_limits[index]
            clamped_position = _clamp(position, lower, upper)
            outside_limits = position < lower or position > upper

            target_positions.append(clamped_position)
            if not self.enable_pos:
                kp_gains.append(0.0)
                kd_gains.append(self.kd[index])
            elif outside_limits:
                kp_gains.append(0.0)
                kd_gains.append(self.kd[index])
                softened_joint_indexes.append(str(index + 1))
            else:
                kp_gains.append(self.kp[index])
                kd_gains.append(self.kd[index])

        if softened_joint_indexes:
            self.get_logger().warn(
                "Leader joints outside MIT limits; softened impedance on joints "
                + ", ".join(softened_joint_indexes),
                throttle_duration_sec=2.0,
            )

        return target_positions, kp_gains, kd_gains

    def _control_loop(self):
        leader_positions = self._extract_joint_positions(self.leader_joint_state)
        if leader_positions is None:
            return

        feedback_scale = self._robot_feedback_scale()
        leader_width = self._extract_gripper_width(self.leader_joint_state)

        follower_efforts = None
        if self.follow_enabled:
            follower_efforts = self._extract_joint_efforts(self.follower_joint_state)
            follower_target = self._compute_follower_target(leader_positions, feedback_scale)
            self._publish_follower_joint_target(follower_target, leader_width)

        leader_target, leader_kp, leader_kd = self._compute_leader_mit_command(
            leader_positions
        )
        leader_torques = self._compute_leader_feedforward(
            follower_efforts,
            feedback_scale,
        )

        self._publish_leader_mit(leader_target, leader_kp, leader_kd, leader_torques)
        self._publish_leader_gripper(leader_width, feedback_scale)
        self.last_leader_gripper_width = leader_width


def main(args=None):
    rclpy.init(args=args)
    node = LeaderTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
