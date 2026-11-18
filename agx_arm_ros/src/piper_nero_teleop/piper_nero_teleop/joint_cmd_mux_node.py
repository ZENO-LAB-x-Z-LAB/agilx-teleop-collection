#!/usr/bin/env python3
"""Small ROS 2 JointState command mux for teleop/policy handoff.

The follower controller has one canonical command input when mux mode is
enabled. This node owns that final hop and switches which upstream source is
forwarded:

- teleop: master joint state + master gripper intent
- policy: policy/replay JointState command

Selection is controlled by a std_msgs/String command topic. Accepted command
values are teleop/human/master or policy/vla/replay.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


HOT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
GRIPPER_NAME = "gripper"


def _normalize_mode(raw: object) -> Optional[str]:
    text = str(raw or "").strip().lower()
    if text in {"teleop", "human", "master", "leader"}:
        return "teleop"
    if text in {"policy", "vla", "replay", "auto", "robot"}:
        return "policy"
    return None


def _stamp_sec(msg: JointState) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _copy_joint_state(msg: JointState) -> JointState:
    out = JointState()
    out.header = msg.header
    out.name = list(msg.name)
    out.position = list(msg.position)
    out.velocity = list(msg.velocity)
    out.effort = list(msg.effort)
    return out


def _ensure_field_len(values, length: int, default: float) -> list:
    out = list(values)
    if len(out) < length:
        out.extend([default] * (length - len(out)))
    return out


class JointCmdMuxNode(Node):
    def __init__(self):
        super().__init__("joint_cmd_mux_node")
        self.declare_parameter("default_source", "teleop")
        self.declare_parameter("teleop_joint_topic", "")
        self.declare_parameter("teleop_gripper_topic", "")
        self.declare_parameter("policy_topic", "vla_joint_cmd")
        self.declare_parameter("output_topic", "joint_cmd_mux/output")
        self.declare_parameter("select_topic", "joint_cmd_mux/select")
        self.declare_parameter("selected_topic", "joint_cmd_mux/selected")
        self.declare_parameter("global_selected_topic", "/mux/selected")
        self.declare_parameter("max_input_staleness_sec", 0.5)
        self.declare_parameter("debug", False)

        default_source = _normalize_mode(self.get_parameter("default_source").value)
        if default_source is None:
            raise ValueError("default_source must be teleop or policy")
        self.selected_source = default_source
        self.max_input_staleness_sec = max(
            0.0, float(self.get_parameter("max_input_staleness_sec").value)
        )
        self.debug = bool(self.get_parameter("debug").value)

        self.teleop_joint_topic = str(self.get_parameter("teleop_joint_topic").value).strip()
        self.teleop_gripper_topic = str(self.get_parameter("teleop_gripper_topic").value).strip()
        self.policy_topic = str(self.get_parameter("policy_topic").value).strip()
        self.output_topic = str(self.get_parameter("output_topic").value).strip()
        self.select_topic = str(self.get_parameter("select_topic").value).strip()
        self.selected_topic = str(self.get_parameter("selected_topic").value).strip()
        self.global_selected_topic = str(self.get_parameter("global_selected_topic").value).strip()

        self.latest_teleop_joint: Optional[JointState] = None
        self.latest_teleop_gripper: Optional[JointState] = None
        self.latest_policy: Optional[JointState] = None
        self.last_publish_by_source: Dict[str, float] = {"teleop": 0.0, "policy": 0.0}

        self.output_pub = self.create_publisher(JointState, self.output_topic, HOT_QOS)
        self.selected_pub = self.create_publisher(String, self.selected_topic, LATCHED_QOS)
        self.global_selected_pub = self.create_publisher(String, self.global_selected_topic, LATCHED_QOS)

        self.create_subscription(JointState, self.teleop_joint_topic, self._on_teleop_joint, HOT_QOS)
        self.create_subscription(JointState, self.teleop_gripper_topic, self._on_teleop_gripper, HOT_QOS)
        self.create_subscription(JointState, self.policy_topic, self._on_policy, HOT_QOS)
        self.create_subscription(String, self.select_topic, self._on_select, LATCHED_QOS)

        self._publish_selected(force=True)
        self.get_logger().info(
            "Joint cmd mux ready: "
            f"default={self.selected_source} teleop_joint={self.teleop_joint_topic} "
            f"teleop_gripper={self.teleop_gripper_topic} policy={self.policy_topic} "
            f"output={self.output_topic} select={self.select_topic}"
        )

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _is_fresh(self, msg: Optional[JointState]) -> bool:
        if msg is None:
            return False
        if self.max_input_staleness_sec <= 0.0:
            return True
        stamp = _stamp_sec(msg)
        if stamp <= 0.0:
            return True
        return (self._now_sec() - stamp) <= self.max_input_staleness_sec

    def _publish_selected(self, *, force: bool = False) -> None:
        del force
        selected_text = self._selected_topic_text()
        msg = String(data=selected_text)
        self.selected_pub.publish(msg)
        self.global_selected_pub.publish(msg)

    def _selected_topic_text(self) -> str:
        if self.selected_source == "teleop":
            return self.teleop_joint_topic
        return self.policy_topic

    def _on_select(self, msg: String) -> None:
        mode = _normalize_mode(msg.data)
        if mode is None:
            data = str(msg.data or "").strip()
            if data == self.teleop_joint_topic:
                mode = "teleop"
            elif data == self.policy_topic:
                mode = "policy"
        if mode is None:
            self.get_logger().warn(
                f"Ignoring unknown mux select command {msg.data!r}; expected teleop or policy.",
                throttle_duration_sec=1.0,
            )
            return
        if mode != self.selected_source:
            self.get_logger().info(
                f"Switch selected source: {self.selected_source} -> {mode} ({self._selected_topic_text()})"
            )
            self.selected_source = mode
        self._publish_selected(force=True)
        self._publish_latest_selected(reason="select")

    def _on_teleop_joint(self, msg: JointState) -> None:
        self.latest_teleop_joint = _copy_joint_state(msg)
        if self.selected_source == "teleop":
            self._publish_latest_selected(reason="teleop_joint")

    def _on_teleop_gripper(self, msg: JointState) -> None:
        self.latest_teleop_gripper = _copy_joint_state(msg)
        if self.selected_source == "teleop":
            self._publish_latest_selected(reason="teleop_gripper")

    def _on_policy(self, msg: JointState) -> None:
        self.latest_policy = _copy_joint_state(msg)
        if self.selected_source == "policy":
            self._publish_latest_selected(reason="policy")

    def _publish_latest_selected(self, *, reason: str) -> None:
        if self.selected_source == "teleop":
            msg = self._build_teleop_command()
        else:
            msg = self.latest_policy if self._is_fresh(self.latest_policy) else None
        if msg is None:
            log = self.get_logger().debug if reason == "select" else self.get_logger().warn
            log(
                f"Selected source {self.selected_source!r} has no fresh command yet; reason={reason}",
                throttle_duration_sec=1.0,
            )
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.output_pub.publish(msg)
        self.last_publish_by_source[self.selected_source] = self._now_sec()
        if self.debug:
            self.get_logger().info(
                f"published source={self.selected_source} reason={reason} output={self.output_topic}",
                throttle_duration_sec=1.0,
            )

    def _build_teleop_command(self) -> Optional[JointState]:
        if not self._is_fresh(self.latest_teleop_joint):
            return None
        assert self.latest_teleop_joint is not None
        out = _copy_joint_state(self.latest_teleop_joint)
        if not out.name:
            out.name = list(JOINT_NAMES[: len(out.position)])
        self._merge_gripper_intent(out)
        return out

    def _merge_gripper_intent(self, out: JointState) -> None:
        gripper_msg = self.latest_teleop_gripper
        if not self._is_fresh(gripper_msg):
            return
        assert gripper_msg is not None
        gripper = self._extract_gripper(gripper_msg)
        if gripper is None:
            return
        pos, vel, effort = gripper
        if GRIPPER_NAME in out.name:
            idx = out.name.index(GRIPPER_NAME)
        else:
            out.name.append(GRIPPER_NAME)
            idx = len(out.name) - 1
        out.position = _ensure_field_len(out.position, idx + 1, 0.0)
        out.velocity = _ensure_field_len(out.velocity, idx + 1, 0.0)
        out.effort = _ensure_field_len(out.effort, idx + 1, 0.0)
        out.position[idx] = float(pos)
        out.velocity[idx] = float(vel)
        out.effort[idx] = float(effort)

    @staticmethod
    def _extract_gripper(msg: JointState) -> Optional[Tuple[float, float, float]]:
        if GRIPPER_NAME in msg.name:
            idx = msg.name.index(GRIPPER_NAME)
        elif len(msg.position) == 1:
            idx = 0
        elif len(msg.position) >= len(JOINT_NAMES):
            idx = len(JOINT_NAMES) - 1
        else:
            return None
        if idx >= len(msg.position):
            return None
        pos = float(msg.position[idx])
        vel = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
        effort = float(msg.effort[idx]) if idx < len(msg.effort) else 0.0
        return pos, vel, effort


def main() -> None:
    rclpy.init()
    node = JointCmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
