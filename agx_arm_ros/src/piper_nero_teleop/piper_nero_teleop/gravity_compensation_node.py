#!/usr/bin/env python3
import copy
import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - exercised at runtime on target machine
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None

_SHUTDOWN_CONVERSION_ERROR_MARKER = "Unable to convert call argument"


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


def _tag_local_name(tag: str) -> str:
    return tag.split("}", 1)[-1]


class GravityCompensationNode(Node):
    def __init__(self):
        super().__init__("gravity_compensation_node")

        self.declare_parameter("joint_names", [f"joint{i}" for i in range(1, 8)])
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("model_package", "agx_arm_urdf")
        self.declare_parameter("model_relpath", "nero/urdf/nero_with_gripper_description.xacro")
        self.declare_parameter("clip_to_limits", True)
        self.declare_parameter("limit_warn_tol", 1e-4)
        self.declare_parameter("limit_push_warn_margin", 0.15)
        self.declare_parameter("unwrap_joint_positions", True)
        self.declare_parameter("max_joint_step", 0.0)
        self.declare_parameter("max_torque_delta_warn", 4.0)
        self.declare_parameter("gravity_joint_scale", [1.0] * 7)
        self.declare_parameter("gravity_joint_sign", [1.0] * 7)
        self.declare_parameter("gravity_joint_position_scale", [1.0] * 7)
        self.declare_parameter("gravity_joint_offset", [0.0] * 7)

        self.arm_joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.arm_joint_count = len(self.arm_joint_names)
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.model_package = str(self.get_parameter("model_package").value)
        self.model_relpath = str(self.get_parameter("model_relpath").value)

        self._refresh_parameters()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._expanded_model_path = self._expand_xacro_model()
        self._load_model()

        self.compat_torque_pub = self.create_publisher(Float64MultiArray, "gravity/torque", 10)
        self.compensated_pub = self.create_publisher(
            JointState,
            "feedback/joint_states_compensated",
            10,
        )
        self.create_subscription(JointState, "feedback/joint_states", self._joint_state_callback, 10)

    def destroy_node(self):
        try:
            if hasattr(self, "_expanded_model_path") and self._expanded_model_path is not None:
                self._expanded_model_path.unlink(missing_ok=True)
        finally:
            return super().destroy_node()

    def _refresh_parameters(self):
        self.clip_to_limits = bool(self.get_parameter("clip_to_limits").value)
        self.limit_warn_tol = float(self.get_parameter("limit_warn_tol").value)
        self.limit_push_warn_margin = float(self.get_parameter("limit_push_warn_margin").value)
        self.unwrap_joint_positions = bool(self.get_parameter("unwrap_joint_positions").value)
        self.max_joint_step = max(0.0, float(self.get_parameter("max_joint_step").value))
        self.max_torque_delta_warn = max(
            0.0, float(self.get_parameter("max_torque_delta_warn").value)
        )
        self.gravity_joint_scale = _normalize_vector(
            self.get_parameter("gravity_joint_scale").value,
            self.arm_joint_count,
            1.0,
        )
        self.gravity_joint_sign = _normalize_vector(
            self.get_parameter("gravity_joint_sign").value,
            self.arm_joint_count,
            1.0,
        )
        self.gravity_joint_position_scale = _normalize_vector(
            self.get_parameter("gravity_joint_position_scale").value,
            self.arm_joint_count,
            1.0,
        )
        self.gravity_joint_offset = _normalize_vector(
            self.get_parameter("gravity_joint_offset").value,
            self.arm_joint_count,
            0.0,
        )
        self.last_model_joint_positions: Optional[List[float]] = None
        self.last_compensated_torques: Optional[List[float]] = None

    def _on_set_parameters(self, parameters):
        staged = {parameter.name: parameter.value for parameter in parameters}
        try:
            gravity_joint_scale = _normalize_vector(
                staged.get("gravity_joint_scale", self.gravity_joint_scale),
                self.arm_joint_count,
                1.0,
            )
            gravity_joint_sign = _normalize_vector(
                staged.get("gravity_joint_sign", self.gravity_joint_sign),
                self.arm_joint_count,
                1.0,
            )
            gravity_joint_position_scale = _normalize_vector(
                staged.get("gravity_joint_position_scale", self.gravity_joint_position_scale),
                self.arm_joint_count,
                1.0,
            )
            gravity_joint_offset = _normalize_vector(
                staged.get("gravity_joint_offset", self.gravity_joint_offset),
                self.arm_joint_count,
                0.0,
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
        if "model_package" in staged or "model_relpath" in staged:
            return SetParametersResult(
                successful=False,
                reason="model_package/model_relpath cannot be changed at runtime",
            )

        self.clip_to_limits = bool(staged.get("clip_to_limits", self.clip_to_limits))
        self.limit_warn_tol = float(staged.get("limit_warn_tol", self.limit_warn_tol))
        self.limit_push_warn_margin = float(
            staged.get("limit_push_warn_margin", self.limit_push_warn_margin)
        )
        self.unwrap_joint_positions = bool(
            staged.get("unwrap_joint_positions", self.unwrap_joint_positions)
        )
        self.max_joint_step = max(0.0, float(staged.get("max_joint_step", self.max_joint_step)))
        self.max_torque_delta_warn = max(
            0.0,
            float(staged.get("max_torque_delta_warn", self.max_torque_delta_warn)),
        )
        self.gravity_joint_scale = gravity_joint_scale
        self.gravity_joint_sign = gravity_joint_sign
        self.gravity_joint_position_scale = gravity_joint_position_scale
        self.gravity_joint_offset = gravity_joint_offset
        return SetParametersResult(successful=True)

    def _expand_xacro_model(self) -> Path:
        model_root = Path(get_package_share_directory(self.model_package))
        model_path = model_root / self.model_relpath
        if not model_path.exists():
            raise RuntimeError(f"Model file not found: {model_path}")

        expanded_root = self._expand_robot_xml(model_path)
        expanded_text = ET.tostring(expanded_root, encoding="unicode")
        temp_file = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            prefix="nero_pinocchio_",
            delete=False,
            encoding="utf-8",
        )
        temp_file.write(expanded_text)
        temp_file.close()
        self.get_logger().info(f"Pinocchio model source: {model_path}")
        self.get_logger().info(f"Expanded URDF written to: {temp_file.name}")
        return Path(temp_file.name)

    def _expand_robot_xml(self, file_path: Path) -> ET.Element:
        root = ET.parse(file_path).getroot()
        expanded_root = copy.deepcopy(root)
        expanded_root[:] = []

        for child in list(root):
            if _tag_local_name(child.tag) == "include":
                include_filename = child.attrib.get("filename")
                if not include_filename:
                    raise RuntimeError(f"Missing filename on include in {file_path}")
                include_path = self._resolve_include_path(include_filename, file_path.parent)
                included_root = self._expand_robot_xml(include_path)
                for included_child in list(included_root):
                    expanded_root.append(copy.deepcopy(included_child))
                continue
            expanded_root.append(copy.deepcopy(child))
        return expanded_root

    def _resolve_include_path(self, filename: str, base_dir: Path) -> Path:
        match = re.match(r"\$\(find\s+([^)]+)\)(/.*)?", filename)
        if match:
            package_name = match.group(1)
            relative = (match.group(2) or "").lstrip("/")
            return Path(get_package_share_directory(package_name)) / relative
        candidate = Path(filename)
        if candidate.is_absolute():
            return candidate
        return (base_dir / candidate).resolve()

    def _load_model(self):
        if pin is None:
            raise RuntimeError(
                "Pinocchio is required for gravity compensation. "
                "Install it with: sudo apt install ros-jazzy-pinocchio"
            ) from _PINOCCHIO_IMPORT_ERROR

        self.model = pin.buildModelFromUrdf(str(self._expanded_model_path))
        self.data = self.model.createData()
        self.lower_limits = list(self.model.lowerPositionLimit)
        self.upper_limits = list(self.model.upperPositionLimit)
        self.q_template = pin.neutral(self.model)

        self.arm_joint_index_map: Dict[str, Tuple[int, int]] = {}
        for joint_name in self.arm_joint_names:
            self.arm_joint_index_map[joint_name] = self._lookup_joint_indexes(joint_name)

        self.gripper_joint1_indexes = self._lookup_joint_indexes("gripper_joint1")
        self.gripper_joint2_indexes = self._lookup_joint_indexes("gripper_joint2")

        self.get_logger().info(
            f"Pinocchio model nq={self.model.nq}, nv={self.model.nv}. "
            "ROS teleop interface remains 8D (7 arm joints + gripper)."
        )

    def _lookup_joint_indexes(self, joint_name: str) -> Tuple[int, int]:
        joint_id = self.model.getJointId(joint_name)
        if joint_id == 0:
            raise RuntimeError(f"Joint '{joint_name}' not found in Pinocchio model")
        joint = self.model.joints[joint_id]
        if joint.nq != 1 or joint.nv != 1:
            raise RuntimeError(
                f"Joint '{joint_name}' must be 1-DOF, got nq={joint.nq}, nv={joint.nv}"
            )
        return int(joint.idx_q), int(joint.idx_v)

    def _extract_arm_positions(self, msg: JointState) -> Optional[List[float]]:
        positions = []
        for joint_name in self.arm_joint_names:
            if joint_name not in msg.name:
                return None
            index = msg.name.index(joint_name)
            if index >= len(msg.position):
                return None
            positions.append(float(msg.position[index]))
        return positions

    def _extract_gripper_width(self, msg: JointState) -> float:
        def _sanitize(value: float) -> float:
            if value < 0.0:
                return 0.0
            if value > 0.100001:
                self.get_logger().warn(
                    f"Received gripper position {value:.6f} > 0.1 m on joint_states; "
                    "treating it as non-width telemetry and ignoring it in gravity model.",
                    throttle_duration_sec=1.0,
                )
                return 0.0
            return value

        if self.gripper_joint_name in msg.name:
            index = msg.name.index(self.gripper_joint_name)
            if index < len(msg.position):
                return _sanitize(abs(float(msg.position[index])))

        if "gripper_joint1" in msg.name:
            index = msg.name.index("gripper_joint1")
            if index < len(msg.position):
                return _sanitize(abs(float(msg.position[index]) * 2.0))

        if "gripper_joint2" in msg.name:
            index = msg.name.index("gripper_joint2")
            if index < len(msg.position):
                return _sanitize(abs(float(msg.position[index]) * 2.0))
        return 0.0

    def _map_arm_positions(self, arm_positions: List[float]) -> List[float]:
        mapped = []
        for index, position in enumerate(arm_positions):
            mapped.append(
                self.gravity_joint_sign[index]
                * (self.gravity_joint_position_scale[index] * position)
                + self.gravity_joint_offset[index]
            )
        return mapped

    def _unwrap_positions(self, positions: List[float]) -> List[float]:
        if not self.unwrap_joint_positions or self.last_model_joint_positions is None:
            return positions

        unwrapped = []
        for current_value, target_value in zip(self.last_model_joint_positions, positions):
            candidate = target_value
            while candidate - current_value > 3.141592653589793:
                candidate -= 6.283185307179586
            while candidate - current_value < -3.141592653589793:
                candidate += 6.283185307179586
            unwrapped.append(candidate)
        return unwrapped

    def _limit_step(self, positions: List[float]) -> List[float]:
        if self.max_joint_step <= 0.0 or self.last_model_joint_positions is None:
            return positions

        limited = []
        for current_value, target_value in zip(self.last_model_joint_positions, positions):
            delta = target_value - current_value
            delta = _clamp(delta, -self.max_joint_step, self.max_joint_step)
            limited.append(current_value + delta)
        return limited

    def _clip_configuration(self, q: np.ndarray) -> Tuple[np.ndarray, List[int]]:
        if not self.clip_to_limits:
            return q, []

        clipped = np.array(q, dtype=float, copy=True)
        clipped_indexes = []
        for index, value in enumerate(clipped):
            bounded = _clamp(value, self.lower_limits[index], self.upper_limits[index])
            if abs(bounded - value) > self.limit_warn_tol:
                clipped_indexes.append(index)
            clipped[index] = bounded
        return clipped, clipped_indexes

    def _joint_positions_message(self, msg: JointState) -> Tuple[List[float], List[float], float]:
        arm_positions = self._extract_arm_positions(msg)
        if arm_positions is None:
            raise ValueError("Missing arm joints in joint_states feedback")
        gripper_width = self._extract_gripper_width(msg)

        mapped_positions = self._map_arm_positions(arm_positions)
        mapped_positions = self._unwrap_positions(mapped_positions)
        mapped_positions = self._limit_step(mapped_positions)
        return arm_positions, mapped_positions, gripper_width

    def _joint_state_callback(self, msg: JointState):
        try:
            raw_arm_positions, model_arm_positions, gripper_width = self._joint_positions_message(msg)
        except ValueError:
            return

        # Pinocchio expects an Eigen-compatible vector here, not a Python list.
        q = np.array(self.q_template, dtype=float, copy=True)
        for index, joint_name in enumerate(self.arm_joint_names):
            idx_q, _ = self.arm_joint_index_map[joint_name]
            q[idx_q] = model_arm_positions[index]

        gripper_half_width = max(0.0, min(0.05, gripper_width * 0.5))
        q[self.gripper_joint1_indexes[0]] = gripper_half_width
        q[self.gripper_joint2_indexes[0]] = -gripper_half_width

        q, clipped_indexes = self._clip_configuration(q)
        if clipped_indexes:
            clipped_arm = []
            for arm_index, joint_name in enumerate(self.arm_joint_names):
                idx_q, _ = self.arm_joint_index_map[joint_name]
                if idx_q in clipped_indexes:
                    clipped_arm.append(str(arm_index + 1))
            if clipped_arm:
                self.get_logger().warn(
                    "Clipped joint positions to Pinocchio limits on joints "
                    + ", ".join(clipped_arm),
                    throttle_duration_sec=1.0,
                )

        gravity_torques = pin.computeGeneralizedGravity(self.model, self.data, q)
        used_arm_positions = [
            float(q[self.arm_joint_index_map[joint_name][0]]) for joint_name in self.arm_joint_names
        ]

        compensated_torques = []
        for index, joint_name in enumerate(self.arm_joint_names):
            _, idx_v = self.arm_joint_index_map[joint_name]
            compensated_torques.append(
                float(gravity_torques[idx_v]) * self.gravity_joint_scale[index]
            )

        if self.limit_push_warn_margin > 0.0:
            for index, joint_name in enumerate(self.arm_joint_names):
                idx_q, _ = self.arm_joint_index_map[joint_name]
                lower = float(self.lower_limits[idx_q])
                upper = float(self.upper_limits[idx_q])
                q_i = float(q[idx_q])
                tau_i = float(compensated_torques[index])
                near_upper = q_i > (upper - self.limit_push_warn_margin)
                near_lower = q_i < (lower + self.limit_push_warn_margin)
                if (near_upper and tau_i > 0.0) or (near_lower and tau_i < 0.0):
                    target = "upper" if near_upper else "lower"
                    self.get_logger().warn(
                        f"Gravity torque pushes toward {target} limit on joint{index + 1}: "
                        f"q={q_i:.3f} lim=[{lower:.3f}, {upper:.3f}] tau={tau_i:.3f}",
                        throttle_duration_sec=0.5,
                    )

        if self.max_torque_delta_warn > 0.0 and self.last_compensated_torques is not None:
            deltas = [
                current - previous
                for current, previous in zip(compensated_torques, self.last_compensated_torques)
            ]
            max_index = max(range(self.arm_joint_count), key=lambda idx: abs(deltas[idx]))
            if abs(deltas[max_index]) > self.max_torque_delta_warn:
                self.get_logger().warn(
                        f"Torque jump on joint{max_index + 1}: d_tau={deltas[max_index]:.3f} "
                        f"prev={self.last_compensated_torques[max_index]:.3f} "
                        f"now={compensated_torques[max_index]:.3f} "
                        f"raw_q={raw_arm_positions[max_index]:.3f} "
                        f"mapped_q={model_arm_positions[max_index]:.3f} "
                        f"used_q={used_arm_positions[max_index]:.3f}",
                        throttle_duration_sec=0.5,
                    )

        self.last_model_joint_positions = list(used_arm_positions)
        self.last_compensated_torques = list(compensated_torques)

        output_msg = JointState()
        output_msg.header = msg.header
        output_msg.name = list(self.arm_joint_names) + [self.gripper_joint_name]
        output_msg.position = list(raw_arm_positions) + [gripper_width]
        output_msg.velocity = [0.0] * (self.arm_joint_count + 1)
        output_msg.effort = list(compensated_torques) + [0.0]

        for index, joint_name in enumerate(self.arm_joint_names):
            if joint_name in msg.name:
                source_index = msg.name.index(joint_name)
                if source_index < len(msg.velocity):
                    output_msg.velocity[index] = float(msg.velocity[source_index])
        if self.gripper_joint_name in msg.name:
            source_index = msg.name.index(self.gripper_joint_name)
            if source_index < len(msg.velocity):
                output_msg.velocity[-1] = float(msg.velocity[source_index])

        self.compat_torque_pub.publish(Float64MultiArray(data=list(compensated_torques)))
        self.compensated_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if _SHUTDOWN_CONVERSION_ERROR_MARKER not in str(exc):
            raise
        node.get_logger().warn(
            "Ignoring shutdown-time JointState conversion error while exiting"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
