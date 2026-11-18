#!/usr/bin/env python3
import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

try:
    import serial
    import serial.tools.list_ports
except ModuleNotFoundError:
    serial = None


SENSOR_DISTRIBUTED_LENGTH = {
    "S1813_elite": 93,
    "S2015_elite": 156,
    "S1813_core": 153,
    "S2716_core": 348,
    "S3013_core": 288,
    "M2826_omega": 381,
    "L3530_omega": 405,
    "S1610_elite": 75,
    "M2324_core": 204,
    "M3025_core": 231,
    "L5325_omega": 717,
    "M2020_elite": 27,
}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def calculate_lrc(data: bytes) -> int:
    return ((~(sum(data) & 0xFF)) + 1) & 0xFF


def signed_u8(value: int) -> int:
    return int(value) if value <= 127 else int(value) - 256


@dataclass
class SensorConfig:
    name: str
    enabled: bool
    robot_ns: str
    finger: str
    port: str
    serial_number: str
    model: str
    point_count: int
    device_addr: int
    normal_axis: str
    normal_sign: float
    deadband_n: float
    scale_n_per_count: float
    data_start: int

    @property
    def distributed_len(self) -> int:
        return int(self.point_count) * 3


class TactileSerialReader:
    def __init__(self, config: SensorConfig, baudrate: int, timeout_sec: float):
        if serial is None:
            raise RuntimeError("pyserial is required for tactile sensor serial access")
        self.config = config
        self.baudrate = int(baudrate)
        self.timeout_sec = float(timeout_sec)
        self.ser: Optional[serial.Serial] = None

    def open(self):
        port = self.config.port or self._find_port_by_serial(self.config.serial_number)
        if not port:
            raise RuntimeError(
                f"{self.config.name}: no port configured and serial_number was not found"
            )
        self.ser = serial.Serial(
            port=port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_sec,
            write_timeout=self.timeout_sec,
            inter_byte_timeout=0.001,
            xonxoff=False,
            rtscts=False,
        )
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def _find_port_by_serial(self, serial_number: str) -> str:
        target = str(serial_number or "").strip()
        if not target:
            return ""
        for port in serial.tools.list_ports.comports():
            if str(getattr(port, "serial_number", "") or "") == target:
                return str(port.device)
        return ""

    def read_points(self) -> List[Tuple[float, float, float]]:
        if self.ser is None or not self.ser.is_open:
            self.open()
        assert self.ser is not None

        request = self._distributed_request_frame()
        self.ser.reset_input_buffer()
        self.ser.write(request)
        response = self._read_response(self.config.data_start + self.config.distributed_len + 1)
        payload = self._extract_payload(response, self.config.distributed_len)
        return self._parse_points(payload)

    def calibrate(self):
        if self.ser is None or not self.ser.is_open:
            self.open()
        assert self.ser is not None
        request = self._calibration_request_frame()
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self._read_response(10)

    def _frame(self, payload_without_lrc: bytes) -> bytes:
        return payload_without_lrc + bytes([calculate_lrc(payload_without_lrc)])

    def _distributed_request_frame(self) -> bytes:
        length = self.config.distributed_len
        base = bytes(
            [
                0x55,
                0xAA,
                0x09,
                0x00,
                self.config.device_addr & 0xFF,
                0x00,
                0xFB,
                0x0E,
                0x04,
                0x00,
                0x00,
                length & 0xFF,
                (length >> 8) & 0xFF,
            ]
        )
        return self._frame(base)

    def _calibration_request_frame(self) -> bytes:
        base = bytes(
            [
                0x55,
                0xAA,
                0x0A,
                0x00,
                self.config.device_addr & 0xFF,
                0x00,
                0x79,
                0x03,
                0x00,
                0x00,
                0x00,
                0x01,
                0x00,
                0x01,
            ]
        )
        return self._frame(base)

    def _read_response(self, min_len: int) -> bytes:
        assert self.ser is not None
        data = b""
        deadline = time.monotonic() + max(0.02, self.timeout_sec)
        last_rx = time.monotonic()
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            if waiting > 0:
                data += self.ser.read(waiting)
                last_rx = time.monotonic()
                if len(data) >= min_len:
                    break
            elif data and time.monotonic() - last_rx > 0.01:
                break
            time.sleep(0.0005)
        if not data:
            raise RuntimeError("empty serial response")
        return data

    def _extract_payload(self, response: bytes, expected_len: int) -> bytes:
        if len(response) < self.config.data_start + expected_len:
            raise RuntimeError(
                f"incomplete response: got {len(response)} bytes, need "
                f"{self.config.data_start + expected_len}"
            )
        if response[:2] != b"\xAA\x55":
            raise RuntimeError(f"bad response header: {response[:2].hex()}")
        if len(response) >= 5:
            body_len = int.from_bytes(response[2:4], byteorder="little")
            lrc_pos = 4 + body_len
            if lrc_pos >= len(response):
                raise RuntimeError(
                    f"incomplete response LRC: lrc_pos={lrc_pos}, len={len(response)}"
                )
            actual_lrc = response[lrc_pos]
            expected_lrc = calculate_lrc(response[:lrc_pos])
            if actual_lrc != expected_lrc:
                raise RuntimeError(
                    f"bad response LRC: expected {expected_lrc:02X}, got {actual_lrc:02X}"
                )
        return response[self.config.data_start : self.config.data_start + expected_len]

    def _parse_points(self, payload: bytes) -> List[Tuple[float, float, float]]:
        points = []
        scale = float(self.config.scale_n_per_count)
        for index in range(0, len(payload), 3):
            chunk = payload[index : index + 3]
            if len(chunk) < 3:
                break
            points.append(
                (
                    signed_u8(chunk[0]) * scale,
                    signed_u8(chunk[1]) * scale,
                    int(chunk[2]) * scale,
                )
            )
        return points


class TactileSensorNode(Node):
    def __init__(self):
        super().__init__("tactile_sensor_node")
        self.declare_parameter("sensors_json", "[]")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("serial_timeout_sec", 0.04)
        self.declare_parameter("calibrate_on_start", False)
        self.declare_parameter("contact_lowpass_alpha", 0.35)
        self.declare_parameter("reopen_backoff_sec", 1.0)
        self.declare_parameter("publish_raw_points", False)

        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout_sec = max(0.005, float(self.get_parameter("serial_timeout_sec").value))
        self.calibrate_on_start = bool(self.get_parameter("calibrate_on_start").value)
        self.contact_lowpass_alpha = _clamp(
            float(self.get_parameter("contact_lowpass_alpha").value), 0.0, 1.0
        )
        self.reopen_backoff_sec = max(0.1, float(self.get_parameter("reopen_backoff_sec").value))
        self.publish_raw_points = bool(self.get_parameter("publish_raw_points").value)

        self.sensor_configs = self._load_sensor_configs()
        self.readers: Dict[str, TactileSerialReader] = {}
        self.joint_publishers = {}
        self.finger_contact_publishers = {}
        self.gripper_force_publishers = {}
        self.latest_contact_by_sensor: Dict[str, float] = {}
        self.filtered_contact_by_sensor: Dict[str, float] = {}
        self.last_error_log_at: Dict[str, float] = {}
        self.last_open_attempt_at: Dict[str, float] = {}

        for config in self.sensor_configs:
            if not config.enabled:
                continue
            self.readers[config.name] = TactileSerialReader(
                config, self.baudrate, self.serial_timeout_sec
            )
            prefix = f"/{config.robot_ns.strip('/')}/tactile/{config.finger}"
            if self.publish_raw_points:
                self.joint_publishers[config.name] = self.create_publisher(
                    JointState, f"{prefix}/joint_states", 10
                )
            self.finger_contact_publishers[config.name] = self.create_publisher(
                Float64, f"{prefix}/contact_force", 10
            )
            robot_ns = config.robot_ns.strip("/")
            if robot_ns not in self.gripper_force_publishers:
                self.gripper_force_publishers[robot_ns] = self.create_publisher(
                    Float64, f"/{robot_ns}/tactile/gripper_force", 10
                )

        self._open_enabled_readers()
        self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.get_logger().info(
            f"Tactile sensor node ready: enabled={len(self.readers)} "
            f"publish_rate={self.publish_rate_hz:.1f}Hz"
        )

    def _load_sensor_configs(self) -> List[SensorConfig]:
        raw = str(self.get_parameter("sensors_json").value or "").strip()
        try:
            payload = json.loads(raw) if raw else []
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"sensors_json is not valid JSON: {exc}") from exc
        if not isinstance(payload, list):
            raise RuntimeError("sensors_json must be a JSON list")

        configs = []
        for index, item in enumerate(payload):
            if not isinstance(item, dict):
                raise RuntimeError(f"sensors_json[{index}] must be an object")
            model = str(item.get("model", "M2020_elite"))
            point_count = int(item.get("point_count", SENSOR_DISTRIBUTED_LENGTH.get(model, 27) // 3))
            configs.append(
                SensorConfig(
                    name=str(item.get("name") or f"sensor_{index}"),
                    enabled=bool(item.get("enabled", False)),
                    robot_ns=str(item.get("robot_ns", "slave_right")).strip("/") or "slave_right",
                    finger=str(item.get("finger", f"finger_{index}")).strip("/") or f"finger_{index}",
                    port=str(item.get("port", "")),
                    serial_number=str(item.get("serial", item.get("serial_number", ""))),
                    model=model,
                    point_count=point_count,
                    device_addr=int(item.get("device_addr", 1)),
                    normal_axis=str(item.get("normal_axis", "z")).lower(),
                    normal_sign=float(item.get("normal_sign", 1.0)),
                    deadband_n=max(0.0, float(item.get("deadband_n", 0.0))),
                    scale_n_per_count=float(item.get("scale_n_per_count", item.get("scale", 0.1))),
                    data_start=int(item.get("data_start", 14)),
                )
            )
        return configs

    def _open_enabled_readers(self):
        for name, reader in self.readers.items():
            try:
                reader.open()
                if self.calibrate_on_start:
                    reader.calibrate()
                self.get_logger().info(
                    f"{name}: opened tactile sensor on {reader.ser.port if reader.ser else 'unknown'}"
                )
            except Exception as exc:
                self.get_logger().warn(f"{name}: initial open failed: {exc}")

    def _timer_callback(self):
        now = self.get_clock().now().to_msg()
        active_robot_names = set()
        for config in self.sensor_configs:
            if not config.enabled or config.name not in self.readers:
                continue
            active_robot_names.add(config.robot_ns.strip("/"))
            reader = self.readers[config.name]
            if not self._reader_ready_to_try(config, reader):
                self.filtered_contact_by_sensor[config.name] = 0.0
                continue
            try:
                points = reader.read_points()
            except Exception as exc:
                self._handle_read_error(config, exc)
                continue

            contact_force = self._compute_contact_force(config, points)
            filtered = self._filter_contact_force(config.name, contact_force)
            self.latest_contact_by_sensor[config.name] = contact_force
            if self.publish_raw_points:
                values = [value for point in points for value in point]
                self._publish_joint_state(config, now, values)
            self._publish_float(self.finger_contact_publishers[config.name], filtered)

        for robot_ns in active_robot_names:
            total = 0.0
            for config in self.sensor_configs:
                if config.enabled and config.robot_ns.strip("/") == robot_ns:
                    total += self.filtered_contact_by_sensor.get(config.name, 0.0)
            self._publish_float(self.gripper_force_publishers[robot_ns], total)

    def _reader_ready_to_try(
        self, config: SensorConfig, reader: TactileSerialReader
    ) -> bool:
        if reader.ser is not None and reader.ser.is_open:
            return True
        now = time.monotonic()
        last_attempt = self.last_open_attempt_at.get(config.name, 0.0)
        if last_attempt > 0.0 and now - last_attempt < self.reopen_backoff_sec:
            return False
        self.last_open_attempt_at[config.name] = now
        return True

    def _handle_read_error(self, config: SensorConfig, exc: Exception):
        now = time.monotonic()
        if now - self.last_error_log_at.get(config.name, 0.0) > 1.0:
            self.get_logger().warn(f"{config.name}: tactile read failed: {exc}")
            self.last_error_log_at[config.name] = now
        self.filtered_contact_by_sensor[config.name] = 0.0
        reader = self.readers.get(config.name)
        if reader is not None:
            reader.close()
        if now - self.last_open_attempt_at.get(config.name, 0.0) > self.reopen_backoff_sec:
            self.last_open_attempt_at[config.name] = now

    def _publish_joint_state(self, config: SensorConfig, stamp, values: List[float]):
        msg = JointState()
        msg.header.stamp = stamp
        names = []
        for point_index in range(config.point_count):
            names.extend(
                [
                    f"p{point_index:02d}_fx",
                    f"p{point_index:02d}_fy",
                    f"p{point_index:02d}_fz",
                ]
            )
        msg.name = names
        msg.effort = list(values[: len(names)])
        self.joint_publishers[config.name].publish(msg)

    def _publish_float(self, publisher, value: float):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def _filter_contact_force(self, sensor_name: str, value: float) -> float:
        if sensor_name not in self.filtered_contact_by_sensor:
            self.filtered_contact_by_sensor[sensor_name] = float(value)
            return float(value)
        alpha = self.contact_lowpass_alpha
        filtered = (
            alpha * float(value)
            + (1.0 - alpha) * self.filtered_contact_by_sensor[sensor_name]
        )
        self.filtered_contact_by_sensor[sensor_name] = filtered
        return filtered

    def _compute_contact_force(
        self, config: SensorConfig, points: List[Tuple[float, float, float]]
    ) -> float:
        axis_index = {"x": 0, "y": 1, "z": 2}.get(config.normal_axis, 2)
        total = 0.0
        for point in points:
            normal = float(point[axis_index]) * float(config.normal_sign)
            total += max(0.0, normal - config.deadband_n)
        return total

    def destroy_node(self):
        for reader in self.readers.values():
            reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
