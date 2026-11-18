#!/usr/bin/env python3
"""Publish local V4L2 cameras as ROS2 Image streams."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import select
import sys
import threading
import time
from pathlib import Path
import re

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from show_connected_cameras import (  # noqa: E402
    CameraDevice,
    _find_device,
    _make_blank_tile,
    _make_tile,
    _open_capture,
    _preferred_devices_for_usb_port,
    _serial_matches,
    choose_devices,
    discover_video_devices,
    import_cv2,
    print_devices,
    probe_device,
    resolve_device_arg,
)

MISSING_ROLE_PREFIX = "__missing__:"
JINHE_ROOT = os.environ.get("APRILTAG_VIDEO_REPO_ROOT")
JINHE_SCRIPTS_DIR = Path(JINHE_ROOT).expanduser() / "scripts" if JINHE_ROOT else None
_JINHE_OPEN_CAMERA = None
_JINHE_CAMERA_CAPTURE = None


class CameraCaptureAdapter:
    def __init__(self, cap, meta: dict[str, object] | None = None) -> None:  # noqa: ANN001
        self.cap = cap
        self.meta = dict(meta or {})

    def isOpened(self) -> bool:
        return bool(getattr(self.cap, "isOpened")())

    def read(self):
        return self.cap.read()

    def release(self) -> None:
        self.cap.release()


def _parse_roles(raw: str, count: int) -> list[str]:
    roles = [item.strip() for item in str(raw or "").split(",") if item.strip()]
    while len(roles) < count:
        roles.append(f"camera_{len(roles) + 1}")
    return roles[:count]


def _requested_roles(raw: str) -> list[str]:
    return [item.strip() for item in str(raw or "").split(",") if item.strip()]


def _topic_token(value: str) -> str:
    token = re.sub(r"[^A-Za-z0-9_]+", "_", str(value).strip().lower()).strip("_")
    return token or "camera"


def _missing_device(role: str) -> CameraDevice:
    token = _topic_token(role)
    return CameraDevice(
        path=Path(f"/missing/{token}"),
        index=-1,
        name=f"{role} camera missing",
        serial="missing",
        usb_serial="missing",
        realsense_serial="missing",
        usb_port="missing",
        usb_iface="missing",
        by_id=(),
        by_path=(),
    )


def _parse_role_bindings(raw_items: list[str], option: str) -> dict[str, str]:
    bindings: dict[str, str] = {}
    for raw in raw_items:
        for item in str(raw or "").split(","):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                raise SystemExit(f"{option} entries must use role=value")
            role, value = item.split("=", 1)
            role = _topic_token(role)
            value = value.strip()
            if not role or not value:
                raise SystemExit(f"{option} entries must use non-empty role=value")
            bindings[role] = value
    return bindings


def _parse_missing_roles(raw_items: list[str]) -> set[str]:
    roles: set[str] = set()
    for raw in raw_items:
        for item in str(raw or "").split(","):
            role = _topic_token(item)
            if role:
                roles.add(role)
    return roles


def _parse_role_capture_sizes(raw_items: list[str]) -> dict[str, tuple[int, int]]:
    role_sizes: dict[str, tuple[int, int]] = {}
    for raw in raw_items:
        for item in str(raw or "").split(","):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                raise SystemExit("--role-capture-size entries must use role=WIDTHxHEIGHT")
            role, size = item.split("=", 1)
            role = _topic_token(role)
            match = re.fullmatch(r"(\d+)x(\d+)", size.strip().lower())
            if match is None:
                raise SystemExit("--role-capture-size entries must use role=WIDTHxHEIGHT")
            width = int(match.group(1))
            height = int(match.group(2))
            if width <= 0 or height <= 0:
                raise SystemExit("--role-capture-size width and height must be positive")
            role_sizes[role] = (width, height)
    return role_sizes


def _parse_role_fourccs(raw_items: list[str]) -> dict[str, str]:
    role_fourccs: dict[str, str] = {}
    for raw in raw_items:
        for item in str(raw or "").split(","):
            item = item.strip()
            if not item:
                continue
            if "=" not in item:
                raise SystemExit("--role-fourcc entries must use role=FOURCC")
            role, fourcc = item.split("=", 1)
            role = _topic_token(role)
            fourcc = fourcc.strip().upper()
            if fourcc in {"", "AUTO", "DEFAULT", "NONE"}:
                continue
            if len(fourcc) != 4:
                raise SystemExit("--role-fourcc FOURCC must be exactly 4 characters, for example MJPG or YUYV.")
            role_fourccs[role] = fourcc
    return role_fourccs


def _load_jinhe_open_camera():
    global _JINHE_OPEN_CAMERA
    if _JINHE_OPEN_CAMERA is not None:
        return _JINHE_OPEN_CAMERA
    _JINHE_OPEN_CAMERA = _load_jinhe_camera_capture().open_camera
    return _JINHE_OPEN_CAMERA


def _load_jinhe_camera_capture():
    global _JINHE_CAMERA_CAPTURE
    if _JINHE_CAMERA_CAPTURE is not None:
        return _JINHE_CAMERA_CAPTURE
    if JINHE_SCRIPTS_DIR is None or not JINHE_SCRIPTS_DIR.is_dir():
        raise RuntimeError(f"ros2_jinhe scripts directory not found: {JINHE_SCRIPTS_DIR}")
    if str(JINHE_SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(JINHE_SCRIPTS_DIR))
    from apriltag_video.internal import camera_capture as jinhe_camera_capture  # type: ignore
    from apriltag_video.internal import camera_realsense as jinhe_camera_realsense  # type: ignore

    for name in ("_select_realsense_device", "_realsense_port_candidates", "_realsense_identifier_values"):
        if hasattr(jinhe_camera_realsense, name):
            setattr(jinhe_camera_capture, name, getattr(jinhe_camera_realsense, name))
    _patch_jinhe_ffmpeg_capture_timeout(jinhe_camera_capture)
    _JINHE_CAMERA_CAPTURE = jinhe_camera_capture
    return _JINHE_CAMERA_CAPTURE


def _ffmpeg_capture_read_timeout_sec() -> float:
    try:
        return max(0.1, float(os.environ.get("WUJI_FFMPEG_CAPTURE_READ_TIMEOUT_SEC", "0.75")))
    except ValueError:
        return 0.75


def _patch_jinhe_ffmpeg_capture_timeout(jinhe_camera_capture) -> None:  # noqa: ANN001
    cls = getattr(jinhe_camera_capture, "FFmpegV4L2MjpegCapture", None)
    if cls is None or getattr(cls, "_wuji_timeout_patch", False):
        return

    def _read_exact_with_timeout(self, size: int) -> bytes | None:  # noqa: ANN001
        if self.process is None or self.process.stdout is None:
            return None
        chunks: list[bytes] = []
        remaining = int(size)
        fd = self.process.stdout.fileno()
        deadline = time.monotonic() + _ffmpeg_capture_read_timeout_sec()
        while remaining > 0:
            timeout = deadline - time.monotonic()
            if timeout <= 0.0:
                return None
            try:
                readable, _, _ = select.select([fd], [], [], timeout)
            except (OSError, ValueError):
                return None
            if not readable:
                return None
            try:
                chunk = os.read(fd, remaining)
            except OSError:
                return None
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    cls._read_exact = _read_exact_with_timeout
    cls._wuji_timeout_patch = True


def _valid_serial(value: object) -> str:
    text = str(value or "").strip()
    if not text or text.lower() == "unknown":
        return ""
    return text


def _serial_binding_value(value: str) -> str:
    text = str(value or "").strip()
    if "@" in text:
        text = text.rsplit("@", 1)[0]
    return _valid_serial(text)


def _realsense_serial(dev: CameraDevice) -> str:
    return _valid_serial(dev.realsense_serial)


def _camera_serial_values(dev: CameraDevice) -> set[str]:
    return {
        value
        for value in (
            _valid_serial(dev.realsense_serial),
            _valid_serial(dev.serial),
            _valid_serial(dev.usb_serial),
        )
        if value
    }


def _is_realsense_device(dev: CameraDevice) -> bool:
    if _realsense_serial(dev):
        return True
    text = " ".join([dev.name, str(dev.path), *dev.by_id, *dev.by_path]).lower()
    return "realsense" in text or "real sense" in text or "intel" in text


def _display_device(dev: CameraDevice) -> str:
    serial = _realsense_serial(dev)
    if serial:
        return f"realsense:{serial}"
    return str(dev.path)


def _jinhe_camera_metadata_from_device(dev: CameraDevice) -> dict[str, object]:
    serial = _realsense_serial(dev)
    path = f"realsense:{serial}" if serial else str(dev.path)
    identifiers = [
        path,
        str(dev.path),
        *[str(value) for value in dev.by_id],
        *[str(value) for value in dev.by_path],
    ]
    for value in (serial, _valid_serial(dev.serial), _valid_serial(dev.usb_serial)):
        if value:
            identifiers.append(f"serial:{value}")
            if serial:
                identifiers.append(f"realsense:{value}")
    deduped: list[str] = []
    seen: set[str] = set()
    for value in identifiers:
        if value and value not in seen:
            seen.add(value)
            deduped.append(value)
    return {
        "path": path,
        "opened_device": path,
        "name": dev.name,
        "score": 1000,
        "by_id": [str(value) for value in dev.by_id],
        "by_path": [str(value) for value in dev.by_path],
        "usb_port": dev.usb_port,
        "usb_iface": dev.usb_iface,
        "usb_serial": _valid_serial(dev.usb_serial),
        "serial": serial or _valid_serial(dev.serial) or _valid_serial(dev.usb_serial),
        "realsense_serial": serial,
        "camera_uid": f"realsense:{serial}" if serial else f"v4l:{dev.path}",
        "identifiers": deduped,
    }


def _open_jinhe_realsense_capture(dev: CameraDevice, width: int, height: int, fps: int, capture_fourcc: str):
    serial = _realsense_serial(dev)
    if not serial:
        raise RuntimeError(f"RealSense device has no SDK serial: {dev.path}")
    module = _load_jinhe_camera_capture()
    meta = _jinhe_camera_metadata_from_device(dev)
    cap = module.RealSenseColorCapture(meta, int(width), int(height), int(fps), str(capture_fourcc or "").strip().upper())
    meta["capture_backend"] = "pyrealsense2"
    meta["actual_width"] = int(getattr(cap, "width", width))
    meta["actual_height"] = int(getattr(cap, "height", height))
    meta["actual_fps"] = float(fps)
    meta["actual_fourcc"] = str(getattr(cap, "format_name", "") or "")
    meta["realsense"] = dict(getattr(cap, "info", {}) or {})
    return cap, meta


def _probe_readable_device(cv2, dev: CameraDevice, args: argparse.Namespace, probes: dict[Path, object]) -> bool:
    result = probes.get(dev.path)
    if result is None:
        result = probe_device(cv2, dev.path, args)
        probes[dev.path] = result
    return bool(getattr(result, "ok", False))


def _probe_readable_device_jinhe(dev: CameraDevice, args: argparse.Namespace, role_fourcc: str) -> bool:
    width = int(getattr(args, "width", 0) or 0)
    height = int(getattr(args, "height", 0) or 0)
    fps = int(round(float(getattr(args, "fps", 0.0) or 0.0)))
    if width <= 0 or height <= 0 or fps <= 0:
        return False
    cap = None
    try:
        if _is_realsense_device(dev):
            cap, _meta = _open_jinhe_realsense_capture(
                dev,
                width,
                height,
                fps,
                str(role_fourcc or getattr(args, "fourcc", "") or "").strip().upper(),
            )
            return bool(cap.isOpened())
        helper_args = argparse.Namespace(
            device=str(dev.path),
            width=width,
            height=height,
            fps=fps,
            capture_fourcc=str(role_fourcc or getattr(args, "fourcc", "") or "").strip().upper(),
        )
        cap, _meta = _load_jinhe_open_camera()(helper_args)
        if not cap.isOpened():
            return False
        ok, frame = cap.read()
        return bool(ok and frame is not None)
    except Exception:
        return False
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def _resolve_role_device(
    cv2,
    role: str,
    kind: str,
    value: str,
    devices: list[CameraDevice],
    args: argparse.Namespace,
    probes: dict[Path, object],
    used_paths: set[Path],
    role_fourccs: dict[str, str] | None = None,
) -> CameraDevice | None:
    matches: list[CameraDevice] = []
    if kind == "device":
        path = resolve_device_arg(value)
        if not path.exists():
            return None
        matches = [_find_device(devices, path)]
    elif kind == "usb-port":
        matches = [dev for dev in devices if dev.usb_port == value]
    elif kind == "serial":
        matches = [dev for dev in devices if _serial_matches(dev, value)]
        requested_serial = _serial_binding_value(value)
        realsense_matches = [
            dev for dev in matches
            if requested_serial and _is_realsense_device(dev) and requested_serial in _camera_serial_values(dev)
        ]
        if realsense_matches:
            matches = _preferred_devices_for_usb_port(realsense_matches)[:1]
    else:
        raise SystemExit(f"unknown role camera binding kind: {kind}")

    probe_args = args
    role_fourcc = (role_fourccs or {}).get(_topic_token(role), "")
    if role_fourcc:
        probe_args = argparse.Namespace(**vars(args))
        probe_args.fourcc = role_fourcc

    if role_fourcc:
        readable = [dev for dev in matches if _probe_readable_device_jinhe(dev, probe_args, role_fourcc)]
        if not readable:
            readable = [dev for dev in matches if _probe_readable_device(cv2, dev, probe_args, probes)]
    else:
        readable = [dev for dev in matches if _probe_readable_device(cv2, dev, probe_args, probes)]
    if not readable:
        return None

    for dev in _preferred_devices_for_usb_port(readable):
        try:
            key = dev.path.resolve()
        except OSError:
            key = dev.path
        if key not in used_paths:
            used_paths.add(key)
            return dev

    return None


def _choose_role_devices(
    devices: list[CameraDevice],
    args: argparse.Namespace,
) -> tuple[list[CameraDevice | None], dict[Path, object]]:
    roles = _requested_roles(args.roles)
    device_bindings = _parse_role_bindings(getattr(args, "role_device", []) or [], "--role-device")
    usb_bindings = _parse_role_bindings(getattr(args, "role_usb_port", []) or [], "--role-usb-port")
    serial_bindings = _parse_role_bindings(getattr(args, "role_serial", []) or [], "--role-serial")
    role_fourccs = _parse_role_fourccs(getattr(args, "role_fourcc", []) or [])
    missing_roles = _parse_missing_roles(getattr(args, "missing_role", []) or [])
    role_keys = set(roles)
    role_keys.update(device_bindings)
    role_keys.update(usb_bindings)
    role_keys.update(serial_bindings)
    role_keys.update(missing_roles)
    if not roles and role_keys:
        roles = sorted(role_keys)
    if not roles:
        return [], {}

    cv2 = import_cv2()
    probes: dict[Path, object] = {}
    used_paths: set[Path] = set()
    role_devices: list[CameraDevice | None] = []
    for role in roles:
        role_key = _topic_token(role)
        value = ""
        kind = ""
        if role_key in missing_roles:
            role_devices.append(None)
            continue
        if role_key in device_bindings:
            kind = "device"
            value = device_bindings[role_key]
        elif role_key in usb_bindings:
            kind = "usb-port"
            value = usb_bindings[role_key]
        elif role_key in serial_bindings:
            kind = "serial"
            value = serial_bindings[role_key]

        if value.startswith(MISSING_ROLE_PREFIX):
            role_devices.append(None)
            continue
        if not kind:
            role_devices.append(None)
            continue

        dev = _resolve_role_device(cv2, role, kind, value, devices, args, probes, used_paths, role_fourccs)
        if dev is None and not bool(getattr(args, "allow_missing", False)):
            raise SystemExit(f"Requested {role} camera {kind} was not found or readable: {value}")
        if dev is None:
            print(f"[camera_mosaic] WARN: {role} camera {kind} unavailable; publishing blank frames", file=sys.stderr)
        role_devices.append(dev)
    return role_devices, probes


class CameraMosaicPublisher:
    def __init__(
        self,
        rclpy,
        image_cls,
        compressed_image_cls,
        node,
        role_devices: list[CameraDevice | None],
        args: argparse.Namespace,
    ) -> None:
        self.rclpy = rclpy
        self.image_cls = image_cls
        self.compressed_image_cls = compressed_image_cls
        self.node = node
        self.args = args
        self.cv2 = import_cv2()
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.role_devices = list(role_devices)
        self.roles = _parse_roles(getattr(args, "roles", ""), len(self.role_devices))
        self.devices = [dev for dev in self.role_devices if dev is not None]
        self.device_roles = {
            dev.path: self.roles[idx]
            for idx, dev in enumerate(self.role_devices)
            if dev is not None and idx < len(self.roles)
        }
        self.role_capture_sizes = _parse_role_capture_sizes(getattr(args, "role_capture_size", []) or [])
        self.role_fourccs = _parse_role_fourccs(getattr(args, "role_fourcc", []) or [])
        self.per_camera_topic_prefix = str(getattr(args, "per_camera_topic_prefix", "") or "").strip().rstrip("/")
        self.compressed_jpeg_quality = int(getattr(args, "compressed_jpeg_quality", 90))
        self.per_camera_jpeg_quality = int(
            getattr(args, "per_camera_compressed_jpeg_quality", self.compressed_jpeg_quality)
        )
        self.per_camera_w = max(0, int(getattr(args, "per_camera_width", 0) or 0))
        self.per_camera_h = max(0, int(getattr(args, "per_camera_height", 0) or 0))
        self.per_camera_publishers = {}
        if self.per_camera_topic_prefix:
            for role in self.roles:
                role = _topic_token(role)
                topic = f"{self.per_camera_topic_prefix}/{role}/image_raw/compressed"
                self.per_camera_publishers[role] = node.create_publisher(compressed_image_cls, topic, qos)

        self.raw_enabled = not bool(getattr(args, "compressed_only", False))
        self.publisher = node.create_publisher(image_cls, args.topic, qos) if self.raw_enabled else None
        self.compressed_publisher = None
        self.compressed_topic = str(getattr(args, "compressed_topic", "") or "").strip()
        if bool(getattr(args, "compressed_only", False)) and not self.compressed_topic and not self.per_camera_publishers:
            self.compressed_topic = f"{str(args.topic).rstrip('/')}/compressed"
        if self.compressed_topic:
            self.compressed_publisher = node.create_publisher(compressed_image_cls, self.compressed_topic, qos)
        if not self.raw_enabled and self.compressed_publisher is None and not self.per_camera_publishers:
            raise RuntimeError("--compressed-only requires --compressed-topic or --per-camera-topic-prefix.")
        self.preview_topic_prefix = str(getattr(args, "preview_topic_prefix", "") or "").strip().rstrip("/")
        self.preview_jpeg_quality = int(getattr(args, "preview_compressed_jpeg_quality", self.compressed_jpeg_quality))
        self.preview_show_stats = bool(getattr(args, "preview_show_stats", False))
        self.preview_w = max(160, int(getattr(args, "preview_width", 0) or getattr(args, "tile_width", 640)))
        self.preview_h = max(120, int(getattr(args, "preview_height", 0) or getattr(args, "tile_height", 480)))
        self.preview_publishers = {}
        if self.preview_topic_prefix:
            for role in self.roles:
                role = _topic_token(role)
                topic = f"{self.preview_topic_prefix}/{role}/image_raw"
                self.preview_publishers[role] = node.create_publisher(image_cls, topic, qos)
        self.captures = {}
        self.last_frames = {}
        self.last_unique_frame_mono = {dev.path: 0.0 for dev in self.devices}
        self.last_frame_hash = {dev.path: None for dev in self.devices}
        self.repeat_frame_count = {dev.path: 0 for dev in self.devices}
        self.repeat_frame_since = {dev.path: 0.0 for dev in self.devices}
        self.stale_reasons = {dev.path: "" for dev in self.devices}
        self.stale_since_mono = {dev.path: 0.0 for dev in self.devices}
        self._last_stale_log_mono = {dev.path: 0.0 for dev in self.devices}
        self._last_reopen_mono = {dev.path: 0.0 for dev in self.devices}
        self._frame_lock = threading.Lock()
        self._stop_capture = threading.Event()
        self._capture_threads = []
        self.frame_counts = {dev.path: 0 for dev in self.devices}
        self.fps_values = {dev.path: 0.0 for dev in self.devices}
        self.fps_t0 = {dev.path: time.monotonic() for dev in self.devices}

        for dev in self.devices:
            cap = self._open_configured_capture(dev)
            if cap.isOpened():
                self.captures[dev.path] = cap
            else:
                cap.release()
                node.get_logger().warning(f"could not open camera {dev.path}")

        self.allow_missing = bool(getattr(args, "allow_missing", False))
        if not self.captures and not self.allow_missing:
            raise RuntimeError("could not open any selected camera")

        role_count = max(1, len(self.role_devices))
        self.cols = max(1, min(int(args.cols), role_count)) if args.cols > 0 else max(1, math.ceil(math.sqrt(role_count)))
        self.rows = max(1, math.ceil(role_count / self.cols))
        self.tile_w = max(160, int(args.tile_width))
        self.tile_h = max(120, int(args.tile_height))
        self.output_w = max(1, int(args.output_width))
        self.output_h = max(1, int(args.output_height))
        self.reverse = bool(args.reverse)
        self.reverse_mode = str(args.reverse_mode)
        self.show_labels = bool(args.show_labels)
        self.overlay_stats = bool(args.overlay_stats)
        self.stats_interval = max(0.0, float(args.stats_interval))
        self.stale_read_timeout_sec = max(0.0, float(getattr(args, "stale_read_timeout_sec", 1.0)))
        self.stale_repeat_sec = max(0.0, float(getattr(args, "stale_repeat_sec", 1.0)))
        self.stale_repeat_count = max(1, int(getattr(args, "stale_repeat_count", 20)))
        self.publish_stale_frames = bool(getattr(args, "publish_stale_frames", False))
        self.auto_reopen_stale_sec = max(0.0, float(getattr(args, "auto_reopen_stale_sec", 0.0)))
        self.auto_reopen_cooldown_sec = max(0.0, float(getattr(args, "auto_reopen_cooldown_sec", 5.0)))
        self.auto_reopen_delay_sec = max(0.0, float(getattr(args, "auto_reopen_delay_sec", 0.2)))
        self._stat_t0 = time.monotonic()
        self._stat_reads = 0
        self._stat_publishes = 0
        self._overlay_t0 = time.monotonic()
        self._overlay_publish_count = 0
        self._overlay_publish_fps = 0.0
        self.direct_publish = (
            len(self.role_devices) == 1
            and self.role_devices[0] is not None
            and not bool(args.no_direct_publish)
        )
        for dev in self.devices:
            cap = self.captures.get(dev.path)
            if cap is None:
                continue
            thread = threading.Thread(target=self._capture_loop, args=(dev,), daemon=True)
            thread.start()
            self._capture_threads.append(thread)
        self.timer = None
        if not self.direct_publish:
            period = 1.0 / max(float(args.publish_fps), 1.0)
            self.timer = node.create_timer(period, self.publish_once)

        shown_items = []
        for idx, dev in enumerate(self.role_devices):
            role = self.roles[idx] if idx < len(self.roles) else f"camera_{idx + 1}"
            if dev is None:
                shown_items.append(f"{role}:missing")
            else:
                shown_items.append(f"{role}:{_display_device(dev)}(usb:{dev.usb_port})")
        shown = ", ".join(shown_items)
        reverse_desc = f", reverse={self.reverse_mode}" if self.reverse else ""
        if self.role_capture_sizes:
            role_size_desc = ", ".join(
                f"{role}={size[0]}x{size[1]}" for role, size in sorted(self.role_capture_sizes.items())
            )
            node.get_logger().info(f"per-role capture size overrides: {role_size_desc}")
        if self.role_fourccs:
            role_fourcc_desc = ", ".join(f"{role}={fourcc}" for role, fourcc in sorted(self.role_fourccs.items()))
            node.get_logger().info(f"per-role capture FOURCC overrides: {role_fourcc_desc}")
        if self.raw_enabled:
            node.get_logger().info(
                f"publishing {len(self.devices)} camera(s) to {args.topic} "
                f"as {self.output_w}x{self.output_h}{reverse_desc}: {shown}"
            )
        elif self.compressed_publisher is not None or self.preview_publishers:
            node.get_logger().info(
                f"raw image topic disabled; building {self.output_w}x{self.output_h}{reverse_desc} mosaic: {shown}"
            )
        else:
            node.get_logger().info(f"raw and mosaic topics disabled; publishing clean per-camera streams: {shown}")
        if self.compressed_publisher is not None:
            node.get_logger().info(
                f"publishing compressed JPEG mosaic to {self.compressed_topic} "
                f"quality={self.compressed_jpeg_quality}"
            )
        if self.per_camera_publishers:
            topics = [
                f"{self.roles[idx]}:{self.per_camera_topic_prefix}/{_topic_token(self.roles[idx])}/image_raw/compressed"
                for idx in range(len(self.roles))
            ]
            size_desc = (
                f"{self.per_camera_w}x{self.per_camera_h}"
                if self.per_camera_w > 0 and self.per_camera_h > 0
                else "source-size"
            )
            node.get_logger().info(
                f"publishing clean per-camera JPEG streams {size_desc} "
                f"quality={self.per_camera_jpeg_quality}: {', '.join(topics)}"
            )
        if self.preview_publishers:
            topics = [
                f"{self.roles[idx]}:{self.preview_topic_prefix}/{_topic_token(self.roles[idx])}/image_raw"
                for idx in range(len(self.roles))
            ]
            node.get_logger().info(
                f"publishing RViz-only raw camera previews {self.preview_w}x{self.preview_h}: {', '.join(topics)}"
            )
        node.get_logger().info(
            "stale camera guard active: "
            f"read_timeout={self.stale_read_timeout_sec:.2f}s "
            f"repeat={self.stale_repeat_count} frames/{self.stale_repeat_sec:.2f}s "
            f"publish_stale_frames={self.publish_stale_frames}"
        )
        if self.auto_reopen_stale_sec > 0.0:
            node.get_logger().info(
                "stale camera auto-reopen active: "
                f"stale_for={self.auto_reopen_stale_sec:.2f}s "
                f"cooldown={self.auto_reopen_cooldown_sec:.2f}s "
                f"delay={self.auto_reopen_delay_sec:.2f}s"
            )

    def close(self) -> None:
        self._stop_capture.set()
        for thread in self._capture_threads:
            thread.join(timeout=1.0)
        for cap in self.captures.values():
            cap.release()
        self.captures.clear()

    def _open_configured_capture(self, dev: CameraDevice):
        role = self._role_for_dev(dev)
        role_key = _topic_token(role)
        capture_fourcc = self.role_fourccs.get(role_key, str(getattr(self.args, "fourcc", "") or "").strip().upper())
        args = self.args
        if capture_fourcc:
            args = argparse.Namespace(**vars(self.args))
            args.fourcc = capture_fourcc
        if not bool(getattr(self.args, "no_jinhe_camera_helper", False)):
            helper_size = self.role_capture_sizes.get(role_key)
            helper_w = helper_size[0] if helper_size is not None else int(getattr(self.args, "width", 0) or 0)
            helper_h = helper_size[1] if helper_size is not None else int(getattr(self.args, "height", 0) or 0)
            helper_fps = int(round(float(getattr(self.args, "fps", 0.0) or 0.0)))
            if helper_w > 0 and helper_h > 0 and helper_fps > 0:
                try:
                    if _is_realsense_device(dev):
                        cap, meta = _open_jinhe_realsense_capture(
                            dev,
                            helper_w,
                            helper_h,
                            helper_fps,
                            capture_fourcc,
                        )
                    else:
                        helper_args = argparse.Namespace(
                            device=str(dev.path),
                            width=helper_w,
                            height=helper_h,
                            fps=helper_fps,
                            capture_fourcc=capture_fourcc,
                        )
                        cap, meta = _load_jinhe_open_camera()(helper_args)
                    adapter = CameraCaptureAdapter(cap, meta)
                    if adapter.isOpened():
                        source = "RealSense SDK" if _is_realsense_device(dev) else "ros2_jinhe helper"
                        self.node.get_logger().info(
                            f"opened {role} with {source}: "
                            f"{meta.get('opened_device') or _display_device(dev)} "
                            f"backend={meta.get('capture_backend', 'unknown')} "
                            f"actual={meta.get('actual_width', '?')}x{meta.get('actual_height', '?')}"
                            f"@{meta.get('actual_fps', '?')} {meta.get('actual_fourcc', '')}"
                        )
                        return adapter
                    adapter.release()
                except Exception as exc:
                    if _is_realsense_device(dev):
                        raise RuntimeError(
                            f"RealSense SDK open failed for {role} {_display_device(dev)}: {exc}"
                        ) from exc
                    self.node.get_logger().warning(
                        f"ros2_jinhe helper failed for {role} {dev.path}; falling back to OpenCV: {exc}"
                    )
        cap = _open_capture(self.cv2, dev.path, args)
        self._apply_role_capture_settings(cap, role)
        if self.args.buffer_size > 0:
            try:
                cap.set(self.cv2.CAP_PROP_BUFFERSIZE, int(self.args.buffer_size))
            except Exception:
                pass
        return cap

    def _apply_role_capture_settings(self, cap, role: str) -> None:
        size = self.role_capture_sizes.get(_topic_token(role))
        if size is None:
            return
        width, height = size
        try:
            cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, float(width))
            cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        except Exception as exc:
            self.node.get_logger().warning(f"could not apply {role} capture size {width}x{height}: {exc}")

    def _as_bgr(self, frame):
        if frame is None:
            return None
        if len(frame.shape) == 2:
            return self.cv2.cvtColor(frame, self.cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:
            return self.cv2.cvtColor(frame, self.cv2.COLOR_BGRA2BGR)
        return frame

    def _reverse_frame(self, frame):
        if not self.reverse:
            return frame
        if self.reverse_mode == "horizontal":
            return self.cv2.flip(frame, 1)
        if self.reverse_mode == "vertical":
            return self.cv2.flip(frame, 0)
        if self.reverse_mode in ("both", "rotate180"):
            return self.cv2.flip(frame, -1)
        return frame

    def _capture_loop(self, dev: CameraDevice) -> None:
        cap = self.captures.get(dev.path)
        while not self._stop_capture.is_set():
            if cap is None or not cap.isOpened():
                cap = self._maybe_reopen_capture(dev, cap, "capture is not open", time.monotonic(), force=True)
                if cap is None:
                    time.sleep(max(0.1, min(self.auto_reopen_cooldown_sec or 1.0, 1.0)))
                    continue
            ok, read_frame = cap.read()
            now = time.monotonic()
            if not ok or read_frame is None:
                self._mark_read_failure_if_stale(dev, now)
                cap = self._maybe_reopen_capture(dev, cap, "read failed", now)
                time.sleep(0.001)
                continue
            frame = self._as_bgr(read_frame)
            frame = self._reverse_frame(frame)
            self._store_capture_frame(dev, frame, now)
            cap = self._maybe_reopen_capture(dev, cap, "stale frame stream", now)
            self._stat_reads += 1
            self.frame_counts[dev.path] += 1
            elapsed = now - self.fps_t0[dev.path]
            if elapsed >= 1.0:
                self.fps_values[dev.path] = self.frame_counts[dev.path] / elapsed
                self.frame_counts[dev.path] = 0
                self.fps_t0[dev.path] = now
            if self.direct_publish:
                live_frame, stale_reason = self._frame_snapshot(dev, now)
                if stale_reason and not self.publish_stale_frames:
                    continue
                if stale_reason:
                    live_frame = _make_blank_tile(
                        self.cv2,
                        dev,
                        self.output_w,
                        self.output_h,
                        f"STALE {stale_reason}",
                    )
                self._publish_bgr(self._fit_output(live_frame))

    def _frame_hash(self, frame) -> bytes:
        contiguous = np.ascontiguousarray(frame)
        return hashlib.blake2b(contiguous, digest_size=8).digest()

    def _set_stale_reason_locked(self, dev: CameraDevice, reason: str, now: float) -> None:
        previous = self.stale_reasons.get(dev.path, "")
        self.stale_reasons[dev.path] = reason
        if not previous:
            self.stale_since_mono[dev.path] = now
        last_log = self._last_stale_log_mono.get(dev.path, 0.0)
        if previous != reason or now - last_log >= 1.0:
            self._last_stale_log_mono[dev.path] = now
            self.node.get_logger().warning(
                f"camera stale: role={self._role_for_dev(dev)} "
                f"dev={dev.path} usb={dev.usb_port} serial={dev.serial}: {reason}"
            )

    def _clear_stale_reason_locked(self, dev: CameraDevice, now: float) -> None:
        if self.stale_reasons.get(dev.path):
            self.node.get_logger().warning(
                f"camera recovered: role={self._role_for_dev(dev)} "
                f"dev={dev.path} usb={dev.usb_port} serial={dev.serial}"
            )
        self.stale_reasons[dev.path] = ""
        self.stale_since_mono[dev.path] = 0.0
        self._last_stale_log_mono[dev.path] = now

    def _topic_for_dev(self, dev: CameraDevice) -> str:
        role = _topic_token(self._role_for_dev(dev))
        if self.per_camera_topic_prefix:
            return f"{self.per_camera_topic_prefix}/{role}/image_raw/compressed"
        if self.compressed_topic:
            return self.compressed_topic
        return str(self.args.topic)

    def _reset_capture_state_for_reopen(self, dev: CameraDevice) -> None:
        with self._frame_lock:
            self.last_frames.pop(dev.path, None)
            self.last_frame_hash[dev.path] = None
            self.last_unique_frame_mono[dev.path] = 0.0
            self.repeat_frame_count[dev.path] = 0
            self.repeat_frame_since[dev.path] = 0.0
            self.frame_counts[dev.path] = 0
            self.fps_values[dev.path] = 0.0
            self.fps_t0[dev.path] = time.monotonic()

    def _maybe_reopen_capture(
        self,
        dev: CameraDevice,
        cap,
        trigger: str,
        now: float,
        force: bool = False,
    ):
        if self.auto_reopen_stale_sec <= 0.0 and not force:
            return cap
        reason = ""
        stale_for = 0.0
        if not force:
            with self._frame_lock:
                reason = self.stale_reasons.get(dev.path, "")
                stale_since = self.stale_since_mono.get(dev.path, 0.0)
            if not reason or stale_since <= 0.0:
                return cap
            stale_for = now - stale_since
            if stale_for < self.auto_reopen_stale_sec:
                return cap
        last_reopen = self._last_reopen_mono.get(dev.path, 0.0)
        if not force and self.auto_reopen_cooldown_sec > 0.0 and now - last_reopen < self.auto_reopen_cooldown_sec:
            return cap

        self._last_reopen_mono[dev.path] = now
        role = self._role_for_dev(dev)
        detail = reason or trigger
        suffix = f" stale_for={stale_for:.2f}s" if stale_for > 0.0 else ""
        self.node.get_logger().warning(
            f"reopening camera capture: role={role} topic={self._topic_for_dev(dev)} "
            f"dev={dev.path} usb={dev.usb_port} serial={dev.serial} trigger={trigger}: {detail}{suffix}"
        )
        if cap is not None:
            try:
                cap.release()
            except Exception as exc:
                self.node.get_logger().warning(f"could not release camera {dev.path}: {exc}")
        self.captures.pop(dev.path, None)
        self._reset_capture_state_for_reopen(dev)
        if self.auto_reopen_delay_sec > 0.0:
            time.sleep(self.auto_reopen_delay_sec)
        if self._stop_capture.is_set():
            return None

        new_cap = self._open_configured_capture(dev)
        if not new_cap.isOpened():
            new_cap.release()
            self.node.get_logger().warning(f"camera reopen failed: role={role} dev={dev.path}")
            return None

        self.captures[dev.path] = new_cap
        self.node.get_logger().warning(f"camera capture reopened: role={role} dev={dev.path}")
        return new_cap

    def _store_capture_frame(self, dev: CameraDevice, frame, now: float) -> None:
        frame_hash = self._frame_hash(frame)
        with self._frame_lock:
            previous_hash = self.last_frame_hash.get(dev.path)
            if previous_hash == frame_hash:
                self.repeat_frame_count[dev.path] = self.repeat_frame_count.get(dev.path, 0) + 1
                if self.repeat_frame_since.get(dev.path, 0.0) <= 0.0:
                    self.repeat_frame_since[dev.path] = self.last_unique_frame_mono.get(dev.path, now) or now
                repeated_for = now - self.repeat_frame_since[dev.path]
                if (
                    self.stale_repeat_sec > 0.0
                    and self.repeat_frame_count[dev.path] >= self.stale_repeat_count
                    and repeated_for >= self.stale_repeat_sec
                ):
                    self._set_stale_reason_locked(
                        dev,
                        f"repeated identical frame x{self.repeat_frame_count[dev.path]} "
                        f"for {repeated_for:.2f}s",
                        now,
                    )
                return

            self.last_frames[dev.path] = frame
            self.last_frame_hash[dev.path] = frame_hash
            self.last_unique_frame_mono[dev.path] = now
            self.repeat_frame_count[dev.path] = 1
            self.repeat_frame_since[dev.path] = now
            self._clear_stale_reason_locked(dev, now)

    def _mark_read_failure_if_stale(self, dev: CameraDevice, now: float) -> None:
        if self.stale_read_timeout_sec <= 0.0:
            return
        with self._frame_lock:
            last_unique = self.last_unique_frame_mono.get(dev.path, 0.0)
            if last_unique <= 0.0:
                return
            stale_for = now - last_unique
            if stale_for >= self.stale_read_timeout_sec:
                self._set_stale_reason_locked(dev, f"no new readable frame for {stale_for:.2f}s", now)

    def _role_for_dev(self, dev: CameraDevice) -> str:
        return self.device_roles.get(dev.path, "camera")

    def _frame_snapshot(self, dev: CameraDevice, now: float):
        with self._frame_lock:
            frame = self.last_frames.get(dev.path)
            reason = self.stale_reasons.get(dev.path, "")
            last_unique = self.last_unique_frame_mono.get(dev.path, 0.0)
            if (
                frame is not None
                and not reason
                and self.stale_read_timeout_sec > 0.0
                and last_unique > 0.0
                and now - last_unique >= self.stale_read_timeout_sec
            ):
                reason = f"no new unique frame for {now - last_unique:.2f}s"
                self._set_stale_reason_locked(dev, reason, now)
            return frame, reason

    def _fit_tile(self, frame):
        return self._fit_to(frame, self.tile_w, self.tile_h)

    def _fit_preview(self, frame):
        return self._fit_to(frame, self.preview_w, self.preview_h)

    def _fit_per_camera(self, frame):
        if self.per_camera_w > 0 and self.per_camera_h > 0:
            return self._fit_to(frame, self.per_camera_w, self.per_camera_h)
        return frame

    def _per_camera_blank_size(self, role: str, frame=None) -> tuple[int, int]:
        if self.per_camera_w > 0 and self.per_camera_h > 0:
            return self.per_camera_w, self.per_camera_h
        if frame is not None:
            source_h, source_w = frame.shape[:2]
            return int(source_w), int(source_h)
        role_size = self.role_capture_sizes.get(_topic_token(role))
        if role_size is not None:
            return role_size
        requested_w = int(getattr(self.args, "width", 0) or 0)
        requested_h = int(getattr(self.args, "height", 0) or 0)
        if requested_w > 0 and requested_h > 0:
            return requested_w, requested_h
        return self.tile_w, self.tile_h

    def _blank_per_camera_frame(self, role: str, dev: CameraDevice | None, text: str, frame=None):
        width, height = self._per_camera_blank_size(role, frame)
        return _make_blank_tile(self.cv2, dev or _missing_device(role), width, height, text)

    def _fit_to(self, frame, target_w: int, target_h: int):
        h, w = frame.shape[:2]
        if w == target_w and h == target_h:
            return frame
        scale = min(target_w / max(w, 1), target_h / max(h, 1))
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        resized = self.cv2.resize(frame, (new_w, new_h), interpolation=self.cv2.INTER_AREA)
        tile = np.zeros((target_h, target_w, 3), dtype=resized.dtype)
        x0 = (target_w - new_w) // 2
        y0 = (target_h - new_h) // 2
        tile[y0 : y0 + new_h, x0 : x0 + new_w] = resized
        return tile

    def _blank_tile_for_role(self, role: str, tile_w: int, tile_h: int, text: str):
        return _make_blank_tile(self.cv2, _missing_device(role), tile_w, tile_h, text)

    def _read_tile(self, role: str, dev: CameraDevice | None, now: float):
        if dev is None:
            return self._blank_tile_for_role(role, self.tile_w, self.tile_h, "camera missing")
        frame, stale_reason = self._frame_snapshot(dev, now)
        if frame is None:
            return _make_blank_tile(self.cv2, dev, self.tile_w, self.tile_h, "waiting for frame")
        if stale_reason:
            return _make_blank_tile(self.cv2, dev, self.tile_w, self.tile_h, f"STALE {stale_reason}")
        if self.show_labels:
            return _make_tile(self.cv2, frame, dev, self.fps_values[dev.path], self.tile_w, self.tile_h)
        return self._fit_tile(frame)

    def _build_mosaic(self):
        now = time.monotonic()
        tiles = [
            self._read_tile(self.roles[idx] if idx < len(self.roles) else f"camera_{idx + 1}", dev, now)
            for idx, dev in enumerate(self.role_devices)
        ]
        blank = np.zeros((self.tile_h, self.tile_w, 3), dtype=np.uint8)
        while len(tiles) < self.rows * self.cols:
            tiles.append(blank.copy())
        row_images = [
            self.cv2.hconcat(tiles[r * self.cols : (r + 1) * self.cols])
            for r in range(self.rows)
        ]
        mosaic = self.cv2.vconcat(row_images)
        return self._fit_output(mosaic)

    def _fit_output(self, frame):
        if frame.shape[1] == self.output_w and frame.shape[0] == self.output_h:
            return frame
        return self.cv2.resize(frame, (self.output_w, self.output_h), interpolation=self.cv2.INTER_AREA)

    def publish_once(self) -> None:
        if not self.rclpy.ok():
            return
        if self.publisher is None and self.compressed_publisher is None and self.per_camera_publishers and not self.preview_publishers:
            header = self._make_header(self.args.frame_id)
            self._publish_clean_camera_frames(header)
            self._stat_publishes += 1
            self._maybe_log_stats()
            return
        self._publish_bgr(self._build_mosaic())

    def _make_header(self, frame_id: str):
        holder = self.image_cls()
        holder.header.stamp = self.node.get_clock().now().to_msg()
        holder.header.frame_id = frame_id
        return holder.header

    def _publish_bgr(self, mosaic_bgr) -> None:
        self._update_overlay_publish_fps()
        if self.overlay_stats:
            mosaic_bgr = self._draw_stats_overlay(mosaic_bgr)

        msg = self.image_cls()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.height = int(mosaic_bgr.shape[0])
        msg.width = int(mosaic_bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(msg.width * 3)
        msg.data = mosaic_bgr.tobytes()
        try:
            if self.publisher is not None:
                self.publisher.publish(msg)
            self._publish_compressed_bgr(mosaic_bgr, msg.header)
            self._publish_clean_camera_frames(msg.header)
            self._publish_preview_frames(msg.header)
            self._stat_publishes += 1
        except Exception:
            if self.rclpy.ok():
                raise
        self._maybe_log_stats()

    def _publish_compressed_bgr(self, mosaic_bgr, header) -> None:
        if self.compressed_publisher is None:
            return
        self._publish_compressed_to(self.compressed_publisher, mosaic_bgr, header, self.compressed_jpeg_quality)

    def _publish_compressed_to(self, publisher, image_bgr, header, jpeg_quality: int, frame_id: str | None = None) -> None:
        ok, encoded = self.cv2.imencode(
            ".jpg",
            image_bgr,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
        )
        if not ok:
            return
        msg = self.compressed_image_cls()
        msg.header.stamp = header.stamp
        msg.header.frame_id = frame_id if frame_id is not None else header.frame_id
        msg.format = "bgr8; jpeg compressed bgr8"
        msg.data = encoded.tobytes()
        publisher.publish(msg)

    def _publish_raw_to(self, publisher, image_bgr, header, frame_id: str | None = None) -> None:
        msg = self.image_cls()
        msg.header.stamp = header.stamp
        msg.header.frame_id = frame_id if frame_id is not None else header.frame_id
        msg.height = int(image_bgr.shape[0])
        msg.width = int(image_bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(msg.width * 3)
        msg.data = image_bgr.tobytes()
        publisher.publish(msg)

    def _publish_clean_camera_frames(self, header) -> None:
        if not self.per_camera_publishers:
            return
        now = time.monotonic()
        for idx, dev in enumerate(self.role_devices):
            role = self.roles[idx] if idx < len(self.roles) else f"camera_{idx + 1}"
            role_token = _topic_token(role)
            publisher = self.per_camera_publishers.get(role_token)
            if publisher is None:
                continue
            if dev is None:
                image = self._blank_per_camera_frame(role, None, "camera missing")
            else:
                frame, stale_reason = self._frame_snapshot(dev, now)
                if frame is None:
                    if not self.allow_missing:
                        continue
                    image = self._blank_per_camera_frame(role, dev, "waiting for frame")
                elif stale_reason:
                    if not self.allow_missing and not self.publish_stale_frames:
                        continue
                    image = self._blank_per_camera_frame(role, dev, f"STALE {stale_reason}", frame=frame)
                else:
                    image = self._fit_per_camera(frame)
            self._publish_compressed_to(
                publisher,
                image,
                header,
                self.per_camera_jpeg_quality,
                frame_id=f"{self.args.frame_id}_{_topic_token(role)}",
            )

    def _publish_preview_frames(self, header) -> None:
        if not self.preview_publishers:
            return
        now = time.monotonic()
        for idx, dev in enumerate(self.role_devices):
            role = self.roles[idx] if idx < len(self.roles) else f"camera_{idx + 1}"
            publisher = self.preview_publishers.get(_topic_token(role))
            if publisher is None:
                continue
            if dev is None:
                image = self._blank_tile_for_role(role, self.preview_w, self.preview_h, "camera missing")
            else:
                frame, stale_reason = self._frame_snapshot(dev, now)
                if frame is None:
                    image = _make_blank_tile(self.cv2, dev, self.preview_w, self.preview_h, "waiting for frame")
                elif stale_reason:
                    image = _make_blank_tile(self.cv2, dev, self.preview_w, self.preview_h, f"STALE {stale_reason}")
                else:
                    image = self._build_preview_frame(idx, dev, frame)
            self._publish_raw_to(publisher, image, header, frame_id=f"{self.args.frame_id}_{_topic_token(role)}")

    def _build_preview_frame(self, idx: int, dev: CameraDevice, frame):
        source_h, source_w = frame.shape[:2]
        image = self._fit_preview(frame)
        if not self.preview_show_stats:
            return image

        role = self.roles[idx] if idx < len(self.roles) else f"camera_{idx + 1}"
        capture_fps = self.fps_values.get(dev.path, 0.0)
        requested_fps = float(getattr(self.args, "fps", 0.0) or 0.0)
        publish_fps = float(getattr(self.args, "publish_fps", 0.0) or 0.0)
        fourcc = str(getattr(self.args, "fourcc", "") or "default").upper()
        requested_size = self.role_capture_sizes.get(_topic_token(role))
        if requested_size is not None:
            requested_desc = f"{requested_size[0]}x{requested_size[1]}"
        else:
            requested_w = int(getattr(self.args, "width", 0) or 0)
            requested_h = int(getattr(self.args, "height", 0) or 0)
            requested_desc = f"{requested_w}x{requested_h}" if requested_w > 0 and requested_h > 0 else "default"
        lines = [
            f"{role.upper()}  {capture_fps:.1f} FPS  {source_w}x{source_h}",
            f"usb:{dev.usb_port}  iface:{dev.usb_iface}  {dev.path}",
            f"capture:{fourcc}@{requested_fps:g} req:{requested_desc}  publish:{publish_fps:g}Hz",
        ]
        self._draw_text_box(image, lines)
        self.cv2.rectangle(image, (0, 0), (self.preview_w - 1, self.preview_h - 1), (0, 220, 255), 2)
        return image

    def _draw_text_box(self, image_bgr, lines: list[str]) -> None:
        if not lines:
            return
        line_h = 24
        box_h = 14 + line_h * len(lines)
        self.cv2.rectangle(image_bgr, (0, 0), (image_bgr.shape[1], box_h), (0, 0, 0), -1)
        y = 24
        for idx, line in enumerate(lines):
            color = (90, 255, 90) if idx == 0 else (255, 255, 255)
            self.cv2.putText(image_bgr, line, (10, y), self.cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, self.cv2.LINE_AA)
            self.cv2.putText(image_bgr, line, (10, y), self.cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, self.cv2.LINE_AA)
            y += line_h

    def _update_overlay_publish_fps(self) -> None:
        self._overlay_publish_count += 1
        now = time.monotonic()
        elapsed = now - self._overlay_t0
        if elapsed >= 1.0:
            self._overlay_publish_fps = self._overlay_publish_count / elapsed
            self._overlay_publish_count = 0
            self._overlay_t0 = now

    def _draw_stats_overlay(self, image_bgr):
        frame = image_bgr.copy()
        height, width = frame.shape[:2]
        camera_fps_values = [fps for fps in self.fps_values.values() if fps > 0.0]
        if camera_fps_values:
            fps = min(camera_fps_values)
        else:
            fps = self._overlay_publish_fps

        lines = [f"{fps:.1f} FPS  {width}x{height}"]
        if len(self.devices) == 1:
            dev = self.devices[0]
            lines.append(f"{dev.path}  usb:{dev.usb_port}")

        x = 10
        y = 26
        for line in lines:
            self.cv2.putText(frame, line, (x, y), self.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, self.cv2.LINE_AA)
            self.cv2.putText(frame, line, (x, y), self.cv2.FONT_HERSHEY_SIMPLEX, 0.62, (80, 255, 80), 2, self.cv2.LINE_AA)
            y += 26
        return frame

    def _maybe_log_stats(self) -> None:
        if self.stats_interval <= 0.0:
            return
        now = time.monotonic()
        elapsed = now - self._stat_t0
        if elapsed < self.stats_interval:
            return
        self.node.get_logger().info(
            f"camera stats: reads={self._stat_reads / elapsed:.1f}fps "
            f"publishes={self._stat_publishes / elapsed:.1f}fps"
        )
        self._stat_t0 = now
        self._stat_reads = 0
        self._stat_publishes = 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish local V4L2 cameras as a ROS2 image mosaic for RViz.")
    parser.add_argument("--list", action="store_true", help="Print detected /dev/video* devices.")
    parser.add_argument("--probe", action="store_true", help="Probe cameras before publishing.")
    parser.add_argument("--device", action="append", default=[], help="Camera device. Repeat for multiple devices.")
    parser.add_argument("--usb-port", action="append", default=[], help="USB_PORT value from --list output. Repeat for multiple ports.")
    parser.add_argument("--serial", action="append", default=[], help="Camera serial number from --list output. Repeat for multiple cameras.")
    parser.add_argument("--role-device", action="append", default=[], help="Role-specific camera device, role=/dev/videoX.")
    parser.add_argument("--role-usb-port", action="append", default=[], help="Role-specific USB_PORT binding, role=USB_PORT.")
    parser.add_argument("--role-serial", action="append", default=[], help="Role-specific serial binding, role=SERIAL.")
    parser.add_argument("--missing-role", action="append", default=[], help="Camera role to publish as a blank stream.")
    parser.add_argument("--allow-missing", action="store_true", help="Keep publishing blank streams for missing cameras.")
    parser.add_argument("--all-nodes", action="store_true", help="Use every readable /dev/video* node.")
    parser.add_argument("--cols", type=int, default=0, help="Mosaic columns. 0 chooses automatically.")
    parser.add_argument("--tile-width", type=int, default=640, help="Width of each camera tile.")
    parser.add_argument("--tile-height", type=int, default=480, help="Height of each camera tile.")
    parser.add_argument("--output-width", type=int, default=640, help="Published ROS image width.")
    parser.add_argument("--output-height", type=int, default=480, help="Published ROS image height.")
    parser.add_argument("--reverse", action="store_true", help="Reverse/flip camera frames before publishing.")
    parser.add_argument("--show-labels", action="store_true", help="Draw camera labels/FPS on the output image.")
    parser.add_argument("--overlay-stats", action="store_true", help="Draw FPS and resolution in the published image.")
    parser.add_argument("--no-direct-publish", action="store_true", help="Use timer-based mosaic publishing even for one camera.")
    parser.add_argument("--stats-interval", type=float, default=0.0, help="Seconds between internal FPS log lines. 0 disables stats.")
    parser.add_argument(
        "--stale-read-timeout-sec",
        type=float,
        default=1.0,
        help="Mark a camera stale if no new unique frame is seen for this many seconds. 0 disables this guard.",
    )
    parser.add_argument(
        "--stale-repeat-sec",
        type=float,
        default=1.0,
        help="Mark a camera stale after identical frames persist for this many seconds. 0 disables repeat-frame stale detection.",
    )
    parser.add_argument(
        "--stale-repeat-count",
        type=int,
        default=20,
        help="Minimum identical-frame read count before repeat-frame stale detection can trigger.",
    )
    parser.add_argument(
        "--publish-stale-frames",
        action="store_true",
        help="Publish visible STALE placeholder images instead of suppressing stale per-camera data topics.",
    )
    parser.add_argument(
        "--auto-reopen-stale-sec",
        type=float,
        default=0.0,
        help="Reopen only the stale camera capture after it remains stale for this many seconds. 0 disables auto-reopen.",
    )
    parser.add_argument(
        "--auto-reopen-cooldown-sec",
        type=float,
        default=5.0,
        help="Minimum seconds between auto-reopen attempts for one camera.",
    )
    parser.add_argument(
        "--auto-reopen-delay-sec",
        type=float,
        default=0.2,
        help="Seconds to wait after releasing a stale camera before reopening it.",
    )
    parser.add_argument(
        "--reverse-mode",
        choices=("horizontal", "vertical", "both", "rotate180"),
        default="horizontal",
        help="How to reverse frames when --reverse is set.",
    )
    parser.add_argument("--backend", choices=("v4l2", "any"), default="v4l2", help="OpenCV capture backend.")
    parser.add_argument(
        "--no-jinhe-camera-helper",
        action="store_true",
        default=not bool(JINHE_ROOT),
        help="Disable ros2_jinhe camera helper and use the legacy OpenCV capture path.",
    )
    parser.add_argument("--width", type=int, default=0, help="Requested capture width. 0 keeps camera default.")
    parser.add_argument("--height", type=int, default=0, help="Requested capture height. 0 keeps camera default.")
    parser.add_argument("--fps", type=float, default=0.0, help="Requested camera FPS. 0 keeps camera default.")
    parser.add_argument("--fourcc", default="MJPG", help="Optional pixel format, for example MJPG or YUYV.")
    parser.add_argument(
        "--role-capture-size",
        action="append",
        default=[],
        help="Per-role capture size override, role=WIDTHxHEIGHT. Repeat or separate entries with commas.",
    )
    parser.add_argument(
        "--role-fourcc",
        action="append",
        default=[],
        help="Per-role pixel format override, role=FOURCC, for example camera1=MJPG,camera2=YUYV.",
    )
    parser.add_argument("--buffer-size", type=int, default=0, help="Requested V4L2 buffer size. 0 keeps camera default.")
    parser.add_argument("--probe-timeout", type=float, default=1.5, help="Seconds to wait for a readable frame.")
    parser.add_argument("--publish-fps", type=float, default=30.0, help="ROS image publish FPS.")
    parser.add_argument("--topic", default="/camera_mosaic/image_raw", help="Output sensor_msgs/Image topic.")
    parser.add_argument("--compressed-only", action="store_true", help="Publish only CompressedImage; do not publish raw Image.")
    parser.add_argument(
        "--compressed-topic",
        default="",
        help="Optional output sensor_msgs/CompressedImage JPEG topic, e.g. /camera_mosaic/image_raw/compressed.",
    )
    parser.add_argument("--compressed-jpeg-quality", type=int, default=90, help="JPEG quality for --compressed-topic.")
    parser.add_argument("--roles", default="", help="Comma-separated camera role labels, for example left,right,top.")
    parser.add_argument(
        "--per-camera-topic-prefix",
        default="",
        help="Optional clean per-camera CompressedImage prefix, e.g. /teleop_camera.",
    )
    parser.add_argument(
        "--per-camera-width",
        type=int,
        default=0,
        help="Clean per-camera output width. 0 keeps the captured source width.",
    )
    parser.add_argument(
        "--per-camera-height",
        type=int,
        default=0,
        help="Clean per-camera output height. 0 keeps the captured source height.",
    )
    parser.add_argument(
        "--per-camera-compressed-jpeg-quality",
        type=int,
        default=90,
        help="JPEG quality for clean per-camera topics.",
    )
    parser.add_argument(
        "--preview-topic-prefix",
        default="",
        help="Optional RViz-only raw per-camera preview prefix, e.g. /teleop_camera_preview.",
    )
    parser.add_argument("--preview-width", type=int, default=0, help="RViz-only per-camera preview width. 0 uses --tile-width.")
    parser.add_argument("--preview-height", type=int, default=0, help="RViz-only per-camera preview height. 0 uses --tile-height.")
    parser.add_argument("--preview-show-stats", action="store_true", help="Draw role/FPS/resolution on preview-only topics.")
    parser.add_argument("--preview-compressed-jpeg-quality", type=int, default=90, help="JPEG quality for preview-only topics.")
    parser.add_argument("--frame-id", default="camera_mosaic", help="Image header frame_id.")
    return parser


def main() -> int:
    args, ros_args = make_parser().parse_known_args()
    devices = discover_video_devices()
    role_bound = bool(args.role_device or args.role_usb_port or args.role_serial or args.missing_role)
    if role_bound:
        selected, probes = _choose_role_devices(devices, args)
    else:
        try:
            selected_devices, probes = choose_devices(devices, args)
        except SystemExit as exc:
            if not bool(getattr(args, "allow_missing", False)):
                raise
            print(f"[camera_mosaic] WARN: {exc}; publishing blank camera roles", file=sys.stderr)
            selected_devices = []
            probes = {}
        roles = _requested_roles(args.roles)
        selected = list(selected_devices)
        if bool(getattr(args, "allow_missing", False)) and roles and len(selected) < len(roles):
            selected.extend([None] * (len(roles) - len(selected)))
    if args.list or args.probe:
        print_devices(devices, probes)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import CompressedImage, Image

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = rclpy.create_node("camera_mosaic_publisher")
    publisher = None
    try:
        publisher = CameraMosaicPublisher(rclpy, Image, CompressedImage, node, selected, args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if publisher is not None:
            publisher.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
