#!/usr/bin/env python3
"""Resolve teleop camera roles from stable camera serial numbers.

The cache stores role -> serial by default.  /dev/video* nodes are treated as
runtime-only endpoints because Linux can renumber them after reconnect/reboot.
USB-port identity is still available as an explicit compatibility mode.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from show_connected_cameras import (  # noqa: E402
    CameraDevice,
    _make_blank_tile,
    _make_tile,
    _open_capture,
    _preferred_devices_for_usb_port,
    discover_video_devices,
    import_cv2,
    print_devices,
    probe_device,
)

CACHE_VERSION = 2
DEFAULT_ROLES = ("left", "right", "top")
IDENTITY_CHOICES = ("serial", "usb-port")
MISSING_IDENTITY_PREFIX = "__missing__:"


@dataclass(frozen=True)
class CameraCandidate:
    role_index: int
    device: Any
    identity_value: str


def _cache_default_path() -> Path:
    cache_home = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return cache_home / "piper_teleop" / "camera_roles_bimanual_3cam.json"


def _role_list(raw: str) -> list[str]:
    roles = [item.strip() for item in raw.split(",") if item.strip()]
    if not roles:
        roles = list(DEFAULT_ROLES)
    if len(set(roles)) != len(roles):
        raise SystemExit(f"Duplicate camera role in --roles: {raw}")
    return roles


def _identity_field(identity: str) -> str:
    if identity == "usb-port":
        return "usb_port"
    return "serial"


def _identity_label(identity: str) -> str:
    return "USB port" if identity == "usb-port" else "serial"


def _missing_value(role: str) -> str:
    return f"{MISSING_IDENTITY_PREFIX}{role}"


def _is_missing_value(value: str) -> bool:
    return value.startswith(MISSING_IDENTITY_PREFIX)


def _device_identity(dev: CameraDevice, identity: str) -> str:
    return str(getattr(dev, _identity_field(identity))).strip()


def _physical_camera_key(dev: CameraDevice) -> str:
    if dev.usb_port != "unknown":
        return f"usb:{dev.usb_port}"
    if dev.by_id:
        return f"by-id:{dev.by_id[0]}"
    if dev.by_path:
        return f"by-path:{dev.by_path[0]}"
    return f"device:{dev.path}"


def _make_probe_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        backend=args.backend,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        probe_timeout=args.probe_timeout,
    )


def _read_cache(path: Path) -> dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    except Exception as exc:
        print(f"[camera-select] ignoring unreadable cache {path}: {exc}", file=sys.stderr)
        return {}
    if not isinstance(data, dict) or data.get("version") not in {1, CACHE_VERSION}:
        return {}
    roles = data.get("roles")
    if not isinstance(roles, dict):
        return {}
    return data


def _write_cache(path: Path, roles: Iterable[str], mapping: dict[str, str], identity: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    field = _identity_field(identity)
    payload = {
        "version": CACHE_VERSION,
        "identity": identity,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "roles_order": list(roles),
        "roles": {role: {field: mapping[role]} for role in roles},
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _mapping_from_cache(data: dict[str, object], roles: list[str], identity: str) -> dict[str, str]:
    raw_roles = data.get("roles")
    if not isinstance(raw_roles, dict):
        return {}
    if data.get("version") == 1 and identity != "usb-port":
        return {}
    if data.get("version") == CACHE_VERSION and data.get("identity") not in {None, identity}:
        return {}
    field = _identity_field(identity)
    mapping: dict[str, str] = {}
    for role in roles:
        item = raw_roles.get(role)
        if not isinstance(item, dict):
            return {}
        value = str(item.get(field) or "").strip()
        if not value:
            return {}
        mapping[role] = value
    return mapping


def _group_candidates(
    devices: list[CameraDevice],
    probes: dict[Path, object],
    allow_unknown: bool,
    identity: str,
) -> list[CameraCandidate]:
    all_by_camera: dict[str, list[CameraDevice]] = {}
    for dev in devices:
        all_by_camera.setdefault(_physical_camera_key(dev), []).append(dev)
    all_preferred_devices: list[CameraDevice] = []
    for key in sorted(all_by_camera):
        preferred = _preferred_devices_for_usb_port(all_by_camera[key])
        if preferred:
            all_preferred_devices.append(preferred[0])
    serial_counts = Counter(dev.serial for dev in all_preferred_devices if dev.serial and dev.serial != "unknown")

    by_camera: dict[str, list[CameraDevice]] = {}
    for dev in devices:
        result = probes.get(dev.path)
        if result is None or not getattr(result, "ok", False):
            continue
        by_camera.setdefault(_physical_camera_key(dev), []).append(dev)

    preferred_devices: list[CameraDevice] = []
    for key in sorted(by_camera):
        preferred = _preferred_devices_for_usb_port(by_camera[key])
        if preferred:
            preferred_devices.append(preferred[0])

    candidates: list[CameraCandidate] = []
    for dev in preferred_devices:
        value = _device_identity(dev, identity)
        if identity == "serial" and value and value != "unknown" and serial_counts[value] > 1:
            value = f"{value}@{dev.usb_port}"
        if (not value or value == "unknown") and not allow_unknown:
            continue
        candidates.append(CameraCandidate(role_index=len(candidates) + 1, device=dev, identity_value=value))
    return candidates


def _candidate_by_identity(candidates: list[CameraCandidate], identity: str) -> dict[str, CameraCandidate]:
    del identity
    return {candidate.identity_value: candidate for candidate in candidates}


def _validate_mapping(
    mapping: dict[str, str],
    roles: list[str],
    candidates: list[CameraCandidate],
    identity: str,
    allow_missing: bool = False,
) -> tuple[bool, str]:
    by_identity = _candidate_by_identity(candidates, identity)
    seen: set[str] = set()
    for role in roles:
        value = mapping.get(role, "").strip()
        if not value:
            return False, f"missing role {role}"
        if allow_missing and _is_missing_value(value):
            continue
        if value in seen:
            return False, f"{_identity_label(identity)} {value} is assigned more than once"
        if value not in by_identity:
            return False, f"cached {role} {_identity_label(identity)} is not readable now: {value}"
        seen.add(value)
    return True, ""


def _mapping_with_missing(
    mapping: dict[str, str],
    roles: list[str],
    candidates: list[CameraCandidate],
    identity: str,
) -> dict[str, str]:
    by_identity = _candidate_by_identity(candidates, identity)
    seen: set[str] = set()
    patched: dict[str, str] = {}
    for role in roles:
        value = mapping.get(role, "").strip()
        if value and value in by_identity and value not in seen:
            patched[role] = value
            seen.add(value)
        else:
            patched[role] = _missing_value(role)
    return patched


def _default_mapping_with_missing(roles: list[str], candidates: list[CameraCandidate]) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for idx, role in enumerate(roles):
        if idx < len(candidates):
            mapping[role] = candidates[idx].identity_value
        else:
            mapping[role] = _missing_value(role)
    return mapping


def _mapping_to_values(roles: list[str], mapping: dict[str, str]) -> str:
    return ",".join(mapping[role] for role in roles)


def _describe_mapping(roles: list[str], mapping: dict[str, str], candidates: list[CameraCandidate], identity: str) -> None:
    by_identity = _candidate_by_identity(candidates, identity)
    print("[camera-select] camera role mapping:", file=sys.stderr)
    for role in roles:
        value = mapping[role]
        if _is_missing_value(value):
            print(f"  {role:<5} {_identity_label(identity)}:missing", file=sys.stderr)
            continue
        candidate = by_identity.get(value)
        if candidate is None:
            print(f"  {role:<5} {_identity_label(identity)}:{value}", file=sys.stderr)
            continue
        dev = candidate.device
        print(
            f"  {role:<5} id:{candidate.identity_value:<24} serial:{dev.serial:<14} "
            f"usb:{dev.usb_port:<12} endpoint:{dev.path} iface:{dev.usb_iface} name:{dev.name}",
            file=sys.stderr,
        )


def _print_devices_stderr(devices: list[CameraDevice], probes: dict[Path, object] | None) -> None:
    with contextlib.redirect_stdout(sys.stderr):
        print_devices(devices, probes)


class _RealSensePreviewCapture:
    def __init__(self, serial: str, args: argparse.Namespace):
        import pyrealsense2 as rs  # type: ignore

        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_device(str(serial))
        width = int(getattr(args, "width", getattr(args, "camera_width", 640)))
        height = int(getattr(args, "height", getattr(args, "camera_height", 480)))
        fps = float(getattr(args, "fps", getattr(args, "camera_fps", 30.0)))
        self._config.enable_stream(
            rs.stream.color,
            width,
            height,
            rs.format.bgr8,
            int(round(fps)),
        )
        self._timeout_ms = max(1, int(getattr(args, "realsense_frame_timeout_ms", 250)))
        self._pipeline.start(self._config)
        self._opened = True

    def isOpened(self) -> bool:  # noqa: N802
        return self._opened

    def read(self) -> tuple[bool, Any]:
        import numpy as np

        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=self._timeout_ms)
            color = frames.get_color_frame()
            if not color:
                return False, None
            return True, np.asanyarray(color.get_data()).copy()
        except Exception:
            return False, None

    def release(self) -> None:
        if not self._opened:
            return
        try:
            self._pipeline.stop()
        finally:
            self._opened = False


def _is_realsense_preview_device(dev: Any) -> bool:
    return getattr(dev, "backend", "") == "realsense" or str(getattr(dev, "path", "")).startswith("realsense:")


def _realsense_serial(dev: Any) -> str:
    serial = str(getattr(dev, "realsense_serial", "") or getattr(dev, "serial", "")).strip()
    if serial:
        return serial
    raw_path = str(getattr(dev, "path", "")).strip()
    if raw_path.startswith("realsense:"):
        return raw_path.split(":", 1)[1]
    return ""


def _open_captures(cv2, candidates: list[CameraCandidate], args: argparse.Namespace):
    captures = {}
    for candidate in candidates:
        if _is_realsense_preview_device(candidate.device):
            serial = _realsense_serial(candidate.device)
            if not serial:
                continue
            try:
                cap = _RealSensePreviewCapture(serial, args)
            except Exception as exc:
                print(
                    f"[camera-select] cannot open RealSense preview {serial}: {exc}",
                    file=sys.stderr,
                )
                continue
        else:
            cap = _open_capture(cv2, candidate.device.path, args)
        if cap.isOpened():
            captures[candidate.device.path] = cap
        else:
            cap.release()
    return captures


def _draw_header(cv2, image, lines: list[str]) -> None:
    y = 26
    for line in lines:
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        y += 26


def _role_for_identity(mapping: dict[str, str], value: str) -> str:
    for role, mapped in mapping.items():
        if mapped == value:
            return role
    return ""


def _build_mosaic_frame(
    cv2,
    candidates: list[CameraCandidate],
    captures,
    args: argparse.Namespace,
    mapping: dict[str, str],
    prompt: str,
    mode: str,
):
    import numpy as np

    tile_w = max(160, int(args.tile_width))
    tile_h = max(120, int(args.tile_height))
    cols = max(1, min(int(args.cols), len(candidates))) if args.cols > 0 else max(1, math.ceil(math.sqrt(len(candidates))))
    rows = max(1, math.ceil(len(candidates) / cols))
    tiles = []

    for candidate in candidates:
        dev = candidate.device
        cap = captures.get(dev.path)
        frame = None
        if cap is not None:
            ok, read_frame = cap.read()
            if ok and read_frame is not None:
                frame = read_frame
        if frame is None:
            tile = _make_blank_tile(cv2, dev, tile_w, tile_h, "waiting for frame")
        else:
            tile = _make_tile(cv2, frame, dev, 0.0, tile_w, tile_h)
        role = _role_for_identity(mapping, candidate.identity_value)
        index_label = f"[{candidate.role_index}]"
        if role:
            index_label = f"{index_label} {role.upper()}"
        cv2.putText(tile, index_label, (12, tile_h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(tile, index_label, (12, tile_h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2, cv2.LINE_AA)
        tiles.append(tile)

    blank = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    while len(tiles) < rows * cols:
        tiles.append(blank.copy())
    row_images = [cv2.hconcat(tiles[r * cols : (r + 1) * cols]) for r in range(rows)]
    mosaic = cv2.vconcat(row_images)
    header_h = 88
    canvas = np.zeros((mosaic.shape[0] + header_h, mosaic.shape[1], 3), dtype=np.uint8)
    canvas[header_h:, :] = mosaic
    if mode == "select":
        if getattr(args, "allow_missing", False):
            lines = [prompt, "Click/number selects next role. Enter accepts selected cameras. r resets, q cancels."]
        else:
            lines = [prompt, "Click a tile or press number. Enter accepts after all roles. r resets, q cancels."]
    else:
        lines = [prompt, "Enter/y accepts cached mapping. r reselects, q cancels."]
    _draw_header(cv2, canvas, lines)
    return canvas, cols, tile_w, tile_h, header_h


def _select_with_window(
    candidates: list[CameraCandidate],
    roles: list[str],
    args: argparse.Namespace,
    initial_mapping: dict[str, str] | None = None,
    confirm_only: bool = False,
) -> tuple[str, dict[str, str] | None]:
    cv2 = import_cv2()
    window = "teleop camera role selector"
    captures = _open_captures(cv2, candidates, args)
    if not captures:
        raise SystemExit("No selected camera endpoint could be opened for preview.")

    mapping = dict(initial_mapping or {})
    current_role_index = 0
    if confirm_only:
        current_role_index = len(roles)
    clicked_index = {"value": 0}

    def on_mouse(event, x, y, flags, param):  # noqa: ANN001
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        frame_info = clicked_index.get("frame_info")
        if not frame_info:
            return
        cols, tile_w, tile_h, header_h = frame_info
        if y < header_h:
            return
        col = min(cols - 1, max(0, x // tile_w))
        row = max(0, (y - header_h) // tile_h)
        idx = row * cols + col + 1
        clicked_index["value"] = int(idx)

    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window, on_mouse)
        while True:
            mode = "confirm" if confirm_only else "select"
            if confirm_only:
                prompt = "Cached teleop camera roles found."
            elif current_role_index < len(roles):
                prompt = f"Select {roles[current_role_index]} camera."
            else:
                prompt = "All camera roles selected."
            frame, cols, tile_w, tile_h, header_h = _build_mosaic_frame(
                cv2, candidates, captures, args, mapping, prompt, mode
            )
            clicked_index["frame_info"] = (cols, tile_w, tile_h, header_h)
            cv2.imshow(window, frame)
            key = cv2.waitKey(30) & 0xFF

            selected_idx = 0
            if clicked_index["value"]:
                selected_idx = int(clicked_index["value"])
                clicked_index["value"] = 0
            elif ord("1") <= key <= ord("9"):
                selected_idx = key - ord("0")

            if selected_idx and 1 <= selected_idx <= len(candidates):
                if confirm_only:
                    confirm_only = False
                    mapping = {}
                    current_role_index = 0
                if current_role_index < len(roles):
                    role = roles[current_role_index]
                    value = candidates[selected_idx - 1].identity_value
                    if value in mapping.values():
                        print(f"[camera-select] {_identity_label(args.identity)} already selected: {value}", file=sys.stderr)
                    else:
                        mapping[role] = value
                        current_role_index += 1
                continue

            if key in (27, ord("q")):
                return "abort", None
            if key == ord("r"):
                confirm_only = False
                mapping = {}
                current_role_index = 0
                continue
            if key in (13, 10, ord("y")):
                if args.allow_missing:
                    for role in roles:
                        mapping.setdefault(role, _missing_value(role))
                valid, reason = _validate_mapping(mapping, roles, candidates, args.identity, allow_missing=args.allow_missing)
                if valid:
                    return "accept", mapping
                print(f"[camera-select] cannot accept yet: {reason}", file=sys.stderr)
    finally:
        for cap in captures.values():
            cap.release()
        cv2.destroyWindow(window)


def _select_with_terminal(
    candidates: list[CameraCandidate],
    roles: list[str],
    identity: str,
    initial_mapping: dict[str, str] | None = None,
    confirm_only: bool = False,
    allow_missing: bool = False,
) -> tuple[str, dict[str, str] | None]:
    def ask(prompt: str) -> str:
        print(prompt, end="", file=sys.stderr, flush=True)
        return input().strip()

    if confirm_only and initial_mapping:
        _describe_mapping(roles, initial_mapping, candidates, identity)
        raw = ask("Use cached camera mapping? [Y/r/q]: ").lower()
        if raw in {"", "y", "yes"}:
            return "accept", dict(initial_mapping)
        if raw in {"q", "quit", "n", "no"}:
            return "abort", None

    print(f"[camera-select] readable camera {_identity_label(identity)} values:", file=sys.stderr)
    for candidate in candidates:
        dev = candidate.device
        print(
            f"  {candidate.role_index}. id:{candidate.identity_value:<24} serial:{dev.serial:<14} usb:{dev.usb_port:<12} "
            f"endpoint:{dev.path} iface:{dev.usb_iface} name:{dev.name}",
            file=sys.stderr,
        )

    mapping: dict[str, str] = {}
    for role in roles:
        while True:
            suffix = " (Enter skips)" if allow_missing else ""
            raw = ask(f"Select {role} camera index{suffix}: ")
            if raw.lower() in {"q", "quit", "cancel"}:
                return "abort", None
            if not raw and allow_missing:
                mapping[role] = _missing_value(role)
                break
            if raw.isdigit() and 1 <= int(raw) <= len(candidates):
                value = candidates[int(raw) - 1].identity_value
                if value in mapping.values():
                    print(f"{_identity_label(identity)} already selected: {value}", file=sys.stderr)
                    continue
                mapping[role] = value
                break
            print(f"Invalid camera index: {raw}", file=sys.stderr)
    return "accept", mapping


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Select left/right/top teleop cameras by serial number.")
    parser.add_argument("--roles", default=",".join(DEFAULT_ROLES), help="Comma-separated role names in mosaic order.")
    parser.add_argument(
        "--identity",
        choices=IDENTITY_CHOICES,
        default="serial",
        help="Stable camera identity to cache and return. Default: serial.",
    )
    parser.add_argument("--cache-file", default=str(_cache_default_path()), help="Role cache JSON path.")
    parser.add_argument("--force-select", action="store_true", help="Ignore valid cache and ask for a fresh selection.")
    parser.add_argument("--non-interactive", action="store_true", help="Do not open prompts or preview windows.")
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="Return missing placeholders for unreadable roles instead of failing selection.",
    )
    parser.add_argument(
        "--allow-default-selection",
        action="store_true",
        help="For smoke tests: select the first N camera identities if no cache exists.",
    )
    parser.add_argument("--no-window", action="store_true", help="Use terminal prompts instead of the OpenCV selection window.")
    parser.add_argument(
        "--allow-unknown-usb-port",
        action="store_true",
        help="Compatibility option: allow unknown values for the selected identity.",
    )
    parser.add_argument("--print-devices", action="store_true", help="Print probed /dev/video* table to stderr.")
    parser.add_argument("--cols", type=int, default=3, help="Preview columns.")
    parser.add_argument("--tile-width", type=int, default=640, help="Preview tile width.")
    parser.add_argument("--tile-height", type=int, default=480, help="Preview tile height.")
    parser.add_argument("--backend", choices=("v4l2", "any"), default="v4l2", help="OpenCV capture backend.")
    parser.add_argument("--width", type=int, default=640, help="Requested capture width.")
    parser.add_argument("--height", type=int, default=480, help="Requested capture height.")
    parser.add_argument("--fps", type=float, default=30.0, help="Requested capture FPS.")
    parser.add_argument("--fourcc", default="MJPG", help="Optional pixel format, for example MJPG or YUYV.")
    parser.add_argument("--probe-timeout", type=float, default=1.5, help="Seconds to wait for a readable frame.")
    return parser


def main() -> int:
    args = make_parser().parse_args()
    roles = _role_list(args.roles)
    cache_file = Path(args.cache_file).expanduser()

    devices = discover_video_devices()
    if not devices:
        print("[camera-select] no /dev/video* devices found", file=sys.stderr)
        if args.allow_missing:
            mapping = {role: _missing_value(role) for role in roles}
            _describe_mapping(roles, mapping, [], args.identity)
            print(_mapping_to_values(roles, mapping))
            return 0
        return 1

    cv2 = import_cv2()
    probe_args = _make_probe_args(args)
    probes = {dev.path: probe_device(cv2, dev.path, probe_args) for dev in devices}
    candidates = _group_candidates(devices, probes, args.allow_unknown_usb_port, args.identity)
    if args.print_devices:
        _print_devices_stderr(devices, probes)
    if len(candidates) < len(roles):
        _print_devices_stderr(devices, probes)
        print(
            f"[camera-select] need {len(roles)} readable camera {_identity_label(args.identity)} values, "
            f"found {len(candidates)}",
            file=sys.stderr,
        )
        if not args.allow_missing:
            return 2

    cached_mapping = _mapping_from_cache(_read_cache(cache_file), roles, args.identity)
    cache_valid = False
    if cached_mapping:
        cache_valid, reason = _validate_mapping(cached_mapping, roles, candidates, args.identity)
        if not cache_valid:
            print(f"[camera-select] cache invalid: {reason}", file=sys.stderr)

    mapping: dict[str, str] | None = None
    if cache_valid and args.non_interactive and not args.force_select:
        mapping = cached_mapping
        _describe_mapping(roles, mapping, candidates, args.identity)
    elif args.allow_missing and cached_mapping and args.non_interactive and not args.force_select:
        mapping = _mapping_with_missing(cached_mapping, roles, candidates, args.identity)
        _describe_mapping(roles, mapping, candidates, args.identity)
    elif args.allow_missing and len(candidates) < len(roles) and args.non_interactive:
        mapping = _default_mapping_with_missing(roles, candidates)
        _describe_mapping(roles, mapping, candidates, args.identity)
    elif args.non_interactive:
        if args.allow_default_selection:
            if args.allow_missing:
                mapping = _default_mapping_with_missing(roles, candidates)
            else:
                mapping = {role: candidates[idx].identity_value for idx, role in enumerate(roles)}
            _write_cache(cache_file, roles, mapping, args.identity)
            _describe_mapping(roles, mapping, candidates, args.identity)
        elif args.allow_missing:
            mapping = _default_mapping_with_missing(roles, candidates)
            _describe_mapping(roles, mapping, candidates, args.identity)
        else:
            print("[camera-select] no valid cache and non-interactive selection is disabled", file=sys.stderr)
            return 3
    else:
        use_window = not args.no_window and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if use_window:
            try:
                status, mapping = _select_with_window(
                    candidates,
                    roles,
                    args,
                    initial_mapping=cached_mapping if cache_valid else None,
                    confirm_only=cache_valid and not args.force_select,
                )
            except Exception as exc:
                print(
                    f"[camera-select] window selector unavailable ({exc}); falling back to terminal selection.",
                    file=sys.stderr,
                )
                status, mapping = _select_with_terminal(
                    candidates,
                    roles,
                    args.identity,
                    initial_mapping=cached_mapping if cache_valid else None,
                    confirm_only=cache_valid and not args.force_select,
                    allow_missing=args.allow_missing,
                )
        else:
            status, mapping = _select_with_terminal(
                candidates,
                roles,
                args.identity,
                initial_mapping=cached_mapping if cache_valid else None,
                confirm_only=cache_valid and not args.force_select,
                allow_missing=args.allow_missing,
            )
        if status != "accept" or mapping is None:
            print("[camera-select] camera selection cancelled", file=sys.stderr)
            return 4
        _write_cache(cache_file, roles, mapping, args.identity)
        _describe_mapping(roles, mapping, candidates, args.identity)

    valid, reason = _validate_mapping(mapping, roles, candidates, args.identity, allow_missing=args.allow_missing)
    if not valid:
        print(f"[camera-select] invalid mapping: {reason}", file=sys.stderr)
        return 5

    print(_mapping_to_values(roles, mapping))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
