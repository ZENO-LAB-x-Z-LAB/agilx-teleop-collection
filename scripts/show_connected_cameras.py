#!/usr/bin/env python3
"""List local V4L2 cameras and show live previews in one mosaic window.

Usage:
  python3 scripts/show_connected_cameras.py
  python3 scripts/show_connected_cameras.py --list --probe
  python3 scripts/show_connected_cameras.py --device /dev/video0
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class CameraDevice:
    path: Path
    index: int
    name: str
    serial: str
    usb_serial: str
    realsense_serial: str
    usb_port: str
    usb_iface: str
    by_id: tuple[str, ...]
    by_path: tuple[str, ...]


@dataclass(frozen=True)
class ProbeResult:
    ok: bool
    width: int = 0
    height: int = 0
    error: str = ""


def _natural_video_key(path: Path) -> int:
    match = re.fullmatch(r"video(\d+)", path.name)
    if not match:
        return 10**9
    return int(match.group(1))


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def _aliases_for(device: Path, alias_dir: Path) -> tuple[str, ...]:
    if not alias_dir.is_dir():
        return ()
    aliases: list[str] = []
    real_device = device.resolve()
    for item in sorted(alias_dir.iterdir(), key=lambda p: p.name):
        if not item.is_symlink():
            continue
        try:
            if item.resolve() == real_device:
                aliases.append(str(item))
        except OSError:
            continue
    return tuple(aliases)


def _detect_usb_info(dev_name: str) -> tuple[str, str]:
    sys_video = Path("/sys/class/video4linux") / dev_name
    try:
        sys_path = str(sys_video.resolve())
    except OSError:
        sys_path = ""

    match = re.search(r"([0-9]+-[0-9.]+):([0-9.]+)", sys_path)
    if match:
        return match.group(1), match.group(2)

    parent_dev = sys_video / "device"
    try:
        parent_path = str(parent_dev.resolve())
    except OSError:
        parent_path = ""
    match = re.search(r"([0-9]+-[0-9.]+):([0-9.]+)", parent_path)
    if match:
        return match.group(1), match.group(2)

    return "unknown", "unknown"


def _detect_usb_serial(dev_name: str, by_id: tuple[str, ...]) -> str:
    for alias in by_id:
        name = Path(alias).name
        prefix = re.sub(r"-video-index\d+$", "", name)
        tail = prefix.rsplit("_", 1)[-1]
        if re.fullmatch(r"[A-Za-z0-9.:-]{4,}", tail):
            return tail

    sys_video = Path("/sys/class/video4linux") / dev_name
    try:
        current = (sys_video / "device").resolve()
    except OSError:
        return "unknown"

    for path in (current, *current.parents):
        if path == Path("/sys"):
            break
        serial = _read_text(path / "serial")
        if serial:
            return serial
    return "unknown"


def _discover_realsense_serials_by_port() -> dict[str, str]:
    try:
        import pyrealsense2 as rs  # type: ignore
    except Exception:
        return {}

    serial_by_port: dict[str, str] = {}
    try:
        devices = rs.context().query_devices()
    except Exception:
        return serial_by_port

    for dev in devices:
        try:
            physical_port = dev.get_info(rs.camera_info.physical_port)
            serial_number = dev.get_info(rs.camera_info.serial_number)
        except Exception:
            continue
        usb_port = ""
        for part in str(physical_port).split("/"):
            if re.fullmatch(r"[0-9]+-[0-9.]+", part):
                usb_port = part
        if usb_port and serial_number:
            serial_by_port[usb_port] = serial_number
    return serial_by_port


def discover_video_devices() -> list[CameraDevice]:
    devices: list[CameraDevice] = []
    rs_serial_by_port = _discover_realsense_serials_by_port()
    for path in sorted(Path("/dev").glob("video*"), key=_natural_video_key):
        match = re.fullmatch(r"video(\d+)", path.name)
        if not match:
            continue
        index = int(match.group(1))
        name = _read_text(Path("/sys/class/video4linux") / path.name / "name")
        usb_port, usb_iface = _detect_usb_info(path.name)
        by_id = _aliases_for(path, Path("/dev/v4l/by-id"))
        by_path = _aliases_for(path, Path("/dev/v4l/by-path"))
        usb_serial = _detect_usb_serial(path.name, by_id)
        realsense_serial = rs_serial_by_port.get(usb_port, "")
        serial = realsense_serial or usb_serial
        if not serial:
            serial = "unknown"
        devices.append(
            CameraDevice(
                path=path,
                index=index,
                name=name or "unknown",
                serial=serial,
                usb_serial=usb_serial,
                realsense_serial=realsense_serial or "unknown",
                usb_port=usb_port,
                usb_iface=usb_iface,
                by_id=by_id,
                by_path=by_path,
            )
        )
    return devices


def import_cv2():
    try:
        import cv2  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "OpenCV is required for preview/probing.\n"
            "Install it with one of:\n"
            "  sudo apt install -y python3-opencv\n"
            "  python3 -m pip install opencv-python\n"
            f"Import error: {exc}"
        ) from exc
    return cv2


def _backend_flag(cv2, backend: str) -> int:
    if backend == "any":
        return 0
    return int(getattr(cv2, "CAP_V4L2", 200))


def _open_capture(cv2, device: Path, args: argparse.Namespace):
    cap = cv2.VideoCapture(str(device), _backend_flag(cv2, args.backend))
    if args.fourcc:
        fourcc = str(args.fourcc).strip().upper()
        if len(fourcc) != 4:
            raise SystemExit("--fourcc must be exactly 4 characters, for example MJPG or YUYV.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))
    if args.fps > 0:
        cap.set(cv2.CAP_PROP_FPS, float(args.fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def _probe_device_direct(cv2, device: Path, args: argparse.Namespace) -> ProbeResult:
    cap = _open_capture(cv2, device, args)
    if not cap.isOpened():
        cap.release()
        return ProbeResult(ok=False, error="open failed")

    deadline = time.monotonic() + float(args.probe_timeout)
    last_error = "no frame"
    while time.monotonic() < deadline:
        ok, frame = cap.read()
        if ok and frame is not None:
            height, width = frame.shape[:2]
            cap.release()
            return ProbeResult(ok=True, width=int(width), height=int(height))
        last_error = "read failed"
        time.sleep(0.03)

    cap.release()
    return ProbeResult(ok=False, error=last_error)


def _probe_device_worker(device: str, args_dict: dict[str, object], queue) -> None:  # noqa: ANN001
    try:
        cv2 = import_cv2()
        result = _probe_device_direct(cv2, Path(device), argparse.Namespace(**args_dict))
        queue.put(
            {
                "ok": result.ok,
                "width": result.width,
                "height": result.height,
                "error": result.error,
            }
        )
    except BaseException as exc:
        queue.put({"ok": False, "width": 0, "height": 0, "error": str(exc) or exc.__class__.__name__})


def probe_device(cv2, device: Path, args: argparse.Namespace) -> ProbeResult:
    timeout = max(0.1, float(args.probe_timeout))
    if os.environ.get("CAMERA_PROBE_IN_PROCESS"):
        return _probe_device_direct(cv2, device, args)

    args_dict = {
        "backend": args.backend,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "fourcc": args.fourcc,
        "probe_timeout": args.probe_timeout,
    }
    try:
        ctx = mp.get_context("fork")
    except ValueError:
        ctx = mp.get_context()
    queue = ctx.Queue(maxsize=1)
    proc = ctx.Process(target=_probe_device_worker, args=(str(device), args_dict, queue))
    proc.start()
    proc.join(timeout + 0.35)
    if proc.is_alive():
        proc.terminate()
        proc.join(timeout=0.2)
        if proc.is_alive():
            proc.kill()
            proc.join(timeout=0.2)
        return ProbeResult(ok=False, error="probe timeout")

    if proc.exitcode not in (0, None) and queue.empty():
        return ProbeResult(ok=False, error=f"probe exited {proc.exitcode}")
    try:
        payload = queue.get_nowait()
    except Exception:
        return ProbeResult(ok=False, error="no probe result")
    return ProbeResult(
        ok=bool(payload.get("ok")),
        width=int(payload.get("width") or 0),
        height=int(payload.get("height") or 0),
        error=str(payload.get("error") or ""),
    )


def _short_aliases(items: Iterable[str]) -> str:
    names = [Path(x).name for x in items]
    return ", ".join(names) if names else "-"


def print_devices(devices: list[CameraDevice], probes: dict[Path, ProbeResult] | None = None) -> None:
    if not devices:
        print("No /dev/video* devices found.")
        return

    print("Detected video devices:")
    print(f"{'DEVICE':<12} {'STATUS':<16} {'SERIAL':<14} {'USB_PORT':<12} {'IFACE':<8} {'NAME':<34} {'BY_ID'}")
    print(
        f"{'-' * 12:<12} {'-' * 16:<16} {'-' * 14:<14} {'-' * 12:<12} "
        f"{'-' * 8:<8} {'-' * 34:<34} {'-' * 5}"
    )
    for dev in devices:
        status = "not probed"
        if probes is not None and dev.path in probes:
            result = probes[dev.path]
            if result.ok:
                status = f"ok {result.width}x{result.height}"
            else:
                status = f"skip {result.error}"
        print(
            f"{str(dev.path):<12} {status:<16} {dev.serial:<14} {dev.usb_port:<12} {dev.usb_iface:<8} "
            f"{dev.name[:34]:<34} {_short_aliases(dev.by_id)}"
        )


def resolve_device_arg(value: str) -> Path:
    raw = value.strip()
    if re.fullmatch(r"\d+", raw):
        return Path(f"/dev/video{raw}")
    return Path(raw)


def _device_group_key(dev: CameraDevice) -> str:
    if dev.serial != "unknown":
        return f"serial:{dev.serial}"
    if dev.usb_port != "unknown":
        return dev.usb_port
    if dev.by_id:
        return dev.by_id[0]
    if dev.by_path:
        return dev.by_path[0]
    return str(dev.path)


def _alias_video_index(dev: CameraDevice) -> int:
    indexes: list[int] = []
    for alias in (*dev.by_path, *dev.by_id):
        match = re.search(r"video-index(\d+)", Path(alias).name)
        if match:
            indexes.append(int(match.group(1)))
    return min(indexes) if indexes else 999


def _iface_sort_key(usb_iface: str) -> tuple[int, ...]:
    values: list[int] = []
    for part in str(usb_iface).split("."):
        try:
            values.append(int(part))
        except ValueError:
            values.append(-1)
    return tuple(values or [-1])


def _preferred_video_index_rank(dev: CameraDevice) -> int:
    alias_index = _alias_video_index(dev)
    aliases = " ".join((*dev.by_id, *dev.by_path)).lower()
    name = str(dev.name).lower()
    is_d405 = "realsense" in aliases and ("_405_" in aliases or " 405" in name)
    if is_d405:
        return {4: 0, 2: 1, 0: 2}.get(alias_index, 10 + alias_index)
    return alias_index


def _preferred_devices_for_usb_port(devices: list[CameraDevice]) -> list[CameraDevice]:
    """Prefer the RGB-like endpoint for one physical USB camera."""
    return sorted(
        devices,
        key=lambda dev: (
            _preferred_video_index_rank(dev),
            tuple(-x for x in _iface_sort_key(dev.usb_iface)),
            dev.index,
        ),
    )


def _find_device(devices: list[CameraDevice], path: Path) -> CameraDevice:
    resolved = path.resolve()
    for dev in devices:
        try:
            if dev.path.resolve() == resolved:
                return dev
        except OSError:
            continue
    match = re.fullmatch(r"video(\d+)", path.name)
    index = int(match.group(1)) if match else _natural_video_key(path)
    name = _read_text(Path("/sys/class/video4linux") / path.name / "name")
    usb_port, usb_iface = _detect_usb_info(path.name)
    by_id = _aliases_for(path, Path("/dev/v4l/by-id"))
    by_path = _aliases_for(path, Path("/dev/v4l/by-path"))
    usb_serial = _detect_usb_serial(path.name, by_id)
    realsense_serial = _discover_realsense_serials_by_port().get(usb_port, "")
    serial = realsense_serial or usb_serial or "unknown"
    return CameraDevice(
        path=path,
        index=index,
        name=name or "unknown",
        serial=serial,
        usb_serial=usb_serial,
        realsense_serial=realsense_serial or "unknown",
        usb_port=usb_port,
        usb_iface=usb_iface,
        by_id=by_id,
        by_path=by_path,
    )


def _serial_matches(dev: CameraDevice, requested: str) -> bool:
    if "@" in requested:
        serial, usb_port = requested.rsplit("@", 1)
        return usb_port == dev.usb_port and serial in {dev.serial, dev.usb_serial, dev.realsense_serial}
    values = {dev.serial, dev.usb_serial, dev.realsense_serial}
    return requested in values


def choose_devices(devices: list[CameraDevice], args: argparse.Namespace) -> tuple[list[CameraDevice], dict[Path, ProbeResult]]:
    cv2 = import_cv2()
    probes: dict[Path, ProbeResult] = {}

    requested_usb_ports = [str(port).strip() for port in getattr(args, "usb_port", []) if str(port).strip()]
    requested_serials = [str(serial).strip() for serial in getattr(args, "serial", []) if str(serial).strip()]
    if args.device or requested_usb_ports or requested_serials:
        selected: list[CameraDevice] = []
        seen_paths: set[Path] = set()

        def add_device(dev: CameraDevice) -> None:
            try:
                key = dev.path.resolve()
            except OSError:
                key = dev.path
            if key in seen_paths:
                return
            selected.append(dev)
            seen_paths.add(key)

        for raw in args.device:
            path = resolve_device_arg(raw)
            if not path.exists():
                raise SystemExit(f"Requested camera does not exist: {path}")
            dev = _find_device(devices, path)
            result = probe_device(cv2, dev.path, args)
            probes[dev.path] = result
            if not result.ok:
                raise SystemExit(f"Requested camera is not readable: {dev.path} ({result.error})")
            add_device(dev)

        for usb_port in requested_usb_ports:
            matches = [dev for dev in devices if dev.usb_port == usb_port]
            if not matches:
                print_devices(devices, probes)
                raise SystemExit(f"Requested USB port was not found: {usb_port}")

            readable: list[CameraDevice] = []
            for dev in matches:
                result = probes.get(dev.path)
                if result is None:
                    result = probe_device(cv2, dev.path, args)
                    probes[dev.path] = result
                if result.ok:
                    readable.append(dev)

            if not readable:
                print_devices(devices, probes)
                raise SystemExit(f"Requested USB port has no readable camera: {usb_port}")

            preferred = readable if args.all_nodes else _preferred_devices_for_usb_port(readable)[:1]
            for dev in preferred:
                add_device(dev)

        for serial in requested_serials:
            matches = [dev for dev in devices if _serial_matches(dev, serial)]
            if not matches:
                print_devices(devices, probes)
                raise SystemExit(f"Requested camera serial was not found: {serial}")

            readable: list[CameraDevice] = []
            for dev in matches:
                result = probes.get(dev.path)
                if result is None:
                    result = probe_device(cv2, dev.path, args)
                    probes[dev.path] = result
                if result.ok:
                    readable.append(dev)

            if not readable:
                print_devices(devices, probes)
                raise SystemExit(f"Requested camera serial has no readable endpoint: {serial}")

            preferred = readable if args.all_nodes else _preferred_devices_for_usb_port(readable)[:1]
            for dev in preferred:
                add_device(dev)

        return selected, probes

    selected_by_group: dict[str, list[CameraDevice]] = {}
    for dev in devices:
        result = probe_device(cv2, dev.path, args)
        probes[dev.path] = result
        if not result.ok:
            continue
        if args.all_nodes:
            selected_by_group[str(dev.path)] = [dev]
        else:
            selected_by_group.setdefault(_device_group_key(dev), []).append(dev)

    selected = []
    for grouped in selected_by_group.values():
        selected.extend(_preferred_devices_for_usb_port(grouped)[:1])
    selected = sorted(selected, key=lambda d: d.index)
    if not selected:
        print_devices(devices, probes)
        raise SystemExit("No readable camera was found.")
    return selected, probes


def _camera_label(dev: CameraDevice, frame, fps: float, index: int | None = None, role: str = "") -> None:
    import cv2  # type: ignore

    lines = [
        f"[{index}] {role}".strip() if index is not None else "",
        f"{dev.path}  serial:{dev.serial}",
        f"usb:{dev.usb_port} iface:{dev.usb_iface}",
        f"{frame.shape[1]}x{frame.shape[0]}  {fps:.1f} FPS",
    ]
    lines = [line for line in lines if line]
    y = 24
    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
        y += 26


def _make_tile(
    cv2,
    frame,
    dev: CameraDevice,
    fps: float,
    tile_w: int,
    tile_h: int,
    index: int | None = None,
    role: str = "",
):
    import numpy as np

    frame = frame.copy()
    _camera_label(dev, frame, fps, index=index, role=role)
    h, w = frame.shape[:2]
    scale = min(tile_w / max(w, 1), tile_h / max(h, 1))
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    tile = np.zeros((tile_h, tile_w, 3), dtype=resized.dtype)
    x0 = (tile_w - new_w) // 2
    y0 = (tile_h - new_h) // 2
    tile[y0 : y0 + new_h, x0 : x0 + new_w] = resized
    return tile


def _make_blank_tile(
    cv2,
    dev: CameraDevice,
    tile_w: int,
    tile_h: int,
    text: str,
    index: int | None = None,
    role: str = "",
):
    import numpy as np

    tile = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    lines = [
        f"[{index}] {role}".strip() if index is not None else "",
        f"{dev.path}  serial:{dev.serial}",
        f"usb:{dev.usb_port}",
        text,
    ]
    lines = [line for line in lines if line]
    y = 32
    for line in lines:
        cv2.putText(tile, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        y += 30
    return tile


def _has_green_artifact(frame) -> bool:
    """Detect V4L2 startup corruption where part of the frame is solid green."""
    if frame is None or len(getattr(frame, "shape", ())) < 3 or frame.shape[2] < 3:
        return False
    h = frame.shape[0]
    if h < 10:
        return False
    bottom = frame[h // 2 :, :, :]
    greenish = (bottom[:, :, 1] > 170) & (bottom[:, :, 0] < 90) & (bottom[:, :, 2] < 90)
    return float(greenish.mean()) > 0.25


def _opencv_highgui_available(cv2) -> bool:
    try:
        info = cv2.getBuildInformation()
    except Exception:
        return False
    for line in info.splitlines():
        if line.strip().startswith("GUI:") and "NONE" in line.upper():
            return False
    return True


def _read_mosaic_frame(cv2, devices: list[CameraDevice], captures: dict[Path, object], state: dict, args: argparse.Namespace):
    import numpy as np

    cols = state["cols"]
    rows = state["rows"]
    tile_w = state["tile_w"]
    tile_h = state["tile_h"]
    frame_counts = state["frame_counts"]
    fps_values = state["fps_values"]
    fps_t0 = state["fps_t0"]
    last_frames = state["last_frames"]
    roles = state.get("selection_roles", {})

    tiles = []
    now = time.monotonic()
    for index, dev in enumerate(devices):
        cap = captures.get(dev.path)
        frame = None
        if cap is not None:
            ok, read_frame = cap.read()
            if ok and read_frame is not None and not _has_green_artifact(read_frame):
                frame = read_frame
                last_frames[dev.path] = read_frame
                frame_counts[dev.path] += 1
                elapsed = now - fps_t0[dev.path]
                if elapsed >= 1.0:
                    fps_values[dev.path] = frame_counts[dev.path] / elapsed
                    frame_counts[dev.path] = 0
                    fps_t0[dev.path] = now
        if frame is None:
            frame = last_frames.get(dev.path)
        role = roles.get(index, "")
        if frame is None:
            tile = _make_blank_tile(cv2, dev, tile_w, tile_h, "waiting for frame", index, role=role)
        else:
            tile = _make_tile(cv2, frame, dev, fps_values[dev.path], tile_w, tile_h, index, role=role)
        tiles.append(tile)

    blank = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    while len(tiles) < rows * cols:
        tiles.append(blank.copy())
    row_images = [cv2.hconcat(tiles[r * cols : (r + 1) * cols]) for r in range(rows)]
    return cv2.vconcat(row_images)


def _open_preview_captures(cv2, devices: list[CameraDevice], args: argparse.Namespace):
    captures: dict[Path, object] = {}
    for dev in devices:
        cap = _open_capture(cv2, dev.path, args)
        if cap.isOpened():
            captures[dev.path] = cap
            for _ in range(8):
                ok, frame = cap.read()
                if ok and frame is not None and not _has_green_artifact(frame):
                    break
                time.sleep(0.02)
        else:
            cap.release()
    if not captures:
        raise SystemExit("Could not open any readable camera for preview.")
    return captures


def _preview_state(devices: list[CameraDevice], args: argparse.Namespace) -> dict:
    cols = max(1, min(int(args.cols), len(devices))) if args.cols > 0 else max(1, math.ceil(math.sqrt(len(devices))))
    rows = max(1, math.ceil(len(devices) / cols))
    tile_w = max(160, int(args.tile_width))
    tile_h = max(120, int(args.tile_height))
    return {
        "cols": cols,
        "rows": rows,
        "tile_w": tile_w,
        "tile_h": tile_h,
        "frame_counts": {dev.path: 0 for dev in devices},
        "fps_values": {dev.path: 0.0 for dev in devices},
        "fps_t0": {dev.path: time.monotonic() for dev in devices},
        "last_frames": {},
        "selection_roles": {},
    }


def _tile_index_from_xy(x: int, y: int, state: dict, devices: list[CameraDevice]) -> int | None:
    col = int(x) // int(state["tile_w"])
    row = int(y) // int(state["tile_h"])
    if col < 0 or row < 0 or col >= int(state["cols"]) or row >= int(state["rows"]):
        return None
    idx = row * int(state["cols"]) + col
    return idx if 0 <= idx < len(devices) else None


def _write_pair_selection(path: str, devices: list[CameraDevice], left_idx: int, right_idx: int) -> None:
    payload = {
        "schema": "camera_pair_selection_v1",
        "left_index": int(left_idx),
        "right_index": int(right_idx),
        "left_device": str(devices[left_idx].path),
        "right_device": str(devices[right_idx].path),
        "devices": [
            {
                "index": idx,
                "path": str(dev.path),
                "usb_port": dev.usb_port,
                "name": dev.name,
            }
            for idx, dev in enumerate(devices)
        ],
    }
    Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _record_pair_click(args: argparse.Namespace, devices: list[CameraDevice], state: dict, idx: int) -> bool:
    if not getattr(args, "select_pair_output", ""):
        return False
    selected = state.setdefault("selected_pair", [])
    roles = state.setdefault("selection_roles", {})
    if not selected:
        selected.append(idx)
        roles[idx] = "LEFT"
        print(f"[select] LEFT = [{idx}] {devices[idx].path}")
        return False
    if idx == selected[0]:
        print("[select] RIGHT must be a different camera.")
        return False
    selected.append(idx)
    roles[idx] = "RIGHT"
    _write_pair_selection(args.select_pair_output, devices, selected[0], selected[1])
    print(f"[select] RIGHT = [{idx}] {devices[idx].path}")
    print("[select] pair saved; closing preview.")
    return True


def preview_mosaic_tk(cv2, devices: list[CameraDevice], args: argparse.Namespace) -> None:
    try:
        import tkinter as tk
        from PIL import Image, ImageTk
        root = tk.Tk()
    except Exception as exc:
        print(f"Tk/PIL preview is not available: {exc}")
        save_preview_snapshot(cv2, devices, args)
        return

    captures = _open_preview_captures(cv2, devices, args)
    state = _preview_state(devices, args)
    shown = ", ".join(f"{dev.path}(usb:{dev.usb_port})" for dev in devices)
    print(f"Previewing {len(devices)} camera(s) with Tk/PIL: {shown}")
    print("Press q or Esc to quit, s to save the whole mosaic frame.")
    if getattr(args, "select_pair_output", ""):
        print("Click one tile for LEFT, then another tile for RIGHT.")

    root.title(f"camera mosaic - {len(devices)} camera(s)")
    label = tk.Label(root)
    label.pack()
    last_mosaic = {"frame": None}

    def save_current() -> None:
        mosaic = last_mosaic.get("frame")
        if mosaic is None:
            return
        out = Path(f"camera_mosaic_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
        cv2.imwrite(str(out), mosaic)
        print(f"Saved {out}")

    def close() -> None:
        for cap in captures.values():
            cap.release()
        root.destroy()

    def on_key(event) -> None:
        key = str(event.keysym).lower()
        if key in {"q", "escape"}:
            close()
        elif key == "s":
            save_current()

    def on_click(event) -> None:
        idx = _tile_index_from_xy(int(event.x), int(event.y), state, devices)
        if idx is None:
            return
        if _record_pair_click(args, devices, state, idx):
            close()

    def update() -> None:
        try:
            mosaic = _read_mosaic_frame(cv2, devices, captures, state, args)
            last_mosaic["frame"] = mosaic
            rgb = cv2.cvtColor(mosaic, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=image)
            label.configure(image=photo)
            label.image = photo
            root.after(15, update)
        except tk.TclError:
            pass
        except Exception as exc:
            print(f"Preview update failed: {exc}")
            close()

    root.bind("<Key>", on_key)
    label.bind("<Button-1>", on_click)
    root.protocol("WM_DELETE_WINDOW", close)
    root.after(0, update)
    try:
        root.mainloop()
    finally:
        for cap in captures.values():
            cap.release()


def save_preview_snapshot(cv2, devices: list[CameraDevice], args: argparse.Namespace) -> None:
    captures = _open_preview_captures(cv2, devices, args)
    state = _preview_state(devices, args)
    mosaic = None
    try:
        deadline = time.monotonic() + max(0.5, float(args.probe_timeout))
        while time.monotonic() < deadline:
            mosaic = _read_mosaic_frame(cv2, devices, captures, state, args)
            time.sleep(0.03)
    finally:
        for cap in captures.values():
            cap.release()
    if mosaic is None:
        raise SystemExit("Could not capture preview snapshot.")
    out = Path(f"camera_mosaic_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
    cv2.imwrite(str(out), mosaic)
    print(f"OpenCV/Tk preview windows are unavailable. Saved snapshot instead: {out}")


def preview_mosaic(devices: list[CameraDevice], args: argparse.Namespace) -> None:
    cv2 = import_cv2()

    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        print("Warning: DISPLAY is not set. OpenCV may not be able to create a preview window.")

    if not _opencv_highgui_available(cv2):
        print("OpenCV HighGUI is unavailable; using Tk/PIL preview fallback.")
        preview_mosaic_tk(cv2, devices, args)
        return

    captures = _open_preview_captures(cv2, devices, args)
    state = _preview_state(devices, args)
    window = f"camera mosaic - {len(devices)} camera(s)"

    shown = ", ".join(f"{dev.path}(usb:{dev.usb_port})" for dev in devices)
    print(f"Previewing {len(devices)} camera(s): {shown}")
    print("Press q or Esc to quit, s to save the whole mosaic frame.")
    if getattr(args, "select_pair_output", ""):
        print("Click one tile for LEFT, then another tile for RIGHT.")

    def on_mouse(event, x, y, _flags, _param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        idx = _tile_index_from_xy(int(x), int(y), state, devices)
        if idx is None:
            return
        if _record_pair_click(args, devices, state, idx):
            state["close_requested"] = True

    if getattr(args, "select_pair_output", ""):
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)

    try:
        while True:
            mosaic = _read_mosaic_frame(cv2, devices, captures, state, args)
            cv2.imshow(window, mosaic)
            key = cv2.waitKey(1) & 0xFF
            if state.get("close_requested") or key in (27, ord("q")):
                break
            if key == ord("s"):
                out = Path(f"camera_mosaic_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
                cv2.imwrite(str(out), mosaic)
                print(f"Saved {out}")
    finally:
        for cap in captures.values():
            cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect local V4L2 cameras and show live previews in one mosaic.")
    parser.add_argument("--list", action="store_true", help="Print the detected /dev/video* devices first.")
    parser.add_argument(
        "--probe",
        action="store_true",
        help="Test which devices can return frames.",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Only print detection/probe results and do not open the live preview window.",
    )
    parser.add_argument(
        "--device",
        action="append",
        default=[],
        help="Camera to open. Repeat for multiple devices, for example --device /dev/video0 --device /dev/video2.",
    )
    parser.add_argument(
        "--usb-port",
        action="append",
        default=[],
        help="USB_PORT value to open from the --list output, for example --usb-port 1-2. Repeat for multiple ports.",
    )
    parser.add_argument(
        "--serial",
        action="append",
        default=[],
        help="Stable camera serial to open. Repeat for multiple cameras.",
    )
    parser.add_argument(
        "--all-nodes",
        action="store_true",
        help="Preview every readable /dev/video* node. Default is one readable node per USB port.",
    )
    parser.add_argument("--cols", type=int, default=0, help="Mosaic columns. 0 chooses automatically.")
    parser.add_argument("--tile-width", type=int, default=640, help="Width of each camera tile in the mosaic.")
    parser.add_argument("--tile-height", type=int, default=480, help="Height of each camera tile in the mosaic.")
    parser.add_argument("--backend", choices=("v4l2", "any"), default="v4l2", help="OpenCV capture backend.")
    parser.add_argument("--width", type=int, default=0, help="Requested capture width. 0 keeps camera default.")
    parser.add_argument("--height", type=int, default=0, help="Requested capture height. 0 keeps camera default.")
    parser.add_argument("--fps", type=float, default=0.0, help="Requested capture FPS. 0 keeps camera default.")
    parser.add_argument("--fourcc", default="MJPG", help="Optional pixel format, for example MJPG or YUYV.")
    parser.add_argument("--probe-timeout", type=float, default=1.5, help="Seconds to wait for a readable frame.")
    parser.add_argument(
        "--select-pair-output",
        default="",
        help="Write clicked left/right camera selection JSON to this path and close after two clicks.",
    )
    return parser


def main() -> int:
    args = make_parser().parse_args()
    devices = discover_video_devices()

    if args.list:
        selected: list[CameraDevice] = []
        probes = None
        if args.probe or not args.no_preview:
            selected, probes = choose_devices(devices, args)
        print_devices(devices, probes)
        if args.no_preview:
            return 0 if devices else 1
        if not devices:
            return 1
        preview_mosaic(selected, args)
        return 0

    selected, probes = choose_devices(devices, args)
    print_devices(devices, probes)
    if args.no_preview:
        return 0
    preview_mosaic(selected, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
