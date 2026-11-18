#!/usr/bin/env python3
"""Export pure teleop ROS 2 bag episodes to the dataset format of the installed LeRobot library."""

from __future__ import annotations

import argparse
import datetime as dt
import inspect
import json
import math
import os
import re
import shutil
from uuid import uuid4
from pathlib import Path
from typing import Any

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("HF_HOME", str(REPO_ROOT / ".cache" / "huggingface"))
os.environ.setdefault("HF_HUB_CACHE", str(REPO_ROOT / ".cache" / "huggingface" / "hub"))
os.environ.setdefault("HF_DATASETS_CACHE", str(REPO_ROOT / ".cache" / "huggingface" / "datasets"))
JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
VECTOR_NAMES = JOINT_NAMES + ["gripper"]
DEFAULT_CAMERA_TOPIC = "/teleop_camera/right/image_raw/compressed"
DEFAULT_STATE_TOPIC = "/slave_right/joint_states_single"
DEFAULT_ACTION_TOPIC = "/slave_right/joint_cmd_mux/output"


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def safe_slug(text: str, *, max_len: int = 96) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text or "")).strip("._-")
    return (value or "pure_teleop")[:max_len]


def parse_episode_num(path: Path) -> int:
    match = re.fullmatch(r"episode_(\d+)", path.name)
    return int(match.group(1)) if match else 10**9


def read_storage_identifier(bag_dir: Path) -> str:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return "mcap"
    for raw in metadata_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if line.startswith("storage_identifier:"):
            value = line.split(":", 1)[1].strip().strip("'\"")
            return value or "mcap"
    return "mcap"


def is_ros2_bag_dir(path: Path) -> bool:
    return path.is_dir() and (path / "metadata.yaml").is_file() and (any(path.glob("*.mcap")) or any(path.glob("*.db3")))


def resolve_bag_dir(episode_dir: Path) -> Path | None:
    for candidate in (episode_dir / "episode", episode_dir):
        if is_ros2_bag_dir(candidate):
            return candidate
    return None


def collect_episode_dirs(session_dir: Path, *, max_episodes: int = 0) -> list[Path]:
    session_dir = session_dir.expanduser().resolve()
    if not session_dir.is_dir():
        raise RuntimeError(f"session dir not found: {session_dir}")
    if resolve_bag_dir(session_dir) is not None:
        episodes = [session_dir]
    else:
        episodes = [
            path
            for path in sorted(session_dir.iterdir(), key=parse_episode_num)
            if path.is_dir() and path.name.startswith("episode_") and resolve_bag_dir(path) is not None
        ]
    if max_episodes > 0:
        episodes = episodes[: int(max_episodes)]
    if not episodes:
        raise RuntimeError(f"no ROS 2 bag episodes found under: {session_dir}")
    return episodes


def stamp_sec_from_msg(msg: Any, fallback_sec: float) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return fallback_sec
    try:
        value = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9
    except (TypeError, ValueError):
        return fallback_sec
    return value if value > 0.0 and math.isfinite(value) else fallback_sec


def message_time_sec(msg: Any, timestamp_ns: int, *, use_bag_time: bool) -> float:
    bag_sec = float(timestamp_ns) * 1e-9
    return bag_sec if use_bag_time else stamp_sec_from_msg(msg, bag_sec)


def _data_to_uint8(data: Any) -> np.ndarray:
    if isinstance(data, (bytes, bytearray, memoryview)):
        return np.frombuffer(data, dtype=np.uint8)
    return np.asarray(data, dtype=np.uint8)


def decode_image_message(msg: Any, *, resize_hw: tuple[int, int] | None) -> np.ndarray | None:
    if hasattr(msg, "format") and not hasattr(msg, "height"):
        arr = _data_to_uint8(getattr(msg, "data", b""))
        if arr.size == 0:
            return None
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return None
    elif all(hasattr(msg, attr) for attr in ("height", "width", "encoding", "step", "data")):
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        step = int(getattr(msg, "step", 0) or 0)
        encoding = str(getattr(msg, "encoding", "") or "").strip().lower()
        if height <= 0 or width <= 0:
            return None
        arr = _data_to_uint8(getattr(msg, "data", b""))
        if arr.size == 0:
            return None
        if encoding in {"bgr8", "rgb8", "8uc3"}:
            channels = 3
        elif encoding in {"bgra8", "rgba8", "8uc4"}:
            channels = 4
        elif encoding in {"mono8", "8uc1"}:
            channels = 1
        else:
            return None
        row_bytes = step if step > 0 else width * channels
        needed = height * row_bytes
        if arr.size < needed:
            return None
        rows = arr[:needed].reshape(height, row_bytes)
        pixels = rows[:, : width * channels].reshape(height, width, channels)
        if encoding in {"bgr8", "8uc3"}:
            frame = pixels.copy()
        elif encoding == "rgb8":
            frame = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif encoding == "bgra8":
            frame = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        elif encoding in {"rgba8", "8uc4"}:
            frame = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
        else:
            frame = cv2.cvtColor(pixels[:, :, 0], cv2.COLOR_GRAY2BGR)
    else:
        return None
    if resize_hw is not None:
        height, width = resize_hw
        frame = cv2.resize(frame, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(frame)


def joint_vector_from_msg(msg: Any, *, dim: int) -> np.ndarray | None:
    names = [str(value) for value in getattr(msg, "name", []) or []]
    positions = [float(value) for value in getattr(msg, "position", []) or []]
    preferred = JOINT_NAMES + (["gripper"] if dim >= 8 else [])
    if len(preferred) >= dim and all(name in names for name in preferred[:dim]):
        values = [positions[names.index(name)] for name in preferred[:dim]]
    elif len(positions) >= dim:
        values = positions[:dim]
    else:
        return None
    if len(values) != dim or not all(math.isfinite(value) for value in values):
        return None
    return np.asarray(values, dtype=np.float32)


def nearest_indices(source_ts: np.ndarray, query_ts: np.ndarray) -> np.ndarray:
    if source_ts.size == 0:
        raise RuntimeError("nearest_indices source is empty")
    if source_ts.size == 1:
        return np.zeros(query_ts.shape[0], dtype=np.int64)
    idx = np.searchsorted(source_ts, query_ts, side="left")
    idx = np.clip(idx, 1, source_ts.size - 1)
    prev_idx = idx - 1
    prev_dist = np.abs(query_ts - source_ts[prev_idx])
    next_dist = np.abs(source_ts[idx] - query_ts)
    idx[prev_dist <= next_dist] = prev_idx[prev_dist <= next_dist]
    return idx.astype(np.int64)


def sorted_stream(times: list[float], values: list[Any]) -> tuple[np.ndarray, list[Any]]:
    order = np.argsort(np.asarray(times, dtype=np.float64), kind="stable")
    return (
        np.asarray([times[int(index)] for index in order], dtype=np.float64),
        [values[int(index)] for index in order],
    )


def camera_bindings_from_args(args: argparse.Namespace) -> list[dict[str, str]]:
    bindings: list[dict[str, str]] = []
    for index, item in enumerate(args.camera_topic or []):
        text = str(item).strip()
        if not text:
            continue
        if "=" in text:
            key, topic = text.split("=", 1)
            key = key.strip()
            topic = topic.strip()
        else:
            topic = text
            key = "observation.image" if index == 0 else f"observation.camera_{index:02d}_image"
        if not key.startswith("observation."):
            key = f"observation.{key}"
        bindings.append({"key": key, "topic": topic})
    if not bindings:
        bindings.append({"key": "observation.image", "topic": DEFAULT_CAMERA_TOPIC})
    return bindings


def read_episode_streams(
    *,
    episode_dir: Path,
    camera_bindings: list[dict[str, str]],
    state_topic: str,
    action_topic: str,
    use_bag_time: bool,
    resize_hw: tuple[int, int] | None,
    state_dim: int,
    action_dim: int,
) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = resolve_bag_dir(episode_dir)
    if bag_dir is None:
        raise RuntimeError(f"missing ROS 2 bag dir under: {episode_dir}")
    storage_id = read_storage_identifier(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic_to_binding = {str(binding["topic"]): binding for binding in camera_bindings}
    topics = [*topic_to_binding.keys(), state_topic, action_topic]
    missing = [topic for topic in topics if topic not in type_map]
    if missing:
        raise RuntimeError(f"episode {episode_dir.name} missing required topics: {', '.join(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    msg_cls_cache: dict[str, Any] = {}
    image_times_by_key: dict[str, list[float]] = {str(binding["key"]): [] for binding in camera_bindings}
    image_frames_by_key: dict[str, list[np.ndarray]] = {str(binding["key"]): [] for binding in camera_bindings}
    state_times: list[float] = []
    state_vectors: list[np.ndarray] = []
    action_times: list[float] = []
    action_vectors: list[np.ndarray] = []
    dropped_images_by_key: dict[str, int] = {str(binding["key"]): 0 for binding in camera_bindings}
    dropped_state = 0
    dropped_action = 0

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        type_name = type_map.get(topic)
        if not type_name:
            continue
        msg_cls = msg_cls_cache.get(type_name)
        if msg_cls is None:
            msg_cls = get_message(type_name)
            msg_cls_cache[type_name] = msg_cls
        msg = deserialize_message(data, msg_cls)
        timestamp = message_time_sec(msg, int(timestamp_ns), use_bag_time=use_bag_time)
        if topic in topic_to_binding:
            key = str(topic_to_binding[topic]["key"])
            frame = decode_image_message(msg, resize_hw=resize_hw)
            if frame is None:
                dropped_images_by_key[key] += 1
                continue
            image_times_by_key[key].append(timestamp)
            image_frames_by_key[key].append(frame)
        elif topic == state_topic:
            vector = joint_vector_from_msg(msg, dim=state_dim)
            if vector is None:
                dropped_state += 1
                continue
            state_times.append(timestamp)
            state_vectors.append(vector)
        elif topic == action_topic:
            vector = joint_vector_from_msg(msg, dim=action_dim)
            if vector is None:
                dropped_action += 1
                continue
            action_times.append(timestamp)
            action_vectors.append(vector)

    image_streams: dict[str, dict[str, Any]] = {}
    for binding in camera_bindings:
        key = str(binding["key"])
        image_ts, image_frames = sorted_stream(image_times_by_key.get(key, []), image_frames_by_key.get(key, []))
        image_streams[key] = {"topic": str(binding["topic"]), "ts": image_ts, "frames": image_frames, "dropped": int(dropped_images_by_key.get(key, 0))}
    state_ts, state_vectors = sorted_stream(state_times, state_vectors)
    action_ts, action_vectors = sorted_stream(action_times, action_vectors)
    return {
        "episode_dir": episode_dir,
        "bag_dir": bag_dir,
        "storage_id": storage_id,
        "anchor_key": str(camera_bindings[0]["key"]),
        "image_streams": image_streams,
        "state_ts": state_ts,
        "state_vectors": state_vectors,
        "action_ts": action_ts,
        "action_vectors": action_vectors,
        "dropped": {"images": dropped_images_by_key, "state": int(dropped_state), "action": int(dropped_action)},
        "topic_types": {topic: type_map.get(topic, "") for topic in topics},
    }


def sample_episode(
    streams: dict[str, Any],
    *,
    max_sync_lag_sec: float,
    action_offset_steps: int,
    max_frames: int,
    trim_start_sec: float = 0.0,
    trim_end_sec: float = 0.0,
    edge_sec: float = 0.0,
    edge_stride: int = 1,
    middle_stride: int = 1,
) -> dict[str, Any]:
    anchor_key = str(streams["anchor_key"])
    image_streams = streams["image_streams"]
    image_ts = np.asarray(image_streams[anchor_key]["ts"], dtype=np.float64)
    state_ts = np.asarray(streams["state_ts"], dtype=np.float64)
    action_ts = np.asarray(streams["action_ts"], dtype=np.float64)
    if image_ts.size == 0:
        raise RuntimeError(f"missing camera frames for {anchor_key}")
    for key, stream in image_streams.items():
        if np.asarray(stream["ts"], dtype=np.float64).size == 0:
            raise RuntimeError(f"missing camera frames for {key}")
    if state_ts.size == 0:
        raise RuntimeError("missing robot state samples")
    if action_ts.size == 0:
        raise RuntimeError("missing action samples")

    t_start = max([float(state_ts[0]), float(action_ts[0])] + [float(np.asarray(stream["ts"], dtype=np.float64)[0]) for stream in image_streams.values()])
    t_end = min([float(state_ts[-1]), float(action_ts[-1])] + [float(np.asarray(stream["ts"], dtype=np.float64)[-1]) for stream in image_streams.values()])
    if t_end <= t_start:
        raise RuntimeError("no overlapping time window across camera/state/action")

    image_indices = np.flatnonzero((image_ts >= t_start) & (image_ts <= t_end))
    raw_candidate_frames = int(image_indices.size)

    trim_start_sec = max(0.0, float(trim_start_sec))
    trim_end_sec = max(0.0, float(trim_end_sec))
    edge_sec = max(0.0, float(edge_sec))
    edge_stride = max(1, int(edge_stride))
    middle_stride = max(1, int(middle_stride))

    if trim_start_sec > 0.0 or trim_end_sec > 0.0:
        keep_start = t_start + trim_start_sec
        keep_end = t_end - trim_end_sec
        if keep_end <= keep_start:
            raise RuntimeError(
                f"trim removes whole episode: overlap={t_end - t_start:.3f}s "
                f"trim_start={trim_start_sec:.3f}s trim_end={trim_end_sec:.3f}s"
            )
        image_indices = image_indices[(image_ts[image_indices] >= keep_start) & (image_ts[image_indices] <= keep_end)]
        if image_indices.size == 0:
            raise RuntimeError("no camera frames left after trim")

    if edge_sec > 0.0 or edge_stride > 1 or middle_stride > 1:
        active_ts = image_ts[image_indices]
        active_start = float(active_ts[0])
        active_end = float(active_ts[-1])
        head_end = active_start + edge_sec
        tail_start = active_end - edge_sec
        segment_counts = {"head": 0, "middle": 0, "tail": 0}
        selected_indices: list[int] = []
        for index in image_indices:
            timestamp = float(image_ts[int(index)])
            if edge_sec > 0.0 and timestamp <= head_end:
                segment = "head"
            elif edge_sec > 0.0 and timestamp >= tail_start:
                segment = "tail"
            else:
                segment = "middle"
            stride = edge_stride if segment in {"head", "tail"} else middle_stride
            if segment_counts[segment] % stride == 0:
                selected_indices.append(int(index))
            segment_counts[segment] += 1
        image_indices = np.asarray(selected_indices, dtype=np.int64)
        if image_indices.size == 0:
            raise RuntimeError("no camera frames left after stride sampling")

    if max_frames > 0:
        image_indices = image_indices[: int(max_frames)]
    query_ts = image_ts[image_indices]
    state_idx = nearest_indices(state_ts, query_ts)
    action_idx = nearest_indices(action_ts, query_ts)
    state_lag = np.abs(state_ts[state_idx] - query_ts)
    action_lag = np.abs(action_ts[action_idx] - query_ts)
    keep = (state_lag <= float(max_sync_lag_sec)) & (action_lag <= float(max_sync_lag_sec))

    camera_indices_by_key: dict[str, np.ndarray] = {anchor_key: image_indices}
    camera_lag_by_key: dict[str, np.ndarray] = {anchor_key: np.zeros(query_ts.shape[0], dtype=np.float64)}
    for key, stream in image_streams.items():
        if key == anchor_key:
            continue
        ts = np.asarray(stream["ts"], dtype=np.float64)
        idx = nearest_indices(ts, query_ts)
        lag = np.abs(ts[idx] - query_ts)
        camera_indices_by_key[key] = idx
        camera_lag_by_key[key] = lag
        keep &= lag <= float(max_sync_lag_sec)

    state_idx = state_idx[keep]
    action_idx = action_idx[keep]
    query_ts = query_ts[keep]
    if int(action_offset_steps) > 0 and action_idx.size:
        action_idx = np.clip(action_idx + int(action_offset_steps), 0, len(streams["action_vectors"]) - 1)

    frames_by_key = {
        key: [list(stream["frames"])[int(index)] for index in camera_indices_by_key[key][keep]]
        for key, stream in image_streams.items()
    }
    return {
        "timestamps": query_ts,
        "frames_bgr_by_key": frames_by_key,
        "states": [streams["state_vectors"][int(index)] for index in state_idx],
        "actions": [streams["action_vectors"][int(index)] for index in action_idx],
        "raw_candidate_frames": int(raw_candidate_frames),
        "candidate_frames": int(image_indices.size),
        "kept_frames": int(query_ts.size),
        "dropped_sync_frames": int(image_indices.size - query_ts.size),
        "trim_start_sec": float(trim_start_sec),
        "trim_end_sec": float(trim_end_sec),
        "edge_sec": float(edge_sec),
        "edge_stride": int(edge_stride),
        "middle_stride": int(middle_stride),
        "camera_lag": {
            key: {
                "lag_ms_mean": float(np.mean(lag[keep]) * 1000.0) if np.any(keep) else 0.0,
                "lag_ms_max": float(np.max(lag[keep]) * 1000.0) if np.any(keep) else 0.0,
                "raw_frames": int(np.asarray(image_streams[key]["ts"], dtype=np.float64).size),
            }
            for key, lag in camera_lag_by_key.items()
        },
        "state_lag_ms_mean": float(np.mean(state_lag[keep]) * 1000.0) if np.any(keep) else 0.0,
        "state_lag_ms_max": float(np.max(state_lag[keep]) * 1000.0) if np.any(keep) else 0.0,
        "action_lag_ms_mean": float(np.mean(action_lag[keep]) * 1000.0) if np.any(keep) else 0.0,
        "action_lag_ms_max": float(np.max(action_lag[keep]) * 1000.0) if np.any(keep) else 0.0,
    }


def build_features(*, height: int, width: int, state_dim: int, action_dim: int, image_keys: list[str]) -> dict[str, dict[str, Any]]:
    features = {
        "observation.state": {"dtype": "float32", "shape": (int(state_dim),), "names": VECTOR_NAMES[: int(state_dim)]},
        "action": {"dtype": "float32", "shape": (int(action_dim),), "names": VECTOR_NAMES[: int(action_dim)]},
    }
    for key in image_keys:
        features[key] = {"dtype": "video", "shape": (3, int(height), int(width)), "names": ["channels", "height", "width"]}
    return features


def create_lerobot_dataset(
    lerobot_dataset_cls: Any,
    *,
    repo_id: str,
    root: Path,
    fps: int,
    features: dict[str, dict[str, Any]],
    robot_type: str,
    use_videos: bool,
    video_codec: str,
    image_writer_threads: int,
    image_writer_processes: int,
) -> Any:
    params = inspect.signature(lerobot_dataset_cls.create).parameters
    kwargs: dict[str, Any] = {
        "repo_id": repo_id,
        "root": root,
        "fps": fps,
        "features": features,
        "robot_type": robot_type,
        "use_videos": use_videos,
        "image_writer_threads": image_writer_threads,
        "image_writer_processes": image_writer_processes,
    }
    if video_codec != "auto":
        codec_arg = next((key for key in ("vcodec", "video_codec") if key in params), None)
        if codec_arg is None:
            raise ValueError("This LeRobot version selects its own codec. Use --video-codec auto.")
        kwargs[codec_arg] = video_codec
    return lerobot_dataset_cls.create(**{key: value for key, value in kwargs.items() if key in params})


def add_lerobot_frame(dataset: Any, frame_payload: dict[str, Any], *, task: str) -> None:
    params = inspect.signature(dataset.add_frame).parameters
    if "task" in params:
        payload = dict(frame_payload)
        payload.pop("task", None)
        dataset.add_frame(payload, task=task)
        return
    dataset.add_frame(frame_payload)


def finalize_lerobot_dataset(dataset: Any) -> None:
    finalize = getattr(dataset, "finalize", None)
    if callable(finalize):
        finalize()


def write_policy_contract(root: Path, *, info: dict[str, Any], source: dict[str, Any]) -> None:
    write_json(
        root / "meta" / "policy_dataset_contract.json",
        {
            "schema": "policy_dataset_contract_v1",
            "created_at_utc": utc_now_iso(),
            "pre_class_export_schema": "standard_robot_mcap_act_v1",
            "source_family": "mcap_teleop",
            "state_encoding": "joint_position_plus_gripper_width",
            "action_encoding": "joint_position_plus_gripper_width",
            "features": info.get("features") or source.get("features") or {},
            "source": source,
        },
    )


def export_dataset(args: argparse.Namespace) -> dict[str, Any]:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    output_root = Path(args.output_root).expanduser().resolve()
    dataset_id = safe_slug(str(args.dataset_id))
    final_root = output_root / dataset_id
    if final_root.exists():
        raise RuntimeError(f"dataset already exists: {final_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    tmp_root = output_root / f".tmp_pure_teleop_lerobot_{dataset_id}_{uuid4().hex}"
    fps_float = float(args.fps)
    fps = int(round(fps_float))
    if fps <= 0 or abs(fps_float - fps) > 1e-3:
        raise RuntimeError(f"LeRobotDataset.create requires positive integer fps, got {fps_float}")
    resize_hw = None if bool(args.no_resize) else (int(args.image_height), int(args.image_width))
    camera_bindings = camera_bindings_from_args(args)
    image_keys = [str(binding["key"]) for binding in camera_bindings]
    session_dirs = [Path(value).expanduser().resolve() for value in args.session_dir]
    tasks = list(args.task) or ["pure teleop"]
    if len(tasks) == 1:
        tasks *= len(session_dirs)
    if len(tasks) != len(session_dirs):
        raise RuntimeError(f"expected one --task per --session-dir (or one shared task), got {len(tasks)} tasks for {len(session_dirs)} sessions")
    episode_specs: list[tuple[str, Path]] = []
    for task, session_dir in zip(tasks, session_dirs, strict=True):
        episode_specs.extend((task, episode_dir) for episode_dir in collect_episode_dirs(session_dir, max_episodes=int(args.max_episodes or 0)))
    dataset: Any | None = None
    features: dict[str, dict[str, Any]] | None = None
    reports: list[dict[str, Any]] = []
    total_frames = 0
    skipped: list[dict[str, str]] = []

    try:
        for source_index, (episode_task, episode_dir) in enumerate(episode_specs):
            print(f"[pure-export] read {episode_dir.name} ({source_index + 1}/{len(episode_specs)}) task={episode_task!r}", flush=True)
            try:
                streams = read_episode_streams(
                    episode_dir=episode_dir,
                    camera_bindings=camera_bindings,
                    state_topic=str(args.state_topic),
                    action_topic=str(args.action_topic),
                    use_bag_time=bool(args.use_bag_time),
                    resize_hw=resize_hw,
                    state_dim=int(args.state_dim),
                    action_dim=int(args.action_dim),
                )
                sampled = sample_episode(
                    streams,
                    max_sync_lag_sec=float(args.max_sync_lag_sec),
                    action_offset_steps=int(args.action_offset_steps),
                    max_frames=int(args.max_frames_per_episode or 0),
                )
                if int(sampled["kept_frames"]) < int(args.min_frames):
                    raise RuntimeError(f"too few synchronized frames: kept={sampled['kept_frames']} min={args.min_frames}")
            except Exception as exc:
                if bool(args.skip_bad_episodes):
                    skipped.append({"episode_dir": str(episode_dir), "error": str(exc)})
                    print(f"[pure-export][skip] {episode_dir.name}: {exc}", flush=True)
                    continue
                raise

            first_frame = sampled["frames_bgr_by_key"][image_keys[0]][0]
            height, width = first_frame.shape[:2]
            if features is None:
                features = build_features(height=height, width=width, state_dim=int(args.state_dim), action_dim=int(args.action_dim), image_keys=image_keys)
                dataset = create_lerobot_dataset(
                    LeRobotDataset,
                    repo_id=dataset_id,
                    root=tmp_root,
                    fps=fps,
                    features=features,
                    robot_type=str(args.robot_type),
                    use_videos=True,
                    video_codec=str(args.video_codec),
                    image_writer_threads=int(args.image_writer_threads),
                    image_writer_processes=int(args.image_writer_processes),
                )
            assert dataset is not None
            for frame_index, (state_vec, action_vec) in enumerate(zip(sampled["states"], sampled["actions"], strict=True)):
                frame_payload = {
                    "observation.state": np.asarray(state_vec, dtype=np.float32),
                    "action": np.asarray(action_vec, dtype=np.float32),
                    "task": str(episode_task),
                }
                for key in image_keys:
                    frame_payload[key] = cv2.cvtColor(sampled["frames_bgr_by_key"][key][frame_index], cv2.COLOR_BGR2RGB)
                add_lerobot_frame(dataset, frame_payload, task=str(episode_task))
            dataset.save_episode()
            report = {
                "episode_index": len(reports),
                "source_episode_index": int(source_index),
                "source_episode_dir": str(episode_dir),
                "source_bag_dir": str(streams["bag_dir"]),
                "task": str(episode_task),
                "storage_id": str(streams["storage_id"]),
                "candidate_frames": int(sampled["candidate_frames"]),
                "kept_rows": int(sampled["kept_frames"]),
                "dropped_sync_frames": int(sampled["dropped_sync_frames"]),
                "camera_lag": sampled["camera_lag"],
                "state_lag_ms_mean": float(sampled["state_lag_ms_mean"]),
                "state_lag_ms_max": float(sampled["state_lag_ms_max"]),
                "action_lag_ms_mean": float(sampled["action_lag_ms_mean"]),
                "action_lag_ms_max": float(sampled["action_lag_ms_max"]),
                "dropped_decode_or_short": dict(streams["dropped"]),
            }
            reports.append(report)
            total_frames += int(sampled["kept_frames"])
            print(f"[pure-export] episode_{report['episode_index']:06d}: frames={report['kept_rows']}", flush=True)

        if dataset is None or features is None or not reports:
            raise RuntimeError("no episodes were exported")
        finalize_lerobot_dataset(dataset)
        info_path = tmp_root / "meta" / "info.json"
        info = read_json(info_path)
        source_manifest = {
            "schema": "pure_teleop_mcap_lerobot_v30_export_v1",
            "created_at_utc": utc_now_iso(),
            "dataset_id": dataset_id,
            "dataset_root": str(tmp_root),
            "source_session_dirs": [str(value) for value in session_dirs],
            "tasks": tasks,
            "fps": int(fps),
            "robot_type": str(args.robot_type),
            "pre_class_export_schema": "standard_robot_mcap_act_v1",
            "state_encoding": "joint_position_plus_gripper_width",
            "action_encoding": "joint_position_plus_gripper_width",
            "camera_topics": [str(binding["topic"]) for binding in camera_bindings],
            "camera_bindings": camera_bindings,
            "state_topic": str(args.state_topic),
            "action_topic": str(args.action_topic),
            "state_dim": int(args.state_dim),
            "action_dim": int(args.action_dim),
            "state_names": VECTOR_NAMES[: int(args.state_dim)],
            "action_names": VECTOR_NAMES[: int(args.action_dim)],
            "use_bag_time": bool(args.use_bag_time),
            "max_sync_lag_sec": float(args.max_sync_lag_sec),
            "action_offset_steps": int(args.action_offset_steps),
            "resize_images": not bool(args.no_resize),
            "image_width": int(features[image_keys[0]]["shape"][2]),
            "image_height": int(features[image_keys[0]]["shape"][1]),
            "video_codec": info["features"][image_keys[0]].get("info", {}).get("video.codec"),
            "features": features,
            "episodes": reports,
            "skipped_episodes": skipped,
            "total_episodes": len(reports),
            "total_frames": int(total_frames),
        }
        write_json(tmp_root / "meta" / "source_pure_teleop_export.json", source_manifest)
        info["pre_class_export_schema"] = "standard_robot_mcap_act_v1"
        info["state_encoding"] = "joint_position_plus_gripper_width"
        info["action_encoding"] = "joint_position_plus_gripper_width"
        info["camera_bindings"] = camera_bindings
        write_json(info_path, info)
        write_policy_contract(tmp_root, info=info, source=source_manifest)
        loaded = LeRobotDataset(dataset_id, root=tmp_root, download_videos=False)
        if int(loaded.num_frames) != int(total_frames):
            raise RuntimeError(f"LeRobot reload frame mismatch: loaded={loaded.num_frames} expected={total_frames}")
        tmp_root.rename(final_root)
        source_manifest["dataset_root"] = str(final_root)
        write_json(final_root / "meta" / "source_pure_teleop_export.json", source_manifest)
        write_policy_contract(final_root, info=info, source=source_manifest)
        print(f"[pure-export] done dataset={dataset_id} root={final_root} frames={total_frames}", flush=True)
        return source_manifest
    except Exception:
        shutil.rmtree(tmp_root, ignore_errors=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True, action="append", help="ROS bag session directory; repeat with a matching --task to merge sources")
    parser.add_argument("--output-root", default=str(os.environ.get("LEROBOT_ROOT") or REPO_ROOT / "data" / "pure_teleop" / "lerobot"))
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--task", action="append", default=[], help="Task prompt for the matching --session-dir; repeatable")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--camera-topic", action="append", default=[], help="topic or observation.key=/topic; repeatable")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--state-dim", type=int, default=8)
    parser.add_argument("--action-dim", type=int, default=8)
    parser.add_argument("--robot-type", default="nero")
    parser.add_argument("--max-sync-lag-sec", type=float, default=0.08)
    parser.add_argument("--action-offset-steps", type=int, default=0)
    parser.add_argument("--use-bag-time", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--image-width", type=int, default=256)
    parser.add_argument("--image-height", type=int, default=256)
    parser.add_argument("--no-resize", action="store_true")
    parser.add_argument("--video-codec", default="auto", help="Use the library default, or choose a codec if supported by this LeRobot version.")
    parser.add_argument("--image-writer-threads", type=int, default=max(4, min(8, (os.cpu_count() or 4) // 2)))
    parser.add_argument("--image-writer-processes", type=int, default=0)
    parser.add_argument("--min-frames", type=int, default=16)
    parser.add_argument("--skip-bad-episodes", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-episodes", type=int, default=0)
    parser.add_argument("--max-frames-per-episode", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    try:
        export_dataset(parse_args())
        return 0
    except KeyboardInterrupt:
        print("\n[pure-export] interrupted", flush=True)
        return 130
    except Exception as exc:
        print(f"[pure-export][FATAL] {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
