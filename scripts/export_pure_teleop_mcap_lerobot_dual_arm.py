#!/usr/bin/env python3
"""Export synchronized left+right 8D teleoperation streams as one 16D LeRobot set.

The action/state order is always ``left[8], right[8]``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any
from uuid import uuid4

import cv2
import numpy as np

import export_pure_teleop_mcap_lerobot_v30 as base


ARM_ORDER = ("left", "right")
ARM_DIM = 8
DUAL_DIM = len(ARM_ORDER) * ARM_DIM
DUAL_NAMES = [f"{arm}.{name}" for arm in ARM_ORDER for name in base.VECTOR_NAMES]


def read_dual_episode_streams(
    *,
    episode_dir: Path,
    camera_bindings: list[dict[str, str]],
    state_topics: dict[str, str],
    action_topics: dict[str, str],
    use_bag_time: bool,
    resize_hw: tuple[int, int] | None,
) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = base.resolve_bag_dir(episode_dir)
    if bag_dir is None:
        raise RuntimeError(f"missing ROS 2 bag dir under: {episode_dir}")
    storage_id = base.read_storage_identifier(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic_to_binding = {str(binding["topic"]): binding for binding in camera_bindings}
    state_to_arm = {str(topic): arm for arm, topic in state_topics.items()}
    action_to_arm = {str(topic): arm for arm, topic in action_topics.items()}
    topics = list(dict.fromkeys([*topic_to_binding, *state_to_arm, *action_to_arm]))
    missing = [topic for topic in topics if topic not in type_map]
    if missing:
        raise RuntimeError(f"episode {episode_dir.name} missing required topics: {', '.join(missing)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    msg_cls_cache: dict[str, Any] = {}
    image_times = {str(binding["key"]): [] for binding in camera_bindings}
    image_frames = {str(binding["key"]): [] for binding in camera_bindings}
    dropped_images = {str(binding["key"]): 0 for binding in camera_bindings}
    arms = {
        arm: {
            "state_times": [], "state_vectors": [], "action_times": [], "action_vectors": [],
            "dropped_state": 0, "dropped_action": 0,
        }
        for arm in ARM_ORDER
    }
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
        timestamp = base.message_time_sec(msg, int(timestamp_ns), use_bag_time=use_bag_time)
        if topic in topic_to_binding:
            key = str(topic_to_binding[topic]["key"])
            frame = base.decode_image_message(msg, resize_hw=resize_hw)
            if frame is None:
                dropped_images[key] += 1
                continue
            image_times[key].append(timestamp)
            image_frames[key].append(frame)
        elif topic in state_to_arm:
            arm = state_to_arm[topic]
            vector = base.joint_vector_from_msg(msg, dim=ARM_DIM)
            if vector is None:
                arms[arm]["dropped_state"] += 1
                continue
            arms[arm]["state_times"].append(timestamp)
            arms[arm]["state_vectors"].append(vector)
        elif topic in action_to_arm:
            arm = action_to_arm[topic]
            vector = base.joint_vector_from_msg(msg, dim=ARM_DIM)
            if vector is None:
                arms[arm]["dropped_action"] += 1
                continue
            arms[arm]["action_times"].append(timestamp)
            arms[arm]["action_vectors"].append(vector)

    image_streams: dict[str, dict[str, Any]] = {}
    for binding in camera_bindings:
        key = str(binding["key"])
        ts, frames = base.sorted_stream(image_times[key], image_frames[key])
        image_streams[key] = {"topic": str(binding["topic"]), "ts": ts, "frames": frames, "dropped": dropped_images[key]}
    for arm in ARM_ORDER:
        state_ts, state_values = base.sorted_stream(arms[arm]["state_times"], arms[arm]["state_vectors"])
        action_ts, action_values = base.sorted_stream(arms[arm]["action_times"], arms[arm]["action_vectors"])
        arms[arm]["state_ts"] = state_ts
        arms[arm]["state_vectors"] = state_values
        arms[arm]["action_ts"] = action_ts
        arms[arm]["action_vectors"] = action_values
    return {
        "episode_dir": episode_dir,
        "bag_dir": bag_dir,
        "storage_id": storage_id,
        "anchor_key": str(camera_bindings[0]["key"]),
        "image_streams": image_streams,
        "arms": arms,
        "dropped": {
            "images": dropped_images,
            "arms": {
                arm: {"state": int(arms[arm]["dropped_state"]), "action": int(arms[arm]["dropped_action"])}
                for arm in ARM_ORDER
            },
        },
        "topic_types": {topic: type_map.get(topic, "") for topic in topics},
    }


def sample_dual_episode(
    streams: dict[str, Any], *, max_sync_lag_sec: float, action_offset_steps: int, max_frames: int
) -> dict[str, Any]:
    anchor_key = str(streams["anchor_key"])
    image_streams = streams["image_streams"]
    anchor_ts = np.asarray(image_streams[anchor_key]["ts"], dtype=np.float64)
    if anchor_ts.size == 0:
        raise RuntimeError(f"missing camera frames for {anchor_key}")
    all_times = [anchor_ts]
    for key, stream in image_streams.items():
        ts = np.asarray(stream["ts"], dtype=np.float64)
        if ts.size == 0:
            raise RuntimeError(f"missing camera frames for {key}")
        all_times.append(ts)
    for arm in ARM_ORDER:
        for kind in ("state", "action"):
            ts = np.asarray(streams["arms"][arm][f"{kind}_ts"], dtype=np.float64)
            if ts.size == 0:
                raise RuntimeError(f"missing {arm} {kind} samples")
            all_times.append(ts)
    t_start = max(float(ts[0]) for ts in all_times)
    t_end = min(float(ts[-1]) for ts in all_times)
    if t_end <= t_start:
        raise RuntimeError("no overlapping time window across camera/left/right state/action")
    anchor_idx = np.flatnonzero((anchor_ts >= t_start) & (anchor_ts <= t_end))
    if max_frames > 0:
        anchor_idx = anchor_idx[: int(max_frames)]
    if anchor_idx.size == 0:
        raise RuntimeError("no anchor camera frames in synchronized time window")
    query_ts = anchor_ts[anchor_idx]
    keep = np.ones(query_ts.shape[0], dtype=bool)
    camera_idx = {anchor_key: anchor_idx}
    camera_lag = {anchor_key: np.zeros(query_ts.shape[0], dtype=np.float64)}
    for key, stream in image_streams.items():
        if key == anchor_key:
            continue
        ts = np.asarray(stream["ts"], dtype=np.float64)
        idx = base.nearest_indices(ts, query_ts)
        lag = np.abs(ts[idx] - query_ts)
        camera_idx[key] = idx
        camera_lag[key] = lag
        keep &= lag <= float(max_sync_lag_sec)
    indices: dict[str, dict[str, np.ndarray]] = {}
    lags: dict[str, dict[str, np.ndarray]] = {}
    for arm in ARM_ORDER:
        indices[arm], lags[arm] = {}, {}
        for kind in ("state", "action"):
            ts = np.asarray(streams["arms"][arm][f"{kind}_ts"], dtype=np.float64)
            idx = base.nearest_indices(ts, query_ts)
            lag = np.abs(ts[idx] - query_ts)
            indices[arm][kind] = idx
            lags[arm][kind] = lag
            keep &= lag <= float(max_sync_lag_sec)
    if not np.any(keep):
        raise RuntimeError("no frames satisfy dual-arm synchronization tolerance")
    query_ts = query_ts[keep]
    frames_by_key = {
        key: [stream["frames"][int(index)] for index in camera_idx[key][keep]]
        for key, stream in image_streams.items()
    }
    states, actions = [], []
    for output_index in range(query_ts.size):
        state_parts, action_parts = [], []
        for arm in ARM_ORDER:
            state_index = int(indices[arm]["state"][keep][output_index])
            action_index = int(indices[arm]["action"][keep][output_index])
            if int(action_offset_steps) > 0:
                action_index = min(
                    len(streams["arms"][arm]["action_vectors"]) - 1,
                    action_index + int(action_offset_steps),
                )
            state_parts.append(streams["arms"][arm]["state_vectors"][state_index])
            action_parts.append(streams["arms"][arm]["action_vectors"][action_index])
        states.append(np.concatenate(state_parts).astype(np.float32, copy=False))
        actions.append(np.concatenate(action_parts).astype(np.float32, copy=False))
    return {
        "timestamps": query_ts,
        "frames_bgr_by_key": frames_by_key,
        "states": states,
        "actions": actions,
        "candidate_frames": int(anchor_idx.size),
        "kept_frames": int(query_ts.size),
        "dropped_sync_frames": int(anchor_idx.size - query_ts.size),
        "camera_lag": {
            key: {
                "lag_ms_mean": float(np.mean(lag[keep]) * 1000.0) if np.any(keep) else 0.0,
                "lag_ms_max": float(np.max(lag[keep]) * 1000.0) if np.any(keep) else 0.0,
            }
            for key, lag in camera_lag.items()
        },
        "arm_lag": {
            arm: {
                kind: {
                    "lag_ms_mean": float(np.mean(lags[arm][kind][keep]) * 1000.0),
                    "lag_ms_max": float(np.max(lags[arm][kind][keep]) * 1000.0),
                }
                for kind in ("state", "action")
            }
            for arm in ARM_ORDER
        },
    }


def export_dataset(args: argparse.Namespace) -> dict[str, Any]:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    session_dir = Path(args.session_dir).expanduser().resolve()
    output_root = Path(args.output_root).expanduser().resolve()
    dataset_id = base.safe_slug(str(args.dataset_id))
    final_root = output_root / dataset_id
    if final_root.exists():
        raise RuntimeError(f"dataset already exists: {final_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    tmp_root = output_root / f".tmp_pure_teleop_dual_{dataset_id}_{uuid4().hex}"
    fps = int(round(float(args.fps)))
    if fps <= 0 or abs(float(args.fps) - fps) > 1e-3:
        raise RuntimeError(f"fps must be a positive integer, got {args.fps}")
    camera_bindings = base.camera_bindings_from_args(args)
    image_keys = [str(binding["key"]) for binding in camera_bindings]
    episode_dirs = base.collect_episode_dirs(session_dir, max_episodes=int(args.max_episodes or 0))
    state_topics = {"left": args.left_state_topic, "right": args.right_state_topic}
    action_topics = {"left": args.left_action_topic, "right": args.right_action_topic}
    resize_hw = None if args.no_resize else (int(args.image_height), int(args.image_width))
    dataset: Any | None = None
    features: dict[str, Any] | None = None
    reports: list[dict[str, Any]] = []
    skipped: list[dict[str, str]] = []
    total_frames = 0
    try:
        for source_index, episode_dir in enumerate(episode_dirs):
            print(f"[dual-export] read {episode_dir.name} ({source_index + 1}/{len(episode_dirs)})", flush=True)
            try:
                streams = read_dual_episode_streams(
                    episode_dir=episode_dir,
                    camera_bindings=camera_bindings,
                    state_topics=state_topics,
                    action_topics=action_topics,
                    use_bag_time=bool(args.use_bag_time),
                    resize_hw=resize_hw,
                )
                sampled = sample_dual_episode(
                    streams,
                    max_sync_lag_sec=float(args.max_sync_lag_sec),
                    action_offset_steps=int(args.action_offset_steps),
                    max_frames=int(args.max_frames_per_episode or 0),
                )
                if int(sampled["kept_frames"]) < int(args.min_frames):
                    raise RuntimeError(f"too few synchronized frames: kept={sampled['kept_frames']} min={args.min_frames}")
            except Exception as exc:
                if args.skip_bad_episodes:
                    skipped.append({"episode_dir": str(episode_dir), "error": str(exc)})
                    print(f"[dual-export][skip] {episode_dir.name}: {exc}", flush=True)
                    continue
                raise
            if features is None:
                height, width = sampled["frames_bgr_by_key"][image_keys[0]][0].shape[:2]
                features = base.build_features(
                    height=height, width=width, state_dim=DUAL_DIM, action_dim=DUAL_DIM, image_keys=image_keys
                )
                features["observation.state"]["names"] = DUAL_NAMES
                features["action"]["names"] = DUAL_NAMES
                dataset = base.create_lerobot_dataset(
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
            for frame_index, (state, action) in enumerate(zip(sampled["states"], sampled["actions"], strict=True)):
                payload = {"observation.state": state, "action": action, "task": str(args.task)}
                for key in image_keys:
                    payload[key] = cv2.cvtColor(sampled["frames_bgr_by_key"][key][frame_index], cv2.COLOR_BGR2RGB)
                base.add_lerobot_frame(dataset, payload, task=str(args.task))
            dataset.save_episode()
            report = {
                "episode_index": len(reports), "source_episode_index": source_index,
                "source_episode_dir": str(episode_dir), "source_bag_dir": str(streams["bag_dir"]),
                "storage_id": str(streams["storage_id"]), "kept_rows": int(sampled["kept_frames"]),
                "candidate_frames": int(sampled["candidate_frames"]), "dropped_sync_frames": int(sampled["dropped_sync_frames"]),
                "camera_lag": sampled["camera_lag"], "arm_lag": sampled["arm_lag"], "dropped_decode_or_short": streams["dropped"],
            }
            reports.append(report)
            total_frames += int(sampled["kept_frames"])
            print(f"[dual-export] episode_{report['episode_index']:06d}: frames={report['kept_rows']}", flush=True)
        if dataset is None or features is None or not reports:
            raise RuntimeError("no episodes were exported")
        base.finalize_lerobot_dataset(dataset)
        source = {
            "schema": "pure_teleop_mcap_lerobot_dual_arm_v1", "created_at_utc": base.utc_now_iso(),
            "dataset_id": dataset_id, "dataset_root": str(tmp_root), "source_session_dir": str(session_dir),
            "task": str(args.task), "fps": fps, "robot_type": str(args.robot_type),
            "pre_class_export_schema": "dual_arm_vanilla_act_v1", "state_encoding": "left8_then_right8",
            "action_encoding": "left8_then_right8", "arm_order": list(ARM_ORDER), "per_arm_dim": ARM_DIM,
            "state_topics": state_topics, "action_topics": action_topics, "camera_bindings": camera_bindings,
            "features": features, "episodes": reports, "skipped_episodes": skipped,
            "total_episodes": len(reports), "total_frames": total_frames,
        }
        base.write_json(tmp_root / "meta" / "source_pure_teleop_export.json", source)
        info_path = tmp_root / "meta" / "info.json"
        info = base.read_json(info_path)
        info.update({
            "pre_class_export_schema": "dual_arm_vanilla_act_v1", "state_encoding": "left8_then_right8",
            "action_encoding": "left8_then_right8", "arm_order": list(ARM_ORDER), "per_arm_dim": ARM_DIM,
            "camera_bindings": camera_bindings,
        })
        base.write_json(info_path, info)
        base.write_policy_contract(tmp_root, info=info, source=source)
        loaded = LeRobotDataset(dataset_id, root=tmp_root, download_videos=False)
        if int(loaded.num_frames) != total_frames:
            raise RuntimeError(f"LeRobot reload frame mismatch: loaded={loaded.num_frames} expected={total_frames}")
        tmp_root.rename(final_root)
        source["dataset_root"] = str(final_root)
        base.write_json(final_root / "meta" / "source_pure_teleop_export.json", source)
        base.write_policy_contract(final_root, info=info, source=source)
        print(f"[dual-export] done dataset={dataset_id} root={final_root} frames={total_frames}", flush=True)
        return source
    except Exception:
        import shutil
        shutil.rmtree(tmp_root, ignore_errors=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--task", default="pure teleop dual arm")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--camera-topic", action="append", default=[])
    parser.add_argument("--left-state-topic", default="/slave_left/joint_states_single")
    parser.add_argument("--right-state-topic", default="/slave_right/joint_states_single")
    parser.add_argument("--left-action-topic", default="/slave_left/joint_cmd_mux/output")
    parser.add_argument("--right-action-topic", default="/slave_right/joint_cmd_mux/output")
    parser.add_argument("--robot-type", default="nero")
    parser.add_argument("--max-sync-lag-sec", type=float, default=0.08)
    parser.add_argument("--action-offset-steps", type=int, default=0)
    parser.add_argument("--use-bag-time", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--image-width", type=int, default=256)
    parser.add_argument("--image-height", type=int, default=256)
    parser.add_argument("--no-resize", action="store_true")
    parser.add_argument("--video-codec", default="auto", help="Use the library default, or choose a codec if supported by this LeRobot version.")
    parser.add_argument("--image-writer-threads", type=int, default=4)
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
        return 130
    except Exception as exc:
        print(f"[dual-export][FATAL] {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
