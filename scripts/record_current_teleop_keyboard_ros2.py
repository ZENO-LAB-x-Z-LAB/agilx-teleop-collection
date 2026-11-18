#!/usr/bin/env python3
"""Keyboard-driven ROS 2 rosbag recorder for an already-running teleop graph."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


ROOT_DIR = Path(__file__).resolve().parents[1]
SYSTEM_TOPICS = {"/rosout", "/parameter_events"}
TOPIC_PRESETS: Dict[str, Tuple[str, ...]] = {
    "dual_arm": (
        "/slave_left/joint_states_single",
        "/slave_left/gripper_status",
        "/slave_left/joint_cmd_mux/output",
        "/slave_right/joint_states_single",
        "/slave_right/gripper_status",
        "/slave_right/joint_cmd_mux/output",
    ),
    "left_wrist_top": (
        "/slave_left/joint_states_single",
        "/slave_left/gripper_status",
        "/slave_left/joint_cmd_mux/output",
        "/teleop_camera/left/image_raw/compressed",
        "/teleop_camera/top/image_raw/compressed",
    ),
    "right_wrist_top": (
        "/slave_right/joint_states_single",
        "/slave_right/gripper_status",
        "/slave_right/joint_cmd_mux/output",
        "/teleop_camera/right/image_raw/compressed",
        "/teleop_camera/top/image_raw/compressed",
    ),
    "right_wrist_only": (
        "/slave_right/joint_states_single",
        "/slave_right/gripper_status",
        "/slave_right/joint_cmd_mux/output",
        "/teleop_camera/right/image_raw/compressed",
    ),
}


def utc_now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def local_now_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def normalize_topic(topic: str) -> str:
    text = str(topic or "").strip()
    if not text:
        raise ValueError("topic must not be empty")
    if not text.startswith("/"):
        text = "/" + text
    return text.rstrip("/") or "/"


def dedup_keep_order(items: Iterable[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def format_duration(seconds: Optional[float]) -> str:
    if seconds is None:
        return "unknown"
    seconds = max(0.0, float(seconds))
    minutes, sec = divmod(seconds, 60.0)
    hours, minutes = divmod(int(minutes), 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{sec:04.1f}"
    return f"{minutes:02d}:{sec:04.1f}"


def sanitize_topic(topic: str) -> str:
    return topic.strip("/").replace("/", "_").replace(":", "_") or "root"


def parse_topic_list(output: str) -> Dict[str, str]:
    topics: Dict[str, str] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line or not line.startswith("/"):
            continue
        if " " in line:
            topic, raw_type = line.split(" ", 1)
            msg_type = raw_type.strip().strip("[]")
        else:
            topic, msg_type = line, ""
        topics[topic] = msg_type
    return dict(sorted(topics.items()))


def parse_json_arg(text: str) -> Dict[str, Any]:
    raw = str(text or "").strip()
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
    except Exception:
        path = Path(raw).expanduser()
        if not path.is_file():
            return {"parse_error": f"not JSON and file not found: {raw}"}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            return {"parse_error": str(exc), "path": str(path)}
    return payload if isinstance(payload, dict) else {"value": payload}


def topic_is_hidden(topic: str) -> bool:
    return topic.startswith("/_")


def topic_matches_default(topic: str, msg_type: str, teleop_ns: str, robot_ns: str) -> bool:
    teleop_prefix = normalize_topic(teleop_ns) + "/"
    robot_prefix = normalize_topic(robot_ns) + "/"

    if topic in ("/tf", "/tf_static"):
        return True
    if topic.startswith("/camera") or topic.startswith("/realsense"):
        return True
    if msg_type in {
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/CameraInfo",
    }:
        return True

    useful_suffixes = (
        "/arm_status",
        "/gripper_status",
        "/gripper_intent",
        "/gripper_haptic_debug",
        "/gripper_lock",
        "/joint_pos_cmd",
        "/vla_joint_cmd",
        "/joint_cmd_mux",
        "/joint_states",
        "/joint_states_single",
        "/joint_states_compensated",
    )
    useful_infixes = ("/feedback/", "/control/", "/gravity/")
    # Preserve the complete per-taxel measurement in the JointState message.
    # The scalar contact_force topic is derived and is intentionally excluded
    # from raw tactile recordings.
    useful_tactile_match = bool(
        re.match(
            # A recording may map a GUI-selected physical sensor to the
            # generic ``gripper`` slot.  Keep accepting the older per-finger
            # names as well, without making the recorder care about a side.
            r"^/slave_(left|right)/tactile/[^/]+/joint_states$",
            topic,
        )
    )
    if "/tactile/" in topic:
        return bool(useful_tactile_match)
    if topic.startswith(teleop_prefix) or topic.startswith(robot_prefix):
        if (
            topic.endswith(useful_suffixes)
            or any(part in topic for part in useful_infixes)
            or bool(useful_tactile_match)
        ):
            return True

    master_slave_match = re.match(
        r"^/(master|slave)_(left|right)/(arm_status|gripper_status|gripper_intent|"
        r"gripper_haptic_debug|gripper_lock|joint_pos_cmd|vla_joint_cmd|joint_cmd_mux|joint_states|"
        r"joint_states_single|joint_states_compensated)($|/)",
        topic,
    )
    return bool(master_slave_match or useful_tactile_match)


class RawTerminal:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY; keyboard controls need a terminal")
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class Recorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.session_root = Path(args.session_root).expanduser().resolve()
        self.session_name = args.session_name or f"{args.prefix}_{local_now_tag()}"
        if self.session_name in {".", ".."} or Path(self.session_name).name != self.session_name:
            raise ValueError("session name must be a single directory name")
        self.session_dir = self.session_root / self.session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.recording = False
        self.shutdown_requested = False
        self.signal_requested = False
        self.rosbag_proc: Optional[subprocess.Popen] = None
        self.rosbag_log_handle = None
        self.current_episode_name: Optional[str] = None
        self.current_episode_dir: Optional[Path] = None
        self.current_started_utc: Optional[str] = None
        self.current_started_monotonic: Optional[float] = None
        self.current_topics: List[str] = []
        self.current_topic_types: Dict[str, str] = {}
        self.current_preflight: Dict[str, object] = {}
        self.last_space_ts = 0.0
        self.last_status_ts = 0.0
        self.next_episode_index = self.scan_next_episode_index()

        self.extra_topics = [normalize_topic(t) for t in args.extra_topic]
        self.required_topics = [normalize_topic(t) for t in args.required_topic]
        self.topic_preset = str(args.topic_preset or "default").strip()
        self.preset_topics = [normalize_topic(t) for t in TOPIC_PRESETS.get(self.topic_preset, ())]
        if self.preset_topics:
            self.required_topics.extend(self.preset_topics)
        if args.require_robot_topics:
            robot_ns = normalize_topic(args.robot_ns)
            self.required_topics.extend(
                [
                    f"{robot_ns}/joint_states_single",
                    f"{robot_ns}/arm_status",
                ]
            )
            self.required_topics = dedup_keep_order(self.required_topics)
        self.required_effort_topics: List[str] = []
        if args.require_joint_effort:
            self.required_effort_topics = [
                topic for topic in self.preset_topics if topic.endswith("/joint_states_single")
            ]
            if not self.required_effort_topics:
                self.required_effort_topics = [f"{normalize_topic(args.robot_ns)}/joint_states_single"]
        self.focus_topics: List[str] = []
        if args.focus_left_arm_load or args.focus_arm_load:
            for topic in (
                args.left_arm_status_topic,
                args.left_arm_load_joint_topic,
                args.arm_status_topic,
                args.arm_load_joint_topic,
            ):
                if topic:
                    self.focus_topics.append(normalize_topic(topic))
            self.focus_topics = dedup_keep_order(self.focus_topics)

        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, signum, frame) -> None:  # noqa: ANN001
        del signum, frame
        self.signal_requested = True
        self.shutdown_requested = True
        if self.recording:
            self.stop_episode("signal_interrupt")

    def scan_next_episode_index(self) -> int:
        used = set()
        if self.session_dir.exists():
            for entry in self.session_dir.iterdir():
                match = re.match(r"^episode_(\d+)$", entry.name)
                if match:
                    used.add(int(match.group(1)))
        idx = 1
        while idx in used:
            idx += 1
        return idx

    def remove_episode_dir(self, episode_dir: Path, reason: str) -> None:
        try:
            if episode_dir.is_dir():
                shutil.rmtree(episode_dir)
                print(f"[record] removed {episode_dir.name}: {reason}", flush=True)
        except Exception as exc:
            print(f"[record][WARN] could not remove {episode_dir}: {exc}", flush=True)

    def cleanup_empty_session_dir(self) -> None:
        try:
            if self.session_dir.is_dir() and not any(self.session_dir.iterdir()):
                self.session_dir.rmdir()
                print(f"[record] removed empty session: {self.session_dir}", flush=True)
        except Exception as exc:
            print(f"[record][WARN] could not remove empty session {self.session_dir}: {exc}", flush=True)

    def rosbag_message_count(self, info_text: str) -> Optional[int]:
        match = re.search(r"(?im)^\s*Messages:\s*(\d+)\s*$", str(info_text or ""))
        return int(match.group(1)) if match else None

    def rosbag_output_has_payload(self, output_path: Path) -> bool:
        if not output_path.exists():
            return False
        if output_path.is_file():
            return output_path.stat().st_size > 0
        for path in output_path.rglob("*"):
            if path.is_file() and path.suffix in {".mcap", ".db3"} and path.stat().st_size > 0:
                return True
        return False

    def reserve_next_episode_slot(self) -> Tuple[int, str, Path]:
        idx = self.scan_next_episode_index()
        while True:
            name = f"episode_{idx:03d}"
            path = self.session_dir / name
            try:
                path.mkdir()
            except FileExistsError:
                idx += 1
                continue
            return idx, name, path

    def run_ros2(self, args: Sequence[str], timeout: Optional[float] = None) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["ros2", *args],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )

    def snapshot_topics(self) -> Dict[str, str]:
        proc = self.run_ros2(["topic", "list", "-t"], timeout=10.0)
        if proc.returncode != 0:
            raise RuntimeError((proc.stderr or proc.stdout or "ros2 topic list failed").strip())
        return parse_topic_list(proc.stdout)

    def topic_has_sample(self, topic: str) -> bool:
        timeout_sec = max(0.1, float(self.args.required_topic_sample_timeout_sec))
        try:
            proc = self.run_ros2(["topic", "echo", topic, "--once"], timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return False
        return proc.returncode == 0

    def topic_has_effort_sample(self, topic: str) -> bool:
        """Return whether a JointState sample has all eight torque values."""
        timeout_sec = max(0.1, float(self.args.required_topic_sample_timeout_sec))
        try:
            proc = self.run_ros2(["topic", "echo", topic, "--once"], timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return False
        if proc.returncode != 0:
            return False
        output = proc.stdout or ""
        inline = re.search(r"(?m)^effort:\s*\[([^\]]+)\]\s*$", output)
        if inline:
            effort_values = inline.group(1)
        else:
            block = re.search(r"(?ms)^effort:\s*\n((?:^[ \t]*-\s*[^\n]+\n?)+)", output)
            effort_values = block.group(1) if block else ""
        return len(re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", effort_values)) >= 8

    def topic_allowed(self, topic: str) -> bool:
        if not self.args.include_system and topic in SYSTEM_TOPICS:
            return False
        if not self.args.include_hidden and topic_is_hidden(topic):
            return False
        for pattern in self.args.exclude_regex:
            if re.search(pattern, topic):
                return False
        return True

    def select_topics_once(self) -> Tuple[List[str], Dict[str, str], Dict[str, object]]:
        topic_types = self.snapshot_topics()
        selected: List[str] = []

        if self.preset_topics:
            for topic in self.preset_topics:
                if topic in topic_types and self.topic_allowed(topic):
                    selected.append(topic)
        else:
            for topic, msg_type in topic_types.items():
                if not self.topic_allowed(topic):
                    continue
                if self.args.all_current:
                    selected.append(topic)
                    continue
                if topic_matches_default(topic, msg_type, self.args.teleop_ns, self.args.robot_ns):
                    selected.append(topic)
                    continue
                if any(re.search(pattern, topic) for pattern in self.args.include_regex):
                    selected.append(topic)

        explicit_missing: List[str] = []
        for topic in [*self.extra_topics, *self.focus_topics]:
            if topic in topic_types:
                selected.append(topic)
            else:
                explicit_missing.append(topic)

        selected = dedup_keep_order(selected)
        missing_required = [topic for topic in self.required_topics if topic not in topic_types]
        for topic in self.required_topics:
            if topic in topic_types:
                selected.append(topic)
        required_without_sample = [
            topic
            for topic in self.required_topics
            if topic in topic_types and not self.topic_has_sample(topic)
        ]
        required_effort_without_sample = [
            topic
            for topic in self.required_effort_topics
            if topic not in topic_types or not self.topic_has_effort_sample(topic)
        ]

        preflight = {
            "current_topic_count": len(topic_types),
            "selected_topic_count": len(selected),
            "missing_required_topics": missing_required,
            "required_topics_without_sample": required_without_sample,
            "required_effort_topics_without_sample": required_effort_without_sample,
            "explicit_missing_topics": explicit_missing,
            "mode": self.topic_preset if self.preset_topics else "all_current" if self.args.all_current else "default",
        }
        return dedup_keep_order(selected), topic_types, preflight

    def wait_preflight(self) -> Tuple[bool, List[str], Dict[str, str], Dict[str, object]]:
        deadline = time.monotonic() + max(0.1, float(self.args.wait_timeout_sec))
        last_preflight: Dict[str, object] = {}
        last_topic_types: Dict[str, str] = {}
        last_selected: List[str] = []

        while time.monotonic() < deadline:
            selected, topic_types, preflight = self.select_topics_once()
            last_selected, last_topic_types, last_preflight = selected, topic_types, preflight
            if (
                selected
                and not preflight["missing_required_topics"]
                and not preflight["required_topics_without_sample"]
                and not preflight["required_effort_topics_without_sample"]
            ):
                return True, selected, topic_types, preflight
            time.sleep(0.25)

        return False, last_selected, last_topic_types, last_preflight

    def check_disk(self) -> Tuple[bool, float]:
        usage = shutil.disk_usage(self.session_dir)
        free_gb = usage.free / (1024 ** 3)
        return free_gb >= float(self.args.min_free_gb), free_gb

    def build_record_cmd(self, output_path: Path, topics: Sequence[str]) -> List[str]:
        cmd = [
            "ros2",
            "bag",
            "record",
            "--storage",
            self.args.storage,
            "--output",
            str(output_path),
            "--max-cache-size",
            str(self.args.max_cache_size),
            "--disable-keyboard-controls",
            "--log-level",
            "warn",
        ]
        if self.args.storage == "mcap" and self.args.storage_preset != "none":
            cmd.extend(["--storage-preset-profile", self.args.storage_preset])
        if self.args.max_bag_size:
            cmd.extend(["--max-bag-size", str(self.args.max_bag_size)])
        if self.args.max_bag_duration:
            cmd.extend(["--max-bag-duration", str(self.args.max_bag_duration)])
        cmd.extend(["--topics", *topics])
        return cmd

    def start_episode(self) -> None:
        if self.recording:
            return

        disk_ok, free_gb = self.check_disk()
        if not disk_ok:
            print(
                f"[record] Disk free too low: {free_gb:.2f} GB < {self.args.min_free_gb:.2f} GB",
                flush=True,
            )
            return

        print(f"[record] SPACE received, running ROS 2 topic preflight (timeout={self.args.wait_timeout_sec:.1f}s)")
        ok, topics, topic_types, preflight = self.wait_preflight()
        if not ok:
            print("[record] Preflight failed", flush=True)
            missing = preflight.get("missing_required_topics") or []
            if missing:
                print(f"[record] missing required topics: {missing}", flush=True)
            without_sample = preflight.get("required_topics_without_sample") or []
            if without_sample:
                print(f"[record] required topics without samples: {without_sample}", flush=True)
            without_effort = preflight.get("required_effort_topics_without_sample") or []
            if without_effort:
                print(f"[record] required JointState.effort samples unavailable: {without_effort}", flush=True)
            print(f"[record] selected_topics={len(topics)} current_topics={len(topic_types)}", flush=True)
            return

        idx, episode_name, episode_dir = self.reserve_next_episode_slot()
        output_path = episode_dir / "episode"
        topic_snapshot_path = episode_dir / "selected_topics.txt"
        topic_snapshot_path.write_text("\n".join(topics) + "\n", encoding="utf-8")
        (episode_dir / "current_topics.json").write_text(
            json.dumps(topic_types, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        rosbag_log_path = episode_dir / "rosbag_record.log"
        self.rosbag_log_handle = rosbag_log_path.open("w", encoding="utf-8")

        cmd = self.build_record_cmd(output_path, topics)
        self.rosbag_proc = subprocess.Popen(
            cmd,
            stdout=self.rosbag_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        time.sleep(0.4)
        if self.rosbag_proc.poll() is not None:
            self.rosbag_log_handle.flush()
            self.rosbag_log_handle.close()
            self.rosbag_log_handle = None
            tail = rosbag_log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-12:]
            print("[record] ros2 bag record exited early:", flush=True)
            for line in tail:
                print(f"  {line}", flush=True)
            self.rosbag_proc = None
            self.remove_episode_dir(episode_dir, "ros2 bag record exited before recording")
            self.next_episode_index = self.scan_next_episode_index()
            return

        self.recording = True
        self.next_episode_index = self.scan_next_episode_index()
        self.current_episode_name = episode_name
        self.current_episode_dir = episode_dir
        self.current_started_utc = utc_now_iso()
        self.current_started_monotonic = time.monotonic()
        self.current_topics = list(topics)
        self.current_topic_types = dict(topic_types)
        self.current_preflight = dict(preflight)
        self.last_status_ts = 0.0

        print(f"[record] START {episode_name}", flush=True)
        print(f"[record] start_utc={self.current_started_utc}", flush=True)
        print(f"[record] episode_path={episode_dir}", flush=True)
        print(f"[record] output={output_path}", flush=True)
        print(f"[record] topics={len(topics)}", flush=True)

    def request_process_stop(self, proc: subprocess.Popen, name: str) -> int:
        if proc.poll() is not None:
            return int(proc.returncode or 0)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            proc.terminate()
        try:
            return proc.wait(timeout=float(self.args.stop_timeout_sec))
        except subprocess.TimeoutExpired:
            print(f"[record] {name} did not stop on SIGINT; sending SIGTERM", flush=True)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                proc.terminate()
        try:
            return proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print(f"[record] {name} still alive; sending SIGKILL", flush=True)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                proc.kill()
            return proc.wait(timeout=5.0)

    def write_metadata(
        self,
        episode_dir: Path,
        output_path: Path,
        stop_reason: str,
        started_utc: Optional[str],
        ended_utc: str,
        duration_sec: Optional[float],
        returncode: Optional[int],
        rosbag_info_ok: bool,
        rosbag_info_path: Path,
    ) -> None:
        metadata = {
            "schema": "ros2_current_teleop_keyboard_v1",
            "session_name": self.session_name,
            "episode_name": episode_dir.name,
            "episode_dir": str(episode_dir),
            "output_path": str(output_path),
            "started_at_utc": started_utc,
            "ended_at_utc": ended_utc,
            "duration_sec": duration_sec,
            "stop_reason": stop_reason,
            "rosbag_returncode": returncode,
            "rosbag_info_ok": rosbag_info_ok,
            "rosbag_info_path": str(rosbag_info_path),
            "recorded_topics": list(self.current_topics),
            "topic_types": {topic: self.current_topic_types.get(topic, "") for topic in self.current_topics},
            "preflight": dict(self.current_preflight),
            "teleop_ns": self.args.teleop_ns,
            "robot_ns": self.args.robot_ns,
            "notes": self.args.notes,
            "camera_profile_binding": parse_json_arg(self.args.camera_profile_binding_json),
            "focus_left_arm_load": bool(self.args.focus_left_arm_load),
            "focus_arm_load": bool(self.args.focus_arm_load),
            "focus_topics": list(self.focus_topics),
            "left_load_high_effort_threshold": self.args.left_load_high_effort_threshold,
            "health_big_gap_min_sec": self.args.health_big_gap_min_sec,
            "health_big_gap_max_count": self.args.health_big_gap_max_count,
            "health_big_gap_median_mult": self.args.health_big_gap_median_mult,
            "expected_camera_fps": self.args.expected_camera_fps,
            "dropframe_warn_ratio": self.args.dropframe_warn_ratio,
            "script_args": vars(self.args),
        }
        (episode_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def stop_episode(self, stop_reason: str) -> None:
        if not self.recording:
            return

        episode_dir = self.current_episode_dir
        proc = self.rosbag_proc
        if episode_dir is None or proc is None:
            return

        output_path = episode_dir / "episode"
        started_utc = self.current_started_utc
        duration_sec = None
        if self.current_started_monotonic is not None:
            duration_sec = time.monotonic() - self.current_started_monotonic
        ended_utc = utc_now_iso()

        print(f"[record] STOP {episode_dir.name} reason={stop_reason}", flush=True)
        returncode = self.request_process_stop(proc, "ros2 bag record")

        if self.rosbag_log_handle is not None:
            try:
                self.rosbag_log_handle.flush()
            finally:
                self.rosbag_log_handle.close()
                self.rosbag_log_handle = None

        rosbag_info_path = episode_dir / "rosbag_info.txt"
        try:
            info_proc = self.run_ros2(["bag", "info", str(output_path)], timeout=30.0)
            rosbag_info_ok = info_proc.returncode == 0
            rosbag_info_text = info_proc.stdout
            if info_proc.stderr:
                rosbag_info_text += "\n[stderr]\n" + info_proc.stderr
        except subprocess.TimeoutExpired:
            rosbag_info_ok = False
            rosbag_info_text = "ros2 bag info timed out after 30s\n"
        rosbag_info_path.write_text(rosbag_info_text, encoding="utf-8")
        message_count = self.rosbag_message_count(rosbag_info_text) if rosbag_info_ok else None
        empty_recording = message_count == 0 or (
            not rosbag_info_ok and not self.rosbag_output_has_payload(output_path)
        )

        self.write_metadata(
            episode_dir=episode_dir,
            output_path=output_path,
            stop_reason=stop_reason,
            started_utc=started_utc,
            ended_utc=ended_utc,
            duration_sec=duration_sec,
            returncode=returncode,
            rosbag_info_ok=rosbag_info_ok,
            rosbag_info_path=rosbag_info_path,
        )

        self.recording = False
        self.rosbag_proc = None
        self.current_episode_name = None
        self.current_episode_dir = None
        self.current_started_utc = None
        self.current_started_monotonic = None
        self.current_topics = []
        self.current_topic_types = {}
        self.current_preflight = {}
        self.next_episode_index = self.scan_next_episode_index()

        print(f"[record] stop_utc={ended_utc}", flush=True)
        print(f"[record] duration={format_duration(duration_sec)}", flush=True)
        if empty_recording:
            self.remove_episode_dir(episode_dir, "no messages recorded")
            self.next_episode_index = self.scan_next_episode_index()
        else:
            print(f"[record] metadata={episode_dir / 'metadata.json'}", flush=True)
        print(f"[record] next=episode_{self.next_episode_index:03d}", flush=True)

    def emit_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_ts < float(self.args.record_status_interval_sec):
            return
        self.last_status_ts = now
        if self.recording and self.current_episode_name:
            elapsed = None
            if self.current_started_monotonic is not None:
                elapsed = time.monotonic() - self.current_started_monotonic
            print(
                f"[record] ACTIVE {self.current_episode_name} "
                f"elapsed={format_duration(elapsed)} topics={len(self.current_topics)}",
                flush=True,
            )

    def print_banner(self) -> None:
        print("Keyboard ROS 2 teleop recorder", flush=True)
        print("Controls: SPACE=start/stop episode, Q=quit", flush=True)
        print(f"Session: {self.session_dir}", flush=True)
        print(f"Teleop namespaces: teleop={self.args.teleop_ns} robot={self.args.robot_ns}", flush=True)
        print(f"Next episode: episode_{self.next_episode_index:03d}", flush=True)
        print("", flush=True)

    def run(self) -> None:
        self.print_banner()
        if self.args.dry_run:
            topics, topic_types, preflight = self.select_topics_once()
            print(f"[dry-run] current_topics={len(topic_types)} selected_topics={len(topics)}")
            for topic in topics:
                print(f"  {topic} [{topic_types.get(topic, '')}]")
            if preflight.get("missing_required_topics"):
                print(f"[dry-run] missing_required_topics={preflight['missing_required_topics']}")
            return

        try:
            with RawTerminal():
                while not self.shutdown_requested:
                    readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if not readable:
                        self.emit_status()
                        continue
                    ch = sys.stdin.read(1)
                    now = time.monotonic()
                    if ch == " ":
                        if now - self.last_space_ts < float(self.args.debounce_ms) / 1000.0:
                            continue
                        self.last_space_ts = now
                        if self.recording:
                            self.stop_episode("user_stop")
                        else:
                            self.start_episode()
                    elif ch in ("q", "Q"):
                        if self.recording:
                            self.stop_episode("user_stop")
                        self.shutdown_requested = True
                    self.emit_status(force=False)
        finally:
            if self.recording:
                reason = "signal_interrupt" if self.signal_requested else "user_stop"
                self.stop_episode(reason)
            self.cleanup_empty_session_dir()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard-driven ROS 2 rosbag recorder")
    default_session_root = str((ROOT_DIR / "rosbag2_data" / "teleop_keyboard").resolve())
    parser.add_argument("--session-root", default=default_session_root)
    parser.add_argument("--prefix", default="teleop")
    parser.add_argument("--session-name", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument("--camera-profile-binding-json", default="")
    parser.add_argument("--teleop-ns", default=os.environ.get("TELEOP_NS", "master_right"))
    parser.add_argument("--robot-ns", default=os.environ.get("ROBOT_NS", "slave_right"))
    parser.add_argument("--wait-timeout-sec", type=float, default=10.0)
    parser.add_argument("--required-topic-sample-timeout-sec", type=float, default=3.0)
    parser.add_argument("--min-free-gb", type=float, default=5.0)
    parser.add_argument("--debounce-ms", type=int, default=250)
    parser.add_argument("--record-status-interval-sec", type=float, default=1.0)
    parser.add_argument("--stop-timeout-sec", type=float, default=10.0)
    parser.add_argument("--save-workers", type=int, default=1, help="Accepted for ROS1 command compatibility.")

    parser.add_argument("--all-current", action="store_true")
    parser.add_argument("--include-system", action="store_true")
    parser.add_argument("--include-hidden", action="store_true")
    parser.add_argument("--extra-topic", action="append", default=[])
    parser.add_argument("--required-topic", action="append", default=[])
    parser.add_argument(
        "--no-require-robot-topics",
        dest="require_robot_topics",
        action="store_false",
        help="Do not require /<robot-ns>/joint_states_single and /<robot-ns>/arm_status before recording.",
    )
    parser.set_defaults(require_robot_topics=True)
    parser.add_argument("--include-regex", action="append", default=[])
    parser.add_argument("--exclude-regex", action="append", default=[])
    parser.add_argument(
        "--topic-preset",
        choices=("default", *sorted(TOPIC_PRESETS)),
        default="default",
        help="Restrict recording to a named preset. right_wrist_only records right 8D state/action plus only the right wrist camera.",
    )
    parser.add_argument(
        "--require-joint-effort",
        action="store_true",
        help="Require one sampled JointState.effort vector for every arm in the selected preset before starting an episode.",
    )

    parser.add_argument("--storage", choices=("mcap", "sqlite3"), default=os.environ.get("ROSBAG_STORAGE", "mcap"))
    parser.add_argument(
        "--storage-preset",
        choices=("none", "fastwrite", "zstd_fast", "zstd_small"),
        default=os.environ.get("ROSBAG_STORAGE_PRESET", "fastwrite"),
    )
    parser.add_argument("--max-cache-size", type=int, default=int(os.environ.get("ROSBAG_MAX_CACHE_SIZE", "536870912")))
    parser.add_argument("--max-bag-size", type=int, default=int(os.environ.get("ROSBAG_MAX_BAG_SIZE", "0")))
    parser.add_argument("--max-bag-duration", type=int, default=int(os.environ.get("ROSBAG_MAX_BAG_DURATION", "0")))

    parser.add_argument("--focus-arm-load", action="store_true")
    parser.add_argument("--arm-status-topic", default="/slave_right/arm_status")
    parser.add_argument("--arm-load-joint-topic", default="/slave_right/joint_states_single")
    parser.add_argument("--focus-left-arm-load", action="store_true")
    parser.add_argument("--left-arm-status-topic", default="")
    parser.add_argument("--left-arm-load-joint-topic", default="")
    parser.add_argument("--left-load-high-effort-threshold", type=float, default=6.0)

    parser.add_argument("--expected-camera-fps", type=float, default=30.0)
    parser.add_argument("--dropframe-warn-ratio", type=float, default=0.85)
    parser.add_argument("--health-big-gap-min-sec", type=float, default=0.06)
    parser.add_argument("--health-big-gap-median-mult", type=float, default=3.0)
    parser.add_argument("--health-big-gap-max-count", type=int, default=3)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if shutil.which("ros2") is None:
        print("ERROR: ros2 not found. Source scripts/use_teleop.sh or run the .sh wrapper.", file=sys.stderr)
        return 2
    try:
        Recorder(args).run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
