"""Operator menus for NERO teleoperation and demonstration recording."""


from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


REPO_ROOT = Path(__file__).resolve().parents[3]


DEFAULT_ROS2_PYTHON = Path(os.environ.get("PURE_TELEOP_ROS2_PYTHON") or sys.executable)


DEFAULT_STATE_PATH = Path(os.environ.get("PURE_TELEOP_STATE_PATH") or REPO_ROOT / "data" / "pure_teleop" / "state.json").expanduser()


DEFAULT_ROSBAG_ROOT = Path(
    os.environ.get("PURE_TELEOP_ROSBAG_ROOT") or REPO_ROOT / "data" / "pure_teleop" / "rosbag2"
).expanduser()


PURE_TELEOP_TITLE = "NERO 遥操作与示范采集"


DEFAULT_TACTILE_PORTS = tuple(
    port.strip()
    for port in re.split(r"[,\s]+", os.environ.get("PURE_TELEOP_TACTILE_PORTS", "/dev/ttyACM0,/dev/ttyACM1"))
    if port.strip()
) or ("/dev/ttyACM0", "/dev/ttyACM1")


SINGLE_RIGHT_TACTILE_TARGET = "slave_right:gripper"


@dataclass(frozen=True)
class MenuOption:
    choice: str
    label: str
    value: str = ""
    aliases: tuple[str, ...] = ()


@dataclass(frozen=True)
class MenuSpec:
    title: str
    options: tuple[MenuOption, ...]
    label: str = "请选择"
    default: str = "1"
    remember_key: str = ""
    suffix: str = "b 返回"
    cancel_values: tuple[str, ...] = ("b", "q", "quit", "exit", "退出")
    cancel_label: str = "[b] 返回"


def local_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"JSON root is not object: {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as fp:
        json.dump(dict(payload), fp, ensure_ascii=False, indent=2, sort_keys=True)
        fp.write("\n")
    Path(fp.name).replace(path)


def default_state() -> dict[str, Any]:
    return {
        "schema": "pure_teleop_tui_state_v1",
        "ui_memory": {},
        "paths": {
            "rosbag_root": str(DEFAULT_ROSBAG_ROOT),
        },
        "activation": {"last_run_tag": ""},
    }


def load_state(path: Path) -> dict[str, Any]:
    path = Path(path).expanduser()
    if not path.is_file():
        return default_state()
    try:
        state = read_json(path)
    except Exception:
        return default_state()
    base = default_state()
    for key, value in base.items():
        state.setdefault(key, value)
    if not isinstance(state.get("paths"), dict):
        state["paths"] = dict(base["paths"])
    for key, value in base["paths"].items():
        state["paths"].setdefault(key, value)
    return state


def save_state(path: Path, state: dict[str, Any]) -> None:
    write_json(path, state)


def memory_default(state: dict[str, Any], key: str, fallback: str, choices: Sequence[str] = ()) -> str:
    if not key:
        return fallback
    memory = state.setdefault("ui_memory", {})
    if isinstance(memory, dict):
        value = str(memory.get(key) or "").strip()
        if value and (not choices or value in set(choices)):
            return value
    return fallback


def remember_choice(state: dict[str, Any], key: str, value: str) -> None:
    if not key:
        return
    memory = state.setdefault("ui_memory", {})
    if isinstance(memory, dict):
        memory[key] = str(value)


def choice_menu_value(spec: MenuSpec, choice: str) -> str:
    for option in spec.options:
        if option.choice == str(choice):
            return option.value or option.choice
    raise KeyError(choice)


def _normalise_choice(raw: str, spec: MenuSpec) -> str | None:
    raw = str(raw or "").strip().lower()
    for option in spec.options:
        if raw == option.choice.lower() or raw in {alias.lower() for alias in option.aliases}:
            return option.choice
    return None


def print_choice_menu(spec: MenuSpec) -> None:
    if spec.title:
        print(spec.title)
    for option in spec.options:
        print(f"  [{option.choice}] {option.label}")
    if spec.cancel_label:
        print(spec.cancel_label)


def prompt_choice_menu(state: dict[str, Any], spec: MenuSpec, *, default_choice: str | None = None) -> str | None:
    print_choice_menu(spec)
    choices = tuple(option.choice for option in spec.options)
    default = memory_default(state, spec.remember_key, str(default_choice or spec.default or ""), choices)
    suffix = f", {spec.suffix}" if spec.suffix else ""
    cancel_values = {str(value).lower() for value in spec.cancel_values}
    while True:
        raw = input(f"{spec.label} [默认 {default}{suffix}]: ").strip().lower()
        if raw in cancel_values:
            return None
        choice = _normalise_choice(raw or default, spec)
        if choice is not None:
            remember_choice(state, spec.remember_key, choice)
            return choice
        print("无效选择。")


def prompt_text(label: str, default: str = "", *, cancel_values: Sequence[str] = ("b", "q")) -> str | None:
    suffix = f" [默认 {default}]" if default else ""
    raw = input(f"{label}{suffix}: ").strip()
    if raw.lower() in {str(value).strip().lower() for value in cancel_values}:
        return None
    return raw if raw else default


def pause(message: str = "\n按 Enter 继续...") -> None:
    input(message)


def run_interactive(cmd: Sequence[str], *, env: Mapping[str, str] | None = None, print_command: bool = True) -> int:
    merged_env = os.environ.copy()
    if env:
        merged_env.update({str(key): str(value) for key, value in env.items()})
    if print_command:
        print(" ".join(shlex.quote(str(item)) for item in cmd))
    return int(subprocess.call([str(item) for item in cmd], cwd=str(REPO_ROOT), env=merged_env))


def shell_export(name: str, value: str) -> str:
    return f"export {name}={shlex.quote(str(value))}"


def gripper_width_to_raw(width_m: float, *, val_multiple: float = 1.0) -> int:
    width = max(0.0, min(0.1, float(width_m)))
    multiple = max(1e-6, float(val_multiple))
    return max(0, min(100000, int(round(width * 1_000_000.0 * multiple))))


def latest_child_dir(root: Path) -> Path | None:
    root = Path(root).expanduser()
    if not root.is_dir():
        return None
    dirs = [path for path in root.iterdir() if path.is_dir()]
    return max(dirs, key=lambda p: p.stat().st_mtime) if dirs else None


def next_episode_name(session_dir: Path) -> str:
    used = set()
    session_dir = Path(session_dir).expanduser()
    if session_dir.is_dir():
        for entry in session_dir.iterdir():
            match = re.match(r"^episode_(\d+)$", entry.name)
            if match:
                used.add(int(match.group(1)))
    idx = 1
    while idx in used:
        idx += 1
    return f"episode_{idx:03d}"


def activation_mode_role(mode: str) -> str:
    return "follower" if str(mode) == "1" else "leader" if str(mode) == "2" else ""


def activation_scope_label(scope: str) -> str:
    if scope == "dual":
        return "双臂+夹爪"
    if scope == "left":
        return "左臂+夹爪"
    return "右臂+夹爪"


def prompt_scope(state: dict[str, Any], *, remember_key: str, allow_dual: bool) -> str | None:
    options = [MenuOption("1", "左臂+夹爪", "left"), MenuOption("2", "右臂+夹爪", "right")]
    if allow_dual:
        options.append(MenuOption("3", "双臂+夹爪", "dual"))
    menu = MenuSpec(title="\n范围:", options=tuple(options), default="2", remember_key=remember_key)
    choice = prompt_choice_menu(state, menu)
    return None if choice is None else choice_menu_value(menu, choice)


def prompt_gripper_min_width_guard(state: dict[str, Any], *, role: str) -> float | None:
    """Optional follower-side minimum gripper command clamp for teleop."""

    if role != "follower":
        return None
    current_default = os.environ.get("PURE_TELEOP_GRIPPER_MIN_WIDTH_DEFAULT", "0.077310")
    menu = MenuSpec(
        title="\n夹爪最小开度保护（follower 端生效）:",
        options=(
            MenuOption("1", "不启用，允许夹爪收到更小开度", "off"),
            MenuOption("2", f"启用，最小开度使用当前读数 {current_default} m", "current"),
            MenuOption("3", "启用，手动输入最小开度 m", "manual"),
        ),
        default="1",
        remember_key="pure.activation.gripper_min_guard",
    )
    choice = prompt_choice_menu(state, menu)
    if choice is None:
        return None
    value = choice_menu_value(menu, choice)
    if value == "off":
        return None
    if value == "current":
        raw = current_default
    else:
        manual_default = memory_default(
            state,
            "pure.activation.gripper_min_width_m",
            current_default,
        )
        raw = prompt_text("gripper 最小开度 m，范围 0.0~0.1", manual_default)
        if raw is None:
            return None
    try:
        width = max(0.0, min(0.1, float(str(raw).strip())))
    except ValueError:
        print(f"[teleop] gripper 最小开度不是数字: {raw!r}，本次不启用保护。")
        return None
    remember_choice(state, "pure.activation.gripper_min_width_m", f"{width:.5f}")
    return width


def build_activation_mux_select_lines(robot_namespaces: Sequence[str]) -> list[str]:
    ordered: list[str] = []
    for namespace in robot_namespaces:
        token = str(namespace or "").strip().strip("/")
        if token and token not in ordered:
            ordered.append(token)
    if not ordered:
        return []
    lines = [
        "nero_select_mux_teleop_later() {",
        "  local robot_ns=\"${1#/}\"",
        "  local topic=\"/${robot_ns}/joint_cmd_mux/select\"",
        "  (",
        "    sleep \"${PURE_TELEOP_MUX_SELECT_DELAY_SEC:-2.0}\"",
        "    set +u",
        "    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash >/dev/null 2>&1 || true",
        f"    source {shlex.quote(str(REPO_ROOT / 'install' / 'setup.bash'))} >/dev/null 2>&1 || true",
        "    set -u",
        "    if command -v ros2 >/dev/null 2>&1; then",
        "      ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \"$topic\" std_msgs/msg/String \"{data: teleop}\" >/dev/null 2>&1 || true",
        "    fi",
        "  ) &",
        "}",
    ]
    lines.extend(f"nero_select_mux_teleop_later {shlex.quote(namespace)}" for namespace in ordered)
    return lines


def can_interface(side: str, role: str) -> str:
    defaults = {("left", "leader"): "can_ml", ("right", "leader"): "can_mr",
                ("left", "follower"): "can_sl", ("right", "follower"): "can_sr"}
    return os.environ.get(f"NERO_{side.upper()}_{role.upper()}_CAN") or defaults[side, role]


def build_activation_command(
    mode: str,
    scope: str,
    *,
    gripper_min_width_m: float | None = None,
) -> tuple[str, list[str], str, list[tuple[str, str]]]:
    if scope not in {"right", "left", "dual"}:
        raise ValueError(f"unsupported activation scope: {scope}")
    role = activation_mode_role(mode)
    if role not in {"follower", "leader"}:
        raise ValueError(f"unsupported activation mode: {mode}")
    script_lines = [
        "set -e",
        f"cd {shlex.quote(str(REPO_ROOT))}",
        "source scripts/use_teleop.sh",
        "teleop_preset_nero_gripper_v112_no_camera",
        shell_export("ENABLE_CMD_MUX", "true"),
        shell_export("CMD_MUX_DEFAULT_SOURCE", "teleop"),
        shell_export("NERO_FIRMWARE_PROFILE", os.environ.get("PURE_TELEOP_FIRMWARE_PROFILE", "v112")),
        shell_export("NERO_ENABLE_TIMEOUT", os.environ.get("PURE_TELEOP_ENABLE_TIMEOUT", "15.0")),
        shell_export("GRIPPER_STARTUP_RECOVERY_ENABLE", "false"),
        shell_export("GRIPPER_RECOVER_SEED_MASTER_OPEN", "false"),
    ]
    rows = [
        ("类型", role),
        ("范围", activation_scope_label(scope)),
        ("project", str(REPO_ROOT)),
        ("preset", "teleop_preset_nero_gripper_v112_no_camera"),
        ("mux", "teleop"),
    ]
    if gripper_min_width_m is not None:
        min_width = max(0.0, min(0.1, float(gripper_min_width_m)))
        min_raw = gripper_width_to_raw(min_width)
        script_lines.extend(
            [
                shell_export("GRIPPER_CMD_LIMIT_ENABLE", "true"),
                shell_export("GRIPPER_CMD_MIN_RAW", str(min_raw)),
                shell_export("GRIPPER_CMD_MAX_RAW", "100000"),
            ]
        )
        rows.extend(
            [
                ("gripper 最小开度保护", f"{min_width:.5f} m"),
                ("gripper_cmd_min_raw", str(min_raw)),
            ]
        )
    if scope in {"right", "left"}:
        teleop_ns = f"master_{scope}"
        robot_ns = f"slave_{scope}"
        can_port = can_interface(scope, role)
        script_lines.append(shell_export("FOLLOWER_CAN_PORT" if role == "follower" else "LEADER_CAN_PORT", can_port))
        script_lines.extend([shell_export("TELEOP_NS", teleop_ns), shell_export("ROBOT_NS", robot_ns)])
        script_lines.extend(build_activation_mux_select_lines([robot_ns]))
        script_lines.append(f"exec ./scripts/run_teleop.sh {role}")
        rows.extend([("CAN", can_port), ("teleop_ns", teleop_ns), ("robot_ns", robot_ns), ("启动脚本", f"./scripts/run_teleop.sh {role}")])
    else:
        for side, label in (("left", "左 CAN"), ("right", "右 CAN")):
            interface = can_interface(side, role)
            script_lines.append(shell_export(f"{role.upper()}_{side.upper()}_CAN_PORT", interface))
            rows.append((label, interface))
        script_lines.extend(build_activation_mux_select_lines(["slave_left", "slave_right"]))
        script_lines.append(f"exec ./scripts/run_teleop_dual.sh {role}")
        rows.extend([("namespace", "master_left/master_right -> slave_left/slave_right"), ("启动脚本", f"./scripts/run_teleop_dual.sh {role}")])
    script = "\n".join(script_lines)
    return f"激活 {role}({activation_scope_label(scope)})", ["bash", "-c", script], script, rows


def build_record_command(
    session_root: Path,
    *,
    record_tactile: bool = False,
    record_torque: bool = False,
    record_preset: str = "",
    session_name: str = "",
    tactile_record_targets: Sequence[str] = (),
    tactile_config_path: Path | None = None,
) -> tuple[list[str], list[tuple[str, str]]]:
    bag_name = str(session_name or "").strip() or f"pure_teleop_current_{local_tag()}"
    session_dir = Path(session_root).expanduser() / bag_name
    session_mode = "继续已有 session" if session_dir.is_dir() else "新建 session"
    cmd = [
        str(REPO_ROOT / "scripts" / "record_current_pure_teleop_rosbag2.sh"),
        "--bag-root",
        str(Path(session_root).expanduser()),
        "--name",
        bag_name,
    ]
    if record_tactile:
        cmd.append("--with-tactile")
    for target in tactile_record_targets:
        cmd.extend(["--tactile-target", str(target)])
    if record_preset == "left_wrist_top":
        cmd.append("--left-wrist-top")
    elif record_preset == "right_wrist_top":
        cmd.append("--right-wrist-top")
    elif record_preset == "right_wrist_only":
        cmd.append("--right-wrist-only")
    elif record_preset == "dual_arm":
        cmd.append("--dual-arm")
    if record_torque:
        cmd.append("--require-joint-effort")
    if record_preset == "left_wrist_top":
        scope_label = "左臂+gripper state/action/effort + left wrist/top camera"
        camera_label = "OpenCV 选择 left/top -> /teleop_camera/{left,top}"
    elif record_preset == "right_wrist_top" and record_tactile:
        scope_label = (
            "右臂+gripper state/action 8D + "
            f"右臂 {len(tactile_record_targets)} 个原始 tactile JointState + right wrist/top camera"
        )
        camera_label = "OpenCV 选择 right/top -> /teleop_camera/{right,top}"
    elif record_preset == "right_wrist_top":
        scope_label = "右臂+gripper state/action 8D，无力传感器 + right wrist/top camera"
        camera_label = "OpenCV 选择 right/top -> /teleop_camera/{right,top}"
    elif record_preset == "right_wrist_only":
        scope_label = "右臂+gripper state/action 8D，无力传感器 + right wrist camera"
        camera_label = "OpenCV 选择 right -> /teleop_camera/right"
    elif record_preset == "dual_arm":
        scope_label = "双臂+gripper state/action + selected camera"
        camera_label = "OpenCV 选择 -> /teleop_camera/* -> RViz"
    else:
        scope_label = "当前 ROS graph 自动选择 topic"
        camera_label = "OpenCV 选择 -> /teleop_camera/* -> RViz"
    if record_tactile and tactile_record_targets:
        tactile_label = "记录原始 JointState: " + ", ".join(str(target) for target in tactile_record_targets)
    elif record_tactile:
        tactile_label = "记录 configured 原始 tactile JointState"
    else:
        tactile_label = "不记录"
    torque_label = "校验并记录 JointState.effort" if record_torque else "不校验 torque（原始 JointState 仍完整保存）"
    rows = [
        ("范围", scope_label),
        ("输出根目录", str(session_root)),
        ("session", bag_name),
        ("session模式", session_mode),
        ("下一集", next_episode_name(session_dir)),
        ("camera", camera_label),
        ("触觉数据", tactile_label),
        ("关节 torque", torque_label),
        ("控制", "SPACE 开始/停止 episode，Q 退出"),
        ("启动脚本", "scripts/record_current_pure_teleop_rosbag2.sh"),
    ]
    if tactile_config_path is not None:
        rows.append(("tactile_config", str(tactile_config_path)))
    return cmd, rows


def tactile_port_candidates() -> list[tuple[str, str]]:
    candidates: list[tuple[str, str]] = []
    by_id_targets: set[str] = set()
    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.is_dir():
        for item in sorted(by_id_dir.iterdir(), key=lambda path: path.name):
            try:
                resolved = str(item.resolve(strict=True))
            except OSError:
                resolved = str(item.resolve(strict=False))
            by_id_targets.add(resolved)
            try:
                target = os.readlink(item)
            except OSError:
                target = resolved
            candidates.append((str(item), f"{item.name} -> {target}"))

    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for item in sorted(Path("/dev").glob(Path(pattern).name), key=lambda path: path.name):
            try:
                resolved = str(item.resolve(strict=True))
            except OSError:
                resolved = str(item.resolve(strict=False))
            if resolved in by_id_targets:
                continue
            candidates.append((str(item), item.name))
    return candidates


def default_detected_tactile_ports() -> list[str]:
    candidates = tactile_port_candidates()
    if candidates:
        return [path for path, _label in candidates]
    return list(DEFAULT_TACTILE_PORTS)


def prompt_single_tactile_port_via_gui(
    state: dict[str, Any],
    *,
    title: str = "选择单 tactile",
) -> str | None:
    """Test all detected ports live and return one healthy selected tactile port."""
    candidates = tactile_port_candidates()
    ports = [path for path, _label in candidates] or default_detected_tactile_ports()
    if not ports:
        print(f"[{title}] 没有检测到 tactile 串口。")
        pause()
        return None
    runtime_dir = REPO_ROOT / "data" / "pure_teleop" / ".runtime"
    selection_path = runtime_dir / f"tactile_selection_{local_tag()}.txt"
    selection_path.unlink(missing_ok=True)
    cmd, rows = build_tactile_gui_command(
        ports,
        int(os.environ.get("PURE_TELEOP_TACTILE_GUI_PERIOD_MS", "80")),
        selection_file=selection_path,
    )
    print_rows(f"[{title}] 实时测试并选择单 tactile", rows)
    print("每个串口会先自动读取 9 个 taxel：绿色为通过且可选，红色为失败且不可选。关闭窗口会取消。")
    rc = run_interactive(cmd, env={"PYTHONNOUSERSITE": "1"}, print_command=True)
    if rc != 0:
        print(f"[{title}] tactile GUI 退出码 {rc}")
        pause()
        return None
    try:
        selected = selection_path.read_text(encoding="utf-8").strip()
    except OSError:
        print(f"[{title}] 未选择 tactile。")
        return None
    finally:
        selection_path.unlink(missing_ok=True)
    if selected not in ports:
        print(f"[{title}] GUI 返回了无效串口: {selected!r}")
        pause()
        return None
    memory = state.setdefault("ui_memory", {})
    if isinstance(memory, dict):
        memory["pure.tactile.single_port"] = selected
    return selected


def parse_tactile_target(raw: str) -> tuple[str, str]:
    text = str(raw or "").strip().strip("/")
    if not text:
        raise ValueError("empty tactile target")
    if ":" in text:
        robot_ns, finger = text.split(":", 1)
    else:
        parts = [part for part in text.split("/") if part]
        if len(parts) >= 4 and parts[1] == "tactile":
            robot_ns, finger = parts[0], parts[2]
        elif len(parts) >= 2:
            robot_ns, finger = parts[0], parts[1]
        else:
            raise ValueError(f"bad tactile target {raw!r}; expected robot:finger")
    robot_ns = robot_ns.strip("/")
    finger = finger.strip("/")
    if not robot_ns or not finger:
        raise ValueError(f"bad tactile target {raw!r}; expected robot:finger")
    return robot_ns, finger


def _yaml_scalar(text: str, name: str, default: str) -> str:
    match = re.search(rf"(?m)^\s*{re.escape(name)}:\s*(.*?)\s*$", text)
    if not match:
        return default
    value = match.group(1).split("#", 1)[0].strip()
    return value if value else default


def _sensors_json_from_tactile_config(text: str) -> list[dict[str, Any]]:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        match = re.match(r"^(\s*)sensors_json:\s*(.*)$", line)
        if not match:
            continue
        base_indent = len(match.group(1))
        tail = match.group(2).strip()
        if tail and tail not in {">", "|", ">-", "|-"}:
            payload = tail
        else:
            block: list[str] = []
            for child in lines[index + 1 :]:
                if not child.strip():
                    block.append("")
                    continue
                child_indent = len(child) - len(child.lstrip(" "))
                if child_indent <= base_indent:
                    break
                block.append(child[base_indent + 2 :] if len(child) >= base_indent + 2 else child.lstrip())
            payload = "\n".join(block).strip()
        sensors = json.loads(payload)
        if not isinstance(sensors, list):
            raise RuntimeError("sensors_json is not a list")
        return [dict(item) for item in sensors if isinstance(item, dict)]
    raise RuntimeError("sensors_json not found")


def make_tactile_record_config(
    *,
    targets: Sequence[str],
    ports: Sequence[str],
    session_name: str,
    publish_rate_hz: int | None = None,
) -> Path:
    if len(ports) != len(targets):
        raise ValueError(f"需要为 {len(targets)} 个 tactile target 选择 {len(targets)} 个端口；当前选择了 {len(ports)} 个")
    src = Path(
        os.environ.get("TACTILE_CONFIG_FILE")
        or REPO_ROOT / "agx_arm_ros" / "src" / "piper_nero_teleop" / "config" / "tactile_sensors.example.yaml"
    ).expanduser()
    if not src.is_file():
        raise FileNotFoundError(f"tactile config not found: {src}")
    text = src.read_text(encoding="utf-8")
    sensors = _sensors_json_from_tactile_config(text)
    port_by_target = {parse_tactile_target(target): str(port) for target, port in zip(targets, ports, strict=True)}
    matched: set[tuple[str, str]] = set()
    for sensor in sensors:
        key = (str(sensor.get("robot_ns", "")).strip("/"), str(sensor.get("finger", "")).strip("/"))
        if key in port_by_target:
            sensor["port"] = port_by_target[key]
            sensor["enabled"] = True
            matched.add(key)
        else:
            sensor["enabled"] = False
    missing = sorted(set(port_by_target) - matched)
    for robot_ns, finger in list(missing):
        if finger != "gripper":
            continue
        template = next(
            (sensor for sensor in sensors if str(sensor.get("robot_ns", "")).strip("/") == robot_ns),
            None,
        )
        if template is None:
            continue
        sensor = dict(template)
        sensor["name"] = f"{robot_ns}_{finger}"
        sensor["finger"] = finger
        sensor["port"] = port_by_target[(robot_ns, finger)]
        sensor["enabled"] = True
        sensors.append(sensor)
        matched.add((robot_ns, finger))
    missing = sorted(set(port_by_target) - matched)
    if missing:
        formatted = ", ".join(f"{robot}:{finger}" for robot, finger in missing)
        raise RuntimeError(f"base tactile config has no sensor for: {formatted}")
    payload = json.dumps(sensors, ensure_ascii=False, indent=2)
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(session_name or local_tag())).strip("_") or local_tag()
    dst = REPO_ROOT / "data" / "pure_teleop" / ".runtime" / f"{safe_name}_tactile_selected_ports.yaml"
    yaml_text = "\n".join(
        [
            "tactile_sensor_node:",
            "  ros__parameters:",
            # ROS declares this parameter as DOUBLE; YAML `200` would be an
            # INTEGER override and makes rclpy reject the node at startup.
            f"    publish_rate_hz: {float(publish_rate_hz):.1f}"
            if publish_rate_hz
            else f"    publish_rate_hz: {_yaml_scalar(text, 'publish_rate_hz', '20.0')}",
            f"    baudrate: {_yaml_scalar(text, 'baudrate', '921600')}",
            # A long serial read timeout silently caps a timer-driven high-rate
            # stream.  The node itself enforces a 5 ms floor.
            f"    serial_timeout_sec: {max(0.005, min(0.04, 1.0 / int(publish_rate_hz))) if publish_rate_hz else _yaml_scalar(text, 'serial_timeout_sec', '0.12')}",
            f"    calibrate_on_start: {_yaml_scalar(text, 'calibrate_on_start', 'true')}",
            f"    contact_lowpass_alpha: {_yaml_scalar(text, 'contact_lowpass_alpha', '0.35')}",
            f"    reopen_backoff_sec: {_yaml_scalar(text, 'reopen_backoff_sec', '1.0')}",
            "    publish_raw_points: true",
            "    sensors_json: |",
            *("      " + line for line in payload.splitlines()),
            "",
        ]
    )
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(yaml_text, encoding="utf-8")
    return dst


def prompt_int_setting(
    state: dict[str, Any],
    *,
    key: str,
    label: str,
    default: str,
    minimum: int = 1,
) -> int | None:
    remembered = memory_default(state, key, default)
    while True:
        raw = prompt_text(label, remembered)
        if raw is None:
            return None
        try:
            value = int(raw)
        except ValueError:
            print("请输入整数。")
            continue
        if value < minimum:
            print(f"请输入 >= {minimum} 的整数。")
            continue
        memory = state.setdefault("ui_memory", {})
        if isinstance(memory, dict):
            memory[key] = str(value)
        return value


def with_dialout_group(cmd: Sequence[str]) -> list[str]:
    shell_cmd = " ".join(shlex.quote(str(item)) for item in cmd)
    return ["sg", "dialout", "-c", shell_cmd]


def build_tactile_gui_command(
    ports: Sequence[str],
    period_ms: int,
    *,
    selection_file: Path | None = None,
) -> tuple[list[str], list[tuple[str, str]]]:
    script = REPO_ROOT / "scripts" / "tactile_sensor_gui.py"
    base_cmd: list[str] = [str(DEFAULT_ROS2_PYTHON), str(script)]
    for port in ports:
        base_cmd.extend(["--port", str(port)])
    base_cmd.extend(["--period-ms", str(period_ms)])
    if selection_file is not None:
        base_cmd.extend(["--select-one", "--selection-file", str(selection_file)])
    rows = [
        ("动作", "选择单 tactile" if selection_file is not None else "启动触觉 live GUI"),
        ("ports", ", ".join(str(port) for port in ports)),
        ("period_ms", str(period_ms)),
        ("权限", "sg dialout"),
        ("脚本", "scripts/tactile_sensor_gui.py"),
    ]
    if selection_file is not None:
        rows.append(("selection_file", str(selection_file)))
    return with_dialout_group(base_cmd), rows


def print_rows(title: str, rows: Sequence[tuple[str, str]]) -> None:
    print(f"\n{title}")
    for key, value in rows:
        print(f"  {key}: {value}")


def confirm_and_run(cmd: Sequence[str], *, rows: Sequence[tuple[str, str]], title: str, env: Mapping[str, str] | None = None) -> None:
    print_rows(title, rows)
    print("command:")
    print("  " + " ".join(shlex.quote(str(item)) for item in cmd))
    raw = prompt_text("按 Enter 执行，输入 b 返回", "", cancel_values=("b", "q"))
    if raw is None:
        return
    try:
        rc = run_interactive(cmd, env=env or {"PYTHONNOUSERSITE": "1"}, print_command=False)
    except KeyboardInterrupt:
        print("\n已停止。")
        pause()
        return
    if rc != 0:
        print(f"退出码 {rc}")
    pause()


def option_record(state: dict[str, Any], state_path: Path) -> None:
    menu = MenuSpec(
        title="\n录制 episode:",
        options=(
            MenuOption("1", "双臂+gripper（camera selector + RViz + SPACE start/stop）", "record_dual_arm"),
            MenuOption("2", "右臂+gripper 8D（无触觉，right wrist/top camera）", "record_right_wrist_top"),
            MenuOption("3", "右臂+gripper 8D + 单 tactile（GUI 选择编号，right wrist/top camera）", "record_right_wrist_top_tactile"),
            MenuOption("4", "右臂+单 tactile 高频 FAST（200 Hz raw 3×3×XYZ；right/top）", "record_right_tactile_fast"),
            MenuOption("5", "右臂 8D + TACTILE（单触觉，right/top；独立入口）", "record_right_8d_tactile"),
            MenuOption("6", "右臂+gripper 8D（无触觉，仅 right wrist camera）", "record_right_wrist_only"),
        ),
        default="1",
        remember_key="pure.record.kind",
    )
    choice = prompt_choice_menu(state, menu)
    if choice is None:
        return
    paths = state.setdefault("paths", {})
    record_kind = choice_menu_value(menu, choice)
    if record_kind in {
        "record_dual_arm",
        "record_right_wrist_top",
        "record_right_wrist_top_tactile",
        "record_right_tactile_fast",
        "record_right_8d_tactile",
        "record_right_wrist_only",
    }:
        session_root = DEFAULT_ROSBAG_ROOT
        paths["rosbag_root"] = str(session_root)
        session_name = ""
        latest_session = latest_child_dir(session_root)
        if latest_session is not None:
            session_menu = MenuSpec(
                title=f"\n录制 session（保存到 {session_root}）:",
                options=(
                    MenuOption("1", f"继续上次: {latest_session.name} -> {next_episode_name(latest_session)}", "continue"),
                    MenuOption("2", "新建并命名 session", "new"),
                ),
                default="1",
            )
            session_choice = prompt_choice_menu(state, session_menu)
            if session_choice is None:
                return
            if choice_menu_value(session_menu, session_choice) == "continue":
                session_name = latest_session.name
        if not session_name:
            session_name = prompt_text("新 session 名称（仅名称）", f"pure_teleop_{local_tag()}")
            if session_name is None:
                return
            session_name = session_name.strip()
            if not session_name or Path(session_name).name != session_name or session_name in {".", ".."}:
                print("[录制] session 名称不能包含路径分隔符。")
                pause()
                return
        torque_choice = None
        if record_kind == "record_dual_arm":
            torque_menu = MenuSpec(
                title="\n是否记录双臂关节 torque:",
                options=(
                    MenuOption("1", "不校验 torque（仅双臂 position/action）", "no", aliases=("n", "no")),
                    MenuOption("2", "记录 8D torque：JointState.effort（录制前校验双臂数据）", "joint_state_effort", aliases=("y", "yes")),
                ),
                default="1",
                remember_key="pure.record.torque",
            )
            torque_choice = prompt_choice_menu(state, torque_menu)
            if torque_choice is None:
                return
        save_state(state_path, state)
        tactile_record_targets: tuple[str, ...] = ()
        if record_kind in {
            "record_right_wrist_top_tactile",
            "record_right_tactile_fast",
            "record_right_8d_tactile",
        }:
            tactile_record_targets = (SINGLE_RIGHT_TACTILE_TARGET,)
        record_tactile = bool(tactile_record_targets)
        if record_kind in {
            "record_right_wrist_top",
            "record_right_wrist_top_tactile",
            "record_right_tactile_fast",
            "record_right_8d_tactile",
        }:
            record_preset = "right_wrist_top"
        elif record_kind == "record_right_wrist_only":
            record_preset = "right_wrist_only"
        elif record_kind == "record_dual_arm":
            record_preset = "dual_arm"
        else:
            record_preset = ""
        record_torque = (
            torque_choice is not None
            and choice_menu_value(torque_menu, torque_choice) == "joint_state_effort"
        )
        effective_session_name = session_name or f"pure_teleop_current_{local_tag()}"
        tactile_config_path: Path | None = None
        tactile_ports: list[str] = []
        tactile_rate_hz: int | None = None
        if tactile_record_targets:
            if record_kind == "record_right_tactile_fast":
                tactile_rate_hz = prompt_int_setting(
                    state,
                    key="pure.record.tactile_fast.rate_hz",
                    label="tactile 发布/录制频率 Hz（推荐 200；串口实际吞吐不足时会自然低于此值）",
                    default=os.environ.get("PURE_TELEOP_TACTILE_FAST_RATE_HZ", "200"),
                    minimum=50,
                )
                if tactile_rate_hz is None:
                    return
            tactile_port = prompt_single_tactile_port_via_gui(state)
            tactile_ports = [tactile_port] if tactile_port else []
            if not tactile_ports:
                return
            if len(tactile_ports) != len(tactile_record_targets):
                print(
                    f"需要选择 {len(tactile_record_targets)} 个端口来对应 "
                    f"{', '.join(tactile_record_targets)}；当前选择了 {len(tactile_ports)} 个。"
                )
                pause()
                return
            try:
                tactile_config_path = make_tactile_record_config(
                    targets=tactile_record_targets,
                    ports=tactile_ports,
                    session_name=effective_session_name,
                    publish_rate_hz=tactile_rate_hz,
                )
            except Exception as exc:
                print(f"[录制] tactile 配置生成失败: {type(exc).__name__}: {exc}")
                pause()
                return
        cmd, rows = build_record_command(
            session_root,
            record_tactile=record_tactile,
            record_torque=record_torque,
            record_preset=record_preset,
            session_name=effective_session_name,
            tactile_record_targets=tactile_record_targets,
            tactile_config_path=tactile_config_path,
        )
        if tactile_record_targets and tactile_ports:
            rows.append(("tactile_ports", ", ".join(f"{target}={port}" for target, port in zip(tactile_record_targets, tactile_ports, strict=True))))
        run_env = {"PYTHONNOUSERSITE": "1"}
        if tactile_record_targets:
            run_env["TACTILE_RECORD_TARGETS"] = ",".join(tactile_record_targets)
        if tactile_config_path is not None:
            run_env["TACTILE_CONFIG_FILE"] = str(tactile_config_path)
        if tactile_rate_hz is not None:
            rows.append(("FAST tactile rate", f"{tactile_rate_hz} Hz; raw JointState [27]"))
        confirm_and_run(cmd, rows=rows, title="[录制] 键盘 episode recorder", env=run_env)
        return


def option_activation(state: dict[str, Any], state_path: Path) -> None:
    mode_menu = MenuSpec(title="\n激活类型:", options=(MenuOption("1", "follower", "1"), MenuOption("2", "leader", "2")), default="1", remember_key="pure.activation.mode")
    mode_choice = prompt_choice_menu(state, mode_menu)
    if mode_choice is None:
        return
    mode = choice_menu_value(mode_menu, mode_choice)
    scope = prompt_scope(state, remember_key="pure.activation.scope", allow_dual=True)
    if scope is None:
        return
    gripper_min_width_m = prompt_gripper_min_width_guard(state, role=activation_mode_role(mode))
    try:
        title, cmd, script, rows = build_activation_command(
            mode,
            scope,
            gripper_min_width_m=gripper_min_width_m,
        )
    except Exception as exc:
        print(f"[teleop] ERROR: 构建命令失败: {type(exc).__name__}: {exc}")
        pause()
        return
    state.setdefault("activation", {})["last_run_tag"] = local_tag()
    save_state(state_path, state)
    print_rows(f"[teleop] {title}", rows)
    print("command:")
    for line in script.splitlines():
        print(f"  {line}")
    raw = prompt_text("按 Enter 执行，输入 b 返回", "", cancel_values=("b", "q"))
    if raw is None:
        return
    try:
        rc = run_interactive(cmd, env={"PYTHONNOUSERSITE": "1"}, print_command=False)
    except KeyboardInterrupt:
        print("\n[teleop] 已停止。")
        pause()
        return
    if rc != 0:
        print(f"[teleop] 退出码 {rc}")
    pause()


ROOT_MENU = MenuSpec(
    title="NERO 遥操作与示范采集",
    options=(MenuOption("1", "录制 episode", "record"),
             MenuOption("5", "Teleop 激活（主臂 / 从臂）", "activation")),
    default="1",
    remember_key="pure_teleop.root",
    suffix="q 退出",
    cancel_values=("q", "quit", "exit", "退出"),
    cancel_label="[q] 退出",
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PURE_TELEOP_TITLE)
    parser.add_argument("--state", type=Path, default=DEFAULT_STATE_PATH)
    parser.add_argument("--smoke-test", action="store_true", help="验证菜单入口后退出")
    args = parser.parse_args(argv)
    if args.smoke_test:
        print(PURE_TELEOP_TITLE, " | ".join(f"{o.choice}. {o.label}" for o in ROOT_MENU.options))
        return 0
    state_path = args.state.expanduser().resolve()
    state = load_state(state_path)
    try:
        while (choice := prompt_choice_menu(state, ROOT_MENU)) is not None:
            if choice_menu_value(ROOT_MENU, choice) == "record":
                option_record(state, state_path)
            else:
                option_activation(state, state_path)
            save_state(state_path, state)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
