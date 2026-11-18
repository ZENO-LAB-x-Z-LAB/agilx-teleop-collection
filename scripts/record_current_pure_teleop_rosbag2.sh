#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
source "$ROOT_DIR/scripts/env/pure_teleop_env.sh"

usage() {
  cat >&2 <<'EOF'
Usage:
  scripts/record_current_pure_teleop_rosbag2.sh [options] [-- recorder-options...]

Pure-teleop recording path:
  1. optionally select local cameras with the OpenCV camera selector,
  2. publish selected cameras to /teleop_camera/<role>/image_raw/compressed,
  3. open RViz on the raw preview mirror for each selected camera,
  4. enter the keyboard recorder: SPACE starts/stops each episode.

Options:
      --bag-root DIR              Output root. Default: data/pure_teleop/rosbag2.
      --name NAME                 Bag directory name under --bag-root.
      --skip-camera               Do not start camera selector/publisher/RViz.
      --with-tactile              Start configured tactile sensors and record tactile topics.
      --skip-tactile              Do not start tactile sensor publisher.
      --tactile-config FILE       Tactile sensor YAML config.
      --tactile-target TARGET     Record one selected tactile sensor; repeat for multiple targets.
                                  TARGET format: slave_right:gripper.
      --dual-arm                  Record both arms+grippers state/action and cameras selected by the camera selector.
      --left-wrist-top            Record only left arm+gripper state/action/effort, left wrist camera, and top camera.
      --right-wrist-top           Record only right arm+gripper state/action, right wrist camera, and top camera.
      --right-wrist-only          Record only right arm+gripper state/action and right wrist camera.
      --require-joint-effort      Before starting an episode, require eight sampled JointState.effort values for every recorded arm.
      --force-camera-select       Ignore cached camera role mapping.
      --no-rviz                   Start camera publisher but do not open RViz.
  -h, --help                      Show this help.

Environment:
  CAMERA_ROLES                    Default: left,right,top.
  CAMERA_FOURCC                   Default: MJPG. Override with YUYV only for debugging.
  CAMERA_STOP_EXISTING            true/false. Default: false. Stop stale camera_mosaic_ros2 publishers before recording.
  PURE_TELEOP_RECORD_CAMERA       true/false. Default: true.
  PURE_TELEOP_RECORD_RVIZ         true/false. Default: true.
  PURE_TELEOP_RECORD_TACTILE      true/false. Default: false.
  TACTILE_CONFIG_FILE             Default: agx_arm_ros/src/piper_nero_teleop/config/tactile_sensors.local.yaml.
  TACTILE_RECORD_TARGETS          Comma-separated robot:finger pairs. Default: slave_right:gripper.
  TACTILE_RECORD_ONLY_SELECTED    true/false. Default: true. Only start the selected 9-point tactile sensors.
  TACTILE_USE_DIALOUT_GROUP       true/false. Default: true. Run tactile publisher with dialout group when needed.
  TACTILE_STOP_EXISTING           true/false. Default: false. Stop stale tactile_sensor_node before recording.
EOF
}

bool_is_true() {
  local value
  value="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "true" || "${value}" == "1" || "${value}" == "yes" || "${value}" == "on" || "${value}" == "auto" ]]
}

local_tag() {
  date +%Y%m%d_%H%M%S
}

split_csv() {
  local raw="$1"
  local -n out_ref="$2"
  local old_ifs="${IFS}"
  IFS=','
  read -r -a out_ref <<< "${raw}"
  IFS="${old_ifs}"
}

role_title() {
  case "$1" in
    left) echo "Left" ;;
    right) echo "Right" ;;
    top) echo "Top" ;;
    *) printf '%s' "$1" | sed -E 's/(^|_)([a-z])/\U\2/g' ;;
  esac
}

write_rviz_config() {
  local output="$1"
  shift
  mkdir -p "$(dirname "${output}")"
  {
    cat <<'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
EOF
    local role title
    for role in "$@"; do
      title="$(role_title "${role}")"
      cat <<EOF
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: ${title} Camera
      Normalize Range: true
      Queue Size: 2
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /teleop_camera_preview/${role}/image_raw
      Transport Hint: raw
      Value: true
EOF
    done
    cat <<'EOF'
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: camera_mosaic
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
Window Geometry:
  Displays:
    collapsed: false
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1280
  X: 80
  Y: 60
EOF
  } > "${output}"
}

BAG_ROOT="${PURE_TELEOP_ROSBAG_ROOT:-${ROOT_DIR}/data/pure_teleop/rosbag2}"
BAG_NAME="pure_teleop_current_$(local_tag)"
DURATION_SEC=""
START_CAMERA="${PURE_TELEOP_RECORD_CAMERA:-true}"
START_RVIZ="${PURE_TELEOP_RECORD_RVIZ:-true}"
START_TACTILE="${PURE_TELEOP_RECORD_TACTILE:-false}"
CAMERA_FOURCC="${CAMERA_FOURCC:-MJPG}"
CAMERA_STOP_EXISTING="${CAMERA_STOP_EXISTING:-false}"
TACTILE_CONFIG="${TACTILE_CONFIG_FILE:-${ROOT_DIR}/agx_arm_ros/src/piper_nero_teleop/config/tactile_sensors.local.yaml}"
TACTILE_RECORD_TARGETS="${TACTILE_RECORD_TARGETS:-slave_right:gripper}"
TACTILE_RECORD_ONLY_SELECTED="${TACTILE_RECORD_ONLY_SELECTED:-true}"
TACTILE_USE_DIALOUT_GROUP="${TACTILE_USE_DIALOUT_GROUP:-true}"
TACTILE_STOP_EXISTING="${TACTILE_STOP_EXISTING:-false}"
declare -a TACTILE_TARGET_OVERRIDES=()
FORCE_CAMERA_SELECT=false
LEFT_WRIST_TOP_PRESET=false
RIGHT_WRIST_TOP_PRESET=false
RIGHT_WRIST_ONLY_PRESET=false
DUAL_ARM_PRESET=false
declare -a RECORD_ARGS=()
declare -a RECORD_CAMERA_TOPIC_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag-root)
      [[ $# -ge 2 ]] || { echo "--bag-root requires a value" >&2; exit 2; }
      BAG_ROOT="$2"
      shift 2
      ;;
    --name)
      [[ $# -ge 2 ]] || { echo "--name requires a value" >&2; exit 2; }
      BAG_NAME="$2"
      shift 2
      ;;
    --duration)
      [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --skip-camera)
      START_CAMERA=false
      START_RVIZ=false
      shift
      ;;
    --with-tactile)
      START_TACTILE=true
      shift
      ;;
    --skip-tactile)
      START_TACTILE=false
      shift
      ;;
    --tactile-config)
      [[ $# -ge 2 ]] || { echo "--tactile-config requires a value" >&2; exit 2; }
      TACTILE_CONFIG="$2"
      shift 2
      ;;
    --tactile-target)
      [[ $# -ge 2 ]] || { echo "--tactile-target requires a value" >&2; exit 2; }
      START_TACTILE=true
      TACTILE_TARGET_OVERRIDES+=("$2")
      shift 2
      ;;
    --left-wrist-top)
      LEFT_WRIST_TOP_PRESET=true
      shift
      ;;
    --right-wrist-top)
      RIGHT_WRIST_TOP_PRESET=true
      shift
      ;;
    --right-wrist-only)
      RIGHT_WRIST_ONLY_PRESET=true
      shift
      ;;
    --dual-arm)
      DUAL_ARM_PRESET=true
      shift
      ;;
    --require-joint-effort)
      RECORD_ARGS+=(--require-joint-effort)
      shift
      ;;
    --force-camera-select)
      FORCE_CAMERA_SELECT=true
      shift
      ;;
    --no-rviz)
      START_RVIZ=false
      shift
      ;;
    --)
      shift
      RECORD_ARGS+=("$@")
      break
      ;;
    *)
      RECORD_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${#TACTILE_TARGET_OVERRIDES[@]}" -gt 0 ]]; then
  TACTILE_RECORD_TARGETS="$(IFS=,; printf '%s' "${TACTILE_TARGET_OVERRIDES[*]}")"
fi

if [[ -z "$BAG_NAME" || "$BAG_NAME" == */* || "$BAG_NAME" == . || "$BAG_NAME" == .. ]]; then
  echo "--name must be a session name without a directory component" >&2
  exit 2
fi

PRESET_COUNT=0
[[ "${LEFT_WRIST_TOP_PRESET}" == "true" ]] && ((PRESET_COUNT += 1))
[[ "${RIGHT_WRIST_TOP_PRESET}" == "true" ]] && ((PRESET_COUNT += 1))
[[ "${RIGHT_WRIST_ONLY_PRESET}" == "true" ]] && ((PRESET_COUNT += 1))
[[ "${DUAL_ARM_PRESET}" == "true" ]] && ((PRESET_COUNT += 1))
if (( PRESET_COUNT > 1 )); then
  echo "--dual-arm, --left-wrist-top, --right-wrist-top, and --right-wrist-only are mutually exclusive" >&2
  exit 2
fi

if [[ "${DUAL_ARM_PRESET}" == "true" ]]; then
  CAMERA_ROLES="left,right,top"
  RECORD_ARGS+=(
    --topic-preset dual_arm
    --teleop-ns master_right
    --robot-ns slave_right
  )
elif [[ "${LEFT_WRIST_TOP_PRESET}" == "true" ]]; then
  CAMERA_ROLES="left,top"
  RECORD_ARGS+=(
    --topic-preset left_wrist_top
    --teleop-ns master_left
    --robot-ns slave_left
  )
elif [[ "${RIGHT_WRIST_TOP_PRESET}" == "true" ]]; then
  CAMERA_ROLES="right,top"
  RECORD_ARGS+=(
    --topic-preset right_wrist_top
    --teleop-ns master_right
    --robot-ns slave_right
  )
elif [[ "${RIGHT_WRIST_ONLY_PRESET}" == "true" ]]; then
  CAMERA_ROLES="right"
  RECORD_ARGS+=(
    --topic-preset right_wrist_only
    --teleop-ns master_right
    --robot-ns slave_right
  )
fi

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export REQUIRE_ROBOT_TOPICS="${REQUIRE_ROBOT_TOPICS:-false}"
export OPENCV_LOG_LEVEL="${OPENCV_LOG_LEVEL:-ERROR}"

CAMERA_PID=""
RVIZ_PID=""
TACTILE_PID=""
ROS2_PYTHON="$PURE_TELEOP_ROS2_PYTHON"
if [[ ! -x "${ROS2_PYTHON}" ]]; then
  ROS2_PYTHON="python3"
fi

prepare_tactile_record_config() {
  if ! bool_is_true "${TACTILE_RECORD_ONLY_SELECTED}"; then
    return 0
  fi
  local runtime_dir="${ROOT_DIR}/data/pure_teleop/.runtime"
  mkdir -p "${runtime_dir}"
  local safe_targets output_config
  safe_targets="$(printf '%s' "${TACTILE_RECORD_TARGETS}" | tr -c '[:alnum:]_,-' '_')"
  output_config="${runtime_dir}/${BAG_NAME}_tactile_${safe_targets}.yaml"

  TACTILE_CONFIG_IN="${TACTILE_CONFIG}" \
  TACTILE_CONFIG_OUT="${output_config}" \
  TACTILE_RECORD_TARGETS="${TACTILE_RECORD_TARGETS}" \
  python3 - <<'PY'
import json
import os
import re
from pathlib import Path


src = Path(os.environ["TACTILE_CONFIG_IN"]).expanduser()
dst = Path(os.environ["TACTILE_CONFIG_OUT"]).expanduser()
text = src.read_text(encoding="utf-8")


def parse_target(raw: str) -> tuple[str, str]:
    text = raw.strip()
    if not text:
        raise ValueError("empty tactile target")
    if ":" in text:
        robot_ns, finger = text.split(":", 1)
    else:
        parts = [part for part in text.strip("/").split("/") if part]
        if len(parts) >= 4 and parts[1] == "tactile" and parts[-1] == "joint_states":
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


target_pairs = {
    parse_target(item)
    for item in os.environ["TACTILE_RECORD_TARGETS"].split(",")
    if item.strip()
}
if not target_pairs:
    raise SystemExit("TACTILE_RECORD_TARGETS did not contain any tactile target")

def scalar(name: str, default: str) -> str:
    match = re.search(rf"(?m)^\s*{re.escape(name)}:\s*(.*?)\s*$", text)
    if not match:
        return default
    value = match.group(1).split("#", 1)[0].strip()
    return value if value else default


def sensors_json_text() -> str:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        match = re.match(r"^(\s*)sensors_json:\s*(.*)$", line)
        if not match:
            continue
        base_indent = len(match.group(1))
        tail = match.group(2).strip()
        if tail and tail not in {">", "|", ">-", "|-"}:
            return tail
        block = []
        for child in lines[index + 1 :]:
            if not child.strip():
                block.append("")
                continue
            child_indent = len(child) - len(child.lstrip(" "))
            if child_indent <= base_indent:
                break
            block.append(child[base_indent + 2 :] if len(child) >= base_indent + 2 else child.lstrip())
        return "\n".join(block).strip()
    raise SystemExit(f"sensors_json not found in {src}")


sensors = json.loads(sensors_json_text())
matched = set()
for sensor in sensors:
    robot_ns = str(sensor.get("robot_ns", "")).strip("/")
    finger = str(sensor.get("finger", "")).strip("/")
    sensor_key = (robot_ns, finger)
    enabled = sensor_key in target_pairs
    sensor["enabled"] = enabled
    if enabled:
        matched.add(sensor_key)
missing = sorted(target_pairs - matched)
# ``gripper`` is the generic single-tactile slot used by the TUI.  Existing
# installations may still have only left_finger/right_finger template entries
# in their local YAML, so clone one entry for the requested robot namespace.
# A TUI-generated config already contains this entry; this path also keeps the
# direct --tactile-target slave_right:gripper command usable.
for robot_ns, finger in list(missing):
    if finger != "gripper":
        continue
    template = next(
        (
            sensor
            for sensor in sensors
            if str(sensor.get("robot_ns", "")).strip("/") == robot_ns
        ),
        None,
    )
    if template is None:
        continue
    sensor = dict(template)
    sensor["name"] = f"{robot_ns}_{finger}"
    sensor["finger"] = finger
    sensor["enabled"] = True
    sensors.append(sensor)
    matched.add((robot_ns, finger))
missing = sorted(target_pairs - matched)
if missing:
    formatted = ", ".join(f"{robot}:{finger}" for robot, finger in missing)
    raise SystemExit(f"no tactile sensor matches {formatted} in {src}")
payload = json.dumps(sensors, ensure_ascii=False, indent=2)
yaml_text = "\n".join(
    [
        "tactile_sensor_node:",
        "  ros__parameters:",
        f"    publish_rate_hz: {scalar('publish_rate_hz', '50.0')}",
        f"    baudrate: {scalar('baudrate', '921600')}",
        f"    serial_timeout_sec: {scalar('serial_timeout_sec', '0.04')}",
        f"    calibrate_on_start: {scalar('calibrate_on_start', 'false')}",
        f"    contact_lowpass_alpha: {scalar('contact_lowpass_alpha', '0.35')}",
        f"    reopen_backoff_sec: {scalar('reopen_backoff_sec', '1.0')}",
        "    publish_raw_points: true",
        "    sensors_json: |",
        *("      " + line for line in payload.splitlines()),
        "",
    ]
)
dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text(yaml_text, encoding="utf-8")
print(dst)
PY
  TACTILE_CONFIG="${output_config}"
}

tactile_topic_for_target() {
  local raw="$1"
  local target robot finger
  target="${raw#/}"
  target="${target%/}"
  if [[ "${target}" == *:* ]]; then
    robot="${target%%:*}"
    finger="${target#*:}"
  elif [[ "${target}" == */tactile/*/joint_states ]]; then
    robot="${target%%/tactile/*}"
    finger="${target#*/tactile/}"
    finger="${finger%/joint_states}"
  elif [[ "${target}" == */* ]]; then
    robot="${target%%/*}"
    finger="${target#*/}"
  else
    echo "[pure-record] WARN: bad tactile target '${raw}', expected robot:finger" >&2
    return 1
  fi
  robot="${robot#/}"
  robot="${robot%/}"
  finger="${finger#/}"
  finger="${finger%/}"
  if [[ -z "${robot}" || -z "${finger}" ]]; then
    echo "[pure-record] WARN: bad tactile target '${raw}', expected robot:finger" >&2
    return 1
  fi
  printf '/%s/tactile/%s/joint_states\n' "${robot}" "${finger}"
}

append_tactile_required_topic_args() {
  if ! bool_is_true "${START_TACTILE}"; then
    return 0
  fi
  local -a targets=()
  split_csv "${TACTILE_RECORD_TARGETS}" targets
  local target topic
  for target in "${targets[@]}"; do
    target="$(printf '%s' "${target}" | xargs)"
    [[ -n "${target}" ]] || continue
    topic="$(tactile_topic_for_target "${target}")" || continue
    RECORD_ARGS+=(--required-topic "${topic}")
  done
}

stop_existing_tactile_publishers() {
  if ! bool_is_true "${TACTILE_STOP_EXISTING}"; then
    return 0
  fi
  local pids
  pids="$(pgrep -f "piper_nero_teleop/lib/piper_nero_teleop/tactile_sensor_node|ros2 launch piper_nero_teleop tactile.launch.py" || true)"
  if [[ -z "${pids}" ]]; then
    return 0
  fi
  echo "[pure-record] stopping existing tactile publisher(s): ${pids//$'\n'/ }" >&2
  while IFS= read -r pid; do
    [[ -n "${pid}" ]] || continue
    [[ "${pid}" != "$$" ]] || continue
    kill "${pid}" 2>/dev/null || true
  done <<< "${pids}"
  sleep 0.5
}

stop_existing_camera_publishers() {
  if ! bool_is_true "${CAMERA_STOP_EXISTING}"; then
    return 0
  fi
  # Match both the normal absolute-path launch and a relative-path launch
  # (for example a persistent systemd camera service).  Only matching the
  # former leaves V4L2 devices busy and makes the selector see one camera.
  local camera_publisher_pattern='(^|/)camera_mosaic_ros2\.py([[:space:]]|$)'
  local pids
  pids="$(pgrep -f "${camera_publisher_pattern}" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    return 0
  fi
  echo "[pure-record] stopping existing camera publisher(s): ${pids//$'\n'/ }" >&2
  local pid
  while IFS= read -r pid; do
    [[ -n "${pid}" ]] || continue
    [[ "${pid}" != "$$" ]] || continue
    kill "${pid}" 2>/dev/null || true
  done <<< "${pids}"

  # V4L2 releases a device asynchronously after SIGTERM.  Wait briefly so the
  # camera selector does not mistake a just-stopped publisher for a dead camera.
  for _ in {1..20}; do
    if ! pgrep -f "${camera_publisher_pattern}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  echo "[pure-record] WARN: a previous camera publisher is still exiting" >&2
}

tactile_launch_with_permissions() {
  local -a launch_cmd=(ros2 launch piper_nero_teleop tactile.launch.py "config_file:=${TACTILE_CONFIG}")
  local current_user group_line group_members
  current_user="${USER:-$(id -un)}"
  group_line="$(getent group dialout 2>/dev/null || true)"
  group_members="${group_line##*:}"
  if bool_is_true "${TACTILE_USE_DIALOUT_GROUP}" \
    && [[ -n "${group_line}" ]] \
    && [[ ",${group_members}," == *",${current_user},"* ]] \
    && ! id -nG | tr ' ' '\n' | grep -qx dialout \
    && command -v sg >/dev/null 2>&1; then
    local inner_cmd sg_cmd quoted_cmd=""
    local item
    for item in "${launch_cmd[@]}"; do
      quoted_cmd+=" $(printf '%q' "${item}")"
    done
    inner_cmd="set -e; cd $(printf '%q' "${ROOT_DIR}"); source $(printf '%q' "${ROOT_DIR}/scripts/env/pure_teleop_env.sh"); exec${quoted_cmd}"
    sg_cmd="exec bash -lc $(printf '%q' "${inner_cmd}")"
    echo "[pure-record] starting tactile publisher via dialout group" >&2
    sg dialout -c "${sg_cmd}" &
  else
    "${launch_cmd[@]}" &
  fi
}

cleanup() {
  local rc=$?
  if [[ -n "${RVIZ_PID}" ]]; then
    echo "[pure-record] stopping RViz pid=${RVIZ_PID}" >&2
    kill "${RVIZ_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${CAMERA_PID}" ]]; then
    echo "[pure-record] stopping camera publisher pid=${CAMERA_PID}" >&2
    kill "${CAMERA_PID}" 2>/dev/null || true
    wait "${CAMERA_PID}" 2>/dev/null || true
  fi
  if [[ -n "${TACTILE_PID}" ]]; then
    echo "[pure-record] stopping tactile publisher pid=${TACTILE_PID}" >&2
    kill "${TACTILE_PID}" 2>/dev/null || true
    wait "${TACTILE_PID}" 2>/dev/null || true
  fi
  exit "${rc}"
}
trap cleanup EXIT

start_tactile_publisher() {
  if ! bool_is_true "${START_TACTILE}"; then
    echo "[pure-record] tactile disabled; not starting tactile sensor publisher" >&2
    return 0
  fi
  if [[ ! -f "${TACTILE_CONFIG}" ]]; then
    echo "[pure-record] WARN: tactile config not found; skipping tactile publisher: ${TACTILE_CONFIG}" >&2
    return 0
  fi

  prepare_tactile_record_config
  stop_existing_tactile_publishers
  echo "[pure-record] starting tactile sensor publisher: ${TACTILE_CONFIG}" >&2
  tactile_launch_with_permissions
  TACTILE_PID="$!"
  sleep 1.0
  if ! kill -0 "${TACTILE_PID}" 2>/dev/null; then
    wait "${TACTILE_PID}" 2>/dev/null || true
    TACTILE_PID=""
    echo "[pure-record] WARN: tactile publisher exited early; recording without tactile topics" >&2
    return 0
  fi
}

start_camera_preview() {
  if ! bool_is_true "${START_CAMERA}"; then
    echo "[pure-record] camera/RViz disabled; recording current ROS graph only" >&2
    return 0
  fi
  if [[ ! -f "${ROOT_DIR}/scripts/select_teleop_cameras.py" || ! -f "${ROOT_DIR}/scripts/camera_mosaic_ros2.py" ]]; then
    echo "[pure-record] WARN: camera helper scripts missing; recording current ROS graph only" >&2
    return 0
  fi

  stop_existing_camera_publishers

  local roles_csv="${CAMERA_ROLES:-left,right,top}"
  local -a selector_args=(
    --roles "${roles_csv}"
    --identity "${CAMERA_ROLE_IDENTITY:-serial}"
    --allow-missing
    --width "${CAMERA_WIDTH:-640}"
    --height "${CAMERA_HEIGHT:-480}"
    --fps "${CAMERA_FPS:-30}"
  )
  if [[ -n "${CAMERA_FOURCC:-}" ]]; then
    selector_args+=(--fourcc "${CAMERA_FOURCC}")
  fi
  if [[ -n "${CAMERA_ROLE_CACHE_FILE:-}" ]]; then
    selector_args+=(--cache-file "${CAMERA_ROLE_CACHE_FILE}")
  fi
  if [[ "${FORCE_CAMERA_SELECT}" == "true" || "$(printf '%s' "${CAMERA_ROLE_RESELECT:-false}" | tr '[:upper:]' '[:lower:]')" =~ ^(true|1|yes|on)$ ]]; then
    selector_args+=(--force-select)
  fi

  echo "[pure-record] selecting cameras for roles: ${roles_csv}" >&2
  local resolved
  if ! resolved="$("${ROS2_PYTHON}" "${ROOT_DIR}/scripts/select_teleop_cameras.py" "${selector_args[@]}")"; then
    echo "[pure-record] WARN: camera selection failed; recording current ROS graph only" >&2
    return 0
  fi
  if [[ -z "${resolved}" ]]; then
    echo "[pure-record] WARN: camera selector returned no cameras; recording current ROS graph only" >&2
    return 0
  fi

  local -a roles identities
  split_csv "${roles_csv}" roles
  split_csv "${resolved}" identities
  local -a active_roles=()
  local -a active_identities=()
  local -a inactive_roles=()
  local idx role identity
  for idx in "${!roles[@]}"; do
    role="$(printf '%s' "${roles[$idx]}" | xargs)"
    identity="$(printf '%s' "${identities[$idx]:-}" | xargs)"
    if [[ -n "${role}" && -n "${identity}" && "${identity}" != __missing__:* ]]; then
      active_roles+=("${role}")
      active_identities+=("${identity}")
    elif [[ -n "${role}" ]]; then
      inactive_roles+=("${role}")
    fi
  done
  RECORD_CAMERA_TOPIC_ARGS+=(--exclude-regex '^/camera_mosaic/' --exclude-regex '^/teleop_camera_preview/')
  for role in "${inactive_roles[@]}"; do
    RECORD_CAMERA_TOPIC_ARGS+=(--exclude-regex "^/teleop_camera/${role}/")
  done
  if [[ ${#active_roles[@]} -eq 0 ]]; then
    echo "[pure-record] no camera selected; recording current ROS graph only" >&2
    return 0
  fi
  for role in "${active_roles[@]}"; do
    RECORD_CAMERA_TOPIC_ARGS+=(--extra-topic "/teleop_camera/${role}/image_raw/compressed")
  done
  local active_roles_csv
  active_roles_csv="$(IFS=,; echo "${active_roles[*]}")"
  local -a camera_args=(
    --roles "${active_roles_csv}"
    --allow-missing
    --no-jinhe-camera-helper
    --publish-fps "${CAMERA_PUBLISH_FPS:-30}"
    --fps "${CAMERA_FPS:-30}"
    --width "${CAMERA_WIDTH:-640}"
    --height "${CAMERA_HEIGHT:-480}"
    --topic /camera_mosaic/image_raw
    --compressed-topic /camera_mosaic/image_raw/compressed
    --per-camera-topic-prefix /teleop_camera
    --per-camera-width "${CAMERA_PER_CAMERA_WIDTH:-640}"
    --per-camera-height "${CAMERA_PER_CAMERA_HEIGHT:-480}"
    --per-camera-compressed-jpeg-quality "${CAMERA_PER_CAMERA_JPEG_QUALITY:-90}"
    --preview-topic-prefix /teleop_camera_preview
    --preview-width "${CAMERA_PREVIEW_WIDTH:-640}"
    --preview-height "${CAMERA_PREVIEW_HEIGHT:-480}"
    --preview-compressed-jpeg-quality "${CAMERA_PREVIEW_JPEG_QUALITY:-90}"
  )
  if [[ -n "${CAMERA_FOURCC:-}" ]]; then
    camera_args+=(--fourcc "${CAMERA_FOURCC}")
  fi
  for idx in "${!active_roles[@]}"; do
    camera_args+=(--role-serial "${active_roles[$idx]}=${active_identities[$idx]}")
  done

  echo "[pure-record] starting camera ROS2 publisher for roles: ${active_roles_csv}" >&2
  "${ROS2_PYTHON}" "${ROOT_DIR}/scripts/camera_mosaic_ros2.py" "${camera_args[@]}" &
  CAMERA_PID="$!"
  sleep 1.0
  if ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
    wait "${CAMERA_PID}" 2>/dev/null || true
    CAMERA_PID=""
    echo "[pure-record] WARN: camera publisher exited early; recording current ROS graph only" >&2
    return 0
  fi

  if ! bool_is_true "${START_RVIZ}"; then
    return 0
  fi
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "[pure-record] WARN: no DISPLAY/WAYLAND_DISPLAY; skipping RViz" >&2
    return 0
  fi
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "[pure-record] WARN: rviz2 not found; skipping RViz" >&2
    return 0
  fi
  local rviz_config="${CAMERA_RVIZ_CONFIG:-}"
  if [[ -z "${rviz_config}" ]]; then
    rviz_config="${ROOT_DIR}/data/pure_teleop/.runtime/${BAG_NAME}_camera.rviz"
    write_rviz_config "${rviz_config}" "${active_roles[@]}"
  fi
  if [[ -f "${rviz_config}" ]]; then
    rviz2 -d "${rviz_config}" &
  else
    rviz2 &
  fi
  RVIZ_PID="$!"
  echo "[pure-record] RViz pid=${RVIZ_PID}" >&2
  sleep 1.0
}

start_tactile_publisher
start_camera_preview
append_tactile_required_topic_args

declare -a record_cmd=(
  "${ROOT_DIR}/scripts/record_current_teleop_keyboard_ros2.sh"
  --session-root "${BAG_ROOT}"
  --session-name "${BAG_NAME}"
  --prefix "pure_teleop"
  --storage mcap
  --storage-preset fastwrite
  --no-require-robot-topics
  --required-topic-sample-timeout-sec "${TACTILE_REQUIRED_TOPIC_SAMPLE_TIMEOUT_SEC:-3.0}"
)
if [[ -n "${DURATION_SEC}" ]]; then
  echo "[pure-record] WARN: --duration is ignored by keyboard episode recording; use SPACE to stop an episode." >&2
fi
record_cmd+=("${RECORD_CAMERA_TOPIC_ARGS[@]}")
record_cmd+=("${RECORD_ARGS[@]}")

echo "" >&2
echo "[pure-record] 进入键盘 episode 录制器" >&2
echo "[pure-record] session: ${BAG_ROOT%/}/${BAG_NAME}" >&2
echo "[pure-record] 操作: SPACE 开始 episode，再按 SPACE 结束保存；Q 退出。" >&2
"${record_cmd[@]}"
