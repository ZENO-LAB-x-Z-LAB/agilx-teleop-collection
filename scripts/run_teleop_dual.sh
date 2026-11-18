#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 leader|follower [extra ros2 launch args...]" >&2
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

ROLE="$1"
shift

if [[ "${ROLE}" != "leader" && "${ROLE}" != "follower" ]]; then
  usage
  exit 1
fi

if [[ -n "${GRIPPER_MODE_OVERRIDE:-}" && "${GRIPPER_MODE_OVERRIDE}" != "width" ]]; then
  echo "[run_teleop_dual] ERROR: GRIPPER_MODE_OVERRIDE must be width; got '${GRIPPER_MODE_OVERRIDE}'" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"

source "$ROOT_DIR/scripts/env/pure_teleop_env.sh"
if [[ ! -r "$ROOT_DIR/install/local_setup.bash" ]]; then
  echo "Build this checkout first with ./scripts/build_ros2.sh" >&2
  exit 1
fi

CLEANUP_STARTED="false"
SHUTDOWN_SIGNAL=""
SHUTDOWN_NOTICE_PRINTED="false"
TELEOP_LAUNCH_RUNNING="false"
TELEOP_LAUNCH_DONE="false"
TELEOP_LAUNCH_RC=""

signal_exit_code() {
  case "${1:-}" in
    INT) echo 130 ;;
    TERM) echo 143 ;;
    *) echo 128 ;;
  esac
}

describe_exit_status() {
  local rc="${1:-0}"
  case "${rc}" in
    0) echo "completed normally" ;;
    130) echo "stopped by Ctrl-C" ;;
    143) echo "terminated" ;;
    *) echo "exited with rc=${rc}" ;;
  esac
}

request_shutdown() {
  local signal_name="$1"
  if [[ -z "${SHUTDOWN_SIGNAL}" ]]; then
    SHUTDOWN_SIGNAL="${signal_name}"
  fi
  if [[ "${SHUTDOWN_NOTICE_PRINTED}" != "true" ]]; then
    SHUTDOWN_NOTICE_PRINTED="true"
    case "${signal_name}" in
      INT)
        printf '\n[run_teleop_dual] Ctrl-C received for %s; waiting for ROS launch to stop cleanly...\n' "${ROLE}" >&2
        ;;
      TERM)
        printf '\n[run_teleop_dual] termination requested for %s; waiting for ROS launch to stop cleanly...\n' "${ROLE}" >&2
        ;;
      *)
        printf '\n[run_teleop_dual] shutdown requested for %s; waiting for ROS launch to stop cleanly...\n' "${ROLE}" >&2
        ;;
    esac
  fi
  if [[ "${TELEOP_LAUNCH_RUNNING}" != "true" ]]; then
    exit "$(signal_exit_code "${signal_name}")"
  fi
}

cleanup() {
  local rc=$?
  if [[ "${CLEANUP_STARTED}" == "true" ]]; then
    return 0
  fi
  CLEANUP_STARTED="true"

  if [[ "${TELEOP_LAUNCH_DONE}" == "true" ]]; then
    echo "[run_teleop_dual] ${ROLE} ROS launch $(describe_exit_status "${TELEOP_LAUNCH_RC}")." >&2
  elif [[ -n "${SHUTDOWN_SIGNAL}" ]]; then
    echo "[run_teleop_dual] ${ROLE} shutdown requested by ${SHUTDOWN_SIGNAL}." >&2
  elif [[ "${rc}" -ne 0 ]]; then
    echo "[run_teleop_dual] ${ROLE} wrapper exited with rc=${rc}." >&2
  else
    echo "[run_teleop_dual] ${ROLE} wrapper finished." >&2
  fi
}

append_arg() {
  local name="$1"
  local value="${2:-}"
  if [[ -n "${value}" ]]; then
    LAUNCH_ARGS+=("${name}:=${value}")
  fi
}

gravity_enabled() {
  local value
  value="$(printf '%s' "${MIT_ENABLE_GRAVITY:-true}" | tr '[:upper:]' '[:lower:]')"
  [[ "${value}" == "true" || "${value}" == "1" || "${value}" == "yes" || "${value}" == "on" ]]
}

check_gravity_python_env() {
  if [[ "${ROLE}" != "leader" ]] || ! gravity_enabled; then
    return 0
  fi

  local gravity_entry="${ROOT_DIR}/install/piper_nero_teleop/lib/piper_nero_teleop/nero_gravity_node"
  local gravity_python="python3"
  if [[ -f "${gravity_entry}" ]]; then
    local shebang
    shebang="$(head -1 "${gravity_entry}")"
    if [[ "${shebang}" == '#!'* ]]; then
      gravity_python="${shebang#'#!'}"
    fi
  fi

  if ! PYTHONNOUSERSITE="${PYTHONNOUSERSITE}" "${gravity_python}" - <<'PY'
import numpy as np
version = tuple(int(part) for part in np.__version__.split(".")[:1])
if version >= (2,):
    raise RuntimeError(f"nero_gravity_node requires NumPy < 2; got {np.__version__}")
import pinocchio  # noqa: F401
PY
  then
    echo "[run_teleop_dual] ERROR: gravity compensation Python environment is invalid." >&2
    echo "[run_teleop_dual] Fix with:" >&2
    echo "  ${gravity_python} -m pip install --force-reinstall 'numpy<2'" >&2
    return 1
  fi
}

declare -a LAUNCH_ARGS=()
LAUNCH_ARGS+=("role:=${ROLE}")

append_arg "leader_left_can_port" "${LEADER_LEFT_CAN_PORT:-}"
append_arg "leader_right_can_port" "${LEADER_RIGHT_CAN_PORT:-}"
append_arg "follower_left_can_port" "${FOLLOWER_LEFT_CAN_PORT:-}"
append_arg "follower_right_can_port" "${FOLLOWER_RIGHT_CAN_PORT:-}"
append_arg "log_level" "${TELEOP_LOG_LEVEL:-${LOG_LEVEL:-warn}}"
append_arg "enable_cmd_mux" "${ENABLE_CMD_MUX:-}"
append_arg "cmd_mux_default_source" "${CMD_MUX_DEFAULT_SOURCE:-}"
append_arg "firmware_profile" "${NERO_FIRMWARE_PROFILE:-${FIRMWARE_PROFILE:-}}"
append_arg "enable_timeout" "${NERO_ENABLE_TIMEOUT:-${ENABLE_TIMEOUT:-}}"
append_arg "control_sched_policy" "${NERO_CONTROL_SCHED_POLICY:-${CONTROL_SCHED_POLICY:-}}"
append_arg "control_sched_nice" "${NERO_CONTROL_SCHED_NICE:-${CONTROL_SCHED_NICE:-}}"
append_arg "control_sched_rt_priority" "${NERO_CONTROL_SCHED_RT_PRIORITY:-${CONTROL_SCHED_RT_PRIORITY:-}}"
append_arg "control_sched_cpu_list" "${NERO_CONTROL_SCHED_CPU_LIST:-${CONTROL_SCHED_CPU_LIST:-}}"

append_arg "master_slave_enable" "${MASTER_SLAVE_ENABLE:-}"
append_arg "robot_feedback_timeout_sec" "${NERO_ROBOT_FEEDBACK_TIMEOUT_SEC:-${ROBOT_FEEDBACK_TIMEOUT_SEC:-}}"
append_arg "mit_enable_pos" "${MIT_ENABLE_POS:-}"
append_arg "mit_enable_tor" "${MIT_ENABLE_TOR:-}"
append_arg "mit_enable_gravity" "${MIT_ENABLE_GRAVITY:-}"
append_arg "mit_kp" "${MIT_KP:-}"
append_arg "mit_kd" "${MIT_KD:-}"
append_arg "mit_torque_scale" "${MIT_TORQUE_SCALE:-}"
append_arg "mit_torque_feedback_sign" "${MIT_TORQUE_FEEDBACK_SIGN:-}"
append_arg "mit_max_torque_abs" "${MIT_MAX_TORQUE_ABS:-}"
append_arg "follower_control_mode" "${FOLLOWER_CONTROL_MODE:-}"
append_arg "follower_mit_enable_gravity" "${FOLLOWER_MIT_ENABLE_GRAVITY:-}"
append_arg "follower_mit_kp" "${FOLLOWER_MIT_KP:-}"
append_arg "follower_mit_kd" "${FOLLOWER_MIT_KD:-}"
append_arg "follower_mit_max_torque_abs" "${FOLLOWER_MIT_MAX_TORQUE_ABS:-}"
append_arg "follower_gravity_torque_scale" "${FOLLOWER_GRAVITY_TORQUE_SCALE:-}"
append_arg "follower_gravity_debug_enable" "${FOLLOWER_GRAVITY_DEBUG_ENABLE:-}"
append_arg "follower_gravity_debug_interval_sec" "${FOLLOWER_GRAVITY_DEBUG_INTERVAL_SEC:-}"
append_arg "gravity_mix_mode" "${GRAVITY_MIX_MODE:-}"
append_arg "gravity_unwrap_joint_positions" "${GRAVITY_UNWRAP_JOINT_POSITIONS:-}"
append_arg "gravity_clip_to_limits" "${GRAVITY_CLIP_TO_LIMITS:-}"
append_arg "gravity_max_joint_step" "${GRAVITY_MAX_JOINT_STEP:-}"
append_arg "gravity_max_torque_delta_warn" "${GRAVITY_MAX_TORQUE_DELTA_WARN:-}"
append_arg "gravity_joint_scale" "${GRAVITY_JOINT_SCALE:-}"
append_arg "enforce_joint_limits" "${ENFORCE_JOINT_LIMITS:-}"
append_arg "leader_shutdown_cleanup_enable" "${LEADER_SHUTDOWN_CLEANUP_ENABLE:-}"
append_arg "leader_shutdown_ramp_sec" "${LEADER_SHUTDOWN_RAMP_SEC:-}"
append_arg "leader_shutdown_ramp_steps" "${LEADER_SHUTDOWN_RAMP_STEPS:-}"
append_arg "leader_shutdown_final_mode" "${LEADER_SHUTDOWN_FINAL_MODE:-}"
append_arg "leader_shutdown_disable_on_failure" "${LEADER_SHUTDOWN_DISABLE_ON_FAILURE:-}"

append_arg "enable_gripper" "${ENABLE_GRIPPER:-}"
append_arg "enable_gripper_haptic" "${ENABLE_GRIPPER_HAPTIC:-}"
append_arg "gripper_startup_recovery_enable" "${GRIPPER_STARTUP_RECOVERY_ENABLE:-}"
append_arg "gripper_mode_override" "${GRIPPER_MODE_OVERRIDE:-}"
append_arg "gripper_range" "${GRIPPER_RANGE:-}"
append_arg "gripper_reverse" "${GRIPPER_REVERSE:-}"
append_arg "gripper_val_multiple" "${GRIPPER_VAL_MULTIPLE:-}"
append_arg "gripper_follow_force_default" "${GRIPPER_FOLLOW_FORCE_DEFAULT:-}"
append_arg "gripper_max_step" "${GRIPPER_MAX_STEP:-}"
append_arg "gripper_cmd_limit_enable" "${GRIPPER_CMD_LIMIT_ENABLE:-}"
append_arg "gripper_cmd_min_raw" "${GRIPPER_CMD_MIN_RAW:-}"
append_arg "gripper_cmd_max_raw" "${GRIPPER_CMD_MAX_RAW:-}"
append_arg "gripper_cmd_deadband_raw" "${GRIPPER_CMD_DEADBAND_RAW:-}"
append_arg "gripper_filter_enable" "${GRIPPER_FILTER_ENABLE:-}"
append_arg "gripper_filter_alpha_position" "${GRIPPER_FILTER_ALPHA_POSITION:-}"
append_arg "gripper_filter_alpha_effort" "${GRIPPER_FILTER_ALPHA_EFFORT:-}"
append_arg "gripper_haptic_effort_sign" "${GRIPPER_HAPTIC_EFFORT_SIGN:-}"
append_arg "gripper_haptic_effort_bias" "${GRIPPER_HAPTIC_EFFORT_BIAS:-}"
append_arg "gripper_haptic_effort_alpha" "${GRIPPER_HAPTIC_EFFORT_ALPHA:-}"
append_arg "gripper_haptic_release_when_opening" "${GRIPPER_HAPTIC_RELEASE_WHEN_OPENING:-}"
append_arg "gripper_haptic_opening_direction_eps" "${GRIPPER_HAPTIC_OPENING_DIRECTION_EPS:-}"
append_arg "gripper_haptic_force_max_n" "${GRIPPER_HAPTIC_FORCE_MAX_N:-}"
append_arg "gripper_haptic_constant_force_n" "${GRIPPER_HAPTIC_CONSTANT_FORCE_N:-}"
append_arg "gripper_haptic_constant_directional" "${GRIPPER_HAPTIC_CONSTANT_DIRECTIONAL:-}"
append_arg "gripper_haptic_spring_align_enable" "${GRIPPER_HAPTIC_SPRING_ALIGN_ENABLE:-}"
append_arg "gripper_haptic_spring_hold_sec" "${GRIPPER_HAPTIC_SPRING_HOLD_SEC:-}"
append_arg "gripper_haptic_spring_k_n_per_m" "${GRIPPER_HAPTIC_SPRING_K_N_PER_M:-}"
append_arg "gripper_haptic_spring_min_force_n" "${GRIPPER_HAPTIC_SPRING_MIN_FORCE_N:-}"
append_arg "gripper_haptic_mismatch_pulse_enable" "${GRIPPER_HAPTIC_MISMATCH_PULSE_ENABLE:-}"
append_arg "gripper_haptic_mismatch_threshold_m" "${GRIPPER_HAPTIC_MISMATCH_THRESHOLD_M:-}"
append_arg "gripper_haptic_mismatch_reset_threshold_m" "${GRIPPER_HAPTIC_MISMATCH_RESET_THRESHOLD_M:-}"
append_arg "gripper_haptic_mismatch_pulse_sec" "${GRIPPER_HAPTIC_MISMATCH_PULSE_SEC:-}"
append_arg "gripper_haptic_mismatch_force_threshold_n" "${GRIPPER_HAPTIC_MISMATCH_FORCE_THRESHOLD_N:-}"
append_arg "gripper_haptic_mismatch_force_confirm_sec" "${GRIPPER_HAPTIC_MISMATCH_FORCE_CONFIRM_SEC:-}"
append_arg "gripper_haptic_mismatch_force_sign" "${GRIPPER_HAPTIC_MISMATCH_FORCE_SIGN:-}"
append_arg "gripper_haptic_mismatch_closing_only" "${GRIPPER_HAPTIC_MISMATCH_CLOSING_ONLY:-}"
append_arg "gripper_haptic_mismatch_retrigger_sec" "${GRIPPER_HAPTIC_MISMATCH_RETRIGGER_SEC:-}"
append_arg "gripper_haptic_effort_deadband" "${GRIPPER_HAPTIC_EFFORT_DEADBAND:-}"
append_arg "gripper_haptic_effort_max" "${GRIPPER_HAPTIC_EFFORT_MAX:-}"
append_arg "gripper_haptic_cmd_max" "${GRIPPER_HAPTIC_CMD_MAX:-}"
append_arg "gripper_haptic_cmd_enable_threshold" "${GRIPPER_HAPTIC_CMD_ENABLE_THRESHOLD:-}"
append_arg "gripper_haptic_opening_relief" "${GRIPPER_HAPTIC_OPENING_RELIEF:-}"
append_arg "gripper_haptic_opening_sign" "${GRIPPER_HAPTIC_OPENING_SIGN:-}"
append_arg "gripper_haptic_force_ramp_up_n_per_sec" "${GRIPPER_HAPTIC_FORCE_RAMP_UP_N_PER_SEC:-}"
append_arg "gripper_haptic_force_ramp_down_n_per_sec" "${GRIPPER_HAPTIC_FORCE_RAMP_DOWN_N_PER_SEC:-}"
append_arg "gripper_v2_geom_scale" "${GRIPPER_V2_GEOM_SCALE:-}"
append_arg "gripper_v2_geom_offset" "${GRIPPER_V2_GEOM_OFFSET:-}"
append_arg "leader_gripper_min_trusted_angle_position" "${LEADER_GRIPPER_MIN_TRUSTED_ANGLE_POSITION:-}"
append_arg "gripper_recover_seed_master_open" "${GRIPPER_RECOVER_SEED_MASTER_OPEN:-}"
append_arg "gripper_recover_seed_angle_deg" "${GRIPPER_RECOVER_SEED_ANGLE_DEG:-}"
append_arg "gripper_recover_seed_width" "${GRIPPER_RECOVER_SEED_WIDTH:-}"
append_arg "gripper_recover_seed_force" "${GRIPPER_RECOVER_SEED_FORCE:-}"
append_arg "gripper_recover_seed_min_width" "${GRIPPER_RECOVER_SEED_MIN_WIDTH:-}"
append_arg "gripper_v2_velocity_eps" "${GRIPPER_V2_VELOCITY_EPS:-}"
append_arg "gripper_v2_confirm_time_sec" "${GRIPPER_V2_CONFIRM_TIME_SEC:-}"
append_arg "gripper_v2_close_error_on" "${GRIPPER_V2_CLOSE_ERROR_ON:-}"
append_arg "gripper_v2_close_error_off" "${GRIPPER_V2_CLOSE_ERROR_OFF:-}"
append_arg "gripper_v2_open_error_on" "${GRIPPER_V2_OPEN_ERROR_ON:-}"
append_arg "gripper_v2_open_error_off" "${GRIPPER_V2_OPEN_ERROR_OFF:-}"
append_arg "gripper_v2_k_close" "${GRIPPER_V2_K_CLOSE:-}"
append_arg "gripper_v2_d_close" "${GRIPPER_V2_D_CLOSE:-}"
append_arg "gripper_v2_fmax_close" "${GRIPPER_V2_FMAX_CLOSE:-}"
append_arg "gripper_v2_k_open" "${GRIPPER_V2_K_OPEN:-}"
append_arg "gripper_v2_d_open" "${GRIPPER_V2_D_OPEN:-}"
append_arg "gripper_v2_fmax_open" "${GRIPPER_V2_FMAX_OPEN:-}"
append_arg "gripper_v2_force_low_close" "${GRIPPER_V2_FORCE_LOW_CLOSE:-}"
append_arg "gripper_v2_force_high_close" "${GRIPPER_V2_FORCE_HIGH_CLOSE:-}"
append_arg "gripper_v2_force_low_open" "${GRIPPER_V2_FORCE_LOW_OPEN:-}"
append_arg "gripper_v2_force_high_open" "${GRIPPER_V2_FORCE_HIGH_OPEN:-}"
append_arg "gripper_v2_lambda_close" "${GRIPPER_V2_LAMBDA_CLOSE:-}"
append_arg "gripper_v2_lambda_open" "${GRIPPER_V2_LAMBDA_OPEN:-}"

echo "[run_teleop_dual] role=${ROLE}" >&2
echo "[run_teleop_dual] launch args:" >&2
for arg in "${LAUNCH_ARGS[@]}"; do
  echo "  ${arg}" >&2
done
if [[ "${ROLE}" == "follower" ]]; then
  echo "[run_teleop_dual] note: follower nodes internally override leader MIT/gravity flags to false" >&2
fi
if [[ $# -gt 0 ]]; then
  echo "[run_teleop_dual] extra args:" >&2
  for arg in "$@"; do
    echo "  ${arg}" >&2
  done
fi

trap cleanup EXIT
trap 'request_shutdown INT' INT
trap 'request_shutdown TERM' TERM
check_gravity_python_env

TELEOP_LAUNCH_RUNNING="true"
set +e
ros2 launch piper_nero_teleop teleop_dual.launch.py "${LAUNCH_ARGS[@]}" "$@"
TELEOP_LAUNCH_RC=$?
set -e
TELEOP_LAUNCH_RUNNING="false"
TELEOP_LAUNCH_DONE="true"
exit "${TELEOP_LAUNCH_RC}"
