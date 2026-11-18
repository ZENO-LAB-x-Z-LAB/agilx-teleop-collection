#!/usr/bin/env bash
# Shared environment for the TUI, ROS launchers and recorder.
nero_load_environment() {
  local root settings ros_setup had_nounset=0
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  settings="${NERO_CONFIG:-$root/config/local.env}"
  if [[ -f "$settings" ]]; then
    if [[ "${NERO_LOADED_CONFIG:-}" != "$settings" ]]; then
      source "$settings" || return 1
      export NERO_LOADED_CONFIG="$settings"
    fi
  elif [[ -n "${NERO_CONFIG:-}" ]]; then
    echo "NERO_CONFIG does not exist: $settings" >&2
    return 1
  fi
  export PURE_TELEOP_ROS2_PYTHON="${PURE_TELEOP_ROS2_PYTHON:-/usr/bin/python3}"
  export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
  ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  if [[ $- == *u* ]]; then had_nounset=1; set +u; fi
  if [[ -r "$ros_setup" ]]; then source "$ros_setup"; fi
  if [[ -r "$root/install/local_setup.bash" ]]; then source "$root/install/local_setup.bash"; fi
  if (( had_nounset )); then set -u; fi
}
nero_load_environment
