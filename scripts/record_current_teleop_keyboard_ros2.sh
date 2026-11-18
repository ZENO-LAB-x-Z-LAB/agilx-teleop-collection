#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/env/pure_teleop_env.sh"
exec "$PURE_TELEOP_ROS2_PYTHON" "$root/scripts/record_current_teleop_keyboard_ros2.py" "$@"
