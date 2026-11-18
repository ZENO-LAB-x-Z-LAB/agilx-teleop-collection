#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
case "${1:-}" in
  -h|--help) echo "Usage: $0 [--build-only]"; exit 0 ;;
  ""|--build-only) ;;
  *) echo "Usage: $0 [--build-only]" >&2; exit 2 ;;
esac
export PYTHONNOUSERSITE=1
distro="${ROS_DISTRO:-jazzy}"
if [[ ! -r "/opt/ros/$distro/setup.bash" ]]; then
  echo "ROS installation missing: /opt/ros/$distro/setup.bash" >&2
  exit 1
fi
set +u
source "/opt/ros/$distro/setup.bash"
set -u
packages=(agx_arm_ros/src agx_arm_urdf pyAgxArm)
if [[ "${1:-}" != --build-only ]]; then
  rosdep install --from-paths "${packages[@]}" --ignore-src --rosdistro "$distro" -y
fi
colcon build --base-paths "${packages[@]}" \
  --packages-select agx_arm_msgs agx_arm_urdf pyAgxArm agx_arm_ctrl piper_nero_teleop
