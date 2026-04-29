#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
print_only=0

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

if [[ "${1:-}" == "--print" ]]; then
  print_only=1
  shift
fi

if [[ $# -gt 0 ]]; then
  echo "Unexpected arguments: $*" >&2
  echo "Usage: ./run-openmanipulator-teleop.sh [--print]" >&2
  exit 2
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "Could not find a Jazzy or Humble ROS installation under /opt/ros." >&2
  exit 1
fi

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built yet. Run ./setup-openmanipulator-teleop.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "$workspace_dir/install/setup.bash"
set -u

if ros2 pkg prefix open_manipulator_teleop >/dev/null 2>&1; then
  cmd=(ros2 run open_manipulator_teleop open_manipulator_x_teleop)
elif ros2 pkg prefix open_manipulator_x_teleop >/dev/null 2>&1; then
  cmd=(ros2 run open_manipulator_x_teleop open_manipulator_x_teleop)
else
  echo "Could not find an OpenManipulator teleop package in the sourced environment." >&2
  exit 1
fi

printf '%q ' "${cmd[@]}"
printf '\n'

if (( print_only )); then
  exit 0
fi

exec "${cmd[@]}"
