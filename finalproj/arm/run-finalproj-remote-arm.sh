#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

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
  echo "Workspace is not built yet. Build it on this machine first." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "$workspace_dir/install/setup.bash"
set -u

exec ros2 launch finalproj_openmanipulator_control system.launch.py \
  launch_joy:=false \
  launch_controller:=false \
  "$@"
