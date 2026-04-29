#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src_root="$workspace_dir/src/robotis"
ros_distro="${ROS_DISTRO:-}"
skip_build=0
update_repos=0

usage() {
  cat <<'EOF'
Usage: ./setup-openmanipulator-teleop.sh [--ros-distro jazzy|humble] [--skip-build] [--update]

Clones the official ROBOTIS OpenManipulator repositories into this workspace
and builds the packages needed for OpenManipulator-X hardware bringup and
keyboard teleoperation.

Options:
  --ros-distro  Override the ROS distro branch to use (jazzy or humble).
  --skip-build  Clone or update repositories without running colcon build.
  --update      Fast-forward existing repository clones to the selected branch.
  -h, --help    Show this help text.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      if [[ $# -lt 2 ]]; then
        echo "--ros-distro requires a value." >&2
        exit 2
      fi
      ros_distro="$2"
      shift 2
      ;;
    --skip-build)
      skip_build=1
      shift
      ;;
    --update)
      update_repos=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$ros_distro" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ros_distro="jazzy"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_distro="humble"
  else
    echo "Could not find /opt/ros/jazzy or /opt/ros/humble." >&2
    exit 1
  fi
fi

case "$ros_distro" in
  jazzy|humble)
    ;;
  *)
    echo "Unsupported ROS distro: $ros_distro (expected jazzy or humble)." >&2
    exit 2
    ;;
esac

ros_setup="/opt/ros/$ros_distro/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup file not found: $ros_setup" >&2
  exit 1
fi

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "$ros_distro" == "jazzy" && "${VERSION_ID:-}" != "24.04" ]]; then
    echo "Warning: ROS Jazzy is typically paired with Ubuntu 24.04; detected ${PRETTY_NAME:-unknown}." >&2
  fi
  if [[ "$ros_distro" == "humble" && "${VERSION_ID:-}" != "22.04" ]]; then
    echo "Warning: ROS Humble is typically paired with Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}." >&2
  fi
fi

declare -a repos=(
  "https://github.com/ROBOTIS-GIT/DynamixelSDK.git DynamixelSDK"
  "https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git dynamixel_interfaces"
  "https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git dynamixel_hardware_interface"
  "https://github.com/ROBOTIS-GIT/open_manipulator.git open_manipulator"
)

mkdir -p "$src_root"

for repo_spec in "${repos[@]}"; do
  repo_url="${repo_spec% *}"
  repo_name="${repo_spec##* }"
  repo_dir="$src_root/$repo_name"

  if [[ -d "$repo_dir/.git" ]]; then
    echo "Found existing $repo_name clone at $repo_dir"
    if (( update_repos )); then
      git -C "$repo_dir" fetch origin "$ros_distro" --depth 1
      if git -C "$repo_dir" show-ref --verify --quiet "refs/heads/$ros_distro"; then
        git -C "$repo_dir" checkout "$ros_distro"
      else
        git -C "$repo_dir" checkout -b "$ros_distro" --track "origin/$ros_distro"
      fi
      git -C "$repo_dir" pull --ff-only origin "$ros_distro"
    fi
    continue
  fi

  echo "Cloning $repo_name ($ros_distro branch)..."
  git clone --depth 1 --branch "$ros_distro" "$repo_url" "$repo_dir"
done

if (( skip_build )); then
  echo
  echo "Repositories are ready under $src_root"
  exit 0
fi

if [[ "$ros_distro" == "jazzy" ]]; then
  build_targets=(
    open_manipulator_bringup
    open_manipulator_moveit_config
    open_manipulator_teleop
  )
else
  build_targets=(
    open_manipulator_x_bringup
    open_manipulator_x_moveit_config
    open_manipulator_x_teleop
  )
fi

echo
echo "Building: ${build_targets[*]}"
cd "$workspace_dir"
# shellcheck disable=SC1090
set +u
source "$ros_setup"
set -u
colcon build --symlink-install --packages-up-to "${build_targets[@]}"

echo
echo "Build complete."
echo "Source the workspace with:"
echo "  source $ros_setup"
echo "  source $workspace_dir/install/setup.bash"
echo

if [[ "$ros_distro" == "jazzy" ]]; then
  cat <<'EOF'
Hardware bringup:
  ./run-openmanipulator-hardware.sh

OpenCR bringup:
  ./run-openmanipulator-hardware.sh port_name:=/dev/ttyACM0

Keyboard teleop:
  ./run-openmanipulator-teleop.sh
EOF
else
  cat <<'EOF'
Hardware bringup:
  ./run-openmanipulator-hardware.sh

OpenCR bringup:
  ./run-openmanipulator-hardware.sh port_name:=/dev/ttyACM0

Keyboard teleop:
  ./run-openmanipulator-teleop.sh
EOF
fi
