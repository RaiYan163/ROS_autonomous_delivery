#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
jazzy_setup="/opt/ros/jazzy/setup.bash"
clean_first=0

if [[ "${1:-}" == "--clean" ]]; then
  clean_first=1
  shift
fi

if [[ ! -f "$jazzy_setup" ]]; then
  echo "ROS Jazzy is not installed: missing $jazzy_setup" >&2
  exit 1
fi

stale_build=0
for path in "$workspace_dir"/build "$workspace_dir"/install "$workspace_dir"/log; do
  if [[ -e "$path" ]]; then
    if command -v rg >/dev/null 2>&1; then
      if rg -q "/opt/ros/(humble|kilted)" "$path"; then
        stale_build=1
        break
      fi
    elif grep -R -E -q "/opt/ros/(humble|kilted)" "$path" 2>/dev/null; then
      stale_build=1
      break
    fi
  fi
done

if (( stale_build )) && (( ! clean_first )); then
  cat >&2 <<'EOF'
Existing build artifacts reference a different ROS distro.
Run ./build-jazzy.sh --clean to remove build/, install/, and log/ before rebuilding.
EOF
  exit 1
fi

if (( clean_first )); then
  rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
fi

cd "$workspace_dir"
set +u
source "$jazzy_setup"
set -u
colcon build --symlink-install "$@"
