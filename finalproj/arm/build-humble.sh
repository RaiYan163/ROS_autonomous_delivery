#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
humble_setup="/opt/ros/humble/setup.bash"
clean_first=0

if [[ "${1:-}" == "--clean" ]]; then
  clean_first=1
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--clean]" >&2
  exit 2
fi

if [[ ! -f "$humble_setup" ]]; then
  echo "ROS Humble is not installed: missing $humble_setup" >&2
  exit 1
fi

stale_build=0
for path in "$workspace_dir"/build "$workspace_dir"/install "$workspace_dir"/log; do
  if [[ -e "$path" ]]; then
    if command -v rg >/dev/null 2>&1; then
      if rg -q "/opt/ros/(kilted|jazzy)" "$path"; then
        stale_build=1
        break
      fi
    elif grep -R -E -q "/opt/ros/(kilted|jazzy)" "$path" 2>/dev/null; then
      stale_build=1
      break
    fi
  fi
done

if (( stale_build )) && (( ! clean_first )); then
  cat >&2 <<'EOF'
Existing build artifacts reference a different ROS distro.
Run ./build-humble.sh --clean to remove build/, install/, and log/ before rebuilding.
EOF
  exit 1
fi

if (( clean_first )); then
  rm -rf "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"
fi

cd "$workspace_dir"
set +u
source "$humble_setup"
set -u
colcon build "$@"
