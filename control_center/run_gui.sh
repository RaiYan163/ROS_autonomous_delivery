#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# TurtleBot Teach-and-Navigate Control Center — launcher
#
# Usage (from anywhere in the workspace):
#   bash control_center/run_gui.sh
#
# The script sources the ROS2 base layer and the workspace overlay, then
# starts the GUI.  Edit the ROS_DISTRO variable if you are on a different
# ROS2 release (default: jazzy).
# DDS / model / RMW defaults are applied inside turtlebot_gui.py (ROS_* constants).
#
# MCP / OpenAI deps (mcp package) often live in a conda env — point GUI_PYTHON at that interpreter:
#   conda activate sdr
#   export GUI_PYTHON="$(command -v python)"
#   bash control_center/run_gui.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo ""
echo "  ╔══════════════════════════════════════════════════╗"
echo "  ║  TurtleBot Teach-and-Navigate  ·  Control Center ║"
echo "  ╚══════════════════════════════════════════════════╝"
echo ""
echo "  Workspace : $WORKSPACE_ROOT"
echo "  ROS distro: $ROS_DISTRO"
echo ""

# ── Source ROS2 base ─────────────────────────────────────────────────────────
ROS_BASE="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$ROS_BASE" ]; then
    # shellcheck disable=SC1090
    source "$ROS_BASE"
    echo "  [OK] Sourced $ROS_BASE"
else
    echo "  [WARN] ROS base not found at $ROS_BASE"
    echo "         Set ROS_DISTRO or source manually before running."
fi

# ── Source workspace overlay ──────────────────────────────────────────────────
OVERLAY="$WORKSPACE_ROOT/install/setup.bash"
if [ -f "$OVERLAY" ]; then
    # shellcheck disable=SC1090
    source "$OVERLAY"
    echo "  [OK] Sourced workspace overlay"
else
    echo "  [WARN] Workspace overlay not found at $OVERLAY"
    echo "         Run 'colcon build' from $WORKSPACE_ROOT first."
fi

PYTHON_CMD="${GUI_PYTHON:-python3}"

echo ""
echo "  Starting GUI..."
echo "  Python    : $PYTHON_CMD"
echo ""

cd "$WORKSPACE_ROOT"
exec "$PYTHON_CMD" "$SCRIPT_DIR/turtlebot_gui.py" "$@"
