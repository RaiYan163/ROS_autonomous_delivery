#!/usr/bin/env python3
"""
TurtleBot Teach-and-Navigate Control Center
============================================
Standalone GUI — no colcon build required.

Run:
    bash control_center/run_gui.sh
Or manually (from workspace root):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 control_center/turtlebot_gui.py

    Environment defaults (domain, model, RMW) are module constants just below the imports.

Pages
-----
  Turtlebot — main dashboard (panels below).
  Initialisation — map path, ROS CLI diagnostics (topic echo/hz/node list),
                    launch shortcuts, emergency zero cmd_vel.
  AI assistant — separate Toplevel: chat, MCP tool catalogue, tool log; optional OpenAI+MCP (mcp_server/).

Panels (Turtlebot page)
--------
  1. System Control   – launch / kill Nav2 + RViz and joystick teleop
  2. Robot Status     – live pose (from /amcl_pose) and battery
  3. Save Location    – name + save current pose to JSON
  4. Saved Locations  – dropdown of stored waypoints
  5. Auto-Navigation  – send Nav2 goal, cancel, recover, update, delete
  6. Navigation — Enable/Disable stack + goal state / distance / events
  7. Camera — live preview from /image_raw/compressed
  8. Logs — Event log or Navigation log (ros2 launch stdout), toggle buttons
"""

from __future__ import annotations

import json
import math
import os
import queue
import time
import signal
import subprocess
import sys
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 environment defaults — edit here (applied before rclpy / ros2 subprocesses).
# DDS domain must match TurtleBot PC and every terminal that should share topics.
# ─────────────────────────────────────────────────────────────────────────────

ROS_DOMAIN_ID      = 23                      # match TurtleBot3 PC / bashrc (0–232)
TURTLEBOT3_MODEL   = "burger"
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# Compressed image topic for the dashboard preview (image_transport default suffix).
CAMERA_COMPRESSED_TOPIC = "/image_raw/compressed"

os.environ["ROS_DOMAIN_ID"]        = str(ROS_DOMAIN_ID)
os.environ["TURTLEBOT3_MODEL"]     = TURTLEBOT3_MODEL
os.environ["RMW_IMPLEMENTATION"]   = RMW_IMPLEMENTATION

# ─────────────────────────────────────────────────────────────────────────────
# Paths  (all resolved relative to this file so it works from any cwd)
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR     = Path(__file__).parent.resolve()
WORKSPACE_ROOT = SCRIPT_DIR.parent
LOCATIONS_FILE = SCRIPT_DIR / "config" / "saved_locations.json"
LOGS_DIR       = SCRIPT_DIR / "logs"
MAP_YAML                  = WORKSPACE_ROOT / "real_map" / "test_map.yaml"
ARM_TELEOP_SCRIPT         = WORKSPACE_ROOT / "finalproj" / "arm" / "run-finalproj-local-teleop.sh"
SAVED_ARM_POSITIONS_FILE  = SCRIPT_DIR / "config" / "saved_arm_positions.json"
SAVED_ARM_SEQUENCES_FILE  = SCRIPT_DIR / "config" / "saved_arm_sequences.json"

_ARM_JOINTS            = ["joint1", "joint2", "joint3", "joint4"]
_ARM_DEFAULT_POSITIONS = [0.0, -1.0, 1.0, 0.0]   # ROBOTIS home
_ARM_GOTO_DURATION_SEC = 4                         # seconds for trajectory
_ARM_SEQUENCE_DELAY_SEC = 2.0                    # pause between sequence steps (seconds)

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

try:
    from dotenv import load_dotenv

    load_dotenv(WORKSPACE_ROOT / ".env")
except ImportError:
    pass

try:
    from mcp_server.mcp_memory_context import FILE_NAME as MCP_MEMORY_FILE_NAME
    from mcp_server.mcp_memory_context import MemoryContext as MCPMemoryContext
except ImportError:
    MCP_MEMORY_FILE_NAME = "memory_context.txt"
    MCPMemoryContext = None  # type: ignore[misc, assignment]


def _shutdown_ros_launch_process(proc: subprocess.Popen | None) -> None:
    """
    Mimic Ctrl+C in the launch terminal: signal the entire process group so
    ``ros2 launch`` tears down RViz, Nav2, and spawned nodes.

    Launches must be started with ``start_new_session=True`` (POSIX) so children
    share one process group.
    """
    if proc is None or proc.poll() is not None:
        return

    if sys.platform == "win32":
        proc.terminate()
        try:
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        proc.send_signal(signal.SIGINT)
    except OSError:
        proc.terminate()

    try:
        proc.wait(timeout=12)
        return
    except subprocess.TimeoutExpired:
        pass

    if proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        proc.terminate()

    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass

    if proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        proc.kill()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        pass

# ─────────────────────────────────────────────────────────────────────────────
# Design tokens  (Catppuccin Mocha palette — readable dark theme)
# ─────────────────────────────────────────────────────────────────────────────
C = {
    "base":     "#1e1e2e",   # window background
    "mantle":   "#181825",   # input / log background
    "crust":    "#11111b",   # header bar
    "surface0": "#313244",   # panel card background
    "surface1": "#45475a",   # panel header strip / button default
    "surface2": "#585b70",   # inactive indicator
    "overlay0": "#6c7086",   # muted hint text
    "text":     "#cdd6f4",   # primary text
    "subtext":  "#a6adc8",   # label text
    "blue":     "#89b4fa",   # accent / info
    "green":    "#a6e3a1",   # success / running
    "yellow":   "#f9e2af",   # warning / manual mode
    "red":      "#f38ba8",   # error / stop
    "orange":   "#fab387",   # caution
    "mauve":    "#cba6f7",   # update action
    "sky":      "#89dceb",   # auto-nav mode
    "teal":     "#94e2d5",   # save action
}

# ── Typography — larger readable defaults; Tk substitutes if a face is missing ──
if sys.platform == "darwin":
    _FONT_SANS = "Helvetica Neue"
    _FONT_MONO = "Menlo"
elif sys.platform == "win32":
    _FONT_SANS = "Segoe UI"
    _FONT_MONO = "Consolas"
else:
    _FONT_SANS = "DejaVu Sans"
    _FONT_MONO = "DejaVu Sans Mono"

FONT_TITLE = (_FONT_SANS, 16, "bold")
FONT_HEADER = (_FONT_SANS, 12, "bold")
FONT_BODY = (_FONT_SANS, 11)
FONT_BODY_BOLD = (_FONT_SANS, 11, "bold")
FONT_MONO = (_FONT_MONO, 11)
FONT_SMALL = (_FONT_SANS, 10)
FONT_DOT = (_FONT_SANS, 13)

PAD = {"padx": 6, "pady": 4}

# Human-readable Nav2 goal state names (matches action_msgs GoalStatus codes)
GOAL_STATE_NAMES = {
    0: "Unknown", 1: "Accepted", 2: "Executing",
    3: "Canceling", 4: "Succeeded", 5: "Canceled", 6: "Aborted",
}

# AI assistant panel: mirrors mcp_server/mcp_server_ros2.py tool names (keep in sync).
AI_ASSISTANT_TOOL_CATALOG: list[tuple[str, str]] = [
    ("cancel_navigation", "Cancel active async NavigateToPose goal."),
    ("delete_saved_location", "Remove waypoint from shared JSON."),
    ("emergency_stop", "Repeated zero TwistStamped on /cmd_vel."),
    ("execute_navigation", "Blocking NavigateToPose for staged goal."),
    ("execute_navigation_async", "Non-blocking NavigateToPose with feedback."),
    ("get_amcl_pose", "Latest localized pose from /amcl_pose."),
    ("get_battery_status", "Battery summary from /battery_state."),
    ("get_camera_snapshot", "JPEG snapshot (raw image or compressed)."),
    ("get_joy_state", "Joystick axes/buttons plus timeout disconnect."),
    ("get_lidar_scan", "LaserScan sectors or sparse full."),
    ("get_navigation_feedback", "Async Nav2 distance_remaining or outcome."),
    ("get_navigation_help", "Nav2 + launches + waypoint workflow text."),
    ("get_navigation_state", "Stored initial pose and goal."),
    ("get_odometry", "Latest /odom pose and twist."),
    ("get_saved_location", "Read one waypoint from JSON."),
    ("get_status", "Configured ROS topics + paths."),
    ("get_tf_transform", "TF2 lookup between two frames."),
    ("list_managed_launches", "MCP-managed ros2 launch PID status."),
    ("list_saved_locations", "Dump waypoint JSON."),
    ("manual_recovery", "Advisory recovery + cancel async nav."),
    ("navigate_to_saved_location", "Goal from JSON + blocking NavigateToPose."),
    ("publish_message", "std_msgs/String to /chatter."),
    ("rename_saved_location", "Rename waypoint key in JSON."),
    ("ros2_node_list", "`ros2 node list` subprocess."),
    ("ros2_topic_echo_once", "`ros2 topic echo --once` with timeout."),
    ("ros2_topic_hz", "`ros2 topic hz` sample window."),
    ("ros2_topic_info", "`ros2 topic info`."),
    ("ros2_topic_list", "`ros2 topic list` optional regex."),
    ("save_current_location", "Append waypoint from current /amcl_pose."),
    ("set_navigation_goal", "Stage map-frame Nav2 goal."),
    ("set_navigation_initial_pose", "/initialpose publisher for AMCL."),
    ("start_joystick_teleop_launch", "Managed joystick teleop launch."),
    ("start_navigation_launch", "Managed turtlebot3_navigation2 launch."),
    ("stop_all_managed_launches", "Kill all MCP-managed launches."),
    ("stop_joystick_teleop_launch", "Stop managed joystick launch."),
    ("stop_navigation_launch", "Stop managed Nav2 launch."),
    ("teleop_move", "Timed /cmd_vel — direction, duration_sec, optional linear_speed, angular_speed, publish_hz (MCP tool)."),
    ("update_saved_location_coords", "Patch x/y/yaw_deg in waypoint JSON."),
    ("update_saved_location_to_current", "Overwrite waypoint with current AMCL pose."),
    ("wait_for_navigation_result", "Block until async NavigateToPose result."),
]

# Stub reply for AI assistant UI testing (replace when LLM is wired).
AI_ASSISTANT_GENERIC_REPLY = (
    "Thanks — I received your message. This is a fixed placeholder reply for layout "
    "testing; the real assistant will answer here once the LLM bridge is connected."
)

# ─────────────────────────────────────────────────────────────────────────────
# Optional ROS2 import — GUI still opens (in demo mode) if ROS is not sourced
# ─────────────────────────────────────────────────────────────────────────────
ROS_AVAILABLE = False
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState, CompressedImage, Joy
    ROS_AVAILABLE = True
except ImportError:
    pass

CV2_NUMPY_AVAILABLE = False
try:
    import cv2
    import numpy as np
    CV2_NUMPY_AVAILABLE = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 backend node
# ─────────────────────────────────────────────────────────────────────────────
if ROS_AVAILABLE:
    class TeachNavNode(Node):
        """
        Lightweight ROS2 node.
        • Subscribes to /amcl_pose  → pushes ("pose", x, y, yaw_deg)
        • Subscribes to /battery_state → pushes ("battery", pct)
        • Subscribes to CAMERA_COMPRESSED_TOPIC → pushes ("camera_frame", ppm_bytes)
        • Provides send_goal() / cancel_goal() for NavigateToPose
        • cmd_vel: parameters `cmd_vel_topic` (default /cmd_vel) and `cmd_vel_type`
          (default twist_stamped) align with mcp_server/mcp_server_ros2.py and
          turtlebot3_gazebo; set cmd_vel_type:=twist for plain Twist subscribers.
        All feedback and results are forwarded to gui_queue so only the
        main thread touches tkinter widgets.
        """

        def __init__(self, gui_queue: queue.Queue) -> None:
            super().__init__("teach_nav_gui")
            self._q = gui_queue
            self._goal_handle = None
            self._last_joy_ns: float = 0.0

            self.declare_parameter("cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("cmd_vel_type", "twist_stamped")
            cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
            cmd_type = str(self.get_parameter("cmd_vel_type").value).strip().lower()
            if cmd_type not in ("twist", "twist_stamped"):
                self.get_logger().warning(
                    "Invalid cmd_vel_type %r; using twist_stamped (matches MCP server / Gazebo)."
                    % (cmd_type,)
                )
                cmd_type = "twist_stamped"
            self._cmd_vel_type = cmd_type

            self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
            )
            self.create_subscription(
                BatteryState, "/battery_state", self._on_battery, 10
            )
            self.create_subscription(
                Joy, "/joy", self._on_joy, 10
            )
            self.create_subscription(
                Odometry, "/odom", self._on_odom, 10
            )
            if self._cmd_vel_type == "twist_stamped":
                self._cmd_vel_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
                self._cmd_vel_log_label = f"geometry_msgs/msg/TwistStamped on {cmd_topic}"
            else:
                self._cmd_vel_pub = self.create_publisher(Twist, cmd_topic, 10)
                self._cmd_vel_log_label = f"geometry_msgs/msg/Twist on {cmd_topic}"
            self._last_odom_push_ns = 0
            self._last_camera_push_ns = 0
            self._camera_decode_warned = False

            if CV2_NUMPY_AVAILABLE:
                self.create_subscription(
                    CompressedImage,
                    CAMERA_COMPRESSED_TOPIC,
                    self._on_camera_compressed,
                    10,
                )
            else:
                self.get_logger().warning(
                    "OpenCV/NumPy not available — camera preview disabled"
                )
                self._q.put(
                    (
                        "log",
                        "WARNING",
                        "CAMERA",
                        "OpenCV/NumPy missing — camera preview disabled",
                    )
                )

            # Periodic check: if no /joy message arrives for 2 s → disconnected
            self.create_timer(1.0, self._check_joy_timeout)

        # ── Subscription callbacks (run in ROS spin thread) ───────────────────

        def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            # Quaternion → yaw in degrees
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw  = math.degrees(math.atan2(siny, cosy))
            self._q.put(("pose", x, y, yaw))

        def _on_battery(self, msg: BatteryState) -> None:
            pct = msg.percentage * 100.0 if msg.percentage >= 0 else -1.0
            self._q.put(("battery", pct))

        # ── Navigation (called from GUI thread, callbacks run in spin thread) ─

        def send_goal(self, x: float, y: float, yaw_deg: float, name: str) -> None:
            if not self._action_client.server_is_ready():
                self._q.put(("log", "ERROR", "NAV",
                             "Nav2 action server not ready — is Navigation running?"))
                self._q.put(("nav_state", "Error"))
                self._q.put(("goal_active", False))
                return

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp    = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            half = math.radians(yaw_deg) / 2.0
            goal.pose.pose.orientation.z = math.sin(half)
            goal.pose.pose.orientation.w = math.cos(half)
            goal.behavior_tree = ""

            self._q.put(("log", "INFO", "NAV",
                         f"Goal sent → {name}  x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f}°"))
            self._q.put(("nav_goal_name", name))
            self._q.put(("nav_state", "Sending"))

            future = self._action_client.send_goal_async(
                goal, feedback_callback=self._on_feedback
            )
            future.add_done_callback(self._on_goal_response)

        def _on_goal_response(self, future) -> None:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._q.put(("log", "ERROR", "NAV", "Goal rejected by Nav2"))
                self._q.put(("nav_state", "Rejected"))
                self._q.put(("goal_active", False))
                return
            self._goal_handle = handle
            self._q.put(("log", "INFO", "NAV", "Goal accepted"))
            self._q.put(("nav_state", "Navigating"))
            handle.get_result_async().add_done_callback(self._on_result)

        def _on_feedback(self, feedback_msg) -> None:
            dist = feedback_msg.feedback.distance_remaining
            self._q.put(("nav_distance", dist))

        def _on_result(self, future) -> None:
            self._goal_handle = None
            res = future.result()
            if res is None:
                self._q.put(("log", "ERROR", "NAV", "No result received from Nav2"))
                self._q.put(("nav_state", "Error"))
            elif res.status == GoalStatus.STATUS_SUCCEEDED:
                self._q.put(("log", "SUCCESS", "NAV", "Goal reached successfully ✓"))
                self._q.put(("nav_state", "Succeeded"))
            elif res.status == GoalStatus.STATUS_CANCELED:
                self._q.put(("log", "WARNING", "NAV", "Goal canceled"))
                self._q.put(("nav_state", "Canceled"))
            else:
                self._q.put(("log", "ERROR", "NAV", f"Goal failed  (status={res.status})"))
                self._q.put(("nav_state", "Failed"))
            self._q.put(("goal_active", False))

        def cancel_goal(self) -> None:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._q.put(("log", "WARNING", "NAV", "Cancel request sent to Nav2"))
                self._q.put(("nav_state", "Canceling"))

        # ── Joystick status ───────────────────────────────────────────────────

        def _on_joy(self, msg: Joy) -> None:
            """Forward raw /joy axes and button state to the GUI queue."""
            was_disconnected = self._last_joy_ns == 0.0
            self._last_joy_ns = self.get_clock().now().nanoseconds
            if was_disconnected:
                self._q.put(("joy_status", "connected"))
                self._q.put(("log", "INFO", "TELEOP", "Joystick connected on /joy"))

            # axis_forward=1, axis_turn=0  (matches joystick_teleop defaults)
            linear  = msg.axes[1] if len(msg.axes) > 1 else 0.0
            angular = msg.axes[0] if len(msg.axes) > 0 else 0.0
            self._q.put(("joy_axes", linear, angular))

            # Collect all pressed buttons as a readable string  e.g. "B0 (STOP)  B3"
            pressed = []
            for i, val in enumerate(msg.buttons):
                if val:
                    label = " (STOP)" if i == 0 else ""
                    pressed.append(f"B{i}{label}")
            self._q.put(("joy_buttons", "  ".join(pressed) if pressed else "None"))

        def _check_joy_timeout(self) -> None:
            """Called every 1 s; marks joystick as disconnected after 2 s silence."""
            if self._last_joy_ns == 0.0:
                return
            elapsed = (self.get_clock().now().nanoseconds - self._last_joy_ns) / 1e9
            if elapsed > 2.0:
                self._last_joy_ns = 0.0
                self._q.put(("joy_status", "disconnected"))
                self._q.put(("log", "WARNING", "TELEOP", "Joystick signal lost (timeout)"))

        # ── Odometry (throttled) + zero cmd_vel ────────────────────────────────

        def publish_zero_twist(self) -> None:
            """Emergency stop: publish zero velocity on cmd_vel (Twist or TwistStamped)."""
            if self._cmd_vel_type == "twist_stamped":
                z = TwistStamped()
                z.header.frame_id = "base_link"
                z.header.stamp = self.get_clock().now().to_msg()
                self._cmd_vel_pub.publish(z)
            else:
                self._cmd_vel_pub.publish(Twist())

        def _on_odom(self, msg: Odometry) -> None:
            ns = self.get_clock().now().nanoseconds
            if ns - self._last_odom_push_ns < 80_000_000:  # ≤12.5 Hz
                return
            self._last_odom_push_ns = ns
            vx = msg.twist.twist.linear.x
            wz = msg.twist.twist.angular.z
            self._q.put(("odom_vel", vx, wz))

        def _on_camera_compressed(self, msg: CompressedImage) -> None:
            """Decode compressed image, downscale, push PPM bytes for Tk PhotoImage."""
            if not CV2_NUMPY_AVAILABLE or cv2 is None or np is None:
                return
            ns = self.get_clock().now().nanoseconds
            if ns - self._last_camera_push_ns < 100_000_000:  # ≤10 Hz to GUI
                return
            self._last_camera_push_ns = ns

            arr = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                if not self._camera_decode_warned:
                    self._camera_decode_warned = True
                    self._q.put(
                        (
                            "log",
                            "WARNING",
                            "CAMERA",
                            f"imdecode failed (format={msg.format!r})",
                        )
                    )
                return

            h, w = bgr.shape[:2]
            max_w = 480
            if w > max_w:
                scale = max_w / float(w)
                bgr = cv2.resize(
                    bgr,
                    (int(w * scale), int(h * scale)),
                    interpolation=cv2.INTER_AREA,
                )

            ok, buf = cv2.imencode(".ppm", bgr)
            if not ok or buf is None:
                return
            self._q.put(("camera_frame", buf.tobytes()))

# ─────────────────────────────────────────────────────────────────────────────
# Reusable styled widget helpers
# ─────────────────────────────────────────────────────────────────────────────

def make_panel(parent: tk.Widget, title: str) -> tuple[tk.Frame, tk.Frame]:
    """Card with a coloured header strip.  Returns (outer_frame, content_frame)."""
    outer  = tk.Frame(parent, bg=C["surface0"], bd=0, relief="flat")
    header = tk.Frame(outer, bg=C["surface1"])
    header.pack(fill="x")
    tk.Label(header, text=title, font=FONT_HEADER,
             bg=C["surface1"], fg=C["blue"], pady=5, padx=10).pack(side="left")
    inner = tk.Frame(outer, bg=C["surface0"])
    inner.pack(fill="both", expand=True, padx=8, pady=(6, 8))
    return outer, inner


# Shared label width (characters) for all key-value rows — keeps all panels aligned
_LW = 12

def kv_row(parent: tk.Widget, label: str,
           default: str = "—", val_color: str = "text",
           val_font=None) -> tk.Label:
    """One key:value row with a fixed-width label.  Returns the value Label."""
    val_font = val_font or FONT_MONO
    f = tk.Frame(parent, bg=C["surface0"])
    f.pack(fill="x", pady=2)
    tk.Label(f, text=label, font=FONT_BODY, width=_LW, anchor="w",
             bg=C["surface0"], fg=C["subtext"]).pack(side="left")
    val = tk.Label(f, text=default, font=val_font,
                   bg=C["surface0"], fg=C[val_color], anchor="w")
    val.pack(side="left", fill="x", expand=True)
    return val


_BTN_COLORS: dict[str, tuple[str, str]] = {
    "blue":   (C["blue"],    C["base"]),
    "green":  (C["green"],   C["base"]),
    "teal":   (C["teal"],    C["base"]),
    "red":    (C["red"],     C["base"]),
    "yellow": (C["yellow"],  C["base"]),
    "orange": (C["orange"],  C["base"]),
    "mauve":  (C["mauve"],   C["base"]),
    "sky":    (C["sky"],     C["base"]),
    "gray":   (C["surface1"], C["text"]),
}


def make_btn(parent: tk.Widget, text: str, command=None,
             color: str = "blue", width: int = 0) -> tk.Button:
    bg, fg = _BTN_COLORS.get(color, _BTN_COLORS["blue"])
    kw: dict = dict(
        text=text, command=command, font=FONT_BODY,
        bg=bg, fg=fg,
        activebackground=C["surface2"], activeforeground=C["text"],
        relief="flat", cursor="hand2", padx=8, pady=4, bd=0,
    )
    if width:
        kw["width"] = width
    return tk.Button(parent, **kw)


def make_label(parent: tk.Widget, text: str = "", color: str = "text",
               font=FONT_BODY, anchor: str = "w",
               bg: str = C["surface0"]) -> tk.Label:
    return tk.Label(parent, text=text, font=font,
                    bg=bg, fg=C[color], anchor=anchor)


def make_dot(parent: tk.Widget, color: str = "surface2") -> tk.Label:
    """Small colored circle used as a process-running indicator."""
    return tk.Label(parent, text="●", font=FONT_DOT,
                    bg=C["surface0"], fg=C[color])


def make_sep(parent: tk.Widget) -> tk.Frame:
    return tk.Frame(parent, bg=C["surface1"], height=1)


def _parse_joint_states_topic_echo(stdout: str) -> dict | None:
    """
    Parse ``ros2 topic echo --once /joint_states`` stdout.

    The CLI sometimes prints leading noise (e.g. lost-message counters, tabs,
    log lines) before the YAML document. Try several slices / ``---`` chunks
    until we get a dict with ``name`` and ``position`` lists.
    """
    import yaml as _yaml

    raw = stdout.strip()
    if not raw:
        return None

    candidates: list[str] = [raw]
    for part in raw.split("---"):
        p = part.strip()
        if p:
            candidates.append(p)

    for marker in ("header:", "\nheader:"):
        idx = raw.find(marker)
        if idx >= 0:
            candidates.append(raw[idx:])

    seen: set[str] = set()
    for cand in candidates:
        if cand in seen:
            continue
        seen.add(cand)
        try:
            data = _yaml.safe_load(cand)
        except Exception:
            continue
        if (
            isinstance(data, dict)
            and isinstance(data.get("name"), list)
            and isinstance(data.get("position"), list)
            and len(data["name"]) == len(data["position"])
        ):
            return data
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Main GUI class
# ─────────────────────────────────────────────────────────────────────────────

class TeachNavGUI:
    """
    Full teach-and-navigate control panel.

    Threading model
    ---------------
    • main thread  : tkinter event loop + polling root.after()
    • ros_thread   : rclpy.spin(node) — all ROS callbacks live here
    • communication: queue.Queue — ROS thread pushes, main thread pops every 100 ms
    """

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self._q: queue.Queue = queue.Queue()

        # ROS
        self._ros_node   = None
        self._ros_thread = None

        # Subprocess handles  { "nav": Popen, "teleop": Popen }
        self._procs: dict[str, subprocess.Popen] = {}

        # State
        self._goal_active    = False
        self._current_pose   = (0.0, 0.0, 0.0)   # x, y, yaw_deg
        self._locations: dict = {}
        self._log_filter     = "all"              # "all" | "errors"
        self._all_log_lines: list[tuple[str, str]] = []  # [(level, formatted_line)]
        # Right-hand log panel: "event" (GUI) vs "navigation" (ros2 launch stream)
        self._log_panel_mode: str = "event"
        self._nav_launch_log: deque[str] = deque(maxlen=8000)

        # Long-running ros2 CLI helpers (echo / hz) — distinct from nav/teleop launches
        self._tool_procs: dict[str, subprocess.Popen] = {}
        self._init_tool_btns: dict[str, tk.Button] = {}
        self._init_tool_short: dict[str, str] = {}
        self._closing_gui = False
        self._camera_photo_ref: tk.PhotoImage | None = None
        self._ai_assistant_win: tk.Toplevel | None = None
        self._ai_chat_text: tk.Text | None = None
        self._ai_input_var: tk.StringVar | None = None
        self._ai_tool_log: tk.Text | None = None
        self._ai_tool_list: tk.Listbox | None = None
        self._ai_tool_desc_lbl: tk.Label | None = None
        self._ai_input_entry: tk.Entry | None = None
        self._mcp_thread: threading.Thread | None = None
        self._mcp_user_q: queue.Queue | None = None
        self._mcp_tk_q: queue.Queue | None = None
        self._mcp_stop: threading.Event | None = None

        # Arm teleop window state
        self._arm_teleop_win: tk.Toplevel | None = None
        self._arm_teleop_proc: subprocess.Popen | None = None
        self._arm_teleop_log_text: tk.Text | None = None
        self._arm_dot: tk.Label | None = None
        self._arm_proc_lbl: tk.Label | None = None
        # Arm saved positions
        self._arm_positions: dict = {}
        self._arm_pos_name_var: tk.StringVar | None = None
        self._arm_pos_dropdown_var: tk.StringVar | None = None
        self._arm_pos_dropdown: ttk.Combobox | None = None
        self._arm_env: dict = {}
        self._arm_sequence: list[str] = []
        self._arm_sequence_listbox: tk.Listbox | None = None
        self._arm_play_stop_event: threading.Event | None = None
        self._arm_active_goal_proc: subprocess.Popen | None = None
        self._arm_play_thread: threading.Thread | None = None
        self._arm_saved_sequences: dict[str, list[str]] = {}
        self._arm_gap_var: tk.StringVar | None = None
        self._arm_speed_var: tk.IntVar | None = None
        self._arm_speed_dur_lbl: tk.Label | None = None
        self._arm_seq_save_name_var: tk.StringVar | None = None
        self._arm_saved_seq_combo: ttk.Combobox | None = None
        self._arm_saved_seq_combo_var: tk.StringVar | None = None
        self._setup_window()
        self._load_locations()
        self._load_arm_positions()
        self._load_arm_sequences()
        self._build_nav_and_pages()
        self._start_ros()
        self._poll_queue()
        self._poll_processes()
        self._log("INFO", "SYSTEM", "GUI started — TurtleBot Navigation and Control Center")

    # ── Window ───────────────────────────────────────────────────────────────

    def _setup_window(self) -> None:
        self.root.title("TurtleBot Control Center")
        self.root.configure(bg=C["base"])
        # Enough height/width so both columns keep readable logs without squashing.
        self.root.geometry("1280x940")
        self.root.minsize(1100, 820)
        self.root.resizable(True, True)

        # ── Single top banner: title (left) · page nav + ROS (right cluster) ───
        hdr = tk.Frame(self.root, bg=C["crust"])
        hdr.pack(fill="x")

        tk.Label(
            hdr,
            text="  ⬡  TurtleBot Control Center",
            font=FONT_TITLE,
            bg=C["crust"],
            fg=C["blue"],
        ).pack(side="left", padx=(10, 8), pady=(10, 10))

        nav_inner = tk.Frame(hdr, bg=C["crust"])

        self._btn_nav_turtle = tk.Button(
            nav_inner,
            text="  Turtlebot  ",
            font=FONT_BODY,
            command=lambda: self._switch_page("turtlebot"),
            relief="flat",
            bd=0,
            cursor="hand2",
            padx=14,
            pady=6,
        )
        self._btn_nav_turtle.pack(side="left", padx=(0, 6))

        self._btn_nav_init = tk.Button(
            nav_inner,
            text="  Initialisation  ",
            font=FONT_BODY,
            command=lambda: self._switch_page("initialisation"),
            relief="flat",
            bd=0,
            cursor="hand2",
            padx=14,
            pady=6,
        )
        self._btn_nav_init.pack(side="left")

        self._btn_ai_assistant = tk.Button(
            nav_inner,
            text="  AI assistant  ",
            font=FONT_BODY,
            command=self._open_ai_assistant_window,
            relief="flat",
            bd=0,
            cursor="hand2",
            padx=14,
            pady=6,
            bg=C["mauve"],
            fg=C["base"],
            activebackground=C["surface2"],
            activeforeground=C["base"],
        )
        self._btn_ai_assistant.pack(side="left", padx=(8, 0))

        self._btn_arm_teleop_nav = tk.Button(
            nav_inner,
            text="  🦾  Arm Teleop  ",
            font=FONT_BODY,
            command=self._open_arm_teleop_window,
            relief="flat",
            bd=0,
            cursor="hand2",
            padx=14,
            pady=6,
            bg=C["sky"],
            fg=C["base"],
            activebackground=C["surface2"],
            activeforeground=C["base"],
        )
        self._btn_arm_teleop_nav.pack(side="left", padx=(8, 0))

        self._ros_badge = tk.Label(
            hdr,
            text="  ROS: Connecting…  ",
            font=FONT_SMALL,
            bg=C["overlay0"],
            fg=C["crust"],
            padx=8,
            pady=3,
            relief="flat",
        )
        # Pack right-to-left: ROS outermost, nav buttons immediately to its left
        self._ros_badge.pack(side="right", padx=(0, 14), pady=(12, 12))
        nav_inner.pack(side="right", padx=(0, 10), pady=(10, 10))

        # ── Swappable page container (directly under banner) ─────────────────
        self._page_container = tk.Frame(self.root, bg=C["base"])
        self._page_container.pack(fill="both", expand=True, padx=0, pady=(0, 8))

        self._page_turtlebot = tk.Frame(self._page_container, bg=C["base"])
        self._page_init = tk.Frame(self._page_container, bg=C["base"])

    # ── Page switching ───────────────────────────────────────────────────────

    def _build_nav_and_pages(self) -> None:
        """Build Turtlebot dashboard and Initialisation page; default to Turtlebot."""
        self._build_layout(self._page_turtlebot)
        self._build_initialisation_page(self._page_init)
        self._switch_page("turtlebot")

    def _switch_page(self, page: str) -> None:
        """Show one page frame; refresh nav pill styling."""
        self._page_turtlebot.pack_forget()
        self._page_init.pack_forget()

        if page == "turtlebot":
            self._page_turtlebot.pack(fill="both", expand=True)
            self._set_nav_style(self._btn_nav_turtle, active=True)
            self._set_nav_style(self._btn_nav_init, active=False)
        else:
            self._page_init.pack(fill="both", expand=True)
            self._set_nav_style(self._btn_nav_turtle, active=False)
            self._set_nav_style(self._btn_nav_init, active=True)

    @staticmethod
    def _set_nav_style(btn: tk.Button, *, active: bool) -> None:
        if active:
            btn.configure(bg=C["blue"], fg=C["base"])
        else:
            btn.configure(bg=C["surface1"], fg=C["subtext"],
                          activebackground=C["surface2"], activeforeground=C["text"])

    def _build_initialisation_page(self, parent: tk.Widget) -> None:
        """Map path (`real_map`), launch shortcuts, `ros2 topic echo` / `hz`, node list, zero cmd_vel."""
        self._init_tool_btns.clear()
        self._init_tool_short.clear()

        wrapper = tk.Frame(parent, bg=C["base"])
        wrapper.pack(fill="both", expand=True, padx=10, pady=(8, 14))

        # ── Panel 1 — map + same launches as System Control ─────────────────
        outer_map, inner_map = make_panel(wrapper, "🗺   Map & bring-up commands")
        map_resolved = MAP_YAML.resolve()
        tk.Label(
            inner_map,
            text="Navigation map YAML (workspace real_map/):",
            font=FONT_BODY,
            bg=C["surface0"],
            fg=C["subtext"],
            anchor="w",
        ).pack(anchor="w")
        tk.Label(
            inner_map,
            text=str(map_resolved),
            font=FONT_MONO,
            bg=C["surface0"],
            fg=C["sky"],
            anchor="w",
            justify="left",
            wraplength=1100,
        ).pack(anchor="w", pady=(0, 10))

        row_la = tk.Frame(inner_map, bg=C["surface0"])
        row_la.pack(fill="x", pady=(0, 6))
        make_btn(
            row_la, "Start Navigation + RViz", self._start_navigation, color="green"
        ).pack(side="left", padx=(0, 5))
        make_btn(
            row_la, "Start Joystick Teleop", self._start_teleop, color="blue"
        ).pack(side="left", padx=(0, 5))
        make_btn(
            row_la, "Stop Navigation", self._stop_navigation, color="orange"
        ).pack(side="left", padx=(0, 5))
        make_btn(
            row_la, "Stop Teleop", self._stop_teleop, color="orange"
        ).pack(side="left")

        row_lb = tk.Frame(inner_map, bg=C["surface0"])
        row_lb.pack(fill="x")
        make_btn(
            row_lb,
            "Emergency zero /cmd_vel",
            self._emergency_zero_cmd_vel,
            color="red",
        ).pack(side="left", padx=(0, 5))
        make_btn(
            row_lb, "ros2 node list", self._run_ros_node_list, color="gray"
        ).pack(side="left")

        outer_map.pack(fill="x", pady=(0, 8))

        # ── Panel 2 — subprocess mirrors of `ros2 topic echo` / `hz` ───────
        outer_mon, inner_mon = make_panel(
            wrapper, "📡   CLI topic monitors  (ros2 topic echo · ros2 topic hz)"
        )
        make_label(
            inner_mon,
            "Toggle a stream to append lines to the log below. "
            "Turn off again, use Stop all, or quit the app to end the process.",
            color="subtext",
        ).pack(anchor="w", pady=(0, 8))

        echo_topics: list[tuple[str, str]] = [
            ("/amcl_pose", "echo /amcl_pose"),
            ("/battery_state", "echo /battery"),
            ("/odom", "echo /odom"),
            ("/scan", "echo /scan"),
            ("/tf", "echo /tf"),
        ]
        grid_echo = tk.Frame(inner_mon, bg=C["surface0"])
        grid_echo.pack(fill="x", pady=(0, 6))
        for i, (topic, short) in enumerate(echo_topics):
            key = f"echo:{topic}"
            self._init_tool_short[key] = short
            b = make_btn(
                grid_echo,
                f"  {short}  □",
                command=lambda t=topic: self._toggle_topic_echo(t),
                color="gray",
            )
            b.grid(row=i // 3, column=i % 3, padx=(0, 8), pady=4, sticky="w")
            self._init_tool_btns[key] = b

        row_hz = tk.Frame(inner_mon, bg=C["surface0"])
        row_hz.pack(fill="x")
        hz_key = "hz:/amcl_pose"
        self._init_tool_short[hz_key] = "hz /amcl_pose"
        b_hz = make_btn(
            row_hz,
            "  hz /amcl_pose  □",
            self._toggle_topic_hz_amcl,
            color="gray",
        )
        b_hz.pack(side="left", padx=(0, 8))
        self._init_tool_btns[hz_key] = b_hz
        make_btn(
            row_hz,
            "Stop all ROS CLI monitors",
            self._stop_all_init_monitors,
            color="orange",
        ).pack(side="left", padx=(12, 0))

        outer_mon.pack(fill="x", pady=(0, 8))

        # ── Panel 3 — combined stdout from echo/hz + messages ───────────────
        outer_log, inner_log = make_panel(wrapper, "📋   Initialisation log")
        inner_log.columnconfigure(0, weight=1)
        inner_log.rowconfigure(0, weight=1)

        self._init_log_text = tk.Text(
            inner_log,
            font=FONT_MONO,
            bg=C["mantle"],
            fg=C["text"],
            relief="flat",
            bd=4,
            wrap="word",
            state="disabled",
            height=20,
            insertbackground=C["text"],
            selectbackground=C["surface1"],
        )
        sb = tk.Scrollbar(
            inner_log,
            command=self._init_log_text.yview,
            bg=C["surface0"],
            troughcolor=C["mantle"],
            activebackground=C["surface1"],
        )
        self._init_log_text.configure(yscrollcommand=sb.set)
        self._init_log_text.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        sb.grid(row=0, column=1, sticky="ns", pady=(0, 6))

        row_clr = tk.Frame(inner_log, bg=C["surface0"])
        row_clr.grid(row=1, column=0, columnspan=2, sticky="ew")
        make_btn(
            row_clr, "Clear log", lambda: self._clear_init_log(), color="gray"
        ).pack(side="left")

        outer_log.pack(fill="both", expand=True)

        self._append_init_log(
            "Initialisation page — map is real_map/test_map.yaml via Start Navigation + RViz.\n"
            "Use the toggles for the same output as `ros2 topic echo …` and `ros2 topic hz /amcl_pose`.\n\n"
        )

    def _clear_init_log(self) -> None:
        w = self._init_log_text
        w.config(state="normal")
        w.delete("1.0", "end")
        w.config(state="disabled")

    # ── Two-column body layout — Turtlebot page only ──────────────────────────

    def _build_layout(self, page: tk.Widget) -> None:
        """``page`` is the Turtlebot container; inner frame restores margin around the grid."""
        body = tk.Frame(page, bg=C["base"])
        body.pack(fill="both", expand=True, padx=10, pady=8)

        body.columnconfigure(0, weight=0, minsize=350)
        body.columnconfigure(1, weight=1, minsize=520)
        # Joystick row (5) absorbs vertical slack; keep a sane minimum so logs never vanish.
        body.rowconfigure(5, weight=1, minsize=280)
        body.rowconfigure(6, weight=0)

        # ── Left column — all panels pinned to top of their row ──────────
        self._build_system_panel(body)   .grid(row=0, column=0, sticky="new", **PAD)
        self._build_status_panel(body)   .grid(row=1, column=0, sticky="new", **PAD)
        self._build_save_panel(body)     .grid(row=2, column=0, sticky="new", **PAD)
        self._build_dropdown_panel(body) .grid(row=3, column=0, sticky="new", **PAD)
        self._build_autonav_panel(body)  .grid(row=4, column=0, sticky="new", **PAD)
        self._build_joystick_panel(body) .grid(row=5, column=0, sticky="nsew", **PAD)
        tk.Frame(body, bg=C["base"])     .grid(row=6, column=0, sticky="nsew")

        # ── Right column — ONE vertical stack (Nav atop Log immediately below)
        #    Previously Nav + Log were separate grid rows; row 0's height matched
        #    tall System Control, leaving dead space below Nav.  Packing both
        #    into ``right_pane`` fixes this.
        right_pane = tk.Frame(body, bg=C["base"])
        right_pane.grid(row=0, column=1, rowspan=7, sticky="nsew", **PAD)
        right_pane.columnconfigure(0, weight=1)
        # Nav + camera fixed height; log absorbs remaining vertical space.
        right_pane.rowconfigure(0, weight=0, minsize=120)
        right_pane.rowconfigure(1, weight=0, minsize=260)
        right_pane.rowconfigure(2, weight=1, minsize=280)

        self._build_navstatus_panel(right_pane).grid(row=0, column=0, sticky="nsew")
        self._build_camera_panel(right_pane)   .grid(row=1, column=0, sticky="nsew")
        self._build_log_panel(right_pane)      .grid(row=2, column=0, sticky="nsew")

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 1 — System Control
    # ─────────────────────────────────────────────────────────────────────────

    def _build_system_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "⚙   System Control")

        row1 = tk.Frame(inner, bg=C["surface0"])
        row1.pack(fill="x", pady=(0, 5))
        self._btn_start_nav    = make_btn(row1, "Start Navigation + RViz",
                                          self._start_navigation, color="green")
        self._btn_start_nav.pack(side="left", padx=(0, 5))
        self._btn_start_teleop = make_btn(row1, "Start Teleop",
                                          self._start_teleop, color="blue")
        self._btn_start_teleop.pack(side="left")

        row2 = tk.Frame(inner, bg=C["surface0"])
        row2.pack(fill="x", pady=(0, 6))
        make_btn(row2, "Stop Navigation", self._stop_navigation,
                 color="orange").pack(side="left", padx=(0, 5))
        make_btn(row2, "Stop Teleop",     self._stop_teleop,
                 color="orange").pack(side="left", padx=(0, 5))
        make_btn(row2, "Stop All",        self._stop_all,
                 color="red").pack(side="left")

        make_sep(inner).pack(fill="x", pady=(0, 5))

        # Live process status indicators
        row3 = tk.Frame(inner, bg=C["surface0"])
        row3.pack(fill="x")
        self._nav_dot    = make_dot(row3)
        self._nav_dot.pack(side="left")
        self._nav_proc_lbl = tk.Label(row3, text="Navigation: Stopped",
                                      font=FONT_SMALL, bg=C["surface0"], fg=C["subtext"])
        self._nav_proc_lbl.pack(side="left", padx=(2, 18))

        self._teleop_dot = make_dot(row3)
        self._teleop_dot.pack(side="left")
        self._teleop_proc_lbl = tk.Label(row3, text="Teleop: Stopped",
                                         font=FONT_SMALL, bg=C["surface0"], fg=C["subtext"])
        self._teleop_proc_lbl.pack(side="left", padx=(2, 0))

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 2 — Robot Status
    # ─────────────────────────────────────────────────────────────────────────

    def _build_status_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📡  Robot Status")

        # Mode + Battery on same row (two kv_row columns)
        row1 = tk.Frame(inner, bg=C["surface0"])
        row1.pack(fill="x", pady=2)
        tk.Label(row1, text="Mode:", font=FONT_BODY, width=_LW, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")
        self._mode_lbl = tk.Label(row1, text="Idle",
                                  font=FONT_BODY_BOLD,
                                  bg=C["surface0"], fg=C["subtext"], width=14, anchor="w")
        self._mode_lbl.pack(side="left")
        tk.Label(row1, text="Battery:", font=FONT_BODY, width=8, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")
        self._battery_lbl = tk.Label(row1, text="—", font=FONT_BODY,
                                     bg=C["surface0"], fg=C["green"], anchor="w")
        self._battery_lbl.pack(side="left")

        self._pose_lbl = kv_row(inner, "Pose:", "x=—    y=—    yaw=—", "text")
        self._odom_lbl = kv_row(inner, "/odom:", "vx=—    ωz=—", "sky")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 3 — Save Current Location
    # ─────────────────────────────────────────────────────────────────────────

    def _build_save_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📌  Save Current Location")

        row = tk.Frame(inner, bg=C["surface0"])
        row.pack(fill="x")
        tk.Label(row, text="Name:", font=FONT_BODY, width=_LW, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")

        self._loc_name_var = tk.StringVar()
        entry = tk.Entry(
            row, textvariable=self._loc_name_var,
            font=FONT_BODY, bg=C["mantle"], fg=C["text"],
            insertbackground=C["text"], relief="flat", bd=4, width=18,
        )
        entry.pack(side="left", padx=(0, 6))
        entry.bind("<Return>", lambda _: self._save_location())

        make_btn(row, "Save Position", self._save_location,
                 color="teal").pack(side="left")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 4 — Load Location
    # ─────────────────────────────────────────────────────────────────────────

    def _build_dropdown_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📂  Load Location")

        # ── Dropdown row ──────────────────────────────────────────────────
        row1 = tk.Frame(inner, bg=C["surface0"])
        row1.pack(fill="x", pady=(0, 5))
        tk.Label(row1, text="Select:", font=FONT_BODY, width=_LW, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")

        self._selected_loc = tk.StringVar()
        self._loc_combo = ttk.Combobox(
            row1, textvariable=self._selected_loc,
            state="readonly", font=FONT_BODY, width=22,
        )
        style = ttk.Style()
        style.theme_use("default")
        style.configure("TCombobox",
                        fieldbackground=C["mantle"],
                        background=C["surface0"],
                        foreground=C["text"],
                        selectbackground=C["surface1"],
                        selectforeground=C["text"],
                        arrowcolor=C["blue"])
        self._loc_combo.pack(side="left")
        self._refresh_dropdown()

        # ── Action buttons — indented to align under the combobox ─────────
        row2 = tk.Frame(inner, bg=C["surface0"])
        row2.pack(fill="x")
        # Spacer matches the "Select:" label width so buttons sit under combobox
        tk.Label(row2, text="", width=_LW,
                 bg=C["surface0"], fg=C["surface0"]).pack(side="left")
        make_btn(row2, "Save",   self._save_location,   color="teal"  ).pack(side="left", padx=(0, 4))
        make_btn(row2, "Edit",   self._edit_location,   color="sky"   ).pack(side="left", padx=(0, 4))
        make_btn(row2, "Delete", self._delete_location, color="orange").pack(side="left")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 5 — Auto-Navigation Controls
    # ─────────────────────────────────────────────────────────────────────────

    def _build_autonav_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "🚀  Auto-Navigation")

        row1 = tk.Frame(inner, bg=C["surface0"])
        row1.pack(fill="x", pady=(0, 5))
        self._btn_goto   = make_btn(row1, "Go To Selected",  self._goto_selected,
                                    color="green")
        self._btn_goto.pack(side="left", padx=(0, 5))
        self._btn_cancel = make_btn(row1, "Cancel",          self._cancel_navigation,
                                    color="red")
        self._btn_cancel.pack(side="left", padx=(0, 5))
        self._btn_cancel.config(state="disabled")
        make_btn(row1, "Manual Recovery", self._manual_recovery,
                 color="yellow").pack(side="left")

        row2 = tk.Frame(inner, bg=C["surface0"])
        row2.pack(fill="x")
        make_btn(row2, "Update Selected Pose", self._update_pose,
                 color="mauve").pack(side="left", padx=(0, 5))
        make_btn(row2, "Delete Selected",      self._delete_location,
                 color="orange").pack(side="left")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 6 — Joystick Control  (status monitor only)
    # ─────────────────────────────────────────────────────────────────────────

    def _build_joystick_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "🕹   Joystick Control")

        inner.grid_columnconfigure(0, weight=1)
        inner.grid_rowconfigure(1, weight=1, minsize=200)

        # ── Row 0: controls (fixed height strip) ──────────────────────────
        controls = tk.Frame(inner, bg=C["surface0"])
        controls.grid(row=0, column=0, sticky="new")

        top_row = tk.Frame(controls, bg=C["surface0"])
        top_row.pack(fill="x", pady=(0, 5))
        self._btn_joy_toggle = make_btn(top_row, "⏵  Enable Joystick",
                                        self._toggle_joystick, color="green")
        self._btn_joy_toggle.pack(side="left")

        make_sep(controls).pack(fill="x", pady=(0, 5))

        conn_row = tk.Frame(controls, bg=C["surface0"])
        conn_row.pack(fill="x", pady=2)
        tk.Label(conn_row, text="Joy Node:", font=FONT_BODY, width=_LW, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")
        self._joy_dot = make_dot(conn_row, "surface2")
        self._joy_dot.pack(side="left")
        self._joy_status_lbl = tk.Label(conn_row, text=" Waiting…",
                                        font=FONT_MONO, bg=C["surface0"], fg=C["subtext"])
        self._joy_status_lbl.pack(side="left")

        self._joy_linear_lbl  = kv_row(controls, "Linear Axis:",    " 0.000", "sky")
        self._joy_angular_lbl = kv_row(controls, "Angular Axis:",   " 0.000", "sky")
        self._joy_buttons_lbl = kv_row(controls, "Btn Pressed:",    "None",   "yellow")

        make_sep(controls).pack(fill="x", pady=(6, 4))

        make_label(controls, "Teleop Output:", color="subtext", font=FONT_SMALL).pack(anchor="w")

        # ── Row 1: log area (absorbs all extra vertical space) ────────────
        log_frame = tk.Frame(inner, bg=C["surface0"])
        log_frame.grid(row=1, column=0, sticky="nsew", pady=(2, 0))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self._joy_log_text = tk.Text(
            log_frame, font=FONT_MONO, bg=C["mantle"], fg=C["subtext"],
            relief="flat", bd=2, wrap="word", state="disabled", height=12,
            insertbackground=C["text"], selectbackground=C["surface1"],
        )
        joy_scroll = tk.Scrollbar(log_frame, command=self._joy_log_text.yview,
                                  bg=C["surface0"], troughcolor=C["mantle"])
        self._joy_log_text.configure(yscrollcommand=joy_scroll.set)
        self._joy_log_text.grid(row=0, column=0, sticky="nsew")
        joy_scroll.grid(row=0, column=1, sticky="ns")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel — Navigation strip (Enable/Disable · goal status)
    # ─────────────────────────────────────────────────────────────────────────

    def _build_navstatus_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "🧭  Navigation")

        nav_btns = tk.Frame(inner, bg=C["surface0"])
        nav_btns.pack(fill="x", pady=(0, 8))
        self._btn_nav_enable = make_btn(
            nav_btns,
            "Enable",
            self._start_navigation,
            color="green",
            width=10,
        )
        self._btn_nav_enable.pack(side="left", padx=(0, 6))
        self._btn_nav_disable = make_btn(
            nav_btns,
            "Disable",
            self._stop_navigation,
            color="orange",
            width=10,
        )
        self._btn_nav_disable.pack(side="left")

        self._btn_nav_disable.config(state=tk.DISABLED)

        # Two compact side-by-side pairs
        row_a = tk.Frame(inner, bg=C["surface0"])
        row_a.pack(fill="x", pady=4)
        row_b = tk.Frame(inner, bg=C["surface0"])
        row_b.pack(fill="x", pady=4)

        def nav_field(parent, label: str) -> tk.Label:
            tk.Label(parent, text=label, font=FONT_SMALL, width=14, anchor="w",
                     bg=C["surface0"], fg=C["subtext"]).pack(side="left")
            val = tk.Label(parent, text="—", font=FONT_MONO,
                           bg=C["surface0"], fg=C["text"], anchor="w", width=14)
            val.pack(side="left", padx=(0, 10))
            return val

        self._nav_goal_name_lbl = nav_field(row_a, "Current Goal:")
        self._nav_state_lbl     = nav_field(row_a, "Goal State:")
        self._nav_dist_lbl      = nav_field(row_b, "Distance:")
        self._nav_event_lbl     = nav_field(row_b, "Last Event:")

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Panel — Camera (compressed image stream)
    # ─────────────────────────────────────────────────────────────────────────

    def _build_camera_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📷  Camera")

        tk.Label(
            inner,
            text=CAMERA_COMPRESSED_TOPIC,
            font=FONT_SMALL,
            bg=C["surface0"],
            fg=C["subtext"],
            anchor="w",
        ).pack(fill="x", pady=(0, 6))

        cam_wrap = tk.Frame(inner, bg=C["mantle"], highlightthickness=1,
                            highlightbackground=C["surface1"])
        cam_wrap.pack(fill="both", expand=True)

        waiting = (
            "Waiting for stream…"
            if ROS_AVAILABLE
            else "ROS unavailable — source workspace and restart"
        )
        if ROS_AVAILABLE and not CV2_NUMPY_AVAILABLE:
            waiting = "OpenCV/NumPy missing — install requirements.txt"

        self._camera_image_lbl = tk.Label(
            cam_wrap,
            text=waiting,
            font=FONT_BODY,
            bg=C["mantle"],
            fg=C["subtext"],
            anchor="center",
            justify="center",
            wraplength=440,
            padx=8,
            pady=24,
        )
        self._camera_image_lbl.pack(fill="both", expand=True)

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Log split view — Event (GUI messages) vs Navigation (ros2 launch stream)
    # ─────────────────────────────────────────────────────────────────────────

    def _rewrite_event_display(self) -> None:
        """Redraw the upper-right text widget from `_all_log_lines` per `_log_filter`."""
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        for level, line in self._all_log_lines:
            if self._log_filter == "all" or level in ("ERROR", "WARNING"):
                self._log_text.insert("end", line + "\n", level)
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def _rewrite_navigation_display(self) -> None:
        """Redraw navigation launch output (buffer only; no ROS calls)."""
        self._log_text.config(state="normal")
        self._log_text.delete("1.0", "end")
        nav_body = "".join(self._nav_launch_log)
        if nav_body:
            self._log_text.insert("1.0", nav_body, "NAVOUT")
        else:
            self._log_text.insert(
                "1.0",
                "(No navigation launch output yet — use Start Navigation or Enable.)\n",
                "DEBUG",
            )
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def _refresh_log_filter_button_state(self) -> None:
        st = tk.NORMAL if self._log_panel_mode == "event" else tk.DISABLED
        if hasattr(self, "_btn_log_show_all"):
            self._btn_log_show_all.config(state=st)
            self._btn_log_errors.config(state=st)

    def _refresh_log_mode_buttons(self) -> None:
        if getattr(self, "_btn_log_event", None) is None:
            return
        if self._log_panel_mode == "event":
            self._btn_log_event.config(
                bg=C["blue"], fg=C["base"], activeforeground=C["base"])
            self._btn_log_nav.config(
                bg=C["surface1"], fg=C["subtext"], activeforeground=C["text"])
        else:
            self._btn_log_event.config(
                bg=C["surface1"], fg=C["subtext"], activeforeground=C["text"])
            self._btn_log_nav.config(
                bg=C["blue"], fg=C["base"], activeforeground=C["base"])
        self._refresh_log_filter_button_state()

    def _set_log_panel_mode(self, mode: str) -> None:
        if mode not in ("event", "navigation"):
            return
        if mode == self._log_panel_mode:
            self._refresh_log_mode_buttons()
            return
        self._log_panel_mode = mode
        if mode == "event":
            self._rewrite_event_display()
        else:
            self._rewrite_navigation_display()
        self._refresh_log_mode_buttons()

    def _append_nav_launch_line(self, text: str) -> None:
        """Accumulate ros2 navigation launch stdout/stderr; update UI if Navigation log visible."""
        self._nav_launch_log.append(text)
        if self._log_panel_mode != "navigation":
            return
        self._log_text.config(state="normal")
        self._log_text.insert("end", text, "NAVOUT")
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    # ─────────────────────────────────────────────────────────────────────────
    # Panel 7 — Logs (Event vs Navigation)
    # ─────────────────────────────────────────────────────────────────────────

    def _build_log_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📋  Logs")
        inner.rowconfigure(1, weight=1, minsize=280)
        inner.columnconfigure(0, weight=1)

        mode_row = tk.Frame(inner, bg=C["surface0"])
        mode_row.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        tk.Label(
            mode_row,
            text="View:",
            font=FONT_BODY,
            bg=C["surface0"],
            fg=C["subtext"],
        ).pack(side="left", padx=(0, 8))
        self._btn_log_event = tk.Button(
            mode_row,
            text=" Event log ",
            command=lambda: self._set_log_panel_mode("event"),
            font=FONT_BODY,
            relief="flat",
            cursor="hand2",
            bd=0,
            padx=10,
            pady=5,
        )
        self._btn_log_event.pack(side="left", padx=(0, 6))
        self._btn_log_nav = tk.Button(
            mode_row,
            text=" Navigation log ",
            command=lambda: self._set_log_panel_mode("navigation"),
            font=FONT_BODY,
            relief="flat",
            cursor="hand2",
            bd=0,
            padx=10,
            pady=5,
        )
        self._btn_log_nav.pack(side="left")
        make_label(
            mode_row,
            "ros2 launch output when Navigation is enabled",
            color="overlay0",
        ).pack(side="left", padx=(12, 0))

        self._log_text = tk.Text(
            inner, font=FONT_MONO, bg=C["mantle"], fg=C["text"],
            relief="flat", bd=4, wrap="word",
            state="disabled", height=14,
            insertbackground=C["text"],
            selectbackground=C["surface1"],
        )
        scrollbar = tk.Scrollbar(inner, command=self._log_text.yview,
                                 bg=C["surface0"], troughcolor=C["mantle"],
                                 activebackground=C["surface1"])
        self._log_text.configure(yscrollcommand=scrollbar.set)
        self._log_text.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
        scrollbar.grid(row=1, column=1, sticky="ns", pady=(0, 6))

        # Colour tags — each event log level gets its own foreground colour
        self._log_text.tag_configure("INFO",    foreground=C["text"])
        self._log_text.tag_configure("SUCCESS", foreground=C["green"])
        self._log_text.tag_configure("WARNING", foreground=C["yellow"])
        self._log_text.tag_configure("ERROR",   foreground=C["red"])
        self._log_text.tag_configure("DEBUG",   foreground=C["overlay0"])
        self._log_text.tag_configure("NAVOUT",  foreground=C["teal"])

        # Button row below the log
        btn_row = tk.Frame(inner, bg=C["surface0"])
        btn_row.grid(row=2, column=0, columnspan=2, sticky="ew")
        make_btn(btn_row, "Clear",       self._clear_log,        color="gray").pack(side="left", padx=(0, 4))
        make_btn(btn_row, "Save Log",    self._save_log,         color="gray").pack(side="left", padx=(0, 4))
        self._btn_log_show_all = make_btn(btn_row, "Show All", self._show_all_logs, color="gray")
        self._btn_log_show_all.pack(side="left", padx=(0, 4))
        self._btn_log_errors = make_btn(btn_row, "Errors Only", self._show_errors_only, color="red")
        self._btn_log_errors.pack(side="left")

        self._refresh_log_mode_buttons()

        return outer

    # ─────────────────────────────────────────────────────────────────────────
    # Process management
    # ─────────────────────────────────────────────────────────────────────────

    def _start_navigation(self) -> None:
        if self._proc_alive("nav"):
            self._log("WARNING", "SYSTEM", "Navigation is already running")
            return
        cmd = [
            "ros2", "launch", "turtlebot3_navigation2", "navigation2.launch.py",
            "use_sim_time:=false", f"map:={MAP_YAML.resolve()}",
        ]
        self._append_nav_launch_line(
            f"--- starting: {' '.join(cmd)} ---\n",
        )
        self._launch_navigation_streaming(cmd)
        self._log("INFO", "NAV", f"Navigation launched  (map: {MAP_YAML.name})")
        self._set_mode("Manual")

    def _start_teleop(self) -> None:
        if self._proc_alive("teleop"):
            self._log("WARNING", "SYSTEM", "Teleop is already running")
            return
        cmd = ["ros2", "launch", "custom_turtlebot_nodes", "joystick_teleop.launch.py"]
        self._launch_proc("teleop", cmd)
        self._log("INFO", "TELEOP", "Joystick teleop launched")

    def _stop_navigation(self) -> None:
        had_nav = self._proc_alive("nav")
        self._kill_proc("nav")
        if had_nav:
            self._append_nav_launch_line("\n--- navigation process exited ---\n")
        self._log("INFO", "NAV", "Navigation stopped")
        self._set_mode("Idle")

    def _stop_teleop(self) -> None:
        self._kill_proc("teleop")
        self._log("INFO", "TELEOP", "Teleop stopped")

    def _toggle_joystick(self) -> None:
        """Launch or kill the teleop process; stream its output to the joystick log."""
        if self._proc_alive("teleop"):
            self._kill_proc("teleop")
            self._btn_joy_toggle.config(text="⏵  Enable Joystick",
                                        bg=C["green"], fg=C["base"])
            self._log("INFO", "TELEOP", "Joystick teleop disabled")
            self._joy_append("--- teleop stopped ---\n")
        else:
            cmd = ["ros2", "launch", "custom_turtlebot_nodes", "joystick_teleop.launch.py"]
            try:
                tp_kwargs = dict(
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                if sys.platform != "win32":
                    tp_kwargs["start_new_session"] = True
                proc = subprocess.Popen(cmd, **tp_kwargs)
                self._procs["teleop"] = proc
                # Background thread reads lines and pushes them to the queue
                threading.Thread(
                    target=self._read_teleop_output, args=(proc,), daemon=True
                ).start()
            except FileNotFoundError:
                self._log("ERROR", "SYSTEM", "ros2 not found — is ROS sourced?")
                return
            self._btn_joy_toggle.config(text="⏹  Disable Joystick",
                                        bg=C["red"], fg=C["base"])
            self._log("INFO", "TELEOP", "Joystick teleop launched")
            self._joy_append("--- teleop started ---\n")

    def _read_teleop_output(self, proc: subprocess.Popen) -> None:
        """Read stdout/stderr from the teleop process line by line."""
        try:
            for line in iter(proc.stdout.readline, ""):
                self._q.put(("joy_log", line))
        except (OSError, ValueError):
            pass

    def _joy_append(self, text: str) -> None:
        """Append text to the joystick log widget."""
        self._joy_log_text.config(state="normal")
        self._joy_log_text.insert("end", text)
        self._joy_log_text.see("end")
        self._joy_log_text.config(state="disabled")

    def _stop_all(self) -> None:
        if self._goal_active:
            self._cancel_navigation()
        had_nav = self._proc_alive("nav")
        for key in list(self._procs):
            self._kill_proc(key)
        if had_nav:
            self._append_nav_launch_line("\n--- navigation process exited ---\n")
        for key in list(self._tool_procs):
            self._kill_tool_proc(key)
        self._log("WARNING", "SYSTEM", "All processes stopped")
        self._set_mode("Idle")

    def _launch_navigation_streaming(self, cmd: list[str]) -> None:
        """Run Nav2 launch with merged stdout/stderr streamed to Navigation log."""
        try:
            kwargs: dict = {
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "text": True,
                "bufsize": 1,
                "errors": "replace",
            }
            if sys.platform != "win32":
                kwargs["start_new_session"] = True
            proc = subprocess.Popen(cmd, **kwargs)
            self._procs["nav"] = proc
            threading.Thread(
                target=self._read_navigation_output,
                args=(proc,),
                daemon=True,
            ).start()
        except FileNotFoundError:
            self._log("ERROR", "SYSTEM", f"Command not found: '{cmd[0]}' — is ROS sourced?")
            self._append_nav_launch_line("[spawn failed] ros2 not found\n")

    def _read_navigation_output(self, proc: subprocess.Popen) -> None:
        stream = proc.stdout
        if stream is None:
            return
        try:
            for line in iter(stream.readline, ""):
                if not line:
                    break
                self._q.put(("nav_log", line))
        except (OSError, ValueError):
            pass

    def _launch_proc(self, key: str, cmd: list[str]) -> None:
        try:
            kwargs: dict = {
                "stdout": subprocess.DEVNULL,
                "stderr": subprocess.DEVNULL,
            }
            # Own process group → ``killpg(SIGINT)`` reaches RViz & nodes (like Ctrl+C).
            if sys.platform != "win32":
                kwargs["start_new_session"] = True
            self._procs[key] = subprocess.Popen(cmd, **kwargs)
        except FileNotFoundError:
            self._log("ERROR", "SYSTEM", f"Command not found: '{cmd[0]}' — is ROS sourced?")

    def _kill_proc(self, key: str) -> None:
        proc = self._procs.pop(key, None)
        _shutdown_ros_launch_process(proc)

    def _proc_alive(self, key: str) -> bool:
        return key in self._procs and self._procs[key].poll() is None

    # ─────────────────────────────────────────────────────────────────────────
    # Initialisation page — ros2 topic echo / hz (subprocess monitors)
    # ─────────────────────────────────────────────────────────────────────────

    def _append_init_log(self, text: str) -> None:
        w = getattr(self, "_init_log_text", None)
        if w is None:
            return
        w.config(state="normal")
        w.insert("end", text)
        w.see("end")
        w.config(state="disabled")

    def _kill_tool_proc(self, key: str) -> None:
        proc = self._tool_procs.pop(key, None)
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        short = self._init_tool_short.get(key, key.split(":", 1)[-1])
        btn = self._init_tool_btns.get(key)
        if btn is not None:
            btn.config(text=f"  {short}  □", bg=C["surface1"], fg=C["subtext"])

    def _tool_proc_alive(self, key: str) -> bool:
        p = self._tool_procs.get(key)
        return p is not None and p.poll() is None

    def _read_tool_output(self, key: str, proc: subprocess.Popen) -> None:
        stream = proc.stdout
        if stream is None:
            return
        try:
            for line in iter(stream.readline, ""):
                if not line:
                    break
                self._q.put(("init_log", line))
        except (OSError, ValueError):
            pass
        # Process ended — refresh button state on main thread
        self.root.after(
            0, lambda k=key: self._on_tool_proc_ended(k),
        )

    def _on_tool_proc_ended(self, key: str) -> None:
        if key in self._tool_procs:
            p = self._tool_procs.get(key)
            if p is not None and p.poll() is not None:
                self._tool_procs.pop(key, None)
        btn = self._init_tool_btns.get(key)
        if btn is not None:
            short = self._init_tool_short.get(key, key.split(":", 1)[-1])
            btn.config(text=f"  {short}  □", bg=C["surface1"], fg=C["subtext"])

    def _start_tool_stream(self, key: str, cmd: list[str], header: str) -> bool:
        if self._tool_proc_alive(key):
            return True
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(WORKSPACE_ROOT),
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                errors="replace",
            )
        except FileNotFoundError:
            self._append_init_log(f"[error] '{cmd[0]}' not found — source ROS?\n")
            self._log("ERROR", "SYSTEM", f"{cmd[0]} not found — is ROS sourced?")
            return False
        self._tool_procs[key] = proc
        self._q.put(("init_log", header))
        threading.Thread(
            target=self._read_tool_output, args=(key, proc), daemon=True
        ).start()
        btn = self._init_tool_btns.get(key)
        if btn is not None:
            short = self._init_tool_short.get(key, key.split(":", 1)[-1])
            btn.config(text=f"  {short}  ■", bg=C["teal"], fg=C["base"])
        return True

    def _toggle_topic_echo(self, topic: str) -> None:
        key = f"echo:{topic}"
        if self._tool_proc_alive(key):
            self._kill_tool_proc(key)
            self._append_init_log(f"--- stopped: ros2 topic echo {topic} ---\n")
            return
        ok = self._start_tool_stream(
            key,
            ["ros2", "topic", "echo", topic],
            f"\n--- ros2 topic echo {topic} ---\n",
        )
        if not ok:
            self._kill_tool_proc(key)

    def _toggle_topic_hz_amcl(self) -> None:
        key = "hz:/amcl_pose"
        if self._tool_proc_alive(key):
            self._kill_tool_proc(key)
            self._append_init_log("--- stopped: ros2 topic hz /amcl_pose ---\n")
            return
        ok = self._start_tool_stream(
            key,
            ["ros2", "topic", "hz", "/amcl_pose"],
            "\n--- ros2 topic hz /amcl_pose ---\n",
        )
        if not ok:
            self._kill_tool_proc(key)

    def _stop_all_init_monitors(self) -> None:
        for key in list(self._tool_procs):
            self._kill_tool_proc(key)
        self._append_init_log("--- all CLI monitors stopped ---\n")

    def _run_ros_node_list(self) -> None:
        def work() -> None:
            try:
                r = subprocess.run(
                    ["ros2", "node", "list"],
                    cwd=str(WORKSPACE_ROOT),
                    env=os.environ.copy(),
                    capture_output=True,
                    text=True,
                    timeout=45,
                    errors="replace",
                )
                out = (r.stdout or "") + (r.stderr or "")
                self._q.put(("init_log", "\n--- ros2 node list ---\n" + out + "\n"))
            except Exception as exc:
                self._q.put(("init_log", f"\n[ros2 node list error] {exc}\n"))

        threading.Thread(target=work, daemon=True).start()

    def _emergency_zero_cmd_vel(self) -> None:
        if ROS_AVAILABLE and self._ros_node is not None:
            self._ros_node.publish_zero_twist()
            self._append_init_log(
                f"[cmd_vel] published zero ({self._ros_node._cmd_vel_log_label})\n"
            )
            self._log("WARNING", "TELEOP", "Emergency zero velocity on /cmd_vel")
            return
        self._append_init_log(
            "[cmd_vel] ROS node offline — examples:\n"
            "  ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "
            '"{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" -1\n'
            "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
            '"{linear: {x: 0.0}, angular: {z: 0.0}}" -1\n'
        )
        self._log("ERROR", "TELEOP", "Cannot zero /cmd_vel — ROS not connected")

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation control
    # ─────────────────────────────────────────────────────────────────────────

    def _goto_selected(self) -> None:
        name = self._selected_loc.get()
        if not name:
            messagebox.showwarning("No Selection",
                                   "Please select a saved location first.")
            return
        if name not in self._locations:
            messagebox.showerror("Not Found",
                                 f"Location '{name}' not found in the saved file.")
            return
        if not ROS_AVAILABLE or self._ros_node is None:
            self._log("ERROR", "NAV", "ROS not available — cannot send navigation goal")
            return

        loc = self._locations[name]
        self._goal_active = True
        self._refresh_nav_buttons()
        self._set_mode("Auto Navigation")
        self._nav_goal_name_lbl.config(text=name)
        self._nav_state_lbl.config(text="Sending…", fg=C["blue"])
        self._ros_node.send_goal(loc["x"], loc["y"], loc.get("yaw", 0.0), name)

    def _cancel_navigation(self) -> None:
        if ROS_AVAILABLE and self._ros_node:
            self._ros_node.cancel_goal()
        self._goal_active = False
        self._refresh_nav_buttons()
        self._set_mode("Manual")

    def _manual_recovery(self) -> None:
        self._cancel_navigation()
        self._set_mode("Recovery")
        self._mode_lbl.config(fg=C["orange"])
        self._log("INFO", "RECOVERY", "Manual recovery — use joystick to take control")

    # ─────────────────────────────────────────────────────────────────────────
    # Waypoint management
    # ─────────────────────────────────────────────────────────────────────────

    def _save_location(self) -> None:
        name = self._loc_name_var.get().strip()
        if not name:
            messagebox.showwarning("Empty Name", "Enter a name for this location.")
            return
        x, y, yaw = self._current_pose
        self._locations[name] = {
            "x":   round(x,   4),
            "y":   round(y,   4),
            "yaw": round(yaw, 2),
        }
        self._save_locations_file()
        self._refresh_dropdown()
        self._selected_loc.set(name)
        self._loc_name_var.set("")
        self._log("SUCCESS", "WAYPOINT",
                  f"Saved '{name}'  x={x:.3f} y={y:.3f} yaw={yaw:.1f}°")

    def _update_pose(self) -> None:
        name = self._selected_loc.get()
        if not name:
            messagebox.showwarning("No Selection", "Select a location to update.")
            return
        x, y, yaw = self._current_pose
        self._locations[name] = {
            "x":   round(x,   4),
            "y":   round(y,   4),
            "yaw": round(yaw, 2),
        }
        self._save_locations_file()
        self._log("SUCCESS", "WAYPOINT",
                  f"Updated '{name}'  x={x:.3f} y={y:.3f} yaw={yaw:.1f}°")

    def _delete_location(self) -> None:
        name = self._selected_loc.get()
        if not name:
            messagebox.showwarning("No Selection", "Select a location to delete.")
            return
        if not messagebox.askyesno("Confirm Delete", f"Delete saved location '{name}'?"):
            return
        self._locations.pop(name, None)
        self._save_locations_file()
        self._refresh_dropdown()
        self._log("WARNING", "WAYPOINT", f"Deleted '{name}'")

    def _edit_location(self) -> None:
        """Open a dialog to rename a location and/or manually edit its x, y, yaw."""
        name = self._selected_loc.get()
        if not name:
            messagebox.showwarning("No Selection", "Select a location to edit.")
            return
        loc = self._locations.get(name, {})

        # ── Dialog window ─────────────────────────────────────────────────
        dlg = tk.Toplevel(self.root)
        dlg.title("Edit Location")
        dlg.configure(bg=C["base"])
        dlg.resizable(False, False)
        dlg.grab_set()                       # modal — block the main window
        dlg.transient(self.root)

        # Header
        hdr = tk.Frame(dlg, bg=C["surface1"])
        hdr.pack(fill="x")
        tk.Label(hdr, text=f"  ✏   Edit  ·  {name}",
                 font=FONT_HEADER, bg=C["surface1"], fg=C["sky"],
                 pady=8, padx=10).pack(side="left")

        body = tk.Frame(dlg, bg=C["base"], padx=20, pady=14)
        body.pack(fill="both", expand=True)

        def field(label: str, default: str) -> tk.StringVar:
            """One labelled entry row; returns its StringVar."""
            row = tk.Frame(body, bg=C["base"])
            row.pack(fill="x", pady=5)
            tk.Label(row, text=label, width=10, anchor="w",
                     font=FONT_BODY, bg=C["base"], fg=C["subtext"]).pack(side="left")
            var = tk.StringVar(value=default)
            tk.Entry(row, textvariable=var, font=FONT_MONO,
                     bg=C["mantle"], fg=C["text"],
                     insertbackground=C["text"],
                     relief="flat", bd=4, width=22).pack(side="left")
            return var

        var_name = field("Name",  name)
        var_x    = field("X (m)", str(loc.get("x",   0.0)))
        var_y    = field("Y (m)", str(loc.get("y",   0.0)))
        var_yaw  = field("Yaw °", str(loc.get("yaw", 0.0)))

        # ── Save / Cancel buttons ─────────────────────────────────────────
        def apply() -> None:
            new_name = var_name.get().strip()
            if not new_name:
                messagebox.showwarning("Empty Name", "Name cannot be empty.", parent=dlg)
                return
            try:
                new_x   = float(var_x.get())
                new_y   = float(var_y.get())
                new_yaw = float(var_yaw.get())
            except ValueError:
                messagebox.showerror("Invalid Value",
                                     "X, Y and Yaw must be numbers.", parent=dlg)
                return

            # If the name changed, remove the old entry
            if new_name != name:
                self._locations.pop(name, None)

            self._locations[new_name] = {
                "x":   round(new_x,   4),
                "y":   round(new_y,   4),
                "yaw": round(new_yaw, 2),
            }
            self._save_locations_file()
            self._refresh_dropdown()
            self._selected_loc.set(new_name)
            self._log("SUCCESS", "WAYPOINT",
                      f"Edited '{name}' → '{new_name}'  "
                      f"x={new_x:.3f} y={new_y:.3f} yaw={new_yaw:.1f}°")
            dlg.destroy()

        btn_row = tk.Frame(body, bg=C["base"])
        btn_row.pack(fill="x", pady=(10, 0))
        make_btn(btn_row, "Save Changes", apply,       color="teal" ).pack(side="left", padx=(0, 8))
        make_btn(btn_row, "Cancel",       dlg.destroy, color="gray" ).pack(side="left")

        # Center dialog over main window
        self.root.update_idletasks()
        rx, ry = self.root.winfo_x(), self.root.winfo_y()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        dlg.update_idletasks()
        dw, dh = dlg.winfo_width(), dlg.winfo_height()
        dlg.geometry(f"+{rx + (rw - dw)//2}+{ry + (rh - dh)//2}")

    # ─────────────────────────────────────────────────────────────────────────
    # Log panel helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _log(self, level: str, module: str, message: str) -> None:
        now  = datetime.now().strftime("%H:%M:%S")
        line = f"[{now}][{level:<7}][{module:<8}] {message}"
        self._all_log_lines.append((level, line))
        if self._log_panel_mode != "event":
            return
        if self._log_filter == "all" or level in ("ERROR", "WARNING"):
            self._write_log_line(level, line + "\n")

    def _write_log_line(self, level: str, text: str) -> None:
        self._log_text.config(state="normal")
        self._log_text.insert("end", text, level)
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    def _clear_log(self) -> None:
        if self._log_panel_mode == "event":
            self._all_log_lines.clear()
            self._rewrite_event_display()
        else:
            self._nav_launch_log.clear()
            self._rewrite_navigation_display()

    def _save_log(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        LOGS_DIR.mkdir(exist_ok=True)
        if self._log_panel_mode == "navigation":
            path = LOGS_DIR / f"navigation_launch_{ts}.txt"
            with open(path, "w") as fh:
                fh.write("".join(self._nav_launch_log))
        else:
            path = LOGS_DIR / f"session_{ts}.txt"
            with open(path, "w") as fh:
                for _, line in self._all_log_lines:
                    fh.write(line + "\n")
        self._log("INFO", "SYSTEM", f"Log saved → {path.name}")

    def _show_all_logs(self) -> None:
        if self._log_panel_mode != "event":
            return
        self._log_filter = "all"
        self._rewrite_event_display()

    def _show_errors_only(self) -> None:
        if self._log_panel_mode != "event":
            return
        self._log_filter = "errors"
        self._rewrite_event_display()

    # ─────────────────────────────────────────────────────────────────────────
    # Locations file I/O
    # ─────────────────────────────────────────────────────────────────────────

    def _load_locations(self) -> None:
        if LOCATIONS_FILE.exists():
            try:
                with open(LOCATIONS_FILE) as fh:
                    self._locations = json.load(fh)
            except (json.JSONDecodeError, OSError):
                self._locations = {}
        else:
            self._locations = {}

    def _save_locations_file(self) -> None:
        LOCATIONS_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(LOCATIONS_FILE, "w") as fh:
            json.dump(self._locations, fh, indent=2)

    def _refresh_dropdown(self) -> None:
        names = list(self._locations.keys())
        self._loc_combo["values"] = names
        current = self._selected_loc.get()
        if names and current not in names:
            self._selected_loc.set(names[0])
        elif not names:
            self._selected_loc.set("")

    # ─────────────────────────────────────────────────────────────────────────
    # State helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _set_mode(self, mode: str) -> None:
        mode_colors = {
            "Idle":            C["subtext"],
            "Manual":          C["sky"],
            "Auto Navigation": C["green"],
            "Recovery":        C["orange"],
            "Error":           C["red"],
        }
        self._mode_lbl.config(text=mode, fg=mode_colors.get(mode, C["text"]))

    def _refresh_nav_buttons(self) -> None:
        if self._goal_active:
            self._btn_goto  .config(state="disabled")
            self._btn_cancel.config(state="normal")
        else:
            self._btn_goto  .config(state="normal")
            self._btn_cancel.config(state="disabled")

    # ─────────────────────────────────────────────────────────────────────────
    # Queue polling — ROS thread → main thread (runs every 100 ms)
    # ─────────────────────────────────────────────────────────────────────────

    def _poll_queue(self) -> None:
        try:
            while True:
                msg  = self._q.get_nowait()
                kind = msg[0]

                if kind == "pose":
                    _, x, y, yaw = msg
                    self._current_pose = (x, y, yaw)
                    self._pose_lbl.config(
                        text=f"x={x:+.3f}    y={y:+.3f}    yaw={yaw:+.1f}°"
                    )

                elif kind == "odom_vel":
                    _, vx, wz = msg
                    self._odom_lbl.config(
                        text=f"vx={vx:+.3f}    ωz={wz:+.3f}",
                    )

                elif kind == "battery":
                    pct   = msg[1]
                    text  = f"{pct:.0f}%" if pct >= 0 else "—"
                    color = (C["green"]  if pct > 40
                             else C["yellow"] if pct > 15
                             else C["red"])
                    self._battery_lbl.config(text=text, fg=color)

                elif kind == "log":
                    _, level, module, message = msg
                    self._log(level, module, message)

                elif kind == "nav_state":
                    state = msg[1]
                    state_colors = {
                        "Sending":   C["blue"],    "Navigating": C["green"],
                        "Succeeded": C["teal"],    "Canceled":   C["yellow"],
                        "Canceling": C["orange"],  "Rejected":   C["red"],
                        "Failed":    C["red"],     "Error":      C["red"],
                    }
                    fg = state_colors.get(state, C["text"])
                    self._nav_state_lbl.config(text=state, fg=fg)
                    self._nav_event_lbl.config(text=state,  fg=fg)
                    if state in ("Succeeded", "Canceled"):
                        self._set_mode("Manual")
                    elif state in ("Failed", "Rejected", "Error"):
                        self._set_mode("Error")

                elif kind == "nav_distance":
                    self._nav_dist_lbl.config(text=f"{msg[1]:.2f} m")

                elif kind == "nav_goal_name":
                    self._nav_goal_name_lbl.config(text=msg[1])

                elif kind == "goal_active":
                    self._goal_active = msg[1]
                    self._refresh_nav_buttons()

                elif kind == "joy_status":
                    connected = msg[1] == "connected"
                    self._joy_dot.config(fg=C["green"] if connected else C["red"])
                    self._joy_status_lbl.config(
                        text=" Connected" if connected else " Disconnected",
                        fg=C["green"] if connected else C["red"],
                    )

                elif kind == "joy_axes":
                    _, linear, angular = msg
                    self._joy_linear_lbl .config(text=f"{linear:+.3f}")
                    self._joy_angular_lbl.config(text=f"{angular:+.3f}")

                elif kind == "joy_buttons":
                    text = msg[1]
                    self._joy_buttons_lbl.config(
                        text=text,
                        fg=C["yellow"] if text != "None" else C["subtext"],
                    )

                elif kind == "joy_log":
                    self._joy_append(msg[1])

                elif kind == "nav_log":
                    self._append_nav_launch_line(msg[1])

                elif kind == "init_log":
                    self._append_init_log(msg[1])

                elif kind == "camera_frame":
                    ppm_bytes = msg[1]
                    try:
                        photo = tk.PhotoImage(data=ppm_bytes)
                        self._camera_photo_ref = photo
                        self._camera_image_lbl.config(image=photo, text="")
                    except tk.TclError:
                        pass

        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    # Process alive poll — updates the dot indicators every 1.5 s
    def _poll_processes(self) -> None:
        for key, dot, lbl_widget, running_text in (
            ("nav",    self._nav_dot,    self._nav_proc_lbl,    "Navigation"),
            ("teleop", self._teleop_dot, self._teleop_proc_lbl, "Teleop"),
        ):
            if self._proc_alive(key):
                dot.config(fg=C["green"])
                lbl_widget.config(text=f"{running_text}: Running", fg=C["green"])
            else:
                dot.config(fg=C["surface2"])
                lbl_widget.config(text=f"{running_text}: Stopped", fg=C["subtext"])

        # Keep the joystick toggle button label in sync with the actual process state
        if self._proc_alive("teleop"):
            self._btn_joy_toggle.config(text="⏹  Disable Joystick",
                                        bg=C["red"], fg=C["base"])
        else:
            self._btn_joy_toggle.config(text="⏵  Enable Joystick",
                                        bg=C["green"], fg=C["base"])

        # Arm teleop status dot (only when the arm window is open)
        arm_running = (
            self._arm_teleop_proc is not None
            and self._arm_teleop_proc.poll() is None
        )
        arm_dot = self._arm_dot
        arm_lbl = self._arm_proc_lbl
        if arm_dot is not None and arm_lbl is not None:
            try:
                if arm_running:
                    arm_dot.config(fg=C["green"])
                    arm_lbl.config(text="Arm Teleop: Running", fg=C["green"])
                else:
                    arm_dot.config(fg=C["surface2"])
                    arm_lbl.config(text="Arm Teleop: Stopped", fg=C["subtext"])
            except tk.TclError:
                pass

        # Navigation Enable/Disable — avoid double-start while stack is running
        if self._proc_alive("nav"):
            self._btn_nav_enable.config(state=tk.DISABLED)
            self._btn_nav_disable.config(state=tk.NORMAL)
        else:
            self._btn_nav_enable.config(state=tk.NORMAL)
            self._btn_nav_disable.config(state=tk.DISABLED)

        self.root.after(1500, self._poll_processes)

    # ─────────────────────────────────────────────────────────────────────────
    # ROS2 spin
    # ─────────────────────────────────────────────────────────────────────────

    def _start_ros(self) -> None:
        if not ROS_AVAILABLE:
            self._ros_badge.config(text="  ROS: Unavailable  ", bg=C["red"])
            self._log("ERROR", "ROS",
                      "rclpy not importable — source your ROS workspace before running")
            return
        try:
            rclpy.init()
            self._ros_node   = TeachNavNode(self._q)
            self._ros_thread = threading.Thread(
                target=rclpy.spin, args=(self._ros_node,), daemon=True
            )
            self._ros_thread.start()
            self._ros_badge.config(text="  ROS: Connected  ",
                                   bg=C["green"], fg=C["crust"])
            self._log(
                "INFO",
                "ROS",
                f"Node started — /amcl_pose, /battery_state, {CAMERA_COMPRESSED_TOPIC}",
            )
        except Exception as exc:
            self._ros_badge.config(text="  ROS: Error  ", bg=C["red"])
            self._log("ERROR", "ROS", f"Failed to initialise ROS node: {exc}")

    # ─────────────────────────────────────────────────────────────────────────
    # AI assistant (separate Toplevel — chat + tools + tool log)
    # ─────────────────────────────────────────────────────────────────────────

    def _stop_ai_mcp_worker(self) -> None:
        th = self._mcp_thread
        uq = self._mcp_user_q
        st = self._mcp_stop
        self._mcp_thread = None
        self._mcp_user_q = None
        self._mcp_tk_q = None
        self._mcp_stop = None
        if st is not None and uq is not None:
            try:
                from mcp_server.tk_worker import stop_mcp_worker

                stop_mcp_worker(st, uq, th)
            except BaseException:
                pass

    def _start_ai_mcp_worker(self) -> None:
        if self._mcp_thread is not None and self._mcp_thread.is_alive():
            self.root.after(120, self._poll_mcp_ai_queue)
            return
        if not os.environ.get("OPENAI_API_KEY"):
            self._ai_append_tool_log(
                "WARNING",
                "OPENAI_API_KEY not set — set workspace .env for live MCP + OpenAI (stub replies only).",
            )
            return
        try:
            from mcp_server.tk_worker import start_mcp_worker
        except ImportError as exc:
            self._ai_append_tool_log("ERROR", f"mcp_server import failed: {exc}")
            return
        self._mcp_thread, self._mcp_user_q, self._mcp_tk_q, self._mcp_stop = start_mcp_worker(
            WORKSPACE_ROOT
        )
        self._ai_append_tool_log("INFO", "Starting MCP stdio worker (mcp_server/mcp_server_ros2.py)…")
        self.root.after(120, self._poll_mcp_ai_queue)

    def _poll_mcp_ai_queue(self) -> None:
        tk_q = self._mcp_tk_q
        if tk_q is None:
            return
        win = getattr(self, "_ai_assistant_win", None)
        if win is None:
            return
        try:
            if not win.winfo_exists():
                return
        except tk.TclError:
            return
        try:
            while True:
                msg = tk_q.get_nowait()
                if not msg:
                    continue
                kind = msg[0]
                if kind == "reply" and len(msg) >= 3:
                    _u, reply = msg[1], msg[2]
                    routed = msg[3].strip() if len(msg) >= 4 else ""
                    self._ai_append_chat("assistant", reply)
                    if routed.startswith("CALL "):
                        self._ai_append_mcp_tool_calls_line(routed)
                    self._ai_append_tool_log(
                        "INFO",
                        f"reply received ({len(str(reply))} chars)",
                    )
                elif kind == "error":
                    err = msg[1] if len(msg) > 1 else "unknown"
                    self._ai_append_tool_log("ERROR", err)
                    self._ai_append_chat("assistant", f"(Error) {err}")
                elif kind == "info":
                    self._ai_append_tool_log("INFO", msg[1] if len(msg) > 1 else "")
        except queue.Empty:
            pass
        if self._mcp_tk_q is not None and getattr(self, "_ai_assistant_win", None) is not None:
            try:
                if self._ai_assistant_win.winfo_exists():
                    self.root.after(120, self._poll_mcp_ai_queue)
            except tk.TclError:
                pass

    def _close_ai_assistant_window(self) -> None:
        self._stop_ai_mcp_worker()
        self._ai_chat_text = None
        self._ai_input_var = None
        self._ai_tool_log = None
        self._ai_mcp_tool_calls_text = None
        self._ai_tool_list = None
        self._ai_tool_desc_lbl = None
        self._ai_input_entry = None
        w = getattr(self, "_ai_assistant_win", None)
        self._ai_assistant_win = None
        if w is not None:
            try:
                w.destroy()
            except tk.TclError:
                pass

    def _open_ai_assistant_window(self) -> None:
        w = getattr(self, "_ai_assistant_win", None)
        if w is not None:
            try:
                if w.winfo_exists():
                    w.lift()
                    w.focus_force()
                    return
            except tk.TclError:
                self._ai_assistant_win = None
        self._build_ai_assistant_window()

    def _schedule_ai_assistant_sash(
        self, win: tk.Toplevel, paned: tk.PanedWindow
    ) -> None:
        """Bias horizontal split toward chat (~72% left); run after geometry exists."""

        def apply_sash() -> None:
            try:
                win.update_idletasks()
                pw = paned.winfo_width()
                if pw < 100:
                    pw = max(win.winfo_width() - 40, 500)
                x = max(400, int(pw * 0.72))
                paned.sash_place(0, x, 2)
            except tk.TclError:
                pass

        win.after(50, apply_sash)
        win.after(250, apply_sash)

    def _build_ai_assistant_window(self) -> None:
        win = tk.Toplevel(self.root)
        self._ai_assistant_win = win
        win.title("AI assistant — TurtleBot Control Center")
        win.configure(bg=C["base"])
        win.geometry("1100x720")
        win.minsize(820, 520)

        win.protocol("WM_DELETE_WINDOW", self._close_ai_assistant_window)

        hdr = tk.Frame(win, bg=C["crust"])
        hdr.pack(fill="x")
        tk.Label(
            hdr,
            text="  AI assistant",
            font=FONT_TITLE,
            bg=C["crust"],
            fg=C["mauve"],
        ).pack(side="left", padx=(12, 6), pady=(10, 10))
        tk.Label(
            hdr,
            text="Chat · tools · MCP invocations · activity log (OpenAI + mcp_server/ when OPENAI_API_KEY is set)",
            font=FONT_SMALL,
            bg=C["crust"],
            fg=C["overlay0"],
        ).pack(side="left", pady=(12, 12))

        # Bottom bar first so the paned area fills remaining height above it.
        footer = tk.Frame(win, bg=C["crust"])
        footer.pack(side="bottom", fill="x", padx=0, pady=0)
        make_btn(
            footer,
            "Close AI assistant",
            self._close_ai_assistant_window,
            color="red",
        ).pack(side="right", padx=(8, 12), pady=(8, 10))

        paned = tk.PanedWindow(
            win,
            orient=tk.HORIZONTAL,
            sashrelief="flat",
            bg=C["base"],
            sashwidth=6,
        )
        paned.pack(fill="both", expand=True, padx=8, pady=(0, 4))

        # ── Left: chat (transcript + ChatGPT-style composer at bottom) ─────
        left = tk.Frame(paned, bg=C["base"])
        outer_chat, inner_chat = make_panel(left, "💬  Chat")
        outer_chat.pack(fill="both", expand=True)
        inner_chat.rowconfigure(0, weight=1)
        inner_chat.columnconfigure(0, weight=1)

        self._ai_chat_text = tk.Text(
            inner_chat,
            font=FONT_BODY,
            bg=C["mantle"],
            fg=C["text"],
            relief="flat",
            bd=4,
            wrap="word",
            state="disabled",
            insertbackground=C["text"],
            selectbackground=C["surface1"],
        )
        self._ai_chat_text.tag_configure(
            "role_label_user", foreground=C["sky"], font=FONT_BODY_BOLD
        )
        self._ai_chat_text.tag_configure("user_body", foreground=C["text"], font=FONT_BODY)
        self._ai_chat_text.tag_configure(
            "role_label_assistant",
            foreground=C["mauve"],
            font=FONT_BODY_BOLD,
        )
        self._ai_chat_text.tag_configure(
            "assistant_body", foreground=C["text"], font=FONT_BODY
        )
        self._ai_chat_text.tag_configure(
            "role_label_system", foreground=C["overlay0"], font=FONT_BODY_BOLD
        )
        self._ai_chat_text.tag_configure(
            "system_body", foreground=C["overlay0"], font=FONT_BODY
        )
        ch_sb = tk.Scrollbar(
            inner_chat,
            command=self._ai_chat_text.yview,
            bg=C["surface0"],
            troughcolor=C["mantle"],
            activebackground=C["surface1"],
        )
        self._ai_chat_text.configure(yscrollcommand=ch_sb.set)
        self._ai_chat_text.grid(row=0, column=0, sticky="nsew", pady=(0, 0))
        ch_sb.grid(row=0, column=1, sticky="ns")

        # Composer strip (fixed at bottom of chat card — like ChatGPT input bar)
        composer = tk.Frame(
            inner_chat,
            bg=C["crust"],
            highlightthickness=1,
            highlightbackground=C["surface1"],
        )
        composer.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        composer.columnconfigure(0, weight=1)

        tk.Label(
            composer,
            text="Message the assistant",
            font=FONT_SMALL,
            bg=C["crust"],
            fg=C["overlay0"],
            anchor="w",
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(8, 2))

        input_row = tk.Frame(composer, bg=C["crust"])
        input_row.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 6))
        input_row.columnconfigure(0, weight=1)

        self._ai_input_var = tk.StringVar()
        self._ai_input_entry = tk.Entry(
            input_row,
            textvariable=self._ai_input_var,
            font=FONT_BODY,
            bg=C["mantle"],
            fg=C["text"],
            insertbackground=C["text"],
            relief="flat",
            bd=6,
            highlightthickness=0,
        )
        self._ai_input_entry.grid(row=0, column=0, sticky="ew", padx=(0, 8), pady=(0, 6), ipady=8)
        self._ai_input_entry.bind("<Return>", lambda e: self._ai_on_send())
        self._ai_input_entry.focus_set()

        btn_send = make_btn(input_row, "Send", self._ai_on_send, color="blue")
        btn_send.grid(row=0, column=1, padx=(0, 4), pady=(0, 6), sticky="ns")

        bottom_row = tk.Frame(composer, bg=C["crust"])
        bottom_row.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 10))
        make_btn(bottom_row, "Clear chat", self._ai_clear_chat, color="gray").pack(
            side="left"
        )
        tk.Label(
            bottom_row,
            text="Enter sends the message",
            font=FONT_SMALL,
            bg=C["crust"],
            fg=C["overlay0"],
        ).pack(side="right", padx=(8, 0))

        paned.add(left, minsize=420)

        # ── Right: tools (top) + tool log (bottom) — kept narrower via sash (see below).
        right = tk.Frame(paned, bg=C["base"])
        vpaned = tk.PanedWindow(
            right,
            orient=tk.VERTICAL,
            sashrelief="flat",
            bg=C["base"],
            sashwidth=6,
        )
        vpaned.pack(fill="both", expand=True)

        outer_tools, inner_tools = make_panel(vpaned, "🛠  Available tools")
        desc_wrap = tk.Frame(inner_tools, bg=C["surface0"])
        desc_wrap.pack(fill="both", expand=True)
        desc_wrap.rowconfigure(0, weight=2)
        desc_wrap.rowconfigure(1, weight=0)
        desc_wrap.columnconfigure(0, weight=1)

        lb_frame = tk.Frame(desc_wrap, bg=C["surface0"])
        lb_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        lb_frame.rowconfigure(0, weight=1)
        lb_frame.columnconfigure(0, weight=1)
        self._ai_tool_list = tk.Listbox(
            lb_frame,
            font=FONT_MONO,
            bg=C["mantle"],
            fg=C["text"],
            selectbackground=C["surface1"],
            selectforeground=C["text"],
            relief="flat",
            bd=4,
            highlightthickness=0,
            activestyle="dotbox",
        )
        for name, _desc in AI_ASSISTANT_TOOL_CATALOG:
            self._ai_tool_list.insert(tk.END, name)
        lb_sb = tk.Scrollbar(
            lb_frame,
            command=self._ai_tool_list.yview,
            bg=C["surface0"],
            troughcolor=C["mantle"],
        )
        self._ai_tool_list.configure(yscrollcommand=lb_sb.set)
        self._ai_tool_list.grid(row=0, column=0, sticky="nsew")
        lb_sb.grid(row=0, column=1, sticky="ns")
        self._ai_tool_list.bind("<<ListboxSelect>>", self._ai_on_tool_select)

        self._ai_tool_desc_lbl = tk.Label(
            desc_wrap,
            text="Select a tool to see its description.",
            font=FONT_SMALL,
            bg=C["surface0"],
            fg=C["subtext"],
            anchor="nw",
            justify="left",
            wraplength=220,
        )
        self._ai_tool_desc_lbl.grid(row=1, column=0, sticky="ew")

        vpaned.add(outer_tools, minsize=200)

        outer_mcp_tc, inner_mcp_tc = make_panel(vpaned, "📌  MCP tool invocations")
        inner_mcp_tc.rowconfigure(0, weight=1)
        inner_mcp_tc.columnconfigure(0, weight=1)
        self._ai_mcp_tool_calls_text = tk.Text(
            inner_mcp_tc,
            font=FONT_MONO,
            bg=C["mantle"],
            fg=C["teal"],
            relief="flat",
            bd=4,
            wrap="word",
            state="disabled",
            height=7,
            insertbackground=C["text"],
            selectbackground=C["surface1"],
        )
        mc_sb = tk.Scrollbar(
            inner_mcp_tc,
            command=self._ai_mcp_tool_calls_text.yview,
            bg=C["surface0"],
            troughcolor=C["mantle"],
        )
        self._ai_mcp_tool_calls_text.configure(yscrollcommand=mc_sb.set)
        self._ai_mcp_tool_calls_text.grid(row=0, column=0, sticky="nsew")
        mc_sb.grid(row=0, column=1, sticky="ns")
        tk.Label(
            inner_mcp_tc,
            text="CALL lines from the OpenAI router only (tail also in memory_context.txt + "
            "append-only mcp_tool_calls.log at workspace root)",
            font=FONT_SMALL,
            bg=C["surface0"],
            fg=C["overlay0"],
            anchor="w",
            justify="left",
        ).grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        row_mcp_btn = tk.Frame(inner_mcp_tc, bg=C["surface0"])
        row_mcp_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        make_btn(
            row_mcp_btn,
            "Clear tool invocations view",
            self._ai_clear_mcp_tool_calls,
            color="gray",
        ).pack(side="left")

        vpaned.add(outer_mcp_tc, minsize=140)

        outer_log, inner_log = make_panel(vpaned, "📜  Tool / activity log")
        inner_log.rowconfigure(0, weight=1)
        inner_log.columnconfigure(0, weight=1)
        self._ai_tool_log = tk.Text(
            inner_log,
            font=FONT_MONO,
            bg=C["mantle"],
            fg=C["text"],
            relief="flat",
            bd=4,
            wrap="word",
            state="disabled",
            height=10,
            insertbackground=C["text"],
            selectbackground=C["surface1"],
        )
        self._ai_tool_log.tag_configure("INFO", foreground=C["teal"])
        self._ai_tool_log.tag_configure("WARNING", foreground=C["yellow"])
        self._ai_tool_log.tag_configure("ERROR", foreground=C["red"])
        self._ai_tool_log.tag_configure("DEBUG", foreground=C["overlay0"])
        log_sb = tk.Scrollbar(
            inner_log,
            command=self._ai_tool_log.yview,
            bg=C["surface0"],
            troughcolor=C["mantle"],
        )
        self._ai_tool_log.configure(yscrollcommand=log_sb.set)
        self._ai_tool_log.grid(row=0, column=0, sticky="nsew")
        log_sb.grid(row=0, column=1, sticky="ns")

        row_log_btn = tk.Frame(inner_log, bg=C["surface0"])
        row_log_btn.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        make_btn(row_log_btn, "Clear tool log", self._ai_clear_tool_log, color="gray").pack(
            side="left"
        )

        vpaned.add(outer_log, minsize=160)

        paned.add(right, minsize=220)

        self._schedule_ai_assistant_sash(win, paned)

        self._ai_append_tool_log(
            "INFO",
            "Window opened — tools match mcp_server package; memory_context.txt + "
            "mcp_tool_calls.log at workspace root.",
        )
        if os.environ.get("OPENAI_API_KEY"):
            self._ai_append_chat(
                "system",
                "OpenAI + MCP — messages route to MCP tools (same stack as python -m "
                "mcp_server.mcp_client_openai). Ensure ROS + workspace are sourced for this shell.",
            )
        else:
            self._ai_append_chat(
                "system",
                "Stub mode — set OPENAI_API_KEY in workspace .env and reopen this window "
                "for live MCP tool calls.",
            )
        if self._ai_tool_list is not None and self._ai_tool_list.size() > 0:
            self._ai_tool_list.selection_set(0)
            self._ai_tool_list.activate(0)
            self._ai_on_tool_select()
        self._ai_hydrate_mcp_tool_calls_from_memory()
        self._start_ai_mcp_worker()

    def _ai_append_chat(self, role: str, text: str) -> None:
        w = self._ai_chat_text
        if w is None:
            return
        w.config(state="normal")
        if role == "user":
            w.insert("end", "You\n", ("role_label_user",))
            w.insert("end", f"{text}\n\n", ("user_body",))
        elif role == "assistant":
            w.insert("end", "Assistant\n", ("role_label_assistant",))
            w.insert("end", f"{text}\n\n", ("assistant_body",))
        else:
            w.insert("end", "System\n", ("role_label_system",))
            w.insert("end", f"{text}\n\n", ("system_body",))
        w.see("end")
        w.config(state="disabled")

    def _ai_append_mcp_tool_calls_line(self, call_line: str) -> None:
        """Append one router ``CALL …`` line with local timestamp (matches memory/audit format)."""
        w = self._ai_mcp_tool_calls_text
        if w is None:
            return
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        row = f"{ts} | {call_line.strip()}\n"
        w.config(state="normal")
        w.insert("end", row)
        w.see("end")
        w.config(state="disabled")

    def _ai_hydrate_mcp_tool_calls_from_memory(self) -> None:
        """Load saved sliding-window tool lines from memory_context.txt into the panel."""
        w = self._ai_mcp_tool_calls_text
        if w is None or MCPMemoryContext is None:
            return
        try:
            mc = MCPMemoryContext(WORKSPACE_ROOT / MCP_MEMORY_FILE_NAME)
            mc.load()
        except BaseException:
            return
        lines = mc.recent_mcp_calls
        if not lines:
            return
        w.config(state="normal")
        w.delete("1.0", "end")
        for line in lines:
            w.insert("end", f"{line.strip()}\n")
        w.see("end")
        w.config(state="disabled")

    def _ai_clear_mcp_tool_calls(self) -> None:
        """Clear the on-screen MCP tool list only (does not truncate disk logs)."""
        w = self._ai_mcp_tool_calls_text
        if w is None:
            return
        w.config(state="normal")
        w.delete("1.0", "end")
        w.config(state="disabled")
        self._ai_append_tool_log("INFO", "MCP tool invocations view cleared (files unchanged).")

    def _ai_append_tool_log(self, level: str, message: str) -> None:
        w = self._ai_tool_log
        if w is None:
            return
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] [{level}] {message}\n"
        tag = level if level in ("INFO", "WARNING", "ERROR", "DEBUG") else "DEBUG"
        w.config(state="normal")
        w.insert("end", line, (tag,))
        w.see("end")
        w.config(state="disabled")

    def _ai_clear_chat(self) -> None:
        w = self._ai_chat_text
        if w is None:
            return
        w.config(state="normal")
        w.delete("1.0", "end")
        w.config(state="disabled")
        self._ai_append_tool_log("INFO", "Chat cleared.")

    def _ai_clear_tool_log(self) -> None:
        w = self._ai_tool_log
        if w is None:
            return
        w.config(state="normal")
        w.delete("1.0", "end")
        w.config(state="disabled")

    def _ai_on_tool_select(self, _event=None) -> None:
        lb = self._ai_tool_list
        lbl = self._ai_tool_desc_lbl
        if lb is None or lbl is None:
            return
        sel = lb.curselection()
        if not sel:
            return
        idx = int(sel[0])
        if 0 <= idx < len(AI_ASSISTANT_TOOL_CATALOG):
            name, desc = AI_ASSISTANT_TOOL_CATALOG[idx]
            lbl.config(text=f"{name}\n{desc}")

    def _ai_on_send(self) -> None:
        if self._ai_input_var is None:
            return
        raw = self._ai_input_var.get().strip()
        if not raw:
            return
        self._ai_input_var.set("")
        self._ai_append_chat("user", raw)
        if self._mcp_user_q is not None:
            try:
                self._mcp_user_q.put_nowait(raw)
                self._ai_append_tool_log("INFO", "queued for OpenAI router + MCP tools")
            except BaseException as exc:
                self._ai_append_tool_log("ERROR", f"queue: {exc}")
                self._ai_append_chat("assistant", AI_ASSISTANT_GENERIC_REPLY)
        else:
            self._ai_append_chat("assistant", AI_ASSISTANT_GENERIC_REPLY)
            self._ai_append_tool_log(
                "INFO",
                "stub reply — set OPENAI_API_KEY and reopen AI assistant for MCP.",
            )
        ent = self._ai_input_entry
        if ent is not None and ent.winfo_exists():
            ent.focus_set()

    # ─────────────────────────────────────────────────────────────────────────
    # Arm Teleop window
    # ─────────────────────────────────────────────────────────────────────────

    # ─────────────────────────────────────────────────────────────────────────
    # Arm saved positions — persistence
    # ─────────────────────────────────────────────────────────────────────────

    def _load_arm_positions(self) -> None:
        try:
            if SAVED_ARM_POSITIONS_FILE.exists():
                with open(SAVED_ARM_POSITIONS_FILE) as f:
                    self._arm_positions = json.load(f)
        except Exception:
            self._arm_positions = {}

    def _save_arm_positions_file(self) -> None:
        try:
            SAVED_ARM_POSITIONS_FILE.parent.mkdir(parents=True, exist_ok=True)
            with open(SAVED_ARM_POSITIONS_FILE, "w") as f:
                json.dump(self._arm_positions, f, indent=2)
        except Exception as exc:
            self._arm_teleop_log_append(f"[WARNING] Could not save positions file: {exc}\n")

    def _load_arm_sequences(self) -> None:
        try:
            if SAVED_ARM_SEQUENCES_FILE.exists():
                with open(SAVED_ARM_SEQUENCES_FILE) as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    self._arm_saved_sequences = {str(k): list(v) for k, v in data.items()}
                else:
                    self._arm_saved_sequences = {}
        except Exception:
            self._arm_saved_sequences = {}

    def _save_arm_sequences_file(self) -> None:
        try:
            SAVED_ARM_SEQUENCES_FILE.parent.mkdir(parents=True, exist_ok=True)
            with open(SAVED_ARM_SEQUENCES_FILE, "w") as f:
                json.dump(self._arm_saved_sequences, f, indent=2)
        except Exception as exc:
            self._arm_teleop_log_append(f"[WARNING] Could not save sequences file: {exc}\n")

    @staticmethod
    def _arm_trajectory_time_yaml(seconds: float) -> str:
        """ROS trajectory ``time_from_start`` as YAML mapping (sec + nanosec)."""
        sec_i = int(seconds)
        nan = int(round((seconds - sec_i) * 1e9))
        if nan >= 1_000_000_000:
            sec_i += nan // 1_000_000_000
            nan %= 1_000_000_000
        return f"{{sec: {sec_i}, nanosec: {nan}}}"

    def _arm_move_duration_from_speed(self, speed: int) -> float:
        """Higher speed value → shorter trajectory time (snappier). speed ∈ [5,100]."""
        sp = max(5, min(100, int(speed)))
        return max(0.35, min(12.0, 100.0 / float(sp)))

    def _refresh_arm_positions_dropdown(self) -> None:
        cb = self._arm_pos_dropdown
        if cb is None:
            return
        try:
            if not cb.winfo_exists():
                return
        except tk.TclError:
            return
        names = ["Default"] + sorted(self._arm_positions.keys())
        cb["values"] = names
        cur = self._arm_pos_dropdown_var.get() if self._arm_pos_dropdown_var else ""
        if cur not in names:
            if self._arm_pos_dropdown_var:
                self._arm_pos_dropdown_var.set("Default")

    # ─────────────────────────────────────────────────────────────────────────
    # Arm Teleop — save / goto panels & workers
    # ─────────────────────────────────────────────────────────────────────────

    def _build_arm_save_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "📌  Save Current Position")
        row = tk.Frame(inner, bg=C["surface0"])
        row.pack(fill="x")
        tk.Label(row, text="Name:", font=FONT_BODY, width=6, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")
        self._arm_pos_name_var = tk.StringVar()
        entry = tk.Entry(row, textvariable=self._arm_pos_name_var,
                         font=FONT_BODY, bg=C["mantle"], fg=C["text"],
                         insertbackground=C["text"], relief="flat", bd=4, width=20)
        entry.pack(side="left", padx=(0, 6))
        entry.bind("<Return>", lambda _: self._on_save_arm_position())
        make_btn(row, "Save Position", self._on_save_arm_position,
                 color="teal").pack(side="left")
        return outer

    def _build_arm_positions_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "🎯  Saved Positions")
        row = tk.Frame(inner, bg=C["surface0"])
        row.pack(fill="x")
        tk.Label(row, text="Select:", font=FONT_BODY, width=6, anchor="w",
                 bg=C["surface0"], fg=C["subtext"]).pack(side="left")
        self._arm_pos_dropdown_var = tk.StringVar(value="Default")
        self._arm_pos_dropdown = ttk.Combobox(
            row, textvariable=self._arm_pos_dropdown_var,
            state="readonly", font=FONT_BODY, width=22,
        )
        self._arm_pos_dropdown.pack(side="left", padx=(0, 6))
        make_btn(row, "Go To Position", self._on_goto_arm_position,
                 color="green").pack(side="left", padx=(0, 6))
        make_btn(row, "Delete", self._on_delete_arm_position,
                 color="red").pack(side="left")
        self._refresh_arm_positions_dropdown()
        return outer

    def _on_save_arm_position(self) -> None:
        name = (self._arm_pos_name_var.get() if self._arm_pos_name_var else "").strip()
        if not name:
            self._arm_teleop_log_append("[WARNING] Enter a name before saving.\n")
            return
        if name.lower() == "default":
            self._arm_teleop_log_append("[WARNING] 'Default' is reserved — choose a different name.\n")
            return
        self._arm_teleop_log_append(f"[INFO] Capturing joint states for '{name}'...\n")
        threading.Thread(
            target=self._capture_arm_position_worker, args=(name,), daemon=True
        ).start()

    def _capture_arm_position_worker(self, name: str) -> None:
        env = self._arm_env or os.environ.copy()
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/joint_states"],
                capture_output=True, text=True, timeout=6, env=env,
            )
            raw = result.stdout.strip()
            if not raw:
                err = (result.stderr or "").strip()
                self.root.after(0, lambda e=err: self._arm_teleop_log_append(
                    "[ERROR] /joint_states returned no data — is arm teleop running?\n"
                    + (f"[stderr] {e}\n" if e else "")
                ))
                return
            data = _parse_joint_states_topic_echo(raw)
            if data is None:
                snippet = raw[:400].replace("\n", "↵")
                err = (result.stderr or "").strip()
                self.root.after(0, lambda s=snippet, e=err: self._arm_teleop_log_append(
                    "[ERROR] Could not parse /joint_states echo as JointState YAML.\n"
                    f"[hint] First 400 chars: {s}\n"
                    + (f"[stderr] {e}\n" if e else "")
                ))
                return
            joint_names = data.get("name", [])
            positions = data.get("position", [])
            joint_map = {n: float(p) for n, p in zip(joint_names, positions)}
        except subprocess.TimeoutExpired:
            self.root.after(0, lambda: self._arm_teleop_log_append(
                "[ERROR] Timeout reading /joint_states\n"
            ))
            return
        except Exception as exc:
            self.root.after(0, lambda e=exc: self._arm_teleop_log_append(
                f"[ERROR] Failed to read joint states: {e}\n"
            ))
            return

        saved: dict = {j: joint_map.get(j, 0.0) for j in _ARM_JOINTS}
        saved["gripper_left_joint"] = joint_map.get("gripper_left_joint", 0.0)
        self._arm_positions[name] = saved
        self._save_arm_positions_file()

        summary = "  ".join(f"{k}={v:.3f}" for k, v in saved.items())
        self.root.after(0, lambda: (
            self._refresh_arm_positions_dropdown(),
            self._arm_teleop_log_append(f"[INFO] Saved '{name}': {summary}\n"),
            self._arm_pos_name_var.set("") if self._arm_pos_name_var else None,
        ))

    def _on_goto_arm_position(self) -> None:
        name = (self._arm_pos_dropdown_var.get() if self._arm_pos_dropdown_var else "").strip()
        if not name:
            self._arm_teleop_log_append("[WARNING] Select a position first.\n")
            return
        move_dur = _ARM_GOTO_DURATION_SEC
        if self._arm_speed_var is not None:
            move_dur = self._arm_move_duration_from_speed(self._arm_speed_var.get())
        self._arm_teleop_log_append(f"[INFO] Moving to '{name}' (≈{move_dur:.2f}s)...\n")
        threading.Thread(
            target=self._goto_arm_position_worker, args=(name, move_dur), daemon=True
        ).start()

    def _arm_kill_active_goal(self) -> None:
        proc = self._arm_active_goal_proc
        self._arm_active_goal_proc = None
        _shutdown_ros_launch_process(proc)

    def _arm_resolve_pose_for_name(self, name: str) -> tuple[dict[str, float], float | None] | None:
        if name == "Default":
            joints = {j: p for j, p in zip(_ARM_JOINTS, _ARM_DEFAULT_POSITIONS)}
            return joints, None
        data = self._arm_positions.get(name)
        if data is None:
            return None
        joints = {j: float(data[j]) for j in _ARM_JOINTS}
        gripper = float(data.get("gripper_left_joint", 0.0))
        return joints, gripper

    def _arm_goto_execute_name(
        self,
        name: str,
        env: dict,
        stop_event: threading.Event | None,
        move_duration_sec: float,
    ) -> bool:
        """Send arm (+ optional gripper) to named pose. Returns False on stop/timeout/error."""
        resolved = self._arm_resolve_pose_for_name(name)
        if resolved is None:
            self.root.after(
                0, lambda n=name: self._arm_teleop_log_append(f"[ERROR] Position '{n}' not found.\n"),
            )
            return False
        joints, gripper = resolved
        positions_list = [joints[j] for j in _ARM_JOINTS]
        tfs = self._arm_trajectory_time_yaml(move_duration_sec)
        goal_yaml = (
            f"{{trajectory: {{joint_names: {_ARM_JOINTS}, "
            f"points: [{{positions: {positions_list}, "
            f"velocities: [0.0, 0.0, 0.0, 0.0], "
            f"time_from_start: {tfs}}}]}}}}"
        )
        cmd = [
            "ros2",
            "action",
            "send_goal",
            "/arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            goal_yaml,
        ]
        kwargs: dict = {
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "text": True,
            "env": env,
        }
        if sys.platform != "win32":
            kwargs["start_new_session"] = True
        proc = subprocess.Popen(cmd, **kwargs)
        self._arm_active_goal_proc = proc
        deadline = time.monotonic() + float(move_duration_sec) + 15.0
        try:
            while True:
                if stop_event and stop_event.is_set():
                    _shutdown_ros_launch_process(proc)
                    self.root.after(
                        0, lambda: self._arm_teleop_log_append("[INFO] Move cancelled (stop).\n"),
                    )
                    return False
                ret = proc.poll()
                if ret is not None:
                    out = ""
                    if proc.stdout:
                        out = proc.stdout.read() or ""
                    self.root.after(
                        0,
                        lambda o=out.strip(): self._arm_teleop_log_append(
                            f"[INFO] Arm result: {o}\n" if o else "[INFO] Arm trajectory finished.\n",
                        ),
                    )
                    if ret != 0:
                        return False
                    break
                if time.monotonic() > deadline:
                    _shutdown_ros_launch_process(proc)
                    self.root.after(
                        0, lambda: self._arm_teleop_log_append("[WARNING] Arm trajectory timed out.\n"),
                    )
                    return False
                time.sleep(0.05)
        finally:
            if self._arm_active_goal_proc is proc:
                self._arm_active_goal_proc = None

        if gripper is not None:
            gripper_yaml = f"{{command: {{position: {gripper:.4f}, max_effort: 10.0}}}}"
            gcmd = [
                "ros2",
                "action",
                "send_goal",
                "/gripper_controller/gripper_cmd",
                "control_msgs/action/GripperCommand",
                gripper_yaml,
            ]
            gkwargs: dict = {
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "text": True,
                "env": env,
            }
            if sys.platform != "win32":
                gkwargs["start_new_session"] = True
            gproc = subprocess.Popen(gcmd, **gkwargs)
            self._arm_active_goal_proc = gproc
            gdeadline = time.monotonic() + 12.0
            try:
                while True:
                    if stop_event and stop_event.is_set():
                        _shutdown_ros_launch_process(gproc)
                        return False
                    ret = gproc.poll()
                    if ret is not None:
                        self.root.after(
                            0,
                            lambda g=gripper: self._arm_teleop_log_append(f"[INFO] Gripper → {g:.4f}\n"),
                        )
                        return ret == 0
                    if time.monotonic() > gdeadline:
                        _shutdown_ros_launch_process(gproc)
                        self.root.after(
                            0, lambda: self._arm_teleop_log_append("[WARNING] Gripper timed out.\n"),
                        )
                        return False
                    time.sleep(0.05)
            finally:
                if self._arm_active_goal_proc is gproc:
                    self._arm_active_goal_proc = None
        return True

    def _goto_arm_position_worker(self, name: str, move_duration_sec: float) -> None:
        env = self._arm_env or os.environ.copy()
        self._arm_goto_execute_name(name, env, None, move_duration_sec)

    def _on_delete_arm_position(self) -> None:
        name = (self._arm_pos_dropdown_var.get() if self._arm_pos_dropdown_var else "").strip()
        if not name or name == "Default":
            self._arm_teleop_log_append("[WARNING] Cannot delete 'Default' — it is built-in.\n")
            return
        if name not in self._arm_positions:
            return
        del self._arm_positions[name]
        self._save_arm_positions_file()
        self._refresh_arm_positions_dropdown()
        self._arm_teleop_log_append(f"[INFO] Deleted position '{name}'.\n")

    # ── Arm sequence (FIFO playback, interruptible) ─────────────────────────

    def _arm_sequence_refresh_listbox(self) -> None:
        lb = self._arm_sequence_listbox
        if lb is None:
            return
        try:
            if not lb.winfo_exists():
                return
        except tk.TclError:
            return
        lb.delete(0, tk.END)
        for i, name in enumerate(self._arm_sequence, start=1):
            lb.insert(tk.END, f"{i}. {name}")

    def _on_arm_sequence_add(self) -> None:
        name = (self._arm_pos_dropdown_var.get() if self._arm_pos_dropdown_var else "").strip()
        if not name:
            self._arm_teleop_log_append("[WARNING] Select a position before adding to sequence.\n")
            return
        self._arm_sequence.append(name)
        self._arm_sequence_refresh_listbox()
        self._arm_teleop_log_append(f"[INFO] Added '{name}' to sequence (bottom = last added).\n")

    def _on_arm_sequence_drop_last(self) -> None:
        if not self._arm_sequence:
            self._arm_teleop_log_append("[INFO] Sequence is empty.\n")
            return
        removed = self._arm_sequence.pop()
        self._arm_sequence_refresh_listbox()
        self._arm_teleop_log_append(f"[INFO] Dropped last from sequence: '{removed}'.\n")

    def _on_arm_sequence_clear(self) -> None:
        self._arm_sequence.clear()
        self._arm_sequence_refresh_listbox()
        self._arm_teleop_log_append("[INFO] Sequence cleared.\n")

    def _on_arm_sequence_stop(self) -> None:
        ev = self._arm_play_stop_event
        if ev is not None:
            ev.set()
        self._arm_kill_active_goal()
        self._arm_teleop_log_append("[INFO] Stop sequence — cancelling current move if any.\n")

    def _on_arm_sequence_play(self) -> None:
        if self._arm_play_thread is not None and self._arm_play_thread.is_alive():
            self._arm_teleop_log_append("[WARNING] Sequence play is already running.\n")
            return
        if not self._arm_sequence:
            self._arm_teleop_log_append("[WARNING] Sequence is empty — add positions first.\n")
            return
        self._arm_play_stop_event = threading.Event()
        gap_sec, move_sec = self._arm_read_gap_and_move_duration_or_warn()
        if gap_sec is None or move_sec is None:
            return
        self._arm_play_thread = threading.Thread(
            target=self._arm_sequence_play_worker,
            args=(gap_sec, move_sec),
            daemon=True,
        )
        self._arm_play_thread.start()

    def _arm_read_gap_and_move_duration_or_warn(self) -> tuple[float | None, float | None]:
        """Parse pause (s) from entry and move duration from speed slider; log + return (None, None) on error."""
        if self._arm_gap_var is None or self._arm_speed_var is None:
            self._arm_teleop_log_append("[ERROR] Timing controls not initialised.\n")
            return None, None
        try:
            gap = float(self._arm_gap_var.get().strip())
        except ValueError:
            self._arm_teleop_log_append("[ERROR] Pause between steps is not a valid number.\n")
            return None, None
        if gap < 0.0 or gap > 120.0:
            self._arm_teleop_log_append("[ERROR] Pause must be between 0 and 120 seconds.\n")
            return None, None
        move_sec = self._arm_move_duration_from_speed(self._arm_speed_var.get())
        return gap, move_sec

    def _arm_sequence_play_worker(self, gap_sec: float, move_duration_sec: float) -> None:
        env = self._arm_env or os.environ.copy()
        stop_ev = self._arm_play_stop_event
        if stop_ev is None:
            return
        # FIFO: play from first added toward last (top → bottom in the list)
        order = list(self._arm_sequence)
        n_steps = len(order)
        self.root.after(
            0,
            lambda g=gap_sec, m=move_duration_sec: self._arm_teleop_log_append(
                f"[INFO] Playing sequence (FIFO, {n_steps} steps, pause={g:.2f}s, "
                f"move≈{m:.2f}s)…\n",
            ),
        )
        for idx, name in enumerate(order):
            if stop_ev.is_set():
                self.root.after(0, lambda: self._arm_teleop_log_append("[INFO] Sequence stopped.\n"))
                return
            self.root.after(
                0,
                lambda i=idx + 1, n=n_steps, nm=name: self._arm_teleop_log_append(
                    f"[INFO] Sequence step {i}/{n}: '{nm}'\n",
                ),
            )
            ok = self._arm_goto_execute_name(name, env, stop_ev, move_duration_sec)
            if not ok:
                if stop_ev.is_set():
                    self.root.after(0, lambda: self._arm_teleop_log_append("[INFO] Sequence aborted.\n"))
                else:
                    self.root.after(0, lambda: self._arm_teleop_log_append("[WARNING] Sequence halted on error.\n"))
                return
            if idx < n_steps - 1:
                end_wait = time.monotonic() + gap_sec
                while time.monotonic() < end_wait:
                    if stop_ev.is_set():
                        self.root.after(0, lambda: self._arm_teleop_log_append("[INFO] Sequence stopped during pause.\n"))
                        return
                    time.sleep(0.05)
        self.root.after(0, lambda: self._arm_teleop_log_append("[INFO] Sequence play finished.\n"))

    def _on_arm_speed_moved(self, *_args) -> None:
        if self._arm_speed_var is None or self._arm_speed_dur_lbl is None:
            return
        try:
            d = self._arm_move_duration_from_speed(self._arm_speed_var.get())
            self._arm_speed_dur_lbl.config(text=f"≈ {d:.2f} s per move")
        except tk.TclError:
            pass

    def _refresh_saved_sequences_combo(self) -> None:
        cb = self._arm_saved_seq_combo
        if cb is None:
            return
        try:
            if not cb.winfo_exists():
                return
        except tk.TclError:
            return
        names = sorted(self._arm_saved_sequences.keys())
        cb["values"] = names
        if self._arm_saved_seq_combo_var is not None:
            cur = self._arm_saved_seq_combo_var.get()
            if cur not in names:
                self._arm_saved_seq_combo_var.set(names[0] if names else "")

    def _arm_valid_pose_step_name(self, name: str) -> bool:
        return name == "Default" or name in self._arm_positions

    def _on_arm_sequence_save(self) -> None:
        key = (self._arm_seq_save_name_var.get() if self._arm_seq_save_name_var else "").strip()
        if not key:
            self._arm_teleop_log_append("[WARNING] Enter a name to save this sequence.\n")
            return
        if not self._arm_sequence:
            self._arm_teleop_log_append("[WARNING] Sequence list is empty — nothing to save.\n")
            return
        bad = [n for n in self._arm_sequence if not self._arm_valid_pose_step_name(n)]
        if bad:
            self._arm_teleop_log_append(
                f"[ERROR] Invalid step names (use Default or saved positions): {bad}\n"
            )
            return
        self._arm_saved_sequences[key] = list(self._arm_sequence)
        self._save_arm_sequences_file()
        self._refresh_saved_sequences_combo()
        self._arm_teleop_log_append(f"[INFO] Saved sequence '{key}' ({len(self._arm_sequence)} steps).\n")

    def _on_arm_sequence_load(self) -> None:
        key = (self._arm_saved_seq_combo_var.get() if self._arm_saved_seq_combo_var else "").strip()
        if not key:
            self._arm_teleop_log_append("[WARNING] Select a saved sequence to load.\n")
            return
        seq = self._arm_saved_sequences.get(key)
        if not seq:
            self._arm_teleop_log_append(f"[ERROR] Unknown sequence '{key}'.\n")
            return
        cleaned: list[str] = []
        skipped: list[str] = []
        for n in seq:
            if self._arm_valid_pose_step_name(n):
                cleaned.append(n)
            else:
                skipped.append(n)
        self._arm_sequence = cleaned
        self._arm_sequence_refresh_listbox()
        self._arm_teleop_log_append(f"[INFO] Loaded sequence '{key}' ({len(cleaned)} steps).\n")
        if skipped:
            self._arm_teleop_log_append(f"[WARNING] Skipped unknown steps: {skipped}\n")

    def _build_arm_sequence_panel(self, parent: tk.Widget) -> tk.Frame:
        outer, inner = make_panel(parent, "🔁  Position sequence (FIFO play)")
        hint = tk.Label(
            inner,
            text="Build top→bottom (FIFO). Set pause between steps and arm move speed, then Play.",
            font=FONT_SMALL,
            bg=C["surface0"],
            fg=C["overlay0"],
            wraplength=760,
            justify="left",
        )
        hint.pack(anchor="w", pady=(0, 6))

        # Timing: pause + speed slider
        self._arm_gap_var = tk.StringVar(value=str(_ARM_SEQUENCE_DELAY_SEC))
        self._arm_speed_var = tk.IntVar(value=25)

        timing = tk.Frame(inner, bg=C["surface0"])
        timing.pack(fill="x", pady=(0, 6))
        tk.Label(timing, text="Pause between steps (s):", font=FONT_BODY, bg=C["surface0"],
                 fg=C["subtext"]).pack(side="left")
        tk.Entry(
            timing, textvariable=self._arm_gap_var, font=FONT_MONO, width=8,
            bg=C["mantle"], fg=C["text"], insertbackground=C["text"], relief="flat", bd=4,
        ).pack(side="left", padx=(6, 18))

        tk.Label(timing, text="Move speed:", font=FONT_BODY, bg=C["surface0"],
                 fg=C["subtext"]).pack(side="left")
        spd = tk.Scale(
            timing,
            variable=self._arm_speed_var,
            from_=5,
            to=100,
            orient=tk.HORIZONTAL,
            resolution=1,
            length=220,
            showvalue=1,
            font=FONT_SMALL,
            bg=C["surface0"],
            fg=C["text"],
            troughcolor=C["mantle"],
            highlightthickness=0,
            command=lambda _v: self._on_arm_speed_moved(),
        )
        spd.pack(side="left", padx=(6, 6))
        self._arm_speed_dur_lbl = tk.Label(
            timing, text="", font=FONT_SMALL, bg=C["surface0"], fg=C["sky"],
        )
        self._arm_speed_dur_lbl.pack(side="left")
        self._on_arm_speed_moved()

        lb_frame = tk.Frame(inner, bg=C["surface0"])
        lb_frame.pack(fill="both", expand=True, pady=(0, 6))
        self._arm_sequence_listbox = tk.Listbox(
            lb_frame,
            font=FONT_MONO,
            height=5,
            bg=C["mantle"],
            fg=C["text"],
            selectbackground=C["surface1"],
            relief="flat",
            bd=0,
            highlightthickness=0,
        )
        self._arm_sequence_listbox.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(lb_frame, command=self._arm_sequence_listbox.yview)
        sb.pack(side="right", fill="y")
        self._arm_sequence_listbox.config(yscrollcommand=sb.set)
        self._arm_sequence_refresh_listbox()

        row1 = tk.Frame(inner, bg=C["surface0"])
        row1.pack(fill="x", pady=(0, 4))
        make_btn(row1, "Add to sequence", self._on_arm_sequence_add, color="teal").pack(
            side="left", padx=(0, 6)
        )
        make_btn(row1, "Drop last", self._on_arm_sequence_drop_last, color="orange").pack(
            side="left", padx=(0, 6)
        )
        make_btn(row1, "Clear sequence", self._on_arm_sequence_clear, color="gray").pack(side="left")

        row_save = tk.Frame(inner, bg=C["surface0"])
        row_save.pack(fill="x", pady=(0, 4))
        tk.Label(row_save, text="Seq name:", font=FONT_BODY, bg=C["surface0"],
                 fg=C["subtext"]).pack(side="left")
        self._arm_seq_save_name_var = tk.StringVar()
        tk.Entry(
            row_save, textvariable=self._arm_seq_save_name_var, font=FONT_BODY, width=16,
            bg=C["mantle"], fg=C["text"], insertbackground=C["text"], relief="flat", bd=4,
        ).pack(side="left", padx=(0, 6))
        make_btn(row_save, "Save sequence", self._on_arm_sequence_save, color="teal").pack(side="left")

        row_load = tk.Frame(inner, bg=C["surface0"])
        row_load.pack(fill="x", pady=(0, 6))
        tk.Label(row_load, text="Saved:", font=FONT_BODY, bg=C["surface0"],
                 fg=C["subtext"]).pack(side="left")
        self._arm_saved_seq_combo_var = tk.StringVar()
        self._arm_saved_seq_combo = ttk.Combobox(
            row_load,
            textvariable=self._arm_saved_seq_combo_var,
            state="readonly",
            font=FONT_BODY,
            width=22,
        )
        self._arm_saved_seq_combo.pack(side="left", padx=(0, 6))
        make_btn(row_load, "Load sequence", self._on_arm_sequence_load, color="blue").pack(side="left")
        self._refresh_saved_sequences_combo()

        row2 = tk.Frame(inner, bg=C["surface0"])
        row2.pack(fill="x")
        make_btn(row2, "▶  Play sequence", self._on_arm_sequence_play, color="green").pack(
            side="left", padx=(0, 6)
        )
        make_btn(row2, "■  Stop sequence", self._on_arm_sequence_stop, color="red").pack(side="left")
        return outer

    def _open_arm_teleop_window(self) -> None:
        """If the arm teleop window already exists, just raise it; otherwise open config dialog."""
        w = self._arm_teleop_win
        if w is not None:
            try:
                if w.winfo_exists():
                    w.lift()
                    w.focus_force()
                    return
            except tk.TclError:
                self._arm_teleop_win = None

        # ── Config dialog ────────────────────────────────────────────────────
        dlg = tk.Toplevel(self.root)
        dlg.title("Arm Teleop — Configuration")
        dlg.configure(bg=C["base"])
        dlg.resizable(False, False)
        dlg.grab_set()

        hdr = tk.Frame(dlg, bg=C["crust"])
        hdr.pack(fill="x")
        tk.Label(hdr, text="  🦾  Arm Teleop — Launch Configuration",
                 font=FONT_HEADER, bg=C["crust"], fg=C["sky"],
                 pady=10, padx=10).pack(side="left")

        body = tk.Frame(dlg, bg=C["base"], padx=20, pady=16)
        body.pack(fill="both", expand=True)

        def field_row(parent, label_text, default_val):
            row = tk.Frame(parent, bg=C["base"])
            row.pack(fill="x", pady=6)
            tk.Label(row, text=label_text, font=FONT_BODY, width=22, anchor="w",
                     bg=C["base"], fg=C["subtext"]).pack(side="left")
            var = tk.StringVar(value=default_val)
            entry = tk.Entry(row, textvariable=var, font=FONT_MONO,
                             bg=C["mantle"], fg=C["text"],
                             insertbackground=C["text"], relief="flat", bd=4, width=24)
            entry.pack(side="left")
            return var

        domain_var = field_row(body, "ROS Domain ID:", str(ROS_DOMAIN_ID))
        model_var  = field_row(body, "TURTLEBOT3_MODEL:", TURTLEBOT3_MODEL)
        rmw_var    = field_row(body, "RMW_IMPLEMENTATION:", RMW_IMPLEMENTATION)

        tk.Frame(body, bg=C["surface1"], height=1).pack(fill="x", pady=(10, 4))

        btn_row = tk.Frame(body, bg=C["base"])
        btn_row.pack(fill="x", pady=(6, 0))

        def _on_start():
            domain = domain_var.get().strip()
            model  = model_var.get().strip()
            rmw    = rmw_var.get().strip()
            dlg.destroy()
            self._build_arm_teleop_window(domain, model, rmw)

        make_btn(btn_row, "Start", _on_start, color="green").pack(side="right", padx=(6, 0))
        make_btn(btn_row, "Cancel", dlg.destroy, color="gray").pack(side="right")

        dlg.bind("<Return>", lambda _: _on_start())
        dlg.bind("<Escape>", lambda _: dlg.destroy())

        dlg.update_idletasks()
        pw, ph = self.root.winfo_x(), self.root.winfo_y()
        rw, rh = self.root.winfo_width(), self.root.winfo_height()
        dw, dh = dlg.winfo_reqwidth(), dlg.winfo_reqheight()
        dlg.geometry(f"+{pw + (rw - dw) // 2}+{ph + (rh - dh) // 2}")

    def _build_arm_teleop_window(self, domain: str, model: str, rmw: str) -> None:
        """Open the main arm teleop Toplevel window."""
        # Store env for save/goto workers
        self._arm_env = os.environ.copy()
        self._arm_env["ROS_DOMAIN_ID"]      = domain
        self._arm_env["TURTLEBOT3_MODEL"]   = model
        self._arm_env["RMW_IMPLEMENTATION"] = rmw

        self._arm_sequence = []
        self._arm_play_stop_event = None
        self._arm_play_thread = None

        win = tk.Toplevel(self.root)
        self._arm_teleop_win = win
        win.title("Arm Teleop — OpenManipulator-X")
        win.configure(bg=C["base"])
        win.geometry("880x940")
        win.minsize(700, 720)
        win.protocol("WM_DELETE_WINDOW", self._close_arm_teleop_window)

        # ── Header ──────────────────────────────────────────────────────────
        hdr = tk.Frame(win, bg=C["crust"])
        hdr.pack(fill="x")
        tk.Label(hdr, text="  🦾  Arm Teleop — OpenManipulator-X",
                 font=FONT_TITLE, bg=C["crust"], fg=C["sky"],
                 padx=10, pady=10).pack(side="left")
        tk.Label(hdr,
                 text=f"domain={domain}  model={model}  rmw={rmw}",
                 font=FONT_SMALL, bg=C["crust"], fg=C["overlay0"],
                 padx=10, pady=10).pack(side="left")

        # ── Controls ────────────────────────────────────────────────────────
        ctrl_outer, ctrl_inner = make_panel(win, "⚙   Teleop Control")
        ctrl_outer.pack(fill="x", padx=10, pady=(10, 4))

        btn_row = tk.Frame(ctrl_inner, bg=C["surface0"])
        btn_row.pack(fill="x", pady=(0, 6))
        self._btn_arm_start = make_btn(btn_row, "▶  Start Teleop",
                                       lambda: self._start_arm_teleop(domain, model, rmw),
                                       color="green")
        self._btn_arm_start.pack(side="left", padx=(0, 6))
        self._btn_arm_stop = make_btn(btn_row, "■  Stop Teleop",
                                      self._stop_arm_teleop, color="orange")
        self._btn_arm_stop.pack(side="left", padx=(0, 6))
        make_btn(btn_row, "✕  Close",
                 self._close_arm_teleop_window, color="red").pack(side="left")

        make_sep(ctrl_inner).pack(fill="x", pady=(0, 5))

        status_row = tk.Frame(ctrl_inner, bg=C["surface0"])
        status_row.pack(fill="x")
        self._arm_dot = make_dot(status_row)
        self._arm_dot.pack(side="left")
        self._arm_proc_lbl = tk.Label(status_row, text="Arm Teleop: Stopped",
                                      font=FONT_SMALL, bg=C["surface0"], fg=C["subtext"])
        self._arm_proc_lbl.pack(side="left", padx=(2, 0))

        # ── Save current position ────────────────────────────────────────────
        self._build_arm_save_panel(win).pack(fill="x", padx=10, pady=(0, 4))

        # ── Saved positions ──────────────────────────────────────────────────
        self._build_arm_positions_panel(win).pack(fill="x", padx=10, pady=(0, 4))

        # ── Sequence (FIFO play) ─────────────────────────────────────────────
        self._build_arm_sequence_panel(win).pack(fill="x", padx=10, pady=(0, 4))

        # ── Teleop log ───────────────────────────────────────────────────────
        log_outer, log_inner = make_panel(win, "📋  Teleop Log")
        log_outer.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        log_inner.rowconfigure(0, weight=1)
        log_inner.columnconfigure(0, weight=1)

        self._arm_teleop_log_text = tk.Text(
            log_inner,
            font=FONT_MONO,
            bg=C["mantle"], fg=C["text"],
            insertbackground=C["text"],
            relief="flat", bd=0,
            wrap="word",
            state="disabled",
        )
        self._arm_teleop_log_text.grid(row=0, column=0, sticky="nsew")

        sb = ttk.Scrollbar(log_inner, command=self._arm_teleop_log_text.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self._arm_teleop_log_text.config(yscrollcommand=sb.set)

        # ── Footer ──────────────────────────────────────────────────────────
        footer = tk.Frame(win, bg=C["crust"])
        footer.pack(side="bottom", fill="x")
        make_btn(footer, "Close", self._close_arm_teleop_window,
                 color="red").pack(side="right", padx=(8, 12), pady=(8, 10))

        self._arm_teleop_log_append(f"[INFO] Arm Teleop window opened "
                                    f"(domain={domain} model={model} rmw={rmw})\n")
        self._arm_teleop_log_append(
            f"[INFO] Script: {ARM_TELEOP_SCRIPT}\n"
            "[INFO] Press 'Start Teleop' to launch.\n"
        )

    def _arm_teleop_log_append(self, text: str) -> None:
        """Thread-safe append to the arm teleop log widget."""
        w = self._arm_teleop_log_text
        if w is None:
            return
        try:
            if not w.winfo_exists():
                return
        except tk.TclError:
            return
        w.config(state="normal")
        w.insert("end", text)
        w.see("end")
        w.config(state="disabled")

    def _start_arm_teleop(self, domain: str, model: str, rmw: str) -> None:
        if self._arm_teleop_proc is not None and self._arm_teleop_proc.poll() is None:
            self._arm_teleop_log_append("[WARNING] Arm teleop is already running.\n")
            return
        if not ARM_TELEOP_SCRIPT.exists():
            self._arm_teleop_log_append(
                f"[ERROR] Script not found: {ARM_TELEOP_SCRIPT}\n"
                "[ERROR] Make sure the arm workspace is in the expected location.\n"
            )
            return
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"]      = domain
        env["TURTLEBOT3_MODEL"]   = model
        env["RMW_IMPLEMENTATION"] = rmw
        try:
            kwargs: dict = {
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "env": env,
                "text": True,
                "bufsize": 1,
            }
            if sys.platform != "win32":
                kwargs["start_new_session"] = True
            self._arm_teleop_proc = subprocess.Popen(
                [str(ARM_TELEOP_SCRIPT)], **kwargs
            )
        except OSError as exc:
            self._arm_teleop_log_append(f"[ERROR] Failed to start: {exc}\n")
            return

        self._arm_teleop_log_append(
            f"[INFO] Launching: {ARM_TELEOP_SCRIPT.name}  "
            f"(PID {self._arm_teleop_proc.pid})\n"
        )
        threading.Thread(
            target=self._read_arm_teleop_output,
            daemon=True,
        ).start()

    def _read_arm_teleop_output(self) -> None:
        proc = self._arm_teleop_proc
        if proc is None or proc.stdout is None:
            return
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                self.root.after(0, lambda l=line: self._arm_teleop_log_append(l))
        except (OSError, ValueError):
            pass
        self.root.after(0, lambda: self._arm_teleop_log_append(
            "\n[INFO] Arm teleop process ended.\n"
        ))

    def _stop_arm_teleop(self) -> None:
        proc = self._arm_teleop_proc
        self._arm_teleop_proc = None
        _shutdown_ros_launch_process(proc)
        self._arm_teleop_log_append("[INFO] Stop signal sent to arm teleop.\n")

    def _close_arm_teleop_window(self) -> None:
        self._on_arm_sequence_stop()
        self._arm_play_thread = None
        self._arm_play_stop_event = None
        self._arm_sequence_listbox = None
        self._arm_gap_var = None
        self._arm_speed_var = None
        self._arm_speed_dur_lbl = None
        self._arm_seq_save_name_var = None
        self._arm_saved_seq_combo = None
        self._arm_saved_seq_combo_var = None
        self._stop_arm_teleop()
        win = self._arm_teleop_win
        self._arm_teleop_win = None
        self._arm_teleop_log_text = None
        self._arm_dot = None
        self._arm_proc_lbl = None
        if win is not None:
            try:
                win.destroy()
            except tk.TclError:
                pass

    # ─────────────────────────────────────────────────────────────────────────
    # Clean shutdown
    # ─────────────────────────────────────────────────────────────────────────

    def on_close(self) -> None:
        """WM_DELETE_WINDOW — always tear down Tk; ROS shutdown must not block the GUI thread."""
        if getattr(self, "_closing_gui", False):
            try:
                self.root.destroy()
            except tk.TclError:
                pass
            return
        self._closing_gui = True

        try:
            self._close_ai_assistant_window()
        except BaseException:
            pass

        try:
            self._close_arm_teleop_window()
        except BaseException:
            pass

        node_ref = self._ros_node
        self._ros_node = None

        try:
            try:
                self._stop_all()
            except BaseException:
                pass

            def _ros_shutdown_background() -> None:
                """Avoid deadlock: spin runs on another thread; shutdown from main can hang."""
                try:
                    if node_ref is not None:
                        node_ref.destroy_node()
                    if ROS_AVAILABLE and rclpy.ok():
                        rclpy.shutdown()
                except BaseException:
                    pass

            if ROS_AVAILABLE:
                threading.Thread(target=_ros_shutdown_background, daemon=True).start()

        finally:
            try:
                self.root.quit()
            except tk.TclError:
                pass
            try:
                self.root.destroy()
            except tk.TclError:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    root = tk.Tk()
    app  = TeachNavGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
