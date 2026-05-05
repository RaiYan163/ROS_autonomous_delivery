"""MCP server for ROS 2: TurtleBot tools mirroring control_center/turtlebot_gui.py."""

from __future__ import annotations

import atexit
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_ros
try:
    from tf2_ros import TransformException  # type: ignore[attr-defined]
except ImportError:
    try:
        from tf2 import TransformException
    except ImportError:
        TransformException = Exception  # type: ignore[misc,assignment]

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from mcp.server.fastmcp import FastMCP
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def _workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.degrees(_yaw_from_quaternion(x, y, z, w))


def _image_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to BGR uint8 array for cv2.imencode."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "8UC3"):
        return data.reshape((msg.height, msg.width, 3))
    if msg.encoding == "rgb8":
        rgb = data.reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if msg.encoding in ("mono8", "8UC1"):
        gray = data.reshape((msg.height, msg.width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def _quaternion_from_yaw_rad(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def _initial_pose_covariance() -> list[float]:
    c = [0.0] * 36
    c[0] = 0.25
    c[7] = 0.25
    c[35] = 0.0685
    return c


def _env_use_sim_time() -> bool:
    raw = os.environ.get("MCP_USE_SIM_TIME", "").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return False
    if raw in ("1", "true", "yes", "on"):
        return True
    return True


def _shutdown_launch_process(proc: subprocess.Popen | None) -> None:
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
    except (ProcessLookupError, PermissionError, OSError):
        proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=12)
        return
    except subprocess.TimeoutExpired:
        pass
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


def _run_ros2_cmd(
    args: list[str],
    *,
    cwd: str,
    timeout_sec: float,
) -> tuple[int, str, str]:
    full = ["ros2", *args]
    try:
        r = subprocess.run(
            full,
            cwd=cwd,
            env=os.environ.copy(),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            errors="replace",
        )
        return r.returncode, r.stdout or "", r.stderr or ""
    except FileNotFoundError:
        return 127, "", "ros2 command not found — source ROS 2 before starting MCP server."
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or "") + (e.stderr or "")
        return -1, out, f"timeout after {timeout_sec}s"


class WaypointStore:
    """Read/write control_center/config/saved_locations.json (same file as GUI)."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        return self._path

    def read_all(self) -> dict[str, Any]:
        with self._lock:
            if not self._path.is_file():
                return {}
            try:
                with open(self._path, encoding="utf-8") as fh:
                    data = json.load(fh)
                return data if isinstance(data, dict) else {}
            except (json.JSONDecodeError, OSError):
                return {}

    def write_all(self, data: dict[str, Any]) -> None:
        with self._lock:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._path, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)


class LaunchRegistry:
    """Managed ros2 launch subprocesses (nav stack, joystick teleop)."""

    def __init__(self, workspace: Path) -> None:
        self._workspace = workspace
        self._lock = threading.Lock()
        self._procs: dict[str, subprocess.Popen] = {}

    def start(
        self,
        key: str,
        cmd: list[str],
        *,
        replace: bool = True,
    ) -> str:
        with self._lock:
            if key in self._procs and self._procs[key].poll() is None:
                if not replace:
                    return f"{key}: already running (pid={self._procs[key].pid})."
                _shutdown_launch_process(self._procs.pop(key, None))
        try:
            kwargs: dict = {
                "cwd": str(self._workspace),
                "env": os.environ.copy(),
                "stdout": subprocess.DEVNULL,
                "stderr": subprocess.DEVNULL,
            }
            if sys.platform != "win32":
                kwargs["start_new_session"] = True
            proc = subprocess.Popen(cmd, **kwargs)
        except FileNotFoundError:
            return f"{key}: failed to start — '{cmd[0]}' not found (source ROS 2?)."
        with self._lock:
            self._procs[key] = proc
        return f"{key}: started pid={proc.pid} cmd={' '.join(cmd)}"

    def stop(self, key: str) -> str:
        with self._lock:
            proc = self._procs.pop(key, None)
        if proc is None:
            return f"{key}: not running."
        _shutdown_launch_process(proc)
        return f"{key}: stopped."

    def stop_all(self) -> str:
        with self._lock:
            keys = list(self._procs.keys())
        for k in keys:
            self.stop(k)
        return "All managed launches stopped."

    def status(self) -> str:
        lines: list[str] = []
        with self._lock:
            for key, proc in self._procs.items():
                if proc.poll() is None:
                    lines.append(f"{key}: running pid={proc.pid}")
                else:
                    lines.append(f"{key}: exited code={proc.poll()}")
        return "\n".join(lines) if lines else "(no managed launches)"


class TurtleBotMcpNode(Node):
    """ROS node: publishers/subscribers + caches + managed launches + waypoints."""

    def __init__(self) -> None:
        super().__init__(
            "mcp_turtlebot_bridge",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, _env_use_sim_time()),
            ],
        )

        ws = _workspace_root()
        default_map = ws / "real_map" / "test_map.yaml"
        default_wp = ws / "control_center" / "config" / "saved_locations.json"
        wp_env = os.environ.get("MCP_WAYPOINTS_PATH", "").strip()
        waypoints_path = Path(wp_env).expanduser() if wp_env else default_wp

        self.declare_parameter("chatter_topic", "/chatter")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("camera_compressed_topic", "/image_raw/compressed")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("joy_timeout_sec", 2.0)
        self.declare_parameter("snapshot_dir", "temp/mcp_snapshots")
        self.declare_parameter("teleop_linear", 0.2)
        self.declare_parameter("teleop_angular", 0.8)
        self.declare_parameter("teleop_rate_hz", 20.0)
        self.declare_parameter("teleop_sequence_max_steps", 24)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("default_navigation_map", str(default_map))

        self._chatter_topic = str(self.get_parameter("chatter_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._camera_compressed_topic = str(
            self.get_parameter("camera_compressed_topic").value
        )
        self._amcl_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._battery_topic = str(self.get_parameter("battery_topic").value)
        self._joy_topic = str(self.get_parameter("joy_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)
        self._snapshot_dir = str(self.get_parameter("snapshot_dir").value)
        self._teleop_linear = float(self.get_parameter("teleop_linear").value)
        self._teleop_angular = float(self.get_parameter("teleop_angular").value)
        self._teleop_rate_hz = float(self.get_parameter("teleop_rate_hz").value)
        self._teleop_sequence_max_steps = int(
            self.get_parameter("teleop_sequence_max_steps").value
        )
        self._initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self._navigate_action_name = str(
            self.get_parameter("navigate_to_pose_action").value
        )
        self._default_nav_map = str(self.get_parameter("default_navigation_map").value)

        os.makedirs(self._snapshot_dir, exist_ok=True)

        self._chatter_pub = self.create_publisher(String, self._chatter_topic, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self._initial_pose_topic,
            10,
        )
        self._nav_action_client = ActionClient(
            self, NavigateToPose, self._navigate_action_name
        )

        self._waypoints = WaypointStore(waypoints_path)
        self._launches = LaunchRegistry(ws)

        self._lock = threading.Lock()
        self._latest_odom: Optional[Odometry] = None
        self._latest_image: Optional[Image] = None
        self._image_stamp_sec: float = 0.0
        self._latest_compressed: Optional[CompressedImage] = None
        self._compressed_stamp_sec: float = 0.0
        self._latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self._amcl_stamp_sec: float = 0.0
        self._latest_battery: Optional[BatteryState] = None
        self._battery_stamp_sec: float = 0.0
        self._latest_joy: Optional[Joy] = None
        self._joy_stamp_ns: int = 0
        self._latest_scan: Optional[LaserScan] = None
        self._scan_stamp_sec: float = 0.0

        self._nav_initial: Optional[Tuple[float, float, float]] = None
        self._nav_goal: Optional[Tuple[float, float, float]] = None

        self._nav_goal_handle: Any = None
        self._nav_async_result_future: Any = None
        self._nav_distance_remaining: float = -1.0

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Image, self._camera_topic, self._on_image, 10)
        self.create_subscription(
            CompressedImage,
            self._camera_compressed_topic,
            self._on_compressed,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_topic,
            self._on_amcl,
            10,
        )
        self.create_subscription(
            BatteryState,
            self._battery_topic,
            self._on_battery,
            10,
        )
        self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)

        self._stop_spin = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while rclpy.ok() and not self._stop_spin.is_set():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest_image = msg
            self._image_stamp_sec = time.time()

    def _on_compressed(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_compressed = msg
            self._compressed_stamp_sec = time.time()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._latest_amcl = msg
            self._amcl_stamp_sec = time.time()

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._latest_battery = msg
            self._battery_stamp_sec = time.time()

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._latest_joy = msg
            self._joy_stamp_ns = self.get_clock().now().nanoseconds

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._scan_stamp_sec = time.time()

    def _on_nav_feedback(self, feedback_msg: Any) -> None:
        try:
            dist = feedback_msg.feedback.distance_remaining
        except AttributeError:
            return
        with self._lock:
            self._nav_distance_remaining = float(dist)

    def shutdown(self) -> None:
        self._launches.stop_all()
        self._stop_spin.set()
        self._spin_thread.join(timeout=2.0)
        self.destroy_node()

    def publish_chatter(self, text: str) -> None:
        m = String()
        m.data = text
        self._chatter_pub.publish(m)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    def emergency_zero_cmd_vel(self, repeats: int) -> str:
        n = max(1, min(int(repeats), 50))
        period = 1.0 / max(1.0, self._teleop_rate_hz)
        for _ in range(n):
            self.publish_twist(0.0, 0.0)
            time.sleep(period)
        return f"Published {n} zero TwistStamped on {self._cmd_topic}."

    def teleop_for_duration(
        self,
        direction: str,
        duration_sec: float,
        linear_speed: float | None = None,
        angular_speed: float | None = None,
        publish_hz: float | None = None,
    ) -> str:
        direction = direction.strip().lower()
        d = max(0.05, min(float(duration_sec), 15.0))
        rate = float(self._teleop_rate_hz)
        if publish_hz is not None and math.isfinite(float(publish_hz)):
            rate = abs(float(publish_hz))
        rate = max(1.0, min(rate, 50.0))
        period = 1.0 / rate
        lin_mag = float(self._teleop_linear)
        ang_mag = float(self._teleop_angular)
        if linear_speed is not None and math.isfinite(float(linear_speed)):
            v = abs(float(linear_speed))
            if v >= 1e-9:
                lin_mag = v
        if angular_speed is not None and math.isfinite(float(angular_speed)):
            v = abs(float(angular_speed))
            if v >= 1e-9:
                ang_mag = v
        lin_mag = max(1e-4, min(lin_mag, 0.6))
        ang_mag = max(1e-4, min(ang_mag, 2.0))

        if direction in ("forward", "fwd", "f"):
            lx, az = lin_mag, 0.0
        elif direction in ("back", "backward", "rev", "b"):
            lx, az = -lin_mag, 0.0
        elif direction in ("left", "l", "turn_left"):
            lx, az = 0.0, ang_mag
        elif direction in ("right", "r", "turn_right"):
            lx, az = 0.0, -ang_mag
        elif direction in ("stop", "halt"):
            self.publish_twist(0.0, 0.0)
            return "Sent stop (zero cmd_vel)."
        else:
            return (
                f"Unknown direction '{direction}'. "
                "Use forward, back, left, right, or stop."
            )

        end = time.time() + d
        while time.time() < end:
            self.publish_twist(lx, az)
            time.sleep(period)

        self.publish_twist(0.0, 0.0)
        return (
            f"Teleop {direction} for {d:.2f}s on {self._cmd_topic} @ {rate:.1f}Hz "
            f"(linear_x={lx:.3f}, angular_z={az:.3f}), then stopped."
        )

    def teleop_sequence_run(self, steps_json: str) -> str:
        """Run several timed teleop segments in one MCP action (open-loop)."""
        try:
            data = json.loads(steps_json.strip())
        except json.JSONDecodeError as exc:
            return f"Invalid steps_json (not JSON): {exc}"
        if not isinstance(data, list):
            return "steps_json must be a JSON array of step objects"
        if len(data) < 1:
            return "steps_json array is empty"
        if len(data) > self._teleop_sequence_max_steps:
            return f"Too many steps (max {self._teleop_sequence_max_steps})"

        lines: list[str] = []
        for i, step in enumerate(data):
            if not isinstance(step, dict):
                return "\n".join(lines) + f"\nStep {i + 1} is not a JSON object."
            direction = str(step.get("direction", "")).strip()
            try:
                d = float(step.get("duration_sec", 2.0))
            except (TypeError, ValueError):
                return "\n".join(lines) + f"\nStep {i + 1}: invalid duration_sec."
            lin_raw = step.get("linear_speed")
            ang_raw = step.get("angular_speed")
            pub_raw = step.get("publish_hz")
            lin = float(lin_raw) if lin_raw is not None else None
            ang = float(ang_raw) if ang_raw is not None else None
            pub = float(pub_raw) if pub_raw is not None else None
            try:
                r = self.teleop_for_duration(direction, d, lin, ang, pub)
            except Exception as exc:  # noqa: BLE001
                return "\n".join(lines) + (
                    f"\nStep {i + 1} raised {type(exc).__name__}: {exc}"
                )
            lines.append(f"[{i + 1}/{len(data)}] {r}")
            if "unknown direction" in r.lower():
                return "\n".join(lines) + f"\nAborted at step {i + 1} (bad direction)."
        return "\n".join(lines) + f"\nSequence complete ({len(data)} segments)."

    def save_camera_snapshot(self, use_compressed: bool = False) -> str:
        with self._lock:
            raw = None if use_compressed else self._latest_image
            comp = self._latest_compressed if use_compressed else None
            stamp = self._compressed_stamp_sec if use_compressed else self._image_stamp_sec

        if use_compressed:
            if comp is None:
                return (
                    "No compressed image yet. "
                    f"Check {self._camera_compressed_topic} is publishing "
                    "(GUI uses compressed; sim may only have /camera/image_raw)."
                )
            arr = np.frombuffer(comp.data, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return f"cv2.imdecode failed (format={comp.format!r})."
            path = os.path.abspath(
                os.path.join(self._snapshot_dir, f"snapshot_{int(time.time() * 1000)}.jpg")
            )
            if not cv2.imwrite(path, bgr):
                return f"Failed to write image to {path}"
            age = time.time() - stamp
            h, w = bgr.shape[:2]
            return (
                f"Saved snapshot from COMPRESSED ({self._camera_compressed_topic}) to {path} "
                f"({w}x{h}, format={comp.format!r}, frame_age_sec={age:.2f})"
            )

        with self._lock:
            msg = self._latest_image

        if msg is None:
            return (
                "No camera frame received yet. "
                f"Ensure simulation is running and {self._camera_topic} is publishing."
            )

        try:
            bgr = _image_to_bgr(msg)
        except ValueError as e:
            return str(e)

        path = os.path.abspath(
            os.path.join(self._snapshot_dir, f"snapshot_{int(time.time() * 1000)}.jpg")
        )
        if not cv2.imwrite(path, bgr):
            return f"Failed to write image to {path}"

        age = time.time() - self._image_stamp_sec
        return (
            f"Saved snapshot to {path} "
            f"(encoding={msg.encoding}, {msg.width}x{msg.height}, "
            f"frame_age_sec={age:.2f})"
        )

    def get_odometry_summary(self) -> str:
        with self._lock:
            odom = self._latest_odom

        if odom is None:
            return (
                "No odometry received yet. "
                f"Ensure simulation is running and {self._odom_topic} is publishing."
            )

        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        yaw = _yaw_from_quaternion(o.x, o.y, o.z, o.w)
        vx = odom.twist.twist.linear.x
        wz = odom.twist.twist.angular.z

        return (
            f"frame_id={odom.header.frame_id}\n"
            f"position: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}\n"
            f"yaw_rad={yaw:.4f} ({math.degrees(yaw):.2f} deg)\n"
            f"twist: linear_x={vx:.4f}, angular_z={wz:.4f}"
        )

    def get_amcl_pose_summary(self) -> str:
        with self._lock:
            msg = self._latest_amcl
            stamp_wall = self._amcl_stamp_sec

        if msg is None:
            return (
                f"No AMCL pose on {self._amcl_topic} yet. Start Nav2 / localization."
            )
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw_deg = _yaw_deg_from_quaternion(o.x, o.y, o.z, o.w)
        cov = msg.pose.covariance
        trace_xy = cov[0] + cov[7] if len(cov) >= 36 else 0.0
        age = time.time() - stamp_wall
        return (
            f"frame_id={msg.header.frame_id}\n"
            f"position: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}\n"
            f"yaw_deg={yaw_deg:.2f}\n"
            f"cov_trace_xy≈{trace_xy:.6f}\n"
            f"cache_age_wall_sec≈{age:.2f}"
        )

    def get_battery_summary(self) -> str:
        with self._lock:
            msg = self._latest_battery
            stamp_wall = self._battery_stamp_sec

        if msg is None:
            return f"No battery message on {self._battery_topic} yet."
        pct = msg.percentage * 100.0 if msg.percentage >= 0.0 else -1.0
        age = time.time() - stamp_wall
        pct_s = f"{pct:.0f}%" if pct >= 0.0 else "n/a"
        return (
            f"percentage={pct_s}  voltage={msg.voltage:.2f}V  "
            f"present={'yes' if msg.present else 'no'}\n"
            f"cache_age_wall_sec≈{age:.2f}"
        )

    def get_joy_summary(self) -> str:
        with self._lock:
            msg = self._latest_joy
            joy_ns = self._joy_stamp_ns

        now_ns = self.get_clock().now().nanoseconds
        if joy_ns == 0 or msg is None:
            return (
                f"No joystick on {self._joy_topic} yet (or never received)."
            )
        elapsed = (now_ns - joy_ns) / 1e9
        connected = elapsed <= self._joy_timeout_sec
        linear = msg.axes[1] if len(msg.axes) > 1 else 0.0
        angular = msg.axes[0] if len(msg.axes) > 0 else 0.0
        pressed = []
        for i, val in enumerate(msg.buttons):
            if val:
                label = " (STOP)" if i == 0 else ""
                pressed.append(f"B{i}{label}")
        btn_s = "  ".join(pressed) if pressed else "None"
        stat = "connected" if connected else f"DISCONNECTED (> {self._joy_timeout_sec}s since last /joy)"
        return (
            f"status={stat}\n"
            f"axes: linear_axis[1]={linear:+.3f}  turn_axis[0]={angular:+.3f}\n"
            f"buttons: {btn_s}\n"
            f"sec_since_msg={elapsed:.2f}"
        )

    def get_lidar_scan_summary(self, sectors_deg: int, full: bool) -> str:
        with self._lock:
            scan = self._latest_scan
            stamp_wall = self._scan_stamp_sec

        if scan is None:
            return f"No LaserScan on {self._scan_topic} yet."

        sectors = max(4, min(int(sectors_deg), 72))
        n = len(scan.ranges)
        age = time.time() - stamp_wall
        lines = [
            f"frame_id={scan.header.frame_id}",
            f"angle_min_deg={math.degrees(scan.angle_min):.2f} "
            f"angle_max_deg={math.degrees(scan.angle_min + scan.angle_increment * max(0, n - 1)):.2f}",
            f"range_min={scan.range_min:.3f} range_max={scan.range_max:.3f} "
            f"count={n}",
            f"cache_age_wall_sec≈{age:.2f}",
        ]
        if full and n <= 720:
            rstr = ", ".join(f"{r:.3f}" for r in scan.ranges)
            lines.append(f"ranges[{n}]: [{rstr}]")
        elif full:
            lines.append("(full=True but scan too large; use full=False)")
        else:
            if n > 0 and scan.angle_increment != 0.0:
                sector_width = (
                    math.degrees(scan.angle_increment * n) / float(sectors)
                )
                mins: list[float] = []
                for s in range(sectors):
                    i0 = int(s * n / sectors)
                    i1 = int((s + 1) * n / sectors)
                    chunk = scan.ranges[i0:i1]
                    valid = [x for x in chunk if math.isfinite(x) and scan.range_min <= x <= scan.range_max]
                    mins.append(min(valid) if valid else float("nan"))
                ang0 = math.degrees(scan.angle_min)
                parts = []
                for s, mn in enumerate(mins):
                    a0 = ang0 + s * sector_width
                    parts.append(f"sector_{s}_{a0:.0f}-{a0 + sector_width:.0f}deg_min={mn:.3f}")
                lines.append("; ".join(parts))
        return "\n".join(lines)

    def get_tf_transform_str(self, target_frame: str, source_frame: str) -> str:
        t_tgt = target_frame.strip()
        t_src = source_frame.strip()
        if not t_tgt or not t_src:
            return "target_frame and source_frame must be non-empty."
        try:
            tfm = self._tf_buffer.lookup_transform(t_tgt, t_src, Time())
        except TransformException as e:
            return f"TF lookup {t_tgt} <- {t_src} failed: {e}"
        t = tfm.transform.translation
        r = tfm.transform.rotation
        yaw_deg = _yaw_deg_from_quaternion(r.x, r.y, r.z, r.w)
        return (
            f"{t_tgt} <- {source_frame}: "
            f"translation x={t.x:.4f} y={t.y:.4f} z={t.z:.4f}; "
            f"yaw_deg={yaw_deg:.2f}"
        )

    def publish_navigation_initial_pose(self, x: float, y: float, yaw_deg: float) -> str:
        yaw_rad = math.radians(float(yaw_deg))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _quaternion_from_yaw_rad(yaw_rad)
        msg.pose.covariance = _initial_pose_covariance()
        self._initial_pose_pub.publish(msg)
        with self._lock:
            self._nav_initial = (float(x), float(y), float(yaw_deg))
        return (
            f"Published initial pose to {self._initial_pose_topic}: "
            f"map x={x:.4f}, y={y:.4f}, yaw_deg={yaw_deg:.2f}"
        )

    def set_navigation_goal_stored(self, x: float, y: float, yaw_deg: float) -> str:
        with self._lock:
            self._nav_goal = (float(x), float(y), float(yaw_deg))
        return (
            f"Stored navigation goal: map x={x:.4f}, y={y:.4f}, yaw_deg={yaw_deg:.2f} "
            f"(call execute_navigation to drive)."
        )

    def get_navigation_state_str(self) -> str:
        with self._lock:
            ini = self._nav_initial
            goal = self._nav_goal
            h = self._nav_goal_handle
            dist = self._nav_distance_remaining
        ini_s = (
            f"initial_pose: x={ini[0]:.4f}, y={ini[1]:.4f}, yaw_deg={ini[2]:.2f}"
            if ini
            else "initial_pose: (not set)"
        )
        goal_s = (
            f"goal: x={goal[0]:.4f}, y={goal[1]:.4f}, yaw_deg={goal[2]:.2f}"
            if goal
            else "goal: (not set)"
        )
        async_s = "async_handle: active" if h is not None else "async_handle: none"
        d_s = (
            f"last_feedback_distance_m={dist:.3f}"
            if dist >= 0.0
            else "last_feedback_distance_m=(unknown)"
        )
        return (
            f"{ini_s}\n{goal_s}\n{async_s}; {d_s}\n"
            f"topics: initialpose={self._initial_pose_topic}, "
            f"action={self._navigate_action_name}"
        )

    def _clear_async_nav_state(self) -> None:
        with self._lock:
            self._nav_goal_handle = None
            self._nav_async_result_future = None
            self._nav_distance_remaining = -1.0

    def cancel_navigation_goal(self) -> str:
        with self._lock:
            h = self._nav_goal_handle
        if h is None:
            return "No active async NavigateToPose goal handle to cancel."
        h.cancel_goal_async()
        return "Cancel request sent to Nav2 (async goal). Poll wait_for_navigation_result for outcome."

    def manual_recovery_advisory(self) -> str:
        self.cancel_navigation_goal()
        return (
            "Manual recovery: cancelled any async Nav2 goal; operator should "
            "use joystick/MCP teleop_move. Blocking execute_navigation cannot "
            "be interrupted from MCP without async tools."
        )

    def get_navigation_feedback_str(self) -> str:
        with self._lock:
            h = self._nav_goal_handle
            fut = self._nav_async_result_future
            dist = self._nav_distance_remaining
        if fut is not None and fut.done():
            res = fut.result()
            self._clear_async_nav_state()
            if res is None:
                return "Async navigation ended: no result object."
            st = res.status
            ok = GoalStatus.STATUS_SUCCEEDED
            return (
                f"Async navigation finished: status={st} "
                f"({'succeeded' if st == ok else 'not succeeded'})"
            )
        if h is None:
            return (
                "No async navigation active. Start with execute_navigation_async() "
                "or use blocking execute_navigation()."
            )
        d_txt = (
            f"distance_remaining_m={dist:.3f}"
            if dist >= 0.0
            else "distance_remaining_m=(no feedback yet)"
        )
        return f"NavigateToPose async active. {d_txt}"

    def wait_for_navigation_result(self, timeout_sec: float) -> str:
        t = max(0.0, min(float(timeout_sec), 600.0))
        deadline = time.time() + t
        with self._lock:
            fut = self._nav_async_result_future

        if fut is None:
            return "No async NavigateToPose in progress."

        while rclpy.ok() and time.time() < deadline and not fut.done():
            time.sleep(0.05)

        if not fut.done():
            return (
                f"Still navigating after {t:.1f}s (timeout). "
                "Call again with larger timeout or cancel_navigation."
            )

        res = fut.result()
        self._clear_async_nav_state()
        if res is None:
            return "Navigation finished but result is None."
        st = res.status
        if st == GoalStatus.STATUS_SUCCEEDED:
            return "Navigation succeeded (async)."
        return f"Navigation finished with status={st}. See Nav2 logs."

    def execute_navigation_goal(self) -> str:
        self._clear_async_nav_state()
        with self._lock:
            g = self._nav_goal
        if g is None:
            return (
                "No navigation goal set. Use set_navigation_goal first "
                "(map x, y, yaw_deg in meters / degrees)."
            )
        if not self._nav_action_client.wait_for_server(timeout_sec=5.0):
            return f"Action server not available: {self._navigate_action_name}"

        x, y, yaw_deg = g
        yaw_rad = math.radians(yaw_deg)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = _quaternion_from_yaw_rad(yaw_rad)
        goal_msg.behavior_tree = ""

        send_future = self._nav_action_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.02)
        gh = send_future.result()
        if gh is None:
            return "send_goal failed (no handle)."
        if not gh.accepted:
            return "NavigateToPose goal rejected by Nav2."

        result_future = gh.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.02)
        res_wrap = result_future.result()
        if res_wrap is None:
            return "No result from NavigateToPose."
        status = res_wrap.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return (
                f"Navigation succeeded: map x={x:.4f}, y={y:.4f}, yaw_deg={yaw_deg:.2f}"
            )
        return f"Navigation finished with status={status}. See Nav2 logs."

    def execute_navigation_goal_async(self) -> str:
        self._clear_async_nav_state()
        with self._lock:
            g = self._nav_goal
        if g is None:
            return (
                "No navigation goal set. Use set_navigation_goal before "
                "execute_navigation_async."
            )
        if not self._nav_action_client.wait_for_server(timeout_sec=5.0):
            return f"Action server not available: {self._navigate_action_name}"

        x, y, yaw_deg = g
        yaw_rad = math.radians(yaw_deg)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = _quaternion_from_yaw_rad(yaw_rad)
        goal_msg.behavior_tree = ""

        send_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_nav_feedback,
        )
        while rclpy.ok() and not send_future.done():
            time.sleep(0.02)
        gh = send_future.result()
        if gh is None:
            return "send_goal_async failed (no handle)."
        if not gh.accepted:
            return "NavigateToPose goal rejected by Nav2."

        with self._lock:
            self._nav_goal_handle = gh
            self._nav_async_result_future = gh.get_result_async()
            self._nav_distance_remaining = -1.0

        return (
            f"NavigateToPose started (async): map x={x:.4f}, y={y:.4f}, yaw_deg={yaw_deg:.2f}. "
            "Poll get_navigation_feedback; cancel_navigation to stop; "
            "wait_for_navigation_result(timeout_sec) to block."
        )

    # --- ROS 2 CLI mirrors (same invocations as Initialisation page) ---
    def ros2_node_list(self) -> str:
        code, out, err = _run_ros2_cmd(
            ["node", "list"],
            cwd=str(_workspace_root()),
            timeout_sec=45.0,
        )
        body = out + err
        return f"[exit={code}]\n{body}" if body.strip() else f"[exit={code}] (empty)"

    def ros2_topic_list(self, filter_regex: str | None = None) -> str:
        code, out, err = _run_ros2_cmd(
            ["topic", "list"],
            cwd=str(_workspace_root()),
            timeout_sec=45.0,
        )
        topics = out.splitlines()
        filt = filter_regex.strip() if filter_regex else ""
        if filt:
            try:
                pat = re.compile(filt)
                topics = [t for t in topics if pat.search(t)]
            except re.error as e:
                return f"Invalid filter_regex: {e}"
        body = "\n".join(topics)
        return f"[filtered={filt!r} exit={code}]\n{body}"

    def ros2_topic_info(self, topic: str) -> str:
        t = topic.strip()
        if not t.startswith("/"):
            t = "/" + t
        code, out, err = _run_ros2_cmd(
            ["topic", "info", t],
            cwd=str(_workspace_root()),
            timeout_sec=30.0,
        )
        return f"[exit={code}]\n{out}\n{err}".strip()

    def ros2_topic_echo_once(self, topic: str, timeout_sec: float) -> str:
        t = topic.strip()
        if not t.startswith("/"):
            t = "/" + t
        to = max(1.0, min(float(timeout_sec), 120.0))
        code, out, err = _run_ros2_cmd(
            ["topic", "echo", t, "--once"],
            cwd=str(_workspace_root()),
            timeout_sec=to,
        )
        return f"[exit={code}]\n{out}\n{err}".strip()

    def ros2_topic_hz(self, topic: str, sample_seconds: float) -> str:
        t = topic.strip()
        if not t.startswith("/"):
            t = "/" + t
        sec = max(1.0, min(float(sample_seconds), 10.0))
        full = ["ros2", "topic", "hz", t]
        try:
            proc = subprocess.Popen(
                full,
                cwd=str(_workspace_root()),
                env=os.environ.copy(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
            )
        except FileNotFoundError:
            return "ros2 not found — source ROS 2."

        buf: list[str] = []
        start = time.time()
        stream = proc.stdout
        pattern = re.compile(r"average rate:\s*([\d.]+)")
        rate: str | None = None

        assert stream is not None
        while time.time() - start < sec:
            if proc.poll() is not None:
                break
            line = stream.readline()
            if line:
                buf.append(line)
                m = pattern.search(line)
                if m:
                    rate = m.group(1)
                    break
            else:
                time.sleep(0.02)

        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

        tail = "".join(buf[-40:])
        if rate:
            return f"average_rate_hz≈{rate} (after up to {sec:.1f}s)\n---\n{tail}"
        return (
            f"(no rate line parsed in {sec:.1f}s; process exit={proc.poll()})\n---\n{tail}"
        )

    # --- Managed launches (match turtlebot_gui nav + teleop commands) ---
    def start_navigation_launch(
        self,
        use_sim_time: bool,
        map_path: str | None,
    ) -> str:
        mp = map_path.strip() if map_path else ""
        if not mp:
            mp = self._default_nav_map
        resolved = str(Path(mp).expanduser().resolve())
        ust = "true" if use_sim_time else "false"
        cmd = [
            "ros2",
            "launch",
            "turtlebot3_navigation2",
            "navigation2.launch.py",
            f"use_sim_time:={ust}",
            f"map:={resolved}",
        ]
        return self._launches.start("nav", cmd, replace=True)

    def stop_navigation_launch(self) -> str:
        return self._launches.stop("nav")

    def start_joystick_teleop_launch(self) -> str:
        cmd = [
            "ros2",
            "launch",
            "custom_turtlebot_nodes",
            "joystick_teleop.launch.py",
        ]
        return self._launches.start("teleop", cmd, replace=True)

    def stop_joystick_teleop_launch(self) -> str:
        return self._launches.stop("teleop")

    def list_managed_launches(self) -> str:
        return self._launches.status()

    def stop_all_managed_launches(self) -> str:
        return self._launches.stop_all()

    # --- Waypoints (shared JSON) ---
    def list_saved_locations(self) -> str:
        data = self._waypoints.read_all()
        if not data:
            return f"(empty) file={self._waypoints.path}"
        return json.dumps(data, indent=2)

    def get_saved_location(self, name: str) -> str:
        data = self._waypoints.read_all()
        entry = data.get(name.strip())
        if entry is None:
            return f"Location '{name}' not found. file={self._waypoints.path}"
        return json.dumps({name.strip(): entry}, indent=2)

    def save_current_location(self, name: str) -> str:
        key = name.strip()
        if not key:
            return "Name must be non-empty."
        with self._lock:
            msg = self._latest_amcl

        if msg is None:
            return (
                f"No pose on {self._amcl_topic}; cannot save AMCL waypoint."
            )

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = _yaw_deg_from_quaternion(o.x, o.y, o.z, o.w)
        data = self._waypoints.read_all()
        data[key] = {"x": round(p.x, 4), "y": round(p.y, 4), "yaw": round(yaw, 2)}
        self._waypoints.write_all(data)
        return (
            f"Saved '{key}' x={data[key]['x']} y={data[key]['y']} "
            f"yaw={data[key]['yaw']}deg -> {self._waypoints.path}"
        )

    def update_saved_location_coords(
        self,
        name: str,
        x: float | None,
        y: float | None,
        yaw_deg: float | None,
    ) -> str:
        key = name.strip()
        data = self._waypoints.read_all()
        if key not in data:
            return f"Location '{key}' not found."
        entry = dict(data[key])
        if x is not None:
            entry["x"] = round(float(x), 4)
        if y is not None:
            entry["y"] = round(float(y), 4)
        if yaw_deg is not None:
            entry["yaw"] = round(float(yaw_deg), 2)
        data[key] = entry
        self._waypoints.write_all(data)
        return f"Updated '{key}': {json.dumps(entry)}"

    def update_saved_location_to_current(self, name: str) -> str:
        key = name.strip()
        with self._lock:
            msg = self._latest_amcl
        if msg is None:
            return "No AMCL pose to copy."
        data = self._waypoints.read_all()
        if key not in data:
            return f"Location '{key}' not found."
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = _yaw_deg_from_quaternion(o.x, o.y, o.z, o.w)
        data[key] = {"x": round(p.x, 4), "y": round(p.y, 4), "yaw": round(yaw, 2)}
        self._waypoints.write_all(data)
        return f"Updated '{key}' from AMCL pose: {data[key]}"

    def rename_saved_location(self, old_name: str, new_name: str) -> str:
        old_k = old_name.strip()
        new_k = new_name.strip()
        if not new_k:
            return "new_name must be non-empty."
        data = self._waypoints.read_all()
        if old_k not in data:
            return f"Old name '{old_k}' not found."
        if new_k in data and new_k != old_k:
            return f"'{new_k}' already exists."
        data[new_k] = data.pop(old_k)
        self._waypoints.write_all(data)
        return f"Renamed '{old_k}' -> '{new_k}'."

    def delete_saved_location(self, name: str) -> str:
        key = name.strip()
        data = self._waypoints.read_all()
        if key not in data:
            return f"Location '{key}' not found."
        del data[key]
        self._waypoints.write_all(data)
        return f"Deleted '{key}'."

    def navigate_to_saved_location(
        self,
        name: str,
        publish_initial_pose: bool,
    ) -> str:
        key = name.strip()
        data = self._waypoints.read_all()
        entry = data.get(key)
        if entry is None:
            return f"Waypoint '{key}' not found."
        try:
            x = float(entry["x"])
            y = float(entry["y"])
            yaw_deg = float(entry["yaw"])
        except (KeyError, TypeError, ValueError) as e:
            return f"Invalid waypoint payload: {e}"

        msgs: list[str] = []
        if publish_initial_pose:
            msgs.append(self.publish_navigation_initial_pose(x, y, yaw_deg))
        msgs.append(self.set_navigation_goal_stored(x, y, yaw_deg))
        msgs.append(self.execute_navigation_goal())
        return "\n".join(msgs)


_bridge: Optional[TurtleBotMcpNode] = None


def _get_bridge() -> TurtleBotMcpNode:
    global _bridge
    if _bridge is None:
        if not rclpy.ok():
            rclpy.init()
        _bridge = TurtleBotMcpNode()
    return _bridge


def _shutdown_bridge() -> None:
    global _bridge
    if _bridge is not None:
        try:
            _bridge.shutdown()
        finally:
            _bridge = None
            if rclpy.ok():
                rclpy.shutdown()


atexit.register(_shutdown_bridge)

mcp = FastMCP("ROS2 TurtleBot MCP Server")
_bridge = _get_bridge()


@mcp.tool()
def publish_message(text: str) -> str:
    """Publish std_msgs/String to the configured chatter topic (default /chatter)."""
    cleaned = text.strip()
    if not cleaned:
        return "Refused to publish: message is empty."

    _bridge.publish_chatter(cleaned)
    return f"Published to {_bridge._chatter_topic}: {cleaned}"


@mcp.tool()
def get_status() -> str:
    """Configured topics, waypoint file, snapshot dir, and default Nav2 map YAML."""
    b = _bridge
    return (
        "MCP ROS2 bridge alive.\n"
        f"chatter: {b._chatter_topic}\n"
        f"cmd_vel: {b._cmd_topic}\n"
        f"odom: {b._odom_topic}\n"
        f"camera: {b._camera_topic}\n"
        f"camera_compressed: {b._camera_compressed_topic}\n"
        f"amcl_pose: {b._amcl_topic}\n"
        f"battery: {b._battery_topic}\n"
        f"joy: {b._joy_topic}\n"
        f"scan: {b._scan_topic}\n"
        f"initial_pose: {b._initial_pose_topic}\n"
        f"navigate_to_pose: {b._navigate_action_name}\n"
        f"default_navigation_map: {b._default_nav_map}\n"
        f"waypoints_file: {b._waypoints.path}\n"
        f"snapshot_dir: {os.path.abspath(b._snapshot_dir)}"
    )


@mcp.tool()
def get_camera_snapshot(use_compressed: bool = False) -> str:
    """
    Save camera frame to JPEG (raw /camera/image_raw or compressed /image_raw/compressed).

    :param use_compressed: True = GUI-style compressed topic; False = sensor_msgs/Image
    """
    return _bridge.save_camera_snapshot(use_compressed)


@mcp.tool()
def teleop_move(
    direction: str,
    duration_sec: float,
    linear_speed: float | None = None,
    angular_speed: float | None = None,
    publish_hz: float | None = None,
) -> str:
    """
    Publish /cmd_vel TwistStamped for duration then stop.
    Directions: forward | back | left | right | stop
    Optional linear_speed (m/s) for forward/back; angular_speed (rad/s) for left/right.
    Optional publish_hz overrides cmd_vel burst rate for this move (1–50); omit for teleop_rate_hz param.
    Omitted speeds use ROS params teleop_linear / teleop_angular (defaults 0.2 / 0.8).
    """
    return _bridge.teleop_for_duration(
        direction, duration_sec, linear_speed, angular_speed, publish_hz
    )


@mcp.tool()
def teleop_sequence(steps_json: str) -> str:
    """
    Chained timed teleop: steps_json is a JSON array of objects with keys
    direction (forward|back|left|right|stop), duration_sec, and optional
    linear_speed, angular_speed, publish_hz per step (same semantics as teleop_move).
    Open-loop: geometry is approximate (odometry not closed-loop).
    """
    return _bridge.teleop_sequence_run(steps_json)


@mcp.tool()
def emergency_stop(repeats: int = 5) -> str:
    """Repeated zero TwistStamped on /cmd_vel (GUI emergency-zero equivalent)."""
    return _bridge.emergency_zero_cmd_vel(repeats)


@mcp.tool()
def get_odometry() -> str:
    """Latest /odom pose and twist summary."""
    return _bridge.get_odometry_summary()


@mcp.tool()
def get_amcl_pose() -> str:
    """Latest /amcl_pose (map localization) pose summary."""
    return _bridge.get_amcl_pose_summary()


@mcp.tool()
def get_battery_status() -> str:
    """Latest /battery_state summary."""
    return _bridge.get_battery_summary()


@mcp.tool()
def get_joy_state() -> str:
    """Latest gamepad axes/buttons plus GUI-style disconnect after joy_timeout_sec silence."""
    return _bridge.get_joy_summary()


@mcp.tool()
def get_lidar_scan(sectors_deg: int = 8, full: bool = False) -> str:
    """
    Summarize latest /scan. Default: sector-wise min ranges. full=True dumps all ranges if small enough.
    """
    return _bridge.get_lidar_scan_summary(sectors_deg, full)


@mcp.tool()
def get_tf_transform(target_frame: str, source_frame: str) -> str:
    """Lookup transform target_frame <- source_frame (TF2 buffer)."""
    return _bridge.get_tf_transform_str(target_frame, source_frame)


@mcp.tool()
def ros2_node_list() -> str:
    """Equivalent to: ros2 node list."""
    return _bridge.ros2_node_list()


@mcp.tool()
def ros2_topic_list(filter_regex: str | None = None) -> str:
    """Equivalent to: ros2 topic list, optional regex filter on topic names."""
    return _bridge.ros2_topic_list(filter_regex)


@mcp.tool()
def ros2_topic_info(topic: str) -> str:
    """Equivalent to: ros2 topic info <topic>."""
    return _bridge.ros2_topic_info(topic)


@mcp.tool()
def ros2_topic_echo_once(topic: str, timeout_sec: float = 10.0) -> str:
    """Equivalent to: ros2 topic echo <topic> --once."""
    return _bridge.ros2_topic_echo_once(topic, timeout_sec)


@mcp.tool()
def ros2_topic_hz(topic: str, sample_seconds: float = 3.0) -> str:
    """Run ros2 topic hz <topic> for up to sample_seconds (capped at 10s); parse average rate."""
    return _bridge.ros2_topic_hz(topic, sample_seconds)


@mcp.tool()
def start_navigation_launch(use_sim_time: bool = False, map_path: str | None = None) -> str:
    """
    Spawn turtlebot3_navigation2 navigation2.launch.py (managed subprocess).
    Default map matches GUI real_map/test_map.yaml when map_path omitted.
    """
    return _bridge.start_navigation_launch(use_sim_time, map_path)


@mcp.tool()
def stop_navigation_launch() -> str:
    """Stop managed Nav2 launch (SIGINT whole process group)."""
    return _bridge.stop_navigation_launch()


@mcp.tool()
def start_joystick_teleop_launch() -> str:
    """Spawn custom_turtlebot_nodes joystick_teleop.launch.py (managed)."""
    return _bridge.start_joystick_teleop_launch()


@mcp.tool()
def stop_joystick_teleop_launch() -> str:
    """Stop managed joystick teleop launch."""
    return _bridge.stop_joystick_teleop_launch()


@mcp.tool()
def list_managed_launches() -> str:
    """Status of MCP-managed ros2 launch processes (nav, teleop)."""
    return _bridge.list_managed_launches()


@mcp.tool()
def stop_all_managed_launches() -> str:
    """Kill all MCP-managed launches."""
    return _bridge.stop_all_managed_launches()


@mcp.tool()
def list_saved_locations() -> str:
    """JSON dump of saved waypoints (control_center/config/saved_locations.json path in get_status)."""
    return _bridge.list_saved_locations()


@mcp.tool()
def get_saved_location(name: str) -> str:
    """Get one saved waypoint entry by name."""
    return _bridge.get_saved_location(name)


@mcp.tool()
def save_current_location(name: str) -> str:
    """Save current /amcl_pose under name (same semantics as GUI)."""
    return _bridge.save_current_location(name)


@mcp.tool()
def update_saved_location_coords(
    name: str,
    x: float | None = None,
    y: float | None = None,
    yaw_deg: float | None = None,
) -> str:
    """Patch x and/or y and/or yaw of a saved waypoint (degrees for yaw like JSON file)."""
    return _bridge.update_saved_location_coords(name, x, y, yaw_deg)


@mcp.tool()
def update_saved_location_to_current(name: str) -> str:
    """Overwrite saved waypoint coords from latest /amcl_pose."""
    return _bridge.update_saved_location_to_current(name)


@mcp.tool()
def rename_saved_location(old_name: str, new_name: str) -> str:
    """Rename a saved waypoint key in JSON."""
    return _bridge.rename_saved_location(old_name, new_name)


@mcp.tool()
def delete_saved_location(name: str) -> str:
    """Delete saved waypoint."""
    return _bridge.delete_saved_location(name)


@mcp.tool()
def navigate_to_saved_location(
    name: str,
    publish_initial_pose: bool = False,
) -> str:
    """
    Load waypoint JSON; optionally publish /initialpose; set goal and run blocking NavigateToPose.
    """
    return _bridge.navigate_to_saved_location(name, publish_initial_pose)


@mcp.tool()
def get_navigation_help() -> str:
    """Explain Nav2 + waypoint + managed launch workflows for this MCP server."""
    return (
        "Navigation:\n"
        "1) start_navigation_launch (Nav2 stack) typically before goals.\n"
        "2) set_navigation_initial_pose(x,y,yaw_deg) or rely on RViz pose estimate.\n"
        "3) set_navigation_goal(x,y,yaw_deg) then execute_navigation (blocking)\n"
        "   OR execute_navigation_async then get_navigation_feedback / "
        "wait_for_navigation_result / cancel_navigation.\n"
        "4) navigate_to_saved_location(name) wraps JSON waypoint + NavigateToPose.\n"
        "Waypoints JSON is shared with the GUI save file (see get_status).\n"
        "Avoid teleop_move while Nav2 is driving (/cmd_vel conflict)."
    )


@mcp.tool()
def set_navigation_initial_pose(x: float, y: float, yaw_deg: float) -> str:
    """Publish /initialpose (map frame)."""
    return _bridge.publish_navigation_initial_pose(x, y, yaw_deg)


@mcp.tool()
def set_navigation_goal(x: float, y: float, yaw_deg: float = 0.0) -> str:
    """Stage Nav2 goal in map frame (does not drive until execute)."""
    return _bridge.set_navigation_goal_stored(x, y, yaw_deg)


@mcp.tool()
def get_navigation_state() -> str:
    """Stored poses, async-nav handle hints, configured topics."""
    return _bridge.get_navigation_state_str()


@mcp.tool()
def execute_navigation() -> str:
    """Blocking NavigateToPose for staged goal."""
    return _bridge.execute_navigation_goal()


@mcp.tool()
def execute_navigation_async() -> str:
    """Non-blocking NavigateToPose with feedback — use cancel / wait_for / get_navigation_feedback."""
    return _bridge.execute_navigation_goal_async()


@mcp.tool()
def cancel_navigation() -> str:
    """Cancel async NavigateToPose (from execute_navigation_async)."""
    return _bridge.cancel_navigation_goal()


@mcp.tool()
def get_navigation_feedback() -> str:
    """Distance remaining during async Nav2, or finish status if completed."""
    return _bridge.get_navigation_feedback_str()


@mcp.tool()
def wait_for_navigation_result(timeout_sec: float = 120.0) -> str:
    """Block up to timeout_sec for async NavigateToPose result."""
    return _bridge.wait_for_navigation_result(timeout_sec)


@mcp.tool()
def manual_recovery() -> str:
    """Advisory manual recovery plus cancel_navigation on async goals."""
    return _bridge.manual_recovery_advisory()


if __name__ == "__main__":
    print("Starting ROS2 MCP server on stdio transport...")
    mcp.run()
