"""MCP server for ROS 2: chatter demo, TurtleBot camera snapshot, teleop, odometry."""

from __future__ import annotations

import atexit
import math
import os
import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
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
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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
    """Diagonal uncertainty for x, y, yaw (map frame)."""
    c = [0.0] * 36
    c[0] = 0.25
    c[7] = 0.25
    c[35] = 0.0685
    return c


def _env_use_sim_time() -> bool:
    """
    Default True so cmd_vel stamps follow Gazebo /clock (use_sim_time).

    Disable for a real robot or wall-clock-only runs: MCP_USE_SIM_TIME=0
    """
    raw = os.environ.get("MCP_USE_SIM_TIME", "").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return False
    if raw in ("1", "true", "yes", "on"):
        return True
    return True


class TurtleBotMcpNode(Node):
    """ROS node: publishers/subscribers + thread-safe caches for MCP tools."""

    def __init__(self) -> None:
        super().__init__(
            "mcp_turtlebot_bridge",
            parameter_overrides=[
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    _env_use_sim_time(),
                ),
            ],
        )

        self.declare_parameter("chatter_topic", "/chatter")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("snapshot_dir", "temp/mcp_snapshots")
        self.declare_parameter("teleop_linear", 0.2)
        self.declare_parameter("teleop_angular", 0.8)
        self.declare_parameter("teleop_rate_hz", 20.0)
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")

        self._chatter_topic = str(self.get_parameter("chatter_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._camera_topic = str(self.get_parameter("camera_topic").value)
        self._snapshot_dir = str(self.get_parameter("snapshot_dir").value)
        self._teleop_linear = float(self.get_parameter("teleop_linear").value)
        self._teleop_angular = float(self.get_parameter("teleop_angular").value)
        self._teleop_rate_hz = float(self.get_parameter("teleop_rate_hz").value)
        self._initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self._navigate_action_name = str(self.get_parameter("navigate_to_pose_action").value)

        os.makedirs(self._snapshot_dir, exist_ok=True)

        self._chatter_pub = self.create_publisher(String, self._chatter_topic, 10)
        # turtlebot3_gazebo bridge is configured as geometry_msgs/msg/TwistStamped on ROS side.
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self._initial_pose_topic,
            10,
        )
        self._nav_action_client = ActionClient(self, NavigateToPose, self._navigate_action_name)

        self._lock = threading.Lock()
        self._latest_odom: Optional[Odometry] = None
        self._latest_image: Optional[Image] = None
        self._image_stamp_sec: float = 0.0
        self._nav_initial: Optional[Tuple[float, float, float]] = None
        self._nav_goal: Optional[Tuple[float, float, float]] = None

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Image, self._camera_topic, self._on_image, 10)

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

    def shutdown(self) -> None:
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

    def teleop_for_duration(self, direction: str, duration_sec: float) -> str:
        direction = direction.strip().lower()
        d = max(0.05, min(float(duration_sec), 15.0))
        lin = self._teleop_linear
        ang = self._teleop_angular

        if direction in ("forward", "fwd", "f"):
            lx, az = lin, 0.0
        elif direction in ("back", "backward", "rev", "b"):
            lx, az = -lin, 0.0
        elif direction in ("left", "l", "turn_left"):
            lx, az = 0.0, ang
        elif direction in ("right", "r", "turn_right"):
            lx, az = 0.0, -ang
        elif direction in ("stop", "halt"):
            self.publish_twist(0.0, 0.0)
            return "Sent stop (zero cmd_vel)."
        else:
            return (
                f"Unknown direction '{direction}'. "
                "Use forward, back, left, right, or stop."
            )

        period = 1.0 / max(1.0, self._teleop_rate_hz)
        end = time.time() + d
        while time.time() < end:
            self.publish_twist(lx, az)
            time.sleep(period)

        self.publish_twist(0.0, 0.0)
        return (
            f"Teleop {direction} for {d:.2f}s on {self._cmd_topic} "
            f"(linear_x={lx:.3f}, angular_z={az:.3f}), then stopped."
        )

    def save_camera_snapshot(self) -> str:
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

    def publish_navigation_initial_pose(self, x: float, y: float, yaw_deg: float) -> str:
        """Publish AMCL initial pose on /initialpose (map frame)."""
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
        """Store goal for execute_navigation (does not drive yet)."""
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
        return (
            f"{ini_s}\n{goal_s}\n"
            f"topics: initialpose={self._initial_pose_topic}, "
            f"action={self._navigate_action_name}"
        )

    def execute_navigation_goal(self) -> str:
        """Send NavigateToPose using stored goal (policy: does not re-publish initial pose)."""
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
    """
    Publish a text message to ROS 2 topic /chatter.

    :param text: Message payload for std_msgs/String
    :return: Confirmation text
    """
    cleaned = text.strip()
    if not cleaned:
        return "Refused to publish: message is empty."

    _bridge.publish_chatter(cleaned)
    return f"Published to {_bridge._chatter_topic}: {cleaned}"


@mcp.tool()
def get_status() -> str:
    """Return basic server and configured ROS topic names."""
    b = _bridge
    return (
        "MCP ROS2 bridge alive.\n"
        f"chatter: {b._chatter_topic}\n"
        f"cmd_vel: {b._cmd_topic}\n"
        f"odom: {b._odom_topic}\n"
        f"camera: {b._camera_topic}\n"
        f"initial_pose: {b._initial_pose_topic}\n"
        f"navigate_to_pose: {b._navigate_action_name}\n"
        f"snapshot_dir: {os.path.abspath(b._snapshot_dir)}"
    )


@mcp.tool()
def get_camera_snapshot() -> str:
    """
    Save the latest camera frame from /camera/image_raw to a JPEG file.

    :return: Path to saved image or error message
    """
    return _bridge.save_camera_snapshot()


@mcp.tool()
def teleop_move(direction: str, duration_sec: float) -> str:
    """
    Publish /cmd_vel for a short time (forward/back/left/right) then stop.

    :param direction: forward | back | left | right | stop
    :param duration_sec: How long to command motion (clamped ~0.05–15s; ignored for stop)
    :return: Status message
    """
    return _bridge.teleop_for_duration(direction, duration_sec)


@mcp.tool()
def get_odometry() -> str:
    """
    Return latest pose and twist from /odom.

    :return: Human-readable odometry summary
    """
    return _bridge.get_odometry_summary()


@mcp.tool()
def get_navigation_help() -> str:
    """
    Nav2 workflow for this MCP server (read before navigating).

    Order: (1) Set initial pose for AMCL (map frame). (2) Set navigation goal (map x, y, yaw_deg).
    (3) Call execute_navigation to send NavigateToPose. You can update initial pose or goal anytime.
    Requires Nav2 running. Goals use the map frame (meters, yaw in degrees). Do not use teleop
    while Nav2 is actively driving.
    """
    return (
        "Navigation workflow:\n"
        "1) set_navigation_initial_pose(x, y, yaw_deg) — publishes /initialpose (AMCL).\n"
        "2) set_navigation_goal(x, y, yaw_deg) — stores goal; yaw_deg defaults to 0 if omitted in API.\n"
        "3) execute_navigation() — sends NavigateToPose for the stored goal only.\n"
        "Use get_navigation_state() to see what is stored. Nav2 and localization must be running."
    )


@mcp.tool()
def set_navigation_initial_pose(x: float, y: float, yaw_deg: float) -> str:
    """
    Publish initial pose estimate to /initialpose (geometry_msgs/PoseWithCovarianceStamped, map frame).

    :param x: Map x (meters)
    :param y: Map y (meters)
    :param yaw_deg: Heading in degrees (map frame)
    """
    return _bridge.publish_navigation_initial_pose(x, y, yaw_deg)


@mcp.tool()
def set_navigation_goal(x: float, y: float, yaw_deg: float = 0.0) -> str:
    """
    Store a Nav2 goal pose in the map frame (does not move the robot until execute_navigation).

    :param x: Map x (meters)
    :param y: Map y (meters)
    :param yaw_deg: Goal heading in degrees (default 0)
    """
    return _bridge.set_navigation_goal_stored(x, y, yaw_deg)


@mcp.tool()
def get_navigation_state() -> str:
    """Return stored initial pose and goal, or note if unset."""
    return _bridge.get_navigation_state_str()


@mcp.tool()
def execute_navigation() -> str:
    """
    Send NavigateToPose action for the stored goal (set_navigation_goal). Does not re-publish
    initial pose; localize first (RViz or set_navigation_initial_pose).
    """
    return _bridge.execute_navigation_goal()


if __name__ == "__main__":
    print("Starting ROS2 MCP server on stdio transport...")
    mcp.run()
