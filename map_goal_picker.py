#!/usr/bin/env python3
"""
Show a saved occupancy map (YAML + PGM) from the map/ folder. Click for map (x, y).
Press g to send a Nav2 NavigateToPose goal (yaw = 0).

Default map file: map/turtlebot3_world_explored.yaml (run from mcp_approach).

  cd /path/to/mcp_approach
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 map_goal_picker.py

  # coordinates only:
  python3 map_goal_picker.py --no-send

  # live /map topic instead of files:
  python3 map_goal_picker.py --from-topic
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional, Tuple

import numpy as np
import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _use_sim_time_from_env() -> bool:
    raw = os.environ.get("MCP_USE_SIM_TIME", "").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return False
    if raw in ("1", "true", "yes", "on"):
        return True
    return True


def _map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def load_map_from_files(yaml_path: str) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    """
    Load map_server-style YAML + PGM. Returns grayscale image [0,1] and imshow extent
    (xmin, xmax, ymin, ymax) in the map frame.
    """
    yaml_path = os.path.abspath(yaml_path)
    map_dir = os.path.dirname(yaml_path)
    with open(yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    image_name = meta["image"]
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError("map yaml must have origin: [x, y, ...]")
    ox, oy = float(origin[0]), float(origin[1])
    negate = int(meta.get("negate", 0))

    pgm_path = os.path.join(map_dir, image_name)
    if not os.path.isfile(pgm_path):
        raise FileNotFoundError(f"PGM not found: {pgm_path}")

    try:
        import cv2
    except ImportError as e:
        raise RuntimeError("OpenCV required to load PGM: pip install opencv-python-headless") from e

    gray = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if gray is None:
        raise ValueError(f"Could not read image: {pgm_path}")
    # ROS map row 0 = bottom; typical PGM row 0 = top
    gray = np.flipud(gray)
    img = gray.astype(np.float32) / 255.0
    if negate:
        img = 1.0 - img

    h, w = img.shape[:2]
    xmin, xmax = ox, ox + w * resolution
    ymin, ymax = oy, oy + h * resolution
    extent = (xmin, xmax, ymin, ymax)
    return img, extent


def _occupancy_to_image(msg: OccupancyGrid) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
    info = msg.info
    w, h = int(info.width), int(info.height)
    if w <= 0 or h <= 0 or len(msg.data) != w * h:
        raise ValueError("Invalid OccupancyGrid dimensions.")

    raw = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
    img = np.where(raw < 0, 0.5, np.clip(raw.astype(np.float32) / 100.0, 0.0, 1.0))

    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    res = float(info.resolution)
    xmin, xmax = ox, ox + w * res
    ymin, ymax = oy, oy + h * res
    extent = (xmin, xmax, ymin, ymax)
    return img, extent


class NavGoalNode(Node):
    """Minimal node: optional /map subscription, NavigateToPose client."""

    def __init__(self, *, map_topic: str, action_name: str, enable_send: bool, subscribe_map: bool) -> None:
        super().__init__(
            "map_goal_picker",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, _use_sim_time_from_env()),
            ],
        )
        self._map: Optional[OccupancyGrid] = None
        self._last_xy: Optional[Tuple[float, float]] = None
        self._enable_send = enable_send
        self._action_name = action_name

        if subscribe_map:
            self.create_subscription(OccupancyGrid, map_topic, self._on_map, _map_qos())
        self._action_client: Optional[ActionClient] = None
        if enable_send:
            self._action_client = ActionClient(self, NavigateToPose, action_name)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def wait_for_map(self, timeout_sec: float = 60.0) -> bool:
        start = self.get_clock().now()
        while rclpy.ok() and self._map is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout_sec:
                return False
        return self._map is not None

    def get_map(self) -> Optional[OccupancyGrid]:
        return self._map

    def set_last_click(self, x: float, y: float) -> None:
        self._last_xy = (float(x), float(y))

    def send_nav_goal(self) -> str:
        if not self._enable_send or self._action_client is None:
            return "Sending disabled (--no-send)."
        if self._last_xy is None:
            return "No click yet — click the map first."
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return f"Action server not available: {self._action_name}"

        x, y = self._last_xy
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        goal.behavior_tree = ""

        send_future = self._action_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        gh = send_future.result()
        if gh is None:
            return "send_goal failed (no handle)."
        if not gh.accepted:
            return "Goal rejected by Nav2."

        result_future = gh.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        res_wrap = result_future.result()
        if res_wrap is None:
            return "No result from action."
        status = res_wrap.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return f"Succeeded: goal map x={x:.3f}, y={y:.3f}, yaw=0."
        return f"Finished with status={status} (see Nav2 logs). Result: {res_wrap.result}"


def main() -> None:
    default_yaml = os.path.join(os.getcwd(), "map", "turtlebot3_world_explored.yaml")

    parser = argparse.ArgumentParser(
        description="Click map for (x,y) in map frame; g = Nav2 goal (yaw=0). "
        "Default: load map YAML/PGM from map/ directory."
    )
    parser.add_argument(
        "--map-yaml",
        default=default_yaml,
        help=f"Path to map.yaml (default: {default_yaml})",
    )
    parser.add_argument(
        "--from-topic",
        action="store_true",
        help="Use OccupancyGrid from ROS /map instead of files.",
    )
    parser.add_argument("--map-topic", default="/map", help="With --from-topic: topic name (default: /map)")
    parser.add_argument(
        "--action",
        default="/navigate_to_pose",
        help="NavigateToPose action name (default: /navigate_to_pose)",
    )
    parser.add_argument("--no-send", action="store_true", help="Only show map and coordinates; do not send goals.")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        print("Install matplotlib: pip install matplotlib", file=sys.stderr)
        raise e

    img: np.ndarray
    extent: Tuple[float, float, float, float]
    node: Optional[NavGoalNode] = None

    if args.from_topic:
        rclpy.init()
        node = NavGoalNode(
            map_topic=args.map_topic,
            action_name=args.action,
            enable_send=not args.no_send,
            subscribe_map=True,
        )
        print("Waiting for map on", args.map_topic, "...")
        if not node.wait_for_map():
            print("Timed out waiting for /map.", file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        msg = node.get_map()
        assert msg is not None
        img, extent = _occupancy_to_image(msg)
    else:
        yaml_path = os.path.abspath(args.map_yaml)
        if not os.path.isfile(yaml_path):
            print(
                f"Map file not found: {yaml_path}\n"
                f"Run from the mcp_approach directory or pass --map-yaml /full/path/to/map.yaml",
                file=sys.stderr,
            )
            sys.exit(1)
        print("Loading map from", yaml_path)
        img, extent = load_map_from_files(yaml_path)
        if not args.no_send:
            rclpy.init()
            node = NavGoalNode(
                map_topic=args.map_topic,
                action_name=args.action,
                enable_send=True,
                subscribe_map=False,
            )

    fig, ax = plt.subplots()
    ax.imshow(img, cmap="gray", origin="lower", extent=extent, interpolation="nearest", vmin=0, vmax=1)
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    src = "file" if not args.from_topic else "/map topic"
    ax.set_title(f"Map ({src}). Click = (x,y). g = Nav2 goal (yaw=0). q = quit.")
    ax.set_aspect("equal")

    status_text = fig.text(0.02, 0.02, "", transform=fig.transFigure, fontsize=9, family="monospace")

    timer = None
    if node is not None:

        def ros_spin(_evt: object) -> None:
            if plt.fignum_exists(fig.number):
                rclpy.spin_once(node, timeout_sec=0.0)

        timer = fig.canvas.new_timer(interval=50)
        timer.add_callback(ros_spin)
        timer.start()

    def on_click(event: object) -> None:
        if getattr(event, "inaxes", None) != ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        x, y = float(event.xdata), float(event.ydata)
        if node is not None:
            node.set_last_click(x, y)
        line = f"map x={x:.4f}  y={y:.4f}  (yaw fixed at 0)"
        print(line)
        status_text.set_text(line)
        fig.canvas.draw_idle()

    def on_key(event: object) -> None:
        key = getattr(event, "key", None)
        if key in ("q", "Q"):
            plt.close(fig)
            return
        if key in ("g", "G"):
            if node is None:
                print("Nav2 goal sending requires ROS (omit --no-send and use Nav2 running).")
                return
            result = node.send_nav_goal()
            print(result)
            prev = status_text.get_text()
            status_text.set_text((prev + "\n" + result) if prev else result)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)

    plt.tight_layout()
    plt.show()

    if timer is not None:
        timer.stop()
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
