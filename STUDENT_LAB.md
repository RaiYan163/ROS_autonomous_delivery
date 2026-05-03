# Lab: TurtleBot3, ROS 2, and the MCP bridge

This document is for **students**. Your teacher’s MCP server lets an assistant (or a small Python client) call **tools** that talk to ROS 2 on the TurtleBot simulation—publish velocity, read odometry, save a camera frame, and optionally run Nav2 goals.

**Repo layout (what matters for this lab):**

| File | Role |
|------|------|
| `mcp_server/mcp_server_ros2.py` | MCP server: defines all tools and the ROS node that connects to topics. |
| `mcp_server/mcp_client_openai.py` | Demo client: you type English; it calls the right tools. It **starts** the server for you. |
| `requirements.txt` | Python packages to install with pip (in a venv). |
| `mcp_ros2_turtlebot_notes.txt` | Extra reference (topics, troubleshooting, SLAM). |

---

## Big picture

1. **Gazebo** runs the TurtleBot3 world and the virtual robot.
2. **`mcp_server/mcp_client_openai.py`** (run as `python3 -m mcp_server.mcp_client_openai`) starts **`mcp_server/mcp_server_ros2.py`** in the background and sends it MCP tool requests over stdio.
3. The **server** uses **rclpy** to publish/subscribe on the same topics as any other ROS 2 node.

You do **not** need to run `mcp_server/mcp_server_ros2.py` in a second terminal for this lab—the client launches it automatically.

---

## What the MCP server can do (tools)

The MCP server mirrors the TurtleBot GUI: **topic snapshots**, **`ros2` CLI helpers**, **managed launches** (`ros2 launch`), **Nav2**, **waypoints JSON** (`control_center/config/saved_locations.json`), and **TF**. Unless noted, velocities use **`geometry_msgs/msg/TwistStamped`** on `/cmd_vel`.

**Core**

| Tool | What it does |
|------|----------------|
| `publish_message(text)` | Publishes a string on `/chatter`. |
| `get_status()` | Topic names, default map YAML, waypoint file path, snapshot dir. |
| `get_camera_snapshot(use_compressed=false)` | JPEG to `temp/mcp_snapshots/` from raw `/camera/image_raw` or compressed `/image_raw/compressed`. |
| `teleop_move(direction, duration_sec)` | Timed bursts on `/cmd_vel`, then stop. Directions: `forward`, `back`, `left`, `right`, `stop`. |
| `emergency_stop(repeats=5)` | Repeated zero velocity on `/cmd_vel`. |
| `get_odometry()` | Latest `/odom` pose and twist. |

**Robot state**

| Tool | What it does |
|------|----------------|
| `get_amcl_pose()` | Latest `/amcl_pose` pose (localized map frame). |
| `get_battery_status()` | Latest `/battery_state`. |
| `get_joy_state()` | Latest `/joy` axes/buttons plus GUI-style disconnect if silent >2 s. |
| `get_lidar_scan(sectors_deg=8, full=false)` | Summarize `/scan` (sectors or sparse full dump when small). |
| `get_tf_transform(target_frame, source_frame)` | TF2 lookup (`map`, `base_link`, etc.). |

**ROS CLI (same behaviour as GUI Initialisation)**

| Tool | CLI equivalent |
|------|----------------|
| `ros2_node_list()` | `ros2 node list` |
| `ros2_topic_list(filter_regex=null)` | `ros2 topic list` with optional substring filter |
| `ros2_topic_info(topic)` | `ros2 topic info <topic>` |
| `ros2_topic_echo_once(topic, timeout_sec=10)` | `ros2 topic echo <topic> --once` |
| `ros2_topic_hz(topic, sample_seconds=3)` | `ros2 topic hz <topic>` (sample window capped at 10 s). |

**Managed launches**

| Tool | Equivalent GUI action |
|------|-----------------------|
| `start_navigation_launch(use_sim_time=false, map_path=null)` | `navigation2.launch.py`; default map `real_map/test_map.yaml`. |
| `stop_navigation_launch()` | Stop Nav2 launch process group |
| `start_joystick_teleop_launch()` | `custom_turtlebot_nodes/joystick_teleop.launch.py` |
| `stop_joystick_teleop_launch()` | Stop teleop launch |
| `list_managed_launches()` | Alive / exited status |
| `stop_all_managed_launches()` | Stop every MCP-managed launch |

**Nav2**

| Tool | Notes |
|------|--------|
| `get_navigation_help()` | Workflow text (starts Nav2 launch, poses, conflicts with teleop). |
| `set_navigation_initial_pose(x, y, yaw_deg)` | `/initialpose` (map frame, degrees). |
| `set_navigation_goal(x, y, yaw_deg=0)` | Store goal — does not drive until execute. |
| `get_navigation_state()` | Stored initial pose / goal plus async-handle hints. |
| `execute_navigation()` | **Blocking** `NavigateToPose` for stored goal. |
| `execute_navigation_async()` | Non-blocking goal with feedback callbacks. |
| `get_navigation_feedback()` | Distance remaining or finished status. |
| `wait_for_navigation_result(timeout_sec=120)` | Block for async navigation result. |
| `cancel_navigation()` | Cancel current **async** goal. |
| `manual_recovery()` | Cancel async goal + advisory; use joystick if stuck. |

**Waypoints (`saved_locations.json`, shared with GUI)**

| Tool | What it does |
|------|----------------|
| `list_saved_locations()` | Full JSON snapshot. |
| `get_saved_location(name)` | One entry by name. |
| `save_current_location(name)` | Save current `/amcl_pose`. |
| `update_saved_location_coords(name, x?, y?, yaw_deg?)` | Partial edit (degrees match JSON yaw). |
| `update_saved_location_to_current(name)` | Overwrite with current `/amcl_pose`. |
| `rename_saved_location(old_name, new_name)` | Rename waypoint key. |
| `delete_saved_location(name)` | Remove entry. |
| `navigate_to_saved_location(name, publish_initial_pose=false)` | Optionally publish `/initialpose` then blocking goal + navigate. |

**Directions for `teleop_move`:** `forward`, `back`, `left`, `right`, `stop` (duration is ignored for `stop`).

**Safety / behavior notes:**

- With **Gazebo**, use `export MCP_USE_SIM_TIME=1` when running the MCP client so command timestamps match simulation time.
- Do **not** teleop with `teleop_move` while **Nav2** is actively driving—both use `/cmd_vel`.
- For **camera** in simulation, use **`export TURTLEBOT3_MODEL=waffle_pi`** (burger often has no camera in this stack).

---

## Before the lab (one-time per computer)

Your instructor may have done some of this already.

1. **ROS 2 Jazzy** is installed and you can `source /opt/ros/jazzy/setup.bash`.
2. This workspace is built from the repo root (one line; run once after cloning):

   ```bash
   cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
   ```

   After that, in **every** new terminal before ROS or MCP: `source /opt/ros/jazzy/setup.bash && source ~/Desktop/mcp_approach/install/setup.bash` (adjust the path if your folder is not on the Desktop).

3. **Python:** Use **Python 3.12** (matches Jazzy). Prefer a **virtual environment** in the project folder:

   ```bash
   cd ~/Desktop/mcp_approach && python3.12 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
   ```
4. **OpenAI (for the chat client only):** A file named `.env` in the repo root must contain `OPENAI_API_KEY=` (your key). Optional: `OPENAI_MODEL=...`
5. If imports fail, your teacher may add packages (e.g. PyYAML, numpy, opencv) to the same venv.

---

## Start here: spawn TurtleBot3, teleop, then check camera and topics

Use **three terminals** (or two if you skip keyboard teleop). Replace `~/Desktop/mcp_approach` if your clone lives elsewhere. Model **`waffle_pi`** gives **camera + lidar** in this sim stack.

### 1) Spawn TurtleBot3 in Gazebo (Terminal 1 — leave running)

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait until the robot appears in Gazebo before opening other terminals.

### 2) Teleop (Terminal 2)

**Keyboard teleop** (install once if needed: `sudo apt install -y ros-jazzy-turtlebot3-teleop`):

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 run turtlebot3_teleop teleop_keyboard
```

Use the keys shown in that terminal (`w` / `x` / `a` / `d`, etc.). **If the robot does not move**, this sim’s bridge may expect **`TwistStamped`** on `/cmd_vel` while the keyboard node publishes plain `Twist`. Then use **MCP teleop** instead: run the MCP client (Lab A Step 2) and say e.g. “drive forward for 3 seconds”, or ask your teacher for a compatible teleop node.

### 3) Check image topics, lidar, cmd_vel, odom (Terminal 3)

**List topics** related to camera, lidar, velocity, and odometry:

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic list | grep -E 'camera|image|scan|cmd_vel|odom|clock'
```

**Camera frame rate** (Ctrl+C to stop):

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic hz /camera/image_raw
```

**Who publishes/subscribes on the camera topic:**

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic info /camera/image_raw
```

**Camera info (intrinsics):**

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic info /camera/camera_info
```

**Lidar topic info:**

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic info /scan
```

**`/cmd_vel` type and endpoints:**

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic info /cmd_vel
```

**All nodes** (long list; pipe to `grep` to narrow):

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 node list
```

**Nodes with “camera” or “image” in the name:**

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 node list | grep -iE 'camera|image|bridge|gz'
```

**Optional — open the camera in a viewer** (another terminal; install if missing: `sudo apt install -y ros-jazzy-rqt-image-view`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/Desktop/mcp_approach/install/setup.bash && ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## Lab A — Gazebo + natural-language MCP (main lab)

### Step 1 — Terminal 1: start the simulation

Replace the path if your project is not on the Desktop.

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait until the robot appears in Gazebo.

### Step 2 — Terminal 2: start the MCP chat client

Use the **same** `TURTLEBOT3_MODEL` and **source ROS + install** again:

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export MCP_USE_SIM_TIME=1 && { [ ! -f .venv/bin/activate ] || . .venv/bin/activate; } && python3 -m mcp_server.mcp_client_openai
```

If you do **not** use a venv, use this line instead:

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export MCP_USE_SIM_TIME=1 && python3 -m mcp_server.mcp_client_openai
```

You should see something like **OpenAI MCP client ready**. Type at the `You:` prompt. Type `exit` or `quit` to leave.

### Step 3 — Things to try (say these in your own words)

- **Status:** Ask what topics the bridge uses → `get_status`
- **Chatter:** Ask to publish a short message to chatter → `publish_message`
- **Drive:** “Drive forward for 3 seconds”, “turn left 2 seconds”, “stop the robot” → `teleop_move`
- **Odometry:** Ask where the robot is / what the odometry says → `get_odometry`
- **Camera:** Ask for a camera snapshot → `get_camera_snapshot`; open the JPEG path it prints (under `temp/mcp_snapshots/`)

### Step 4 — Optional: confirm ROS without MCP

In a third terminal (sourced):

List topics (one line):

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic list
```

Camera rate (runs until you press Ctrl+C):

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 topic hz /camera/image_raw
```

### Step 5 — Shutdown

- In the client terminal: `exit`
- In Gazebo: Ctrl+C  
If something is stuck, your instructor may run:

```bash
cd ~/Desktop/mcp_approach && bash ./kill.sh
```

---

## Lab B — Optional: see the camera in rqt

```bash
source /opt/ros/jazzy/setup.bash && source ~/Desktop/mcp_approach/install/setup.bash && ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

If the window is blank, pick `/camera/image_raw` from the topic dropdown.

---

## Lab C — Extension: Nav2 + MCP (only if your teacher runs Nav2)

**Extra terminals:**

1. Gazebo (same as Lab A).
2. Nav2 + RViz, with **sim time** and a map file (paths may be adjusted by your teacher), for example:

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$PWD/map/turtlebot3_world_explored.yaml
```

3. MCP client (same as Lab A, with `MCP_USE_SIM_TIME=1`).

**Typical order in chat:**

1. If you need instructions: “How do I navigate?” → `get_navigation_help`
2. Set **initial pose** in the map (x, y, yaw in degrees) so localization matches where the robot sits—or use RViz **2D Pose Estimate**.
3. Set a **goal** (map x, y, yaw).
4. Say you want to **go to the goal** / **execute navigation** → `execute_navigation`
5. “Navigation status” → `get_navigation_state`

Nav2 must be running and the `NavigateToPose` action must be available before `execute_navigation` will succeed.

---

## Quick troubleshooting

| Problem | What to check |
|---------|----------------|
| `OPENAI_API_KEY is not set` | `.env` in repo root with a valid key. |
| `No module named 'rclpy'` | Sourced `/opt/ros/jazzy/setup.bash` and `install/setup.bash` before `python3 -m mcp_server.mcp_client_openai`. |
| Robot does not move | `MCP_USE_SIM_TIME=1` with Gazebo; model matches Gazebo terminal. |
| No camera / snapshot fails | `TURTLEBOT3_MODEL=waffle_pi`; `ros2 topic hz /camera/image_raw`. |
| Nav2 / map looks wrong in sim | Launches use `use_sim_time:=true` where your teacher’s guide says so. |

For more detail, see `mcp_ros2_turtlebot_notes.txt`.
