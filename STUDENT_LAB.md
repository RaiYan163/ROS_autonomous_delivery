# Lab: TurtleBot3, ROS 2, and the MCP bridge

This document is for **students**. Your teacher’s MCP server lets an assistant (or a small Python client) call **tools** that talk to ROS 2 on the TurtleBot simulation—publish velocity, read odometry, save a camera frame, and optionally run Nav2 goals.

**Repo layout (what matters for this lab):**

| File | Role |
|------|------|
| `mcp_server_ros2.py` | MCP server: defines all tools and the ROS node that connects to topics. |
| `mcp_client_openai.py` | Demo client: you type English; it calls the right tools. It **starts** the server for you. |
| `requirements.txt` | Python packages to install with pip (in a venv). |
| `mcp_ros2_turtlebot_notes.txt` | Extra reference (topics, troubleshooting, SLAM). |

---

## Big picture

1. **Gazebo** runs the TurtleBot3 world and the virtual robot.
2. **`mcp_client_openai.py`** starts **`mcp_server_ros2.py`** in the background and sends it MCP tool requests over stdio.
3. The **server** uses **rclpy** to publish/subscribe on the same topics as any other ROS 2 node.

You do **not** need to run `mcp_server_ros2.py` in a second terminal for this lab—the client launches it automatically.

---

## What the MCP server can do (tools)

| Tool | What it does |
|------|----------------|
| `publish_message(text)` | Publishes a string on `/chatter` (demo topic). |
| `get_status()` | Shows configured topic names and where camera snapshots are saved. |
| `get_camera_snapshot()` | Saves the latest `/camera/image_raw` frame to `temp/mcp_snapshots/*.jpg`. |
| `teleop_move(direction, duration_sec)` | Sends motion on `/cmd_vel` for a few seconds, then stops. |
| `get_odometry()` | Prints pose and twist from `/odom`. |
| `get_navigation_help()` | Short text explaining the Nav2 workflow (initial pose → goal → execute). |
| `set_navigation_initial_pose(x, y, yaw_deg)` | Publishes `/initialpose` for localization (map frame, meters, degrees). |
| `set_navigation_goal(x, y, yaw_deg)` | Stores a navigation goal (does not move until you execute). |
| `get_navigation_state()` | Shows stored initial pose and goal. |
| `execute_navigation()` | Tells Nav2 to drive to the stored goal (`NavigateToPose`). |

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
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export MCP_USE_SIM_TIME=1 && { [ ! -f .venv/bin/activate ] || . .venv/bin/activate; } && python3 mcp_client_openai.py
```

If you do **not** use a venv, use this line instead:

```bash
cd ~/Desktop/mcp_approach && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export MCP_USE_SIM_TIME=1 && python3 mcp_client_openai.py
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
| `No module named 'rclpy'` | Sourced `/opt/ros/jazzy/setup.bash` and `install/setup.bash` before `python3 mcp_client_openai.py`. |
| Robot does not move | `MCP_USE_SIM_TIME=1` with Gazebo; model matches Gazebo terminal. |
| No camera / snapshot fails | `TURTLEBOT3_MODEL=waffle_pi`; `ros2 topic hz /camera/image_raw`. |
| Nav2 / map looks wrong in sim | Launches use `use_sim_time:=true` where your teacher’s guide says so. |

For more detail, see `mcp_ros2_turtlebot_notes.txt`.
