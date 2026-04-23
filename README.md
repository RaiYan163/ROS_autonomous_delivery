# Minimal MCP + ROS2 + OpenAI

This is a minimal starter setup with:

- `mcp_server_ros2.py`: MCP server exposing ROS2 publish tools
- `mcp_client_openai.py`: OpenAI-based wrapper that decides when to call MCP tools

## 1) Prerequisites

- ROS 2 installed (with `rclpy` and `std_msgs`)
- Python 3.10+

## 2) Install Python dependencies

```bash
pip install -r requirements.txt
```

## 3) Configure `.env`

```bash
cp .env .env.local 2>/dev/null || true
```

Then edit `.env` and set:

```env
OPENAI_API_KEY=your_openai_key_here
OPENAI_MODEL=gpt-4.1-mini
```

The client auto-loads `.env`, so manual `export` is optional.

## 4) Source ROS 2

```bash
source /opt/ros/<your_distro>/setup.bash
```

Example (Humble):

```bash
source /opt/ros/humble/setup.bash
```

## 5) Verify ROS topic output

In terminal A:

```bash
source /opt/ros/<your_distro>/setup.bash
ros2 topic echo /chatter
```

In terminal B:

```bash
source /opt/ros/<your_distro>/setup.bash
python mcp_client_openai.py
```

Then type:

- `publish hello from ai`
- `send this to chatter: turtlebot online`

The wrapper should call MCP tool `publish_message`, and you should see messages on `/chatter`.
