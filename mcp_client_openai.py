"""OpenAI-based MCP wrapper for ROS2 TurtleBot tools."""

from __future__ import annotations

import asyncio
import os
import re
import sys
from pathlib import Path

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from openai import OpenAI
from dotenv import load_dotenv

from mcp_memory_context import FILE_NAME, MemoryContext

load_dotenv()


class MCPWrapper:
    """Routes user text through OpenAI and optionally calls MCP tools."""

    def __init__(self, session: ClientSession, memory_path: Path) -> None:
        self.session = session
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
        self.model = os.environ.get("OPENAI_MODEL", "gpt-4.1-mini")
        self.memory = MemoryContext(memory_path)

    def _llm_route(self, user_input: str) -> str:
        context_block = self.memory.build_router_context(user_input)
        prompt = f"""
You are a routing assistant for an MCP server connected to ROS2 TurtleBot simulation.

Available MCP tools:
1) publish_message(text: str) -> publishes std_msgs/String to /chatter
2) get_status() -> lists configured ROS topics
3) get_camera_snapshot() -> saves latest /camera/image_raw frame to a JPEG file, returns path
4) teleop_move(direction: str, duration_sec: float) -> publishes /cmd_vel for duration then stops.
   direction: forward | back | left | right | stop (stop ignores duration)
5) get_odometry() -> latest pose/twist from /odom
6) get_navigation_help() -> explains Nav2 workflow (initial pose, goal, execute)
7) set_navigation_initial_pose(x, y, yaw_deg) -> publish /initialpose for AMCL (map frame, meters, degrees)
8) set_navigation_goal(x, y, yaw_deg=0) -> store goal for execute_navigation (does not move until execute)
9) get_navigation_state() -> show stored initial pose and goal
10) execute_navigation() -> send NavigateToPose for stored goal (Nav2 must be running)

CRITICAL — your entire reply must be exactly ONE line and MUST start with either "CALL " or "NO TOOL NEEDED ".
No markdown. No quotes. No questions. No extra sentences.

If the user wants motion but does not name a direction, use direction=forward.
If they give a duration (e.g. "5 seconds"), use that number as duration_sec. If no duration, use duration_sec=2.0.

For navigation:
- Use CALL get_navigation_help ONLY when the user asks HOW navigation works, what steps to take, or wants
  an explanation of the workflow (e.g. "how do I navigate", "what is the order", "explain nav").
- Use CALL execute_navigation when the user wants the robot to actually drive to the stored goal NOW:
  e.g. "go to the goal", "go to target", "navigate", "start navigation", "execute", "execute navigation",
  "drive to the goal", "proceed", "move the robot to navigation", "send it", "take me there".
  Do NOT use get_navigation_help for those — they are commands to run Nav2, not requests for documentation.
- Use CALL get_navigation_state for status: "navigation status", "where is the goal", "what did I set".
If they give explicit map coordinates, fill x y yaw_deg from their message; if yaw omitted use yaw_deg=0 for goals.

Use RECENT HISTORY and ROBOT STATE below to interpret follow-ups ("again", "repeat that", "same as before",
"go to previous goal", "what happened?", "continue"). You must still output only one CALL/NO TOOL line.
Route the intent of the single user message shown under NEW USER QUERY at the bottom of the context block.

Line templates:
- CALL publish_message text=<message>
- CALL get_camera_snapshot
- CALL teleop_move direction=<forward|back|left|right|stop> duration_sec=<number>
- CALL get_odometry
- CALL get_status
- CALL get_navigation_help
- CALL get_navigation_state
- CALL execute_navigation
- CALL set_navigation_initial_pose x=<number> y=<number> yaw_deg=<number>
- CALL set_navigation_goal x=<number> y=<number> yaw_deg=<number>
- NO TOOL NEEDED <brief reply>

--- CONTEXT (sliding window + robot state) ---
{context_block}
""".strip()

        response = self.client.responses.create(
            model=self.model,
            input=prompt,
            temperature=0,
        )
        return response.output_text.strip()

    @staticmethod
    def _extract_result_text(result: object) -> str:
        if (
            hasattr(result, "structuredContent")
            and isinstance(result.structuredContent, dict)
            and "result" in result.structuredContent
        ):
            return str(result.structuredContent["result"])
        if isinstance(result, dict) and "result" in result:
            return str(result["result"])
        return str(result)

    @staticmethod
    def _parse_teleop_from_user_text(text: str) -> tuple[str, float] | None:
        """
        If the LLM returns junk, still honor clear user motion commands.
        Examples: "move the robot for 5 second", "drive forward 3s", "go left for 1.5 seconds"
        """
        s = text.lower().strip()
        dur_m = re.search(
            r"(\d+(?:\.\d+)?)\s*(?:sec|second|seconds?|s)\b",
            s,
        )
        duration = float(dur_m.group(1)) if dur_m else 2.0

        if re.search(r"\bstop\b", s):
            return "stop", duration
        if re.search(r"\b(forward|ahead|straight)\b", s):
            return "forward", duration
        if re.search(r"\b(backward|back|reverse)\b", s):
            return "back", duration
        if re.search(r"\bturn\s+left\b") or (
            re.search(r"\bleft\b", s) and not re.search(r"\bright\b", s)
        ):
            return "left", duration
        if re.search(r"\bturn\s+right\b") or re.search(r"\bright\b", s):
            return "right", duration
        if re.search(r"\b(move|drive|go|teleop)\b", s):
            return "forward", duration
        return None

    @staticmethod
    def _wants_execute_navigation(text: str) -> bool:
        """
        If the router mistakenly picks get_navigation_help, still honor clear go/execute commands.
        """
        s = text.lower().strip()
        if re.search(
            r"\b(execute\s+navigation|run\s+navigation|start\s+navigation)\b",
            s,
        ):
            return True
        if re.search(
            r"\bgo\s+to\s+(the\s+)?(goal|target|destination)\b",
            s,
        ):
            return True
        if re.search(
            r"\b(navigate\s+there|navigate\s+now|drive\s+to\s+(the\s+)?(goal|target))\b",
            s,
        ):
            return True
        if re.search(
            r"\b(proceed|take\s+me\s+there|send\s+(it|the\s+robot)|head\s+to\s+(the\s+)?(goal|target))\b",
            s,
        ):
            return True
        if re.search(
            r"\bmove\s+the\s+robot\s+to\s+navigation\b",
            s,
        ):
            return True
        if (
            "navigate" in s
            and re.search(r"\b(goal|target)\b", s)
            and not re.search(r"\bhow\s+(do|to|does)\b", s)
        ):
            return True
        return False

    async def _do_teleop(
        self, direction: str, duration_sec: float
    ) -> str:
        result = await self.session.call_tool(
            "teleop_move",
            arguments={"direction": direction, "duration_sec": duration_sec},
        )
        return self._extract_result_text(result)

    async def _execute_command(self, command: str, user_input: str) -> str:
        """Run MCP tools for a parsed CALL line (or handle NO TOOL / fallbacks)."""
        if command.startswith("CALL publish_message"):
            match = re.search(r"text=(.*)$", command)
            if not match:
                return "Routing error: no message text found."
            text = match.group(1).strip()
            result = await self.session.call_tool(
                "publish_message",
                arguments={"text": text},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL get_status"):
            result = await self.session.call_tool("get_status", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_camera_snapshot"):
            result = await self.session.call_tool("get_camera_snapshot", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_odometry"):
            result = await self.session.call_tool("get_odometry", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_navigation_help"):
            result = await self.session.call_tool("get_navigation_help", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_navigation_state"):
            result = await self.session.call_tool("get_navigation_state", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL execute_navigation"):
            result = await self.session.call_tool("execute_navigation", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL set_navigation_initial_pose"):
            xm = re.search(r"x=([\d.-]+)", command)
            ym = re.search(r"y=([\d.-]+)", command)
            zm = re.search(r"yaw_deg=([\d.-]+)", command)
            if not xm or not ym or not zm:
                return (
                    "Routing error: set_navigation_initial_pose needs x= y= yaw_deg= "
                    "(all numbers, map frame, yaw in degrees)."
                )
            result = await self.session.call_tool(
                "set_navigation_initial_pose",
                arguments={
                    "x": float(xm.group(1)),
                    "y": float(ym.group(1)),
                    "yaw_deg": float(zm.group(1)),
                },
            )
            return self._extract_result_text(result)

        if command.startswith("CALL set_navigation_goal"):
            xm = re.search(r"x=([\d.-]+)", command)
            ym = re.search(r"y=([\d.-]+)", command)
            zm = re.search(r"yaw_deg=([\d.-]+)", command)
            if not xm or not ym:
                return "Routing error: set_navigation_goal needs x= y= [yaw_deg=] (numbers)."
            yaw = float(zm.group(1)) if zm else 0.0
            result = await self.session.call_tool(
                "set_navigation_goal",
                arguments={
                    "x": float(xm.group(1)),
                    "y": float(ym.group(1)),
                    "yaw_deg": yaw,
                },
            )
            return self._extract_result_text(result)

        if command.startswith("CALL teleop_move"):
            d_match = re.search(r"direction=(\S+)", command)
            t_match = re.search(r"duration_sec=([\d.]+)", command)
            if not d_match:
                parsed = self._parse_teleop_from_user_text(user_input)
                if parsed:
                    direction, duration = parsed
                    return await self._do_teleop(direction, duration)
                return "Routing error: teleop_move needs direction=..."
            direction = d_match.group(1).strip().lower()
            duration = float(t_match.group(1)) if t_match else 2.0
            return await self._do_teleop(direction, duration)

        if command.startswith("NO TOOL NEEDED"):
            return command.replace("NO TOOL NEEDED", "", 1).strip()

        parsed = self._parse_teleop_from_user_text(user_input)
        if parsed:
            direction, duration = parsed
            return await self._do_teleop(direction, duration)

        return f"Unrecognized router output: {command}"

    async def handle(self, user_input: str) -> str:
        self.memory.load()
        command = self._llm_route(user_input)
        if command.startswith("CALL get_navigation_help") and self._wants_execute_navigation(
            user_input
        ):
            command = "CALL execute_navigation"

        reply = await self._execute_command(command, user_input)

        self.memory.append_exchange(user_input, reply)
        self.memory.update_robot_state(command, reply)
        self.memory.save()
        return reply


def _project_root() -> Path:
    return Path(__file__).resolve().parent


async def chat_loop() -> None:
    root = _project_root()
    server_py = root / "mcp_server_ros2.py"
    if not server_py.is_file():
        raise RuntimeError(f"MCP server not found at {server_py}")

    memory_path = root / FILE_NAME
    server_params = StdioServerParameters(
        command=sys.executable,
        args=[str(server_py)],
        env=os.environ.copy(),
        cwd=str(root),
    )

    async with stdio_client(server_params) as (read_stream, write_stream):
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()
            wrapper = MCPWrapper(session, memory_path)

            print("OpenAI MCP client ready. Type 'exit' to quit.")
            print("Try: drive forward for 3 seconds | turn left 2s | stop the robot")
            print(f"Sliding-window memory + robot state: {memory_path}")
            while True:
                user_input = input("You: ").strip()
                if user_input.lower() in {"exit", "quit"}:
                    break
                if not user_input:
                    continue
                reply = await wrapper.handle(user_input)
                print(f"Assistant: {reply}")


if __name__ == "__main__":
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is not set.")
    asyncio.run(chat_loop())
