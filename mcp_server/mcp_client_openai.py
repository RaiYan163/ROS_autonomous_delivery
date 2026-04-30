"""OpenAI-based MCP wrapper for ROS2 TurtleBot tools."""

from __future__ import annotations

import asyncio
import os
import re
import sys
from pathlib import Path

from dotenv import load_dotenv
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from openai import OpenAI

from .mcp_memory_context import FILE_NAME, MemoryContext

load_dotenv()


def workspace_root() -> Path:
    """Repository root (parent of the ``mcp_server`` package)."""
    return Path(__file__).resolve().parent.parent


def package_dir() -> Path:
    return Path(__file__).resolve().parent


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
You are a routing assistant for an MCP server connected to a ROS 2 robot (simulation or hardware)..

AVAILABLE MCP TOOLS (pick one CALL line per reply):

Core / demo:
- publish_message(text)
- get_status()
- get_camera_snapshot(use_compressed=false)  (/camera/image_raw or /image_raw/compressed JPEG)
- teleop_move(direction, duration_sec)
- emergency_stop(repeats=5)
Sensing:
- get_odometry(); get_amcl_pose(); get_battery_status(); get_joy_state(); get_lidar_scan(sectors_deg=8, full=false)
- get_tf_transform(target_frame=map, source_frame=base_link)  adapt frames to user wording
CLI mirrors (same as GUI Initialisation ros2 terminals):
- ros2_node_list(); ros2_topic_list(filter_regex omitted or filter_regex=substring_regex)
- ros2_topic_info(topic=/topic); ros2_topic_echo_once(topic=/topic, timeout_sec=10)
- ros2_topic_hz(topic=/topic, sample_seconds=3)
Managed launches (subprocess ros2 launch, like GUI buttons):
- start_navigation_launch(use_sim_time=false, map_path omit for default map)
- stop_navigation_launch(); start_joystick_teleop_launch(); stop_joystick_teleop_launch()
- list_managed_launches(); stop_all_managed_launches()
Nav2 pose / goal:
- get_navigation_help(); get_navigation_state()
- set_navigation_initial_pose(x, y, yaw_deg); set_navigation_goal(x, y, yaw_deg=0)
- execute_navigation()  blocking NavigateToPose
- execute_navigation_async(); get_navigation_feedback(); wait_for_navigation_result(timeout_sec=120); cancel_navigation()
- manual_recovery()
Waypoints (same JSON as GUI control_center/config/saved_locations.json):
- list_saved_locations(); get_saved_location(name)
- save_current_location(name); update_saved_location_to_current(name)
- update_saved_location_coords(name, x=…, y=…, yaw_deg=…)  omit keys not changed
- rename_saved_location(old_name, new_name); delete_saved_location(name)
- navigate_to_saved_location(name, publish_initial_pose=false)

CRITICAL — your entire reply must be exactly ONE line and MUST start with either "CALL " or "NO TOOL NEEDED ".
No markdown. No quotes. No questions. No extra sentences.

If the user wants motion but does not name a direction, use direction=forward.
If they give a duration (e.g. "5 seconds"), use that number as duration_sec. If no duration, use duration_sec=2.0.

Navigation routing:
- CALL get_navigation_help ONLY for questions about HOW to navigate (workflow explanation).
- CALL execute_navigation when they want to drive to the STORED goal NOW (blocking).
- CALL execute_navigation_async + later wait_for_navigation_result / get_navigation_feedback / cancel_navigation for non-blocking.
- CALL get_navigation_state for "navigation status", "what goal is set".
- For named places from memory or waypoints JSON, CALL navigate_to_saved_location name=<name>.
If coordinates are explicit, CALL set_navigation_goal x= y= yaw_deg= (default yaw 0) then execute_navigation unless they asked async.

Use RECENT HISTORY and ROBOT STATE below to interpret follow-ups. Output only one CALL/NO TOOL line.
Route from NEW USER QUERY at the bottom.

Line templates (examples):
- CALL publish_message text=<message>
- CALL get_camera_snapshot use_compressed=false
- CALL teleop_move direction=<forward|back|left|right|stop> duration_sec=<number>
- CALL emergency_stop repeats=5
- CALL ros2_topic_list filter_regex=<pattern or omit line after list for none>
- CALL ros2_topic_info topic=<topic path>
- CALL ros2_topic_echo_once topic=<topic> timeout_sec=10
- CALL ros2_topic_hz topic=<topic> sample_seconds=3
- CALL start_navigation_launch use_sim_time=false map_path=<path or omit>
- CALL navigate_to_saved_location name=<waypoint_key> publish_initial_pose=false
- CALL update_saved_location_coords name=<n> x=<n> y=<n> yaw_deg=<n>  # only include coords that change
- CALL get_odometry | CALL get_amcl_pose | CALL get_navigation_feedback | CALL wait_for_navigation_result timeout_sec=120
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

    @staticmethod
    def _truthy_token(s: str | None, default: bool = False) -> bool:
        if s is None:
            return default
        v = s.lower().strip()
        if v in ("true", "1", "yes", "on"):
            return True
        if v in ("false", "0", "no", "off"):
            return False
        return default

    async def _execute_command(self, command: str, user_input: str) -> str:
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
            uc_m = re.search(r"use_compressed=(true|false|1|0)", command, re.I)
            use_c = self._truthy_token(uc_m.group(1)) if uc_m else False
            result = await self.session.call_tool(
                "get_camera_snapshot",
                arguments={"use_compressed": use_c},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL emergency_stop"):
            rm = re.search(r"repeats=(\d+)", command)
            reps = int(rm.group(1)) if rm else 5
            result = await self.session.call_tool(
                "emergency_stop",
                arguments={"repeats": reps},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL get_odometry"):
            result = await self.session.call_tool("get_odometry", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_amcl_pose"):
            result = await self.session.call_tool("get_amcl_pose", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_battery_status"):
            result = await self.session.call_tool("get_battery_status", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_joy_state"):
            result = await self.session.call_tool("get_joy_state", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_lidar_scan"):
            sm = re.search(r"sectors_deg=(\d+)", command)
            fm = re.search(r"full=(true|false|1|0)", command, re.I)
            sectors = int(sm.group(1)) if sm else 8
            full = self._truthy_token(fm.group(1)) if fm else False
            result = await self.session.call_tool(
                "get_lidar_scan",
                arguments={"sectors_deg": sectors, "full": full},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL get_tf_transform"):
            tm = re.search(r"target_frame=(\S+)", command)
            sm = re.search(r"source_frame=(\S+)", command)
            if not tm or not sm:
                return "Routing error: get_tf_transform needs target_frame= source_frame="
            result = await self.session.call_tool(
                "get_tf_transform",
                arguments={
                    "target_frame": tm.group(1),
                    "source_frame": sm.group(1),
                },
            )
            return self._extract_result_text(result)

        if command.startswith("CALL ros2_node_list"):
            result = await self.session.call_tool("ros2_node_list", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL ros2_topic_list"):
            fm = re.search(r"filter_regex=(\S+)", command)
            args: dict = {}
            if fm:
                args["filter_regex"] = fm.group(1)
            result = await self.session.call_tool("ros2_topic_list", arguments=args)
            return self._extract_result_text(result)

        if command.startswith("CALL ros2_topic_info"):
            tm = re.search(r"topic=(\S+)", command)
            if not tm:
                return "Routing error: ros2_topic_info needs topic="
            result = await self.session.call_tool(
                "ros2_topic_info",
                arguments={"topic": tm.group(1)},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL ros2_topic_echo_once"):
            tm = re.search(r"topic=(\S+)", command)
            tom = re.search(r"timeout_sec=([\d.]+)", command)
            if not tm:
                return "Routing error: ros2_topic_echo_once needs topic="
            timeout = float(tom.group(1)) if tom else 10.0
            result = await self.session.call_tool(
                "ros2_topic_echo_once",
                arguments={"topic": tm.group(1), "timeout_sec": timeout},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL ros2_topic_hz"):
            tm = re.search(r"topic=(\S+)", command)
            sm = re.search(r"sample_seconds=([\d.]+)", command)
            if not tm:
                return "Routing error: ros2_topic_hz needs topic="
            sec = float(sm.group(1)) if sm else 3.0
            result = await self.session.call_tool(
                "ros2_topic_hz",
                arguments={"topic": tm.group(1), "sample_seconds": sec},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL start_navigation_launch"):
            um = re.search(r"use_sim_time=(true|false|1|0)", command, re.I)
            ust = self._truthy_token(um.group(1)) if um else False
            mm = re.search(r"map_path=(\S+)", command)
            args: dict = {"use_sim_time": ust}
            if mm:
                args["map_path"] = mm.group(1)
            result = await self.session.call_tool(
                "start_navigation_launch",
                arguments=args,
            )
            return self._extract_result_text(result)

        if command.startswith("CALL stop_navigation_launch"):
            result = await self.session.call_tool(
                "stop_navigation_launch", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL start_joystick_teleop_launch"):
            result = await self.session.call_tool(
                "start_joystick_teleop_launch", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL stop_joystick_teleop_launch"):
            result = await self.session.call_tool(
                "stop_joystick_teleop_launch", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL list_managed_launches"):
            result = await self.session.call_tool(
                "list_managed_launches", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL stop_all_managed_launches"):
            result = await self.session.call_tool(
                "stop_all_managed_launches", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL list_saved_locations"):
            result = await self.session.call_tool(
                "list_saved_locations", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL get_saved_location"):
            nm_m = re.search(r"name=(.+)$", command.strip())
            if not nm_m:
                return "Routing error: get_saved_location needs name="
            result = await self.session.call_tool(
                "get_saved_location",
                arguments={"name": nm_m.group(1).strip()},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL save_current_location"):
            nm_m = re.search(r"name=(.+)$", command.strip())
            if not nm_m:
                return "Routing error: save_current_location needs name="
            result = await self.session.call_tool(
                "save_current_location",
                arguments={"name": nm_m.group(1).strip()},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL update_saved_location_to_current"):
            nm_m = re.search(r"name=(.+)$", command.strip())
            if not nm_m:
                return "Routing error: update_saved_location_to_current needs name="
            result = await self.session.call_tool(
                "update_saved_location_to_current",
                arguments={"name": nm_m.group(1).strip()},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL update_saved_location_coords"):
            rest_cmd = command[len("CALL update_saved_location_coords") :].strip()
            mnm = re.search(r"\bname=", rest_cmd)
            if not mnm:
                return "Routing error: update_saved_location_coords needs name="
            blob = rest_cmd[mnm.end() :]  # after "name="
            name_blob = blob
            for delim in (" x=", " y=", " yaw_deg="):
                ix = blob.find(delim)
                if ix != -1:
                    name_blob = blob[:ix]
                    break
            name_val = name_blob.strip()
            if not name_val:
                return "Routing error: empty name in update_saved_location_coords"
            xm = re.search(r"\bx=([\d.-]+)", command)
            ym = re.search(r"\by=([\d.-]+)", command)
            zm = re.search(r"\byaw_deg=([\d.-]+)", command)
            uargs: dict = {"name": name_val}
            if xm:
                uargs["x"] = float(xm.group(1))
            if ym:
                uargs["y"] = float(ym.group(1))
            if zm:
                uargs["yaw_deg"] = float(zm.group(1))
            if len(uargs) == 1:
                return "Routing error: update_saved_location_coords needs at least one of x= y= yaw_deg="
            result = await self.session.call_tool(
                "update_saved_location_coords",
                arguments=uargs,
            )
            return self._extract_result_text(result)

        if command.startswith("CALL rename_saved_location"):
            stem = command[len("CALL rename_saved_location") :].strip()
            parts = re.split(r"\s+new_name=", stem, maxsplit=1, flags=re.I)
            if len(parts) != 2:
                return (
                    "Routing error: rename_saved_location needs "
                    "`old_name=... new_name=...` (exact new_name delimiter)."
                )
            left_blob, new_name = parts[0].strip(), parts[1].strip()
            ol = re.match(r"old_name=(.+)$", left_blob.strip(), flags=re.I)
            if not ol:
                return "Routing error: rename_saved_location expects old_name= as first clause."
            old_name = ol.group(1).strip()
            result = await self.session.call_tool(
                "rename_saved_location",
                arguments={
                    "old_name": old_name,
                    "new_name": new_name.strip(),
                },
            )
            return self._extract_result_text(result)

        if command.startswith("CALL delete_saved_location"):
            nm_m = re.search(r"name=(.+)$", command.strip())
            if not nm_m:
                return "Routing error: delete_saved_location needs name="
            result = await self.session.call_tool(
                "delete_saved_location",
                arguments={"name": nm_m.group(1).strip()},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL navigate_to_saved_location"):
            pub_m = re.search(r"\bpublish_initial_pose=(true|false|1|0)\b", command, re.I)
            pub = self._truthy_token(pub_m.group(1)) if pub_m else False
            truncated = command[: pub_m.start()] if pub_m else command
            nm_m = re.search(r"name=(.+)$", truncated.strip())
            if not nm_m:
                return "Routing error: navigate_to_saved_location needs name="
            wname = nm_m.group(1).strip()
            result = await self.session.call_tool(
                "navigate_to_saved_location",
                arguments={
                    "name": wname,
                    "publish_initial_pose": pub,
                },
            )
            return self._extract_result_text(result)

        if command.startswith("CALL get_navigation_help"):
            result = await self.session.call_tool("get_navigation_help", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_navigation_state"):
            result = await self.session.call_tool("get_navigation_state", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL execute_navigation_async"):
            result = await self.session.call_tool(
                "execute_navigation_async", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL cancel_navigation"):
            result = await self.session.call_tool("cancel_navigation", arguments={})
            return self._extract_result_text(result)

        if command.startswith("CALL get_navigation_feedback"):
            result = await self.session.call_tool(
                "get_navigation_feedback", arguments={}
            )
            return self._extract_result_text(result)

        if command.startswith("CALL wait_for_navigation_result"):
            tm = re.search(r"timeout_sec=([\d.]+)", command)
            tout = float(tm.group(1)) if tm else 120.0
            result = await self.session.call_tool(
                "wait_for_navigation_result",
                arguments={"timeout_sec": tout},
            )
            return self._extract_result_text(result)

        if command.startswith("CALL manual_recovery"):
            result = await self.session.call_tool("manual_recovery", arguments={})
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


async def chat_loop() -> None:
    ws = workspace_root()
    server_py = package_dir() / "mcp_server_ros2.py"
    if not server_py.is_file():
        raise RuntimeError(f"MCP server not found at {server_py}")

    memory_path = ws / FILE_NAME
    server_params = StdioServerParameters(
        command=sys.executable,
        args=[str(server_py)],
        env=os.environ.copy(),
        cwd=str(ws),
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
