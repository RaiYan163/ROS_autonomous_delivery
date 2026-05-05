"""OpenAI-based MCP wrapper for ROS2 TurtleBot tools."""

from __future__ import annotations

import asyncio
import json
import os
import re
import sys
from pathlib import Path

from dotenv import load_dotenv
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from openai import OpenAI

from .mcp_memory_context import FILE_NAME, MemoryContext, TOOL_AUDIT_LOG_NAME

load_dotenv()


def workspace_root() -> Path:
    """Repository root (parent of the ``mcp_server`` package)."""
    return Path(__file__).resolve().parent.parent


def package_dir() -> Path:
    return Path(__file__).resolve().parent


class MCPWrapper:
    """Routes user text through OpenAI and optionally calls MCP tools."""

    _TASK_PLAN_MAX_STEPS = 16

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
- publish_message(text) — ROS topic /chatter ONLY; never for normal chat (see below)
- get_status()
- get_camera_snapshot(use_compressed=false)  (/camera/image_raw or /image_raw/compressed JPEG)
- teleop_move(direction, duration_sec [, linear_speed] [, angular_speed] [, publish_hz])
- teleop_sequence(steps_json) — JSON array of timed move steps (same fields per element as teleop_move); use for squares, multi-segment paths, dance-like motion in one CALL
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

CRITICAL — your entire reply must be exactly ONE line. It MUST start with one of:
- CALL <tool and arguments>  (single tool, includes CALL teleop_sequence steps_json=... )
- NO TOOL NEEDED <brief plain text for the chat user only>
- TASK_PLAN <JSON array>  (see multi-step rules; minified JSON, no newlines)
No markdown. No surrounding quotes. No extra sentences beyond the one line.

Multi-step physical jobs in a single user request (e.g. square, zigzag, several moves, simple dance pattern):
- Prefer ONE CALL teleop_sequence steps_json=[{{"direction":"forward","duration_sec":2}},{{"direction":"left","duration_sec":2,"angular_speed":0.8}},…] when all steps are timed teleop. Open-loop only — durations are approximate.
- OR TASK_PLAN [{{"tool":"teleop_move","arguments":{{"direction":"forward","duration_sec":2}}}},…] with at most 16 steps. Each "tool" must be an MCP tool name the server exposes (same as the GUI "Available tools" list): navigation, sensing, waypoints, teleop, ros2 CLI mirrors, etc. Mix tools as needed (e.g. get_camera_snapshot then execute_navigation after goals are set).
- Use a single CALL teleop_move only when the user asks for one motion. If they ask for a shape or sequence, use teleop_sequence or TASK_PLAN — do not answer NO TOOL NEEDED unless chit-chat only.

Chat vs ROS /chatter:
- This window is a chat with the human. Anything you want to SAY to the user (answers, "I can't dance", jokes, clarifications, suggestions) MUST be NO TOOL NEEDED <brief plain text>. That text appears in the chat only and is NOT published to ROS.
- CALL publish_message ONLY when the user clearly wants a string published on the robot ROS topic /chatter (e.g. "publish hello on chatter", "announce X on /chatter", demo broadcast). Do NOT use publish_message because you are replying conversationally.

If the user wants motion but does not name a direction, use direction=forward.
If they give a duration (e.g. "5 seconds"), use that number as duration_sec. If no duration, use duration_sec=2.0.
For teleop_move: speeds are NOT fixed — pass linear_speed (m/s) on the CALL line for forward/back whenever the user says slow/fast, gives a number, or implies speed (e.g. "very slow" -> linear_speed=0.03). For left/right turns with requested turn rate, add angular_speed (rad/s). Add publish_hz only if they ask for a different cmd_vel publish rate. If they do not mention speed at all, omit linear_speed/angular_speed/publish_hz so the MCP server uses its ROS parameters teleop_linear/teleop_angular/teleop_rate_hz.
Never reply NO TOOL NEEDED to claim teleop speed cannot be changed; use the tool parameters above.

Navigation routing:
- CALL get_navigation_help ONLY for questions about HOW to navigate (workflow explanation).
- CALL execute_navigation when they want to drive to the STORED goal NOW (blocking).
- CALL execute_navigation_async + later wait_for_navigation_result / get_navigation_feedback / cancel_navigation for non-blocking.
- CALL get_navigation_state for "navigation status", "what goal is set".
- For named places from memory or waypoints JSON, CALL navigate_to_saved_location name=<name>.
If coordinates are explicit, CALL set_navigation_goal x= y= yaw_deg= (default yaw 0) then execute_navigation unless they asked async.

Use RECENT HISTORY and ROBOT STATE below to interpret follow-ups. Output exactly one line: CALL …, NO TOOL NEEDED …, or TASK_PLAN ….
Route from NEW USER QUERY at the bottom.

Line templates (examples):
- NO TOOL NEEDED I cannot dance, but tell me forward, back, or turn and I can move the robot.
- CALL teleop_sequence steps_json=[{{"direction":"forward","duration_sec":2}},{{"direction":"left","duration_sec":1.8,"angular_speed":0.85}}]
- TASK_PLAN [{{"tool":"teleop_move","arguments":{{"direction":"forward","duration_sec":2}}}},{{"tool":"get_odometry","arguments":{{}}}}]
- TASK_PLAN [{{"tool":"set_navigation_goal","arguments":{{"x":1.0,"y":0.5,"yaw_deg":0}}}},{{"tool":"execute_navigation","arguments":{{}}}}]
- CALL publish_message text=hello  # ONLY if user asked to publish on /chatter
- CALL get_camera_snapshot use_compressed=false
- CALL teleop_move direction=back duration_sec=2 linear_speed=0.03
- CALL teleop_move direction=<forward|back|left|right|stop> duration_sec=<number> [linear_speed=<m/s>] [angular_speed=<rad/s>] [publish_hz=<1-50>]
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
    def _tool_outcome_is_failure(command: str, tool_reply: str) -> bool:
        """True when a tool plan/CALL outcome looks like an error."""
        cmd = command.strip()
        if not (cmd.startswith("CALL ") or cmd.startswith("TASK_PLAN ")):
            return False
        r = tool_reply.strip()
        low = r.lower()
        if low.startswith("routing error"):
            return True
        if "unrecognized router output" in low:
            return True
        if "invalid task_plan" in low or "invalid task_plan json" in low:
            return True
        if "task_plan must" in low:
            return True
        if re.match(
            r"^(?:type|value|key|attribute|runtime|connection|timeout|modulenotfound)error\s*:",
            low,
        ):
            return True
        if "stopped after failed step" in low:
            return True
        if "aborted at step" in low:
            return True
        if "invalid steps_json" in low:
            return True
        if "too many steps" in low:
            return True
        if "sequence complete" in low:
            return False
        if "plan complete" in low and "stopped after failed" not in low:
            return False
        if "teleop" in low and "then stopped." in low:
            return False
        if "sent stop" in low and "cmd_vel" in low:
            return False
        fail_markers = (
            "rejected",
            "not available",
            "no navigation goal",
            "failed to",
            "unknown direction",
            "refused to publish",
            "unsupported",
            "invalid filter_regex",
            "traceback",
            "no such file",
            "errno ",
        )
        if any(m in low for m in fail_markers):
            return True
        if "invalid " in low and "argument" in low:
            return True
        if "goal rejected" in low or "was rejected" in low:
            return True
        return False

    def _llm_error_recovery_sync(
        self,
        user_input: str,
        routed_command: str,
        tool_reply: str,
    ) -> str:
        """Second-pass LLM (**only** invoked from ``handle`` after ``_tool_outcome_is_failure``)."""
        detail = tool_reply.strip()
        if len(detail) > 2800:
            detail = detail[:2797] + "..."
        prompt = f"""
You are helping the user with a ROS 2 TurtleBot workstation. A tool was run from their request and something went wrong.

USER REQUEST (verbatim):
{user_input.strip()}

ROUTER / TOOL INVOCATION (verbatim):
{routed_command.strip()}

TOOL OR SYSTEM OUTPUT (verbatim):
{detail}

Write a short, helpful reply (plain text, 2–5 sentences):
- Briefly explain what likely went wrong in terms the operator understands (do not invent sensor values or robot state you do not see above).
- Give one or two concrete next steps (e.g. fix a missing direction, start navigation, set a goal, check a topic, retry with corrected parameters).
- If the fix is to rephrase the request, give an example wording.

Do not repeat raw stack traces. Do not output a CALL line unless you are clearly showing example text inside a sentence.
""".strip()
        try:
            response = self.client.responses.create(
                model=self.model,
                input=prompt,
                temperature=0.2,
            )
            return response.output_text.strip()
        except Exception as exc:  # noqa: BLE001
            return (
                f"Something went wrong while running the robot tool.\n\n"
                f"Technical detail:\n{tool_reply.strip()}\n\n"
                f"(Recovery explanation could not be generated: {type(exc).__name__}: {exc})"
            )

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
    def _parse_teleop_from_user_text(
        text: str,
    ) -> tuple[str, float | None, float | None, float | None] | None:
        s = text.lower().strip()
        dur_m = re.search(
            r"(\d+(?:\.\d+)?)\s*(?:sec|second|seconds?|s)\b",
            s,
        )
        duration: float | None = float(dur_m.group(1)) if dur_m else None

        direction: str | None = None
        if re.search(r"\bstop\b", s):
            direction = "stop"
        elif re.search(r"\b(forward|ahead|straight)\b", s):
            direction = "forward"
        elif re.search(r"\b(backward|back|reverse)\b", s):
            direction = "back"
        elif re.search(r"\bturn\s+left\b", s) or (
            re.search(r"\bleft\b", s) and not re.search(r"\bright\b", s)
        ):
            direction = "left"
        elif re.search(r"\bturn\s+right\b", s) or re.search(r"\bright\b", s):
            direction = "right"
        elif re.search(r"\b(move|drive|go|teleop)\b", s):
            direction = "forward"
        if direction is None:
            return None

        linear_override: float | None = None
        angular_override: float | None = None
        lm = re.search(
            r"linear(?:\s*speed)?\s*(?:is|=|:)?\s*(\d+(?:\.\d+)?(?:e[-+]?\d+)?)",
            s,
        )
        am = re.search(
            r"angular(?:\s*speed)?\s*(?:is|=|:)?\s*(\d+(?:\.\d+)?(?:e[-+]?\d+)?)",
            s,
        )
        if lm:
            linear_override = float(lm.group(1))
        if am:
            angular_override = float(am.group(1))
        if linear_override is None and angular_override is None:
            gen = re.search(r"\b(?:around|about|roughly)\s+(\d+(?:\.\d+)?)\b", s)
            if gen and direction != "stop":
                v = float(gen.group(1))
                if direction in ("left", "right"):
                    angular_override = v
                else:
                    linear_override = v
        turn_dirs = ("left", "l", "turn_left", "right", "r", "turn_right")
        if direction != "stop" and direction not in turn_dirs and linear_override is None:
            if re.search(r"\b(very\s+slow|very\s+low|crawl)\b", s):
                linear_override = 0.03
            elif re.search(r"\b(slowly|slow\s+speed)\b", s) or (
                re.search(r"\bslow\b", s) and re.search(r"\b(speed|move|drive|back|forward)\b", s)
            ):
                linear_override = 0.08
        if direction in turn_dirs and angular_override is None:
            if re.search(r"\b(very\s+slow|very\s+low|crawl)\b", s):
                angular_override = 0.15
            elif re.search(r"\bslow\b", s):
                angular_override = 0.35
        return direction, duration, linear_override, angular_override

    @staticmethod
    def _teleop_publish_hz_from_call(command: str) -> float | None:
        m = re.search(r"publish_hz=([\d.eE+-]+)", command)
        return float(m.group(1)) if m else None

    @staticmethod
    def _teleop_speeds_from_call(command: str) -> tuple[float | None, float | None]:
        ls_m = re.search(r"linear_speed=([\d.eE+-]+)", command)
        az_m = re.search(r"angular_speed=([\d.eE+-]+)", command)
        ls = float(ls_m.group(1)) if ls_m else None
        az = float(az_m.group(1)) if az_m else None
        return ls, az

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

    @staticmethod
    def _merge_teleop_parameters(
        direction: str,
        duration_from_call: float | None,
        duration_from_parse: float | None,
        ls_call: float | None,
        az_call: float | None,
        ph_call: float | None,
        pl: float | None,
        pa: float | None,
    ) -> tuple[str, float, float | None, float | None, float | None]:
        """Duration defaults to 2s. Speeds/publish_hz only if CALL line or user text supplied."""
        direction = direction.strip().lower()
        if duration_from_call is not None:
            dur = float(duration_from_call)
        elif duration_from_parse is not None:
            dur = float(duration_from_parse)
        else:
            dur = 2.0
        ls = ls_call if ls_call is not None else pl
        az = az_call if az_call is not None else pa
        ph = ph_call
        return direction, dur, ls, az, ph

    async def _do_teleop(
        self,
        direction: str,
        duration_sec: float,
        linear_speed: float | None = None,
        angular_speed: float | None = None,
        publish_hz: float | None = None,
    ) -> str:
        args: dict = {"direction": direction, "duration_sec": duration_sec}
        if linear_speed is not None:
            args["linear_speed"] = linear_speed
        if angular_speed is not None:
            args["angular_speed"] = angular_speed
        if publish_hz is not None:
            args["publish_hz"] = publish_hz
        result = await self.session.call_tool("teleop_move", arguments=args)
        return self._extract_result_text(result)

    async def _mcp_tool_names(self) -> frozenset[str]:
        """Names advertised by the connected MCP server (matches GUI available tools)."""
        listed = await self.session.list_tools()
        tools = getattr(listed, "tools", None) or []
        names = frozenset(
            str(t.name).strip()
            for t in tools
            if getattr(t, "name", None) and str(t.name).strip()
        )
        if not names:
            raise RuntimeError("MCP list_tools returned no tools.")
        return names

    async def _execute_task_plan(self, plan: object) -> str:
        if not isinstance(plan, list):
            return "TASK_PLAN must be a JSON array of step objects."
        if len(plan) < 1 or len(plan) > self._TASK_PLAN_MAX_STEPS:
            return f"TASK_PLAN must have 1–{self._TASK_PLAN_MAX_STEPS} steps."
        try:
            allowed_tools = await self._mcp_tool_names()
        except Exception as exc:  # noqa: BLE001 — surface listing failure to caller
            return f"TASK_PLAN could not list MCP tools: {exc}"
        parts: list[str] = []
        for i, step in enumerate(plan):
            if not isinstance(step, dict):
                return f"Step {i + 1} is not a JSON object."
            tool = step.get("tool")
            args = step.get("arguments")
            if args is None:
                args = step.get("args")
            if not isinstance(tool, str) or not tool.strip():
                return f"Step {i + 1}: tool must be a non-empty string."
            tool = tool.strip()
            if tool not in allowed_tools:
                sample = ", ".join(sorted(allowed_tools)[:40])
                more = ""
                if len(allowed_tools) > 40:
                    more = f" … (+{len(allowed_tools) - 40} more)"
                return (
                    f"Step {i + 1}: unknown MCP tool {tool!r}. "
                    f"Known tools (sample): {sample}{more}"
                )
            if not isinstance(args, dict):
                return f"Step {i + 1}: arguments must be a JSON object."
            result = await self.session.call_tool(str(tool), arguments=args)
            text = self._extract_result_text(result)
            parts.append(f"[{i + 1}/{len(plan)}] {tool} → {text}")
            probe = f"CALL {tool}"
            if self._tool_outcome_is_failure(probe, text):
                return (
                    "\n".join(parts) + f"\nStopped after failed step {i + 1}."
                )
        return "\n".join(parts) + f"\nPlan complete ({len(plan)} steps)."

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

        if command.startswith("CALL teleop_sequence"):
            m = re.search(r"steps_json=(.+)$", command, re.DOTALL)
            if not m:
                return "Routing error: teleop_sequence needs steps_json=<JSON array>"
            payload = m.group(1).strip()
            result = await self.session.call_tool(
                "teleop_sequence",
                arguments={"steps_json": payload},
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
            ls_call, az_call = self._teleop_speeds_from_call(command)
            ph_call = self._teleop_publish_hz_from_call(command)
            d_match = re.search(r"direction=(\S+)", command)
            t_match = re.search(r"duration_sec=([\d.]+)", command)
            dur_call = float(t_match.group(1)) if t_match else None
            if not d_match:
                parsed = self._parse_teleop_from_user_text(user_input)
                if parsed:
                    direction, du, pl, pa = parsed
                    direction, dur, ls, az, ph = self._merge_teleop_parameters(
                        direction,
                        dur_call,
                        du,
                        ls_call,
                        az_call,
                        ph_call,
                        pl,
                        pa,
                    )
                    return await self._do_teleop(direction, dur, ls, az, ph)
                return "Routing error: teleop_move needs direction=..."
            direction_raw = d_match.group(1).strip().lower()
            parsed = self._parse_teleop_from_user_text(user_input)
            du = pl = pa = None
            if parsed:
                du, pl, pa = parsed[1], parsed[2], parsed[3]
            direction, dur, ls, az, ph = self._merge_teleop_parameters(
                direction_raw,
                dur_call,
                du,
                ls_call,
                az_call,
                ph_call,
                pl,
                pa,
            )
            return await self._do_teleop(direction, dur, ls, az, ph)

        if command.startswith("NO TOOL NEEDED"):
            return command.replace("NO TOOL NEEDED", "", 1).strip()

        parsed = self._parse_teleop_from_user_text(user_input)
        if parsed:
            direction, du, pl, pa = parsed
            direction, dur, ls, az, ph = self._merge_teleop_parameters(
                direction,
                None,
                du,
                None,
                None,
                None,
                pl,
                pa,
            )
            return await self._do_teleop(direction, dur, ls, az, ph)

        return f"Unrecognized router output: {command}"

    async def handle(self, user_input: str) -> tuple[str, str]:
        """One routing OpenAI call; TASK_PLAN runs MCP tools in series; recovery LLM only on failure."""
        self.memory.load()
        route = self._llm_route(user_input).strip()
        if route.startswith("CALL get_navigation_help") and self._wants_execute_navigation(
            user_input
        ):
            route = "CALL execute_navigation"

        command = route
        reply: str
        if route.startswith("TASK_PLAN "):
            payload = route[len("TASK_PLAN ") :].strip()
            try:
                plan = json.loads(payload)
                reply = await self._execute_task_plan(plan)
            except json.JSONDecodeError as exc:
                reply = f"Invalid TASK_PLAN JSON: {exc}"
            except Exception as exc:  # noqa: BLE001
                reply = f"{type(exc).__name__}: {exc}"
        else:
            try:
                reply = await self._execute_command(route, user_input)
            except Exception as exc:  # noqa: BLE001
                reply = f"{type(exc).__name__}: {exc}"

        if command.strip().startswith("CALL ") or command.strip().startswith(
            "TASK_PLAN "
        ):
            self.memory.append_mcp_tool_invocation(command.strip())

        display_reply = reply
        tool_failed = self._tool_outcome_is_failure(command, reply)
        if tool_failed:
            display_reply = await asyncio.to_thread(
                self._llm_error_recovery_sync,
                user_input,
                command.strip(),
                reply,
            )

        self.memory.append_exchange(user_input, display_reply)
        self.memory.update_robot_state(command, reply)
        self.memory.save()
        return display_reply, command


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
            print(f"MCP tool call audit log: {ws / TOOL_AUDIT_LOG_NAME}")
            while True:
                user_input = input("You: ").strip()
                if user_input.lower() in {"exit", "quit"}:
                    break
                if not user_input:
                    continue
                reply, routed = await wrapper.handle(user_input)
                print(f"Assistant: {reply}")
                r0 = routed.strip()
                if r0.startswith("CALL ") or r0.startswith("TASK_PLAN "):
                    tail = r0 if len(r0) <= 500 else r0[:497] + "..."
                    print(f"  [router] {tail}")


if __name__ == "__main__":
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is not set.")
    asyncio.run(chat_loop())
