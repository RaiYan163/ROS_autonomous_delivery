"""
Sliding-window conversation memory + recent MCP router ``CALL …`` tail + robot state.

``memory_context.txt`` (workspace root) stores RECENT HISTORY, RECENT MCP TOOL CALLS,
and ROBOT STATE. ``mcp_tool_calls.log`` is an append-only audit of MCP invocations only.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path


MAX_EXCHANGES = 6
MAX_RECENT_MCP_TOOL_CALLS = 24
FILE_NAME = "memory_context.txt"
# Append-only audit of MCP ``CALL …`` routing lines at workspace root.
TOOL_AUDIT_LOG_NAME = "mcp_tool_calls.log"


@dataclass
class RobotState:
    last_command: str = "(none)"
    last_result: str = "(none)"
    last_goal: str = "(none)"
    last_initial_pose: str = "(none)"
    last_nav_status: str = "(none)"
    last_snapshot: str = "(none)"
    last_waypoint_name: str = "(none)"


@dataclass
class MemoryContext:
    """Recent USER/ASSISTANT pairs + MCP tool-invocation tail + robot state."""

    path: Path
    exchanges: list[tuple[str, str]] = field(default_factory=list)
    recent_mcp_calls: list[str] = field(default_factory=list)
    state: RobotState = field(default_factory=RobotState)

    def load(self) -> None:
        self.exchanges = []
        self.recent_mcp_calls = []
        self.state = RobotState()
        if not self.path.is_file():
            return
        try:
            text = self.path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return
        section: str | None = None
        hist_buf: list[str] = []
        tools_buf: list[str] = []
        for line in text.splitlines():
            s = line.strip()
            if s.startswith("====") and len(s) >= 4:
                continue
            if s == "RECENT HISTORY":
                section = "history"
                hist_buf = []
                continue
            if s == "RECENT MCP TOOL CALLS":
                if section == "history" and hist_buf:
                    self._parse_history_lines(hist_buf)
                section = "tool_calls"
                tools_buf = []
                continue
            if s == "ROBOT STATE":
                if section == "history" and hist_buf:
                    self._parse_history_lines(hist_buf)
                elif section == "tool_calls" and tools_buf:
                    self._parse_tool_calls_lines(tools_buf)
                section = "state"
                continue
            if section == "history":
                hist_buf.append(line)
            elif section == "tool_calls":
                tools_buf.append(line)
            elif section == "state" and ":" in line:
                key, _, val = line.partition(":")
                key = key.strip()
                val = val.strip()
                if key == "last_command":
                    self.state.last_command = val or "(none)"
                elif key == "last_result":
                    self.state.last_result = val or "(none)"
                elif key == "last_goal":
                    self.state.last_goal = val or "(none)"
                elif key == "last_initial_pose":
                    self.state.last_initial_pose = val or "(none)"
                elif key == "last_nav_status":
                    self.state.last_nav_status = val or "(none)"
                elif key == "last_snapshot":
                    self.state.last_snapshot = val or "(none)"
                elif key == "last_waypoint_name":
                    self.state.last_waypoint_name = val or "(none)"
        if section == "history" and hist_buf:
            self._parse_history_lines(hist_buf)
        elif section == "tool_calls" and tools_buf:
            self._parse_tool_calls_lines(tools_buf)

    def _parse_history_lines(self, lines: list[str]) -> None:
        cur_user: str | None = None
        for raw in lines:
            line = raw.rstrip("\n")
            if line.startswith("USER:"):
                cur_user = line[5:].strip()
            elif line.startswith("ASSISTANT:") and cur_user is not None:
                asst = line[10:].strip()
                self.exchanges.append((cur_user, asst))
                cur_user = None
        while len(self.exchanges) > MAX_EXCHANGES:
            self.exchanges.pop(0)

    def append_exchange(self, user: str, assistant: str) -> None:
        self.exchanges.append((user.strip(), _short_text(assistant.strip(), 800)))
        while len(self.exchanges) > MAX_EXCHANGES:
            self.exchanges.pop(0)

    def _parse_tool_calls_lines(self, lines: list[str]) -> None:
        self.recent_mcp_calls = []
        for raw in lines:
            line = raw.strip()
            if not line or line.lower().startswith("(no "):
                continue
            self.recent_mcp_calls.append(line)
        while len(self.recent_mcp_calls) > MAX_RECENT_MCP_TOOL_CALLS:
            self.recent_mcp_calls.pop(0)

    def append_mcp_tool_invocation(self, router_command: str) -> None:
        """Persist one router ``CALL …`` or ``TASK_PLAN …`` line to memory tail + audit log."""
        cmd = router_command.strip()
        if cmd.startswith("CALL "):
            entry_body = cmd
        elif cmd.startswith("TASK_PLAN "):
            entry_body = cmd if len(cmd) <= 2000 else cmd[:1997] + "..."
        else:
            return
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        entry = f"{ts} | {entry_body}"
        self.recent_mcp_calls.append(entry)
        while len(self.recent_mcp_calls) > MAX_RECENT_MCP_TOOL_CALLS:
            self.recent_mcp_calls.pop(0)
        audit = self.path.parent / TOOL_AUDIT_LOG_NAME
        try:
            with audit.open("a", encoding="utf-8") as fp:
                fp.write(entry + "\n")
        except OSError:
            pass

    def build_router_context(self, new_user_query: str) -> str:
        """Block inserted into the OpenAI router prompt (history + state + new query)."""
        lines: list[str] = [
            "==============================",
            "RECENT HISTORY",
            "==============================",
        ]
        if not self.exchanges:
            lines.append("(no prior turns)")
        else:
            for u, a in self.exchanges:
                lines.append(f"USER: {u}")
                lines.append(f"ASSISTANT: {a}")
                lines.append("")
        lines.extend(
            [
                "==============================",
                "RECENT MCP TOOL CALLS",
                "==============================",
            ]
        )
        if not self.recent_mcp_calls:
            lines.append("(no MCP tool invocations yet in sliding window)")
        else:
            for t in self.recent_mcp_calls:
                lines.append(t)
            lines.append("")
        lines.extend(
            [
                "==============================",
                "ROBOT STATE",
                "==============================",
                f"last_command: {self.state.last_command}",
                f"last_result: {self.state.last_result}",
                f"last_goal: {self.state.last_goal}",
                f"last_initial_pose: {self.state.last_initial_pose}",
                f"last_nav_status: {self.state.last_nav_status}",
                f"last_snapshot: {self.state.last_snapshot}",
                f"last_waypoint_name: {self.state.last_waypoint_name}",
                "",
                "==============================",
                "NEW USER QUERY",
                "==============================",
                new_user_query.strip(),
            ]
        )
        return "\n".join(lines)

    def save(self) -> None:
        lines: list[str] = [
            "==============================",
            "RECENT HISTORY",
            "==============================",
            "",
        ]
        for u, a in self.exchanges:
            lines.append(f"USER: {u}")
            lines.append(f"ASSISTANT: {a}")
            lines.append("")
        lines.extend(
            [
                "==============================",
                "RECENT MCP TOOL CALLS",
                "==============================",
            ]
        )
        if not self.recent_mcp_calls:
            lines.append("(no MCP tool invocations yet in sliding window)")
        else:
            for t in self.recent_mcp_calls:
                lines.append(t)
            lines.append("")
        lines.extend(
            [
                "==============================",
                "ROBOT STATE",
                "==============================",
                f"last_command: {self.state.last_command}",
                f"last_result: {self.state.last_result}",
                f"last_goal: {self.state.last_goal}",
                f"last_initial_pose: {self.state.last_initial_pose}",
                f"last_nav_status: {self.state.last_nav_status}",
                f"last_snapshot: {self.state.last_snapshot}",
                f"last_waypoint_name: {self.state.last_waypoint_name}",
                "",
            ]
        )
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("\n".join(lines), encoding="utf-8")

    def update_robot_state(self, command: str, reply: str) -> None:
        """After each turn, patch fields per mcp_instructions.txt."""
        cmd = command.strip()
        rep = reply.strip()
        short_rep = _short_text(rep, 240)

        if cmd.startswith("CALL teleop_move"):
            d = _re_first(r"direction=(\S+)", cmd) or "?"
            t = _re_first(r"duration_sec=([\d.]+)", cmd) or "?"
            ls = _re_first(r"linear_speed=([\d.eE+-]+)", cmd)
            azs = _re_first(r"angular_speed=([\d.eE+-]+)", cmd)
            phz = _re_first(r"publish_hz=([\d.eE+-]+)", cmd)
            extra = ""
            if ls:
                extra += f",linear_speed={ls}"
            if azs:
                extra += f",angular_speed={azs}"
            if phz:
                extra += f",publish_hz={phz}"
            self.state.last_command = (
                f"teleop_move(direction={d},duration_sec={t}{extra})"
            )
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL teleop_sequence"):
            self.state.last_command = "teleop_sequence(...)"
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("TASK_PLAN "):
            self.state.last_command = _short_text(
                "task_plan " + cmd[len("TASK_PLAN ") :].strip(), 160
            )
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL set_navigation_goal"):
            xm = _re_first(r"x=([\d.-]+)", cmd)
            ym = _re_first(r"y=([\d.-]+)", cmd)
            zm = _re_first(r"yaw_deg=([\d.-]+)", cmd)
            if xm and ym:
                yaw = zm if zm else "0"
                self.state.last_goal = f"x={xm},y={ym},yaw={yaw}"

        elif cmd.startswith("CALL set_navigation_initial_pose"):
            xm = _re_first(r"x=([\d.-]+)", cmd)
            ym = _re_first(r"y=([\d.-]+)", cmd)
            zm = _re_first(r"yaw_deg=([\d.-]+)", cmd)
            if xm and ym and zm:
                self.state.last_initial_pose = f"x={xm},y={ym},yaw={zm}"

        elif cmd.startswith("CALL execute_navigation_async"):
            self.state.last_nav_status = "async_started"
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL execute_navigation"):
            self.state.last_nav_status = _nav_status_from_reply(rep)
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL get_camera_snapshot"):
            m = re.search(r"Saved snapshot (?:from COMPRESSED \(\S+\) )?to\s+(\S+)", rep)
            if m:
                self.state.last_snapshot = m.group(1).strip()

        elif cmd.startswith("CALL navigate_to_saved_location"):
            wm = _re_first(r"name=([^\s].*?)(?=\s+publish_|$)", cmd)
            if wm:
                self.state.last_goal = f"saved_waypoint:{wm.strip()}"
                self.state.last_waypoint_name = wm.strip()
                self.state.last_nav_status = _nav_status_from_reply(rep)
                self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL save_current_location"):
            wm = _re_first(r"name=(\S.+)$", cmd) or _re_first(r"name=(\S+)", cmd)
            if wm:
                self.state.last_waypoint_name = wm.strip()
                self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL emergency_stop"):
            self.state.last_command = "emergency_stop"
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL cancel_navigation"):
            self.state.last_nav_status = "cancel_requested"
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith("CALL wait_for_navigation_result"):
            self.state.last_nav_status = _nav_status_from_reply(rep)
            self.state.last_result = _result_bucket(rep, short_rep)

        elif cmd.startswith(
            (
                "CALL start_navigation_launch",
                "CALL stop_navigation_launch",
                "CALL start_joystick_teleop_launch",
                "CALL stop_joystick_teleop_launch",
                "CALL stop_all_managed_launches",
            ),
        ):
            self.state.last_command = _short_text(cmd.replace("CALL ", "").strip(), 120)
            self.state.last_result = _result_bucket(rep, short_rep)


def _short_text(s: str, max_len: int) -> str:
    s = s.replace("\n", " ").strip()
    if len(s) <= max_len:
        return s
    return s[: max_len - 3] + "..."


def _re_first(pat: str, s: str) -> str | None:
    m = re.search(pat, s)
    return m.group(1) if m else None


def _result_bucket(full: str, short: str) -> str:
    low = full.lower()
    fail_markers = (
        "routing error",
        "rejected",
        "not available",
        "no navigation goal",
        "failed to",
        "unknown direction",
        "refused to publish",
        "unsupported",
    )
    if any(m in low for m in fail_markers):
        return f"failed: {short}"
    return f"success — {short}"


def _nav_status_from_reply(rep: str) -> str:
    low = rep.lower()
    if "succeeded" in low:
        return "succeeded"
    if "rejected" in low:
        return "rejected"
    if "no navigation goal" in low or "not set" in low:
        return "failed (no goal)"
    if "status=" in low:
        m = re.search(r"status=(\d+)", rep)
        if m:
            return f"finished status={m.group(1)}"
    if "failed" in low or "error" in low:
        return "failed"
    return _short_text(rep, 120)
