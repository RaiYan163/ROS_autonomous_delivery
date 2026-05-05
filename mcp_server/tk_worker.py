"""
Background asyncio MCP stdio client for Tk (control center AI assistant).

Runs ``mcp_server_ros2.py`` as a child process; forwards user lines to ``MCPWrapper.handle``.
"""

from __future__ import annotations

import asyncio
import os
import queue
import sys
import threading
from pathlib import Path

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client

from .mcp_client_openai import MCPWrapper
from .mcp_memory_context import FILE_NAME


async def _async_main(
    workspace: Path,
    user_q: queue.Queue,
    tk_q: queue.Queue,
    stop: threading.Event,
) -> None:
    pkg = Path(__file__).resolve().parent
    server_py = pkg / "mcp_server_ros2.py"
    if not server_py.is_file():
        tk_q.put(("error", f"MCP server script missing: {server_py}"))
        return

    if not os.environ.get("OPENAI_API_KEY"):
        tk_q.put(("error", "OPENAI_API_KEY is not set — cannot start MCP assistant."))
        return

    memory_path = workspace / FILE_NAME
    server_params = StdioServerParameters(
        command=sys.executable,
        args=[str(server_py)],
        env=os.environ.copy(),
        cwd=str(workspace),
    )

    try:
        async with stdio_client(server_params) as (read_stream, write_stream):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()
                tk_q.put(("info", "MCP session connected (stdio server running)."))
                wrapper = MCPWrapper(session, memory_path)
                while not stop.is_set():
                    try:
                        user_text = await asyncio.to_thread(user_q.get, True, 0.35)
                    except queue.Empty:
                        continue
                    if user_text is None:
                        break
                    if not isinstance(user_text, str):
                        continue
                    text = user_text.strip()
                    if not text:
                        continue
                    try:
                        reply, routed = await wrapper.handle(text)
                        tk_q.put(("reply", text, reply, routed))
                    except Exception as exc:  # noqa: BLE001 — surface to UI
                        tk_q.put(("error", f"{type(exc).__name__}: {exc}"))
    except Exception as exc:  # noqa: BLE001
        tk_q.put(("error", f"MCP worker exited: {type(exc).__name__}: {exc}"))


def start_mcp_worker(
    workspace: Path,
) -> tuple[threading.Thread, queue.Queue, queue.Queue, threading.Event]:
    """
    Start daemon thread running asyncio MCP client.

    Returns ``(thread, user_queue, tk_queue, stop_event)``.
    Push user strings to ``user_queue``; receive ``("reply", user, assistant_text,
    router_line)``, ``("error", msg)``, or ``("info", msg)`` on ``tk_queue``. Push ``None`` to ``user_queue`` to stop the loop.
    """
    user_q: queue.Queue = queue.Queue()
    tk_q: queue.Queue = queue.Queue()
    stop = threading.Event()

    def runner() -> None:
        asyncio.run(_async_main(workspace, user_q, tk_q, stop))

    th = threading.Thread(target=runner, name="mcp_stdio_client", daemon=True)
    th.start()
    return th, user_q, tk_q, stop


def stop_mcp_worker(
    stop: threading.Event,
    user_q: queue.Queue,
    thread: threading.Thread | None,
    timeout: float = 10.0,
) -> None:
    stop.set()
    try:
        user_q.put_nowait(None)
    except queue.Full:
        pass
    if thread is not None and thread.is_alive():
        thread.join(timeout=timeout)
