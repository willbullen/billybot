"""ROS 2 development tool for interacting with the ByYourCommand container.

Provides general shell access and high-level ROS 2 CLI helpers via docker exec,
plus direct file operations on the shared ROS workspace mount.
"""

import asyncio
import os
import re
import shlex
from pathlib import Path
from typing import Any

from nanobot.agent.tools.base import Tool


class Ros2Tool(Tool):
    """Tool to interact with the ROS 2 (ByYourCommand) container for development."""

    # Commands that must never run inside the ROS 2 container
    _DENY_PATTERNS = [
        r"\brm\s+-[rf]{1,2}\s+/\s",       # rm -rf /
        r"\b(format|mkfs|diskpart)\b",     # disk operations
        r"\bdd\s+if=",                     # dd
        r">\s*/dev/sd",                    # write to disk
        r"\b(shutdown|reboot|poweroff)\b", # system power
        r":\(\)\s*\{.*\};\s*:",            # fork bomb
    ]

    def __init__(
        self,
        container_name: str = "billybot-ros2",
        ros_workspace: str | None = None,
        timeout: int = 60,
    ):
        self.container_name = container_name
        self.ros_workspace = Path(ros_workspace) if ros_workspace else None
        self.timeout = timeout

    # ------------------------------------------------------------------
    # Tool interface
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return "ros2"

    @property
    def description(self) -> str:
        return (
            "Interact with the ROS 2 ByYourCommand container for development. "
            "Supports running arbitrary commands, ROS 2 CLI introspection "
            "(topics, nodes, services, params), building packages, viewing logs, "
            "restarting the container, and reading/writing workspace source files."
        )

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": [
                        "exec",
                        "topics", "nodes", "services",
                        "topic_echo", "topic_info",
                        "node_info",
                        "params", "param_get", "param_set",
                        "build", "log", "restart",
                        "read_file", "write_file",
                    ],
                    "description": (
                        "Action to perform. "
                        "'exec' runs an arbitrary command in the container. "
                        "'topics/nodes/services' list ROS 2 entities. "
                        "'topic_echo' echoes messages from a topic. "
                        "'topic_info/node_info' show detailed info. "
                        "'params/param_get/param_set' manage node parameters. "
                        "'build' runs colcon build. "
                        "'log' shows container logs. "
                        "'restart' restarts the container. "
                        "'read_file/write_file' access ROS workspace source files."
                    ),
                },
                "command": {
                    "type": "string",
                    "description": "Shell command for 'exec' action (run inside the ROS 2 container)",
                },
                "topic": {
                    "type": "string",
                    "description": "Topic name for 'topic_echo' or 'topic_info'",
                },
                "node": {
                    "type": "string",
                    "description": "Node name for 'node_info', 'params', 'param_get', 'param_set'",
                },
                "param": {
                    "type": "string",
                    "description": "Parameter name for 'param_get' or 'param_set'",
                },
                "value": {
                    "type": "string",
                    "description": "Value for 'param_set'",
                },
                "count": {
                    "type": "integer",
                    "description": "Number of messages for 'topic_echo' (default 1)",
                    "minimum": 1,
                    "maximum": 100,
                },
                "packages": {
                    "type": "string",
                    "description": "Space-separated package names for 'build' (default: all workspace packages)",
                },
                "path": {
                    "type": "string",
                    "description": "Relative file path within the ROS workspace for 'read_file' or 'write_file'",
                },
                "content": {
                    "type": "string",
                    "description": "File content for 'write_file'",
                },
                "lines": {
                    "type": "integer",
                    "description": "Number of log lines for 'log' (default 100)",
                    "minimum": 1,
                    "maximum": 2000,
                },
                "verbose": {
                    "type": "boolean",
                    "description": "Verbose output for 'topics' (show message types and counts)",
                },
            },
            "required": ["action"],
        }

    async def execute(self, action: str, **kwargs: Any) -> str:
        """Dispatch to the appropriate action handler."""
        handler = getattr(self, f"_action_{action}", None)
        if handler is None:
            return f"Error: Unknown action '{action}'"
        try:
            return await handler(**kwargs)
        except Exception as e:
            return f"Error in ros2 {action}: {e}"

    # ------------------------------------------------------------------
    # Docker exec helper
    # ------------------------------------------------------------------

    async def _docker_exec(self, cmd: str, timeout: int | None = None) -> str:
        """Run a command inside the ROS 2 container with the ROS env sourced."""
        # Safety check
        guard = self._guard(cmd)
        if guard:
            return guard

        # Escape the inner command for bash -c
        escaped = cmd.replace("'", "'\\''")
        full_cmd = (
            f"docker exec {shlex.quote(self.container_name)} "
            f"bash -c 'source /opt/ros/humble/setup.bash && "
            f"source /ros2_ws/install/setup.bash 2>/dev/null; "
            f"{escaped}'"
        )

        return await self._run_shell(full_cmd, timeout or self.timeout)

    async def _docker_cmd(self, cmd: str, timeout: int | None = None) -> str:
        """Run a docker command (not exec) -- e.g. docker logs, docker restart."""
        return await self._run_shell(cmd, timeout or self.timeout)

    async def _run_shell(self, full_cmd: str, timeout: int) -> str:
        """Execute a shell command locally and return output."""
        try:
            process = await asyncio.create_subprocess_shell(
                full_cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            try:
                stdout, stderr = await asyncio.wait_for(
                    process.communicate(), timeout=timeout
                )
            except asyncio.TimeoutError:
                process.kill()
                return f"Error: Command timed out after {timeout}s"

            parts: list[str] = []
            if stdout:
                parts.append(stdout.decode("utf-8", errors="replace"))
            if stderr:
                err = stderr.decode("utf-8", errors="replace").strip()
                if err:
                    parts.append(f"STDERR:\n{err}")
            if process.returncode != 0:
                parts.append(f"\nExit code: {process.returncode}")

            result = "\n".join(parts) if parts else "(no output)"

            # Truncate very long output
            max_len = 10_000
            if len(result) > max_len:
                result = result[:max_len] + f"\n... (truncated, {len(result) - max_len} more chars)"
            return result

        except Exception as e:
            return f"Error executing command: {e}"

    def _guard(self, cmd: str) -> str | None:
        """Block obviously destructive commands."""
        lower = cmd.lower()
        for pattern in self._DENY_PATTERNS:
            if re.search(pattern, lower):
                return "Error: Command blocked by safety guard (dangerous pattern detected)"
        return None

    # ------------------------------------------------------------------
    # Action handlers: General execution
    # ------------------------------------------------------------------

    async def _action_exec(self, command: str = "", **kwargs: Any) -> str:
        """Run an arbitrary command inside the ROS 2 container."""
        if not command:
            return "Error: 'command' parameter is required for exec action"
        return await self._docker_exec(command)

    # ------------------------------------------------------------------
    # Action handlers: ROS 2 introspection
    # ------------------------------------------------------------------

    async def _action_topics(self, verbose: bool = False, **kwargs: Any) -> str:
        """List all ROS 2 topics."""
        cmd = "ros2 topic list -v" if verbose else "ros2 topic list"
        return await self._docker_exec(cmd)

    async def _action_nodes(self, **kwargs: Any) -> str:
        """List all ROS 2 nodes."""
        return await self._docker_exec("ros2 node list")

    async def _action_services(self, **kwargs: Any) -> str:
        """List all ROS 2 services."""
        return await self._docker_exec("ros2 service list")

    async def _action_topic_echo(
        self, topic: str = "", count: int = 1, **kwargs: Any
    ) -> str:
        """Echo messages from a ROS 2 topic."""
        if not topic:
            return "Error: 'topic' parameter is required for topic_echo"
        safe_topic = shlex.quote(topic)
        # Use a short timeout for echo since it blocks waiting for messages
        return await self._docker_exec(
            f"ros2 topic echo {safe_topic} --max-count {count}",
            timeout=min(self.timeout, 15),
        )

    async def _action_topic_info(self, topic: str = "", **kwargs: Any) -> str:
        """Get detailed info about a ROS 2 topic."""
        if not topic:
            return "Error: 'topic' parameter is required for topic_info"
        safe_topic = shlex.quote(topic)
        return await self._docker_exec(f"ros2 topic info {safe_topic} -v")

    async def _action_node_info(self, node: str = "", **kwargs: Any) -> str:
        """Get detailed info about a ROS 2 node."""
        if not node:
            return "Error: 'node' parameter is required for node_info"
        safe_node = shlex.quote(node)
        return await self._docker_exec(f"ros2 node info {safe_node}")

    # ------------------------------------------------------------------
    # Action handlers: Parameters
    # ------------------------------------------------------------------

    async def _action_params(self, node: str = "", **kwargs: Any) -> str:
        """List parameters for a ROS 2 node."""
        if not node:
            return "Error: 'node' parameter is required for params"
        safe_node = shlex.quote(node)
        return await self._docker_exec(f"ros2 param list {safe_node}")

    async def _action_param_get(
        self, node: str = "", param: str = "", **kwargs: Any
    ) -> str:
        """Get a parameter value from a ROS 2 node."""
        if not node or not param:
            return "Error: 'node' and 'param' parameters are required for param_get"
        return await self._docker_exec(
            f"ros2 param get {shlex.quote(node)} {shlex.quote(param)}"
        )

    async def _action_param_set(
        self, node: str = "", param: str = "", value: str = "", **kwargs: Any
    ) -> str:
        """Set a parameter value on a ROS 2 node."""
        if not node or not param or not value:
            return "Error: 'node', 'param', and 'value' parameters are required for param_set"
        return await self._docker_exec(
            f"ros2 param set {shlex.quote(node)} {shlex.quote(param)} {shlex.quote(value)}"
        )

    # ------------------------------------------------------------------
    # Action handlers: Development
    # ------------------------------------------------------------------

    async def _action_build(self, packages: str = "", **kwargs: Any) -> str:
        """Build ROS 2 packages with colcon."""
        if packages:
            pkg_list = packages.strip().split()
            safe_pkgs = " ".join(shlex.quote(p) for p in pkg_list)
            cmd = (
                f"cd /ros2_ws && colcon build "
                f"--packages-select {safe_pkgs} --symlink-install"
            )
        else:
            cmd = "cd /ros2_ws && colcon build --symlink-install"
        return await self._docker_exec(cmd, timeout=120)

    async def _action_log(self, lines: int = 100, **kwargs: Any) -> str:
        """Show recent container logs."""
        return await self._docker_cmd(
            f"docker logs --tail {lines} {shlex.quote(self.container_name)}"
        )

    async def _action_restart(self, **kwargs: Any) -> str:
        """Restart the ROS 2 container."""
        result = await self._docker_cmd(
            f"docker restart {shlex.quote(self.container_name)}"
        )
        if "Error" not in result:
            return f"Container '{self.container_name}' restarted successfully."
        return result

    # ------------------------------------------------------------------
    # Action handlers: File operations (via shared mount)
    # ------------------------------------------------------------------

    async def _action_read_file(self, path: str = "", **kwargs: Any) -> str:
        """Read a file from the ROS 2 workspace."""
        if not path:
            return "Error: 'path' parameter is required for read_file"
        if not self.ros_workspace:
            return "Error: ROS workspace mount not configured"

        resolved = self._resolve_workspace_path(path)
        if isinstance(resolved, str):
            return resolved  # error message

        try:
            content = resolved.read_text(encoding="utf-8", errors="replace")
            # Truncate very large files
            max_len = 15_000
            if len(content) > max_len:
                content = content[:max_len] + f"\n... (truncated, {len(content) - max_len} more chars)"
            return content if content else "(empty file)"
        except FileNotFoundError:
            return f"Error: File not found: {path}"
        except IsADirectoryError:
            # List directory contents instead
            entries = sorted(resolved.iterdir())
            listing = "\n".join(
                f"{'[dir]  ' if e.is_dir() else '       '}{e.name}" for e in entries
            )
            return f"'{path}' is a directory:\n{listing}" if listing else f"'{path}' is an empty directory"
        except Exception as e:
            return f"Error reading file: {e}"

    async def _action_write_file(
        self, path: str = "", content: str = "", **kwargs: Any
    ) -> str:
        """Write or create a file in the ROS 2 workspace."""
        if not path:
            return "Error: 'path' parameter is required for write_file"
        if not self.ros_workspace:
            return "Error: ROS workspace mount not configured"

        resolved = self._resolve_workspace_path(path)
        if isinstance(resolved, str):
            return resolved  # error message

        try:
            resolved.parent.mkdir(parents=True, exist_ok=True)
            resolved.write_text(content, encoding="utf-8")
            return f"File written: {path} ({len(content)} bytes)"
        except Exception as e:
            return f"Error writing file: {e}"

    def _resolve_workspace_path(self, rel_path: str) -> Path | str:
        """Resolve a relative path within the ROS workspace, with safety checks."""
        if not self.ros_workspace:
            return "Error: ROS workspace mount not configured"

        # Prevent path traversal
        if ".." in rel_path:
            return "Error: Path traversal ('..') is not allowed"

        resolved = (self.ros_workspace / rel_path).resolve()

        # Ensure the resolved path is inside the workspace
        try:
            resolved.relative_to(self.ros_workspace.resolve())
        except ValueError:
            return "Error: Path escapes the ROS workspace boundary"

        return resolved
