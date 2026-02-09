"""WebSocket consumers for the BillyBot dashboard."""

import asyncio
import json
import subprocess

from channels.generic.websocket import AsyncWebSocketConsumer
from django.conf import settings


class Ros2BridgeConsumer(AsyncWebSocketConsumer):
    """Bridges browser WebSocket to ROS 2 topics via docker exec.

    Receives commands from the control page (cmd_vel, arm_preset, etc.)
    and publishes them to ROS 2 topics inside the container.
    """

    async def connect(self):
        await self.accept()
        self.container = settings.ROS2_CONTAINER_NAME

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data=None, bytes_data=None):
        if not text_data:
            return
        try:
            data = json.loads(text_data)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "")

        if msg_type == "cmd_vel":
            await self._publish_cmd_vel(data)
        elif msg_type == "arm_preset":
            await self._publish_string("/arm_preset", data.get("preset", ""))
        elif msg_type == "bearing":
            await self._publish_string("/behavior_command", data.get("bearing", ""))
        elif msg_type == "voice_active":
            val = "true" if data.get("active") else "false"
            await self._ros2_exec(
                f"ros2 topic pub --once /voice_active std_msgs/Bool '{{data: {val}}}'"
            )
        elif msg_type == "estop":
            await self._publish_cmd_vel(
                {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}
            )
        elif msg_type == "pantilt":
            pan = data.get("pan", 0)
            tilt = data.get("tilt", 0)
            await self._ros2_exec(
                f"ros2 topic pub --once /pantilt_cmd geometry_msgs/Vector3 "
                f"'{{x: {pan:.3f}, y: {tilt:.3f}, z: 0.0}}'"
            )

    async def _publish_cmd_vel(self, data):
        lin = data.get("linear", {})
        ang = data.get("angular", {})
        lx = lin.get("x", 0)
        az = ang.get("z", 0)
        await self._ros2_exec(
            f"ros2 topic pub --once /cmd_vel geometry_msgs/Twist "
            f"'{{linear: {{x: {lx:.3f}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {az:.3f}}}}}'"
        )

    async def _publish_string(self, topic, value):
        if not value:
            return
        safe_value = value.replace("'", "")
        await self._ros2_exec(
            f"ros2 topic pub --once {topic} std_msgs/String '{{data: \"{safe_value}\"}}'"
        )

    async def _ros2_exec(self, cmd):
        full_cmd = (
            f"docker exec {self.container} bash -c "
            f"'source /opt/ros/humble/setup.bash && "
            f"source /ros2_ws/install/setup.bash 2>/dev/null; {cmd}'"
        )
        try:
            proc = await asyncio.create_subprocess_shell(
                full_cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            await asyncio.wait_for(proc.communicate(), timeout=5)
        except (asyncio.TimeoutError, Exception):
            pass


class TelemetryConsumer(AsyncWebSocketConsumer):
    """Streams telemetry data to the browser."""

    async def connect(self):
        await self.accept()
        self._running = True
        asyncio.ensure_future(self._telemetry_loop())

    async def disconnect(self, close_code):
        self._running = False

    async def _telemetry_loop(self):
        """Periodically check ROS 2 system status and send updates."""
        while self._running:
            try:
                await asyncio.sleep(2)
            except asyncio.CancelledError:
                break

    async def receive(self, text_data=None, bytes_data=None):
        pass


class ChatConsumer(AsyncWebSocketConsumer):
    """Chat interface to nanobot via docker exec."""

    async def connect(self):
        await self.accept()
        self.container = settings.NANOBOT_CONTAINER_NAME

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data=None, bytes_data=None):
        if not text_data:
            return
        try:
            data = json.loads(text_data)
        except json.JSONDecodeError:
            return

        message = data.get("message", "").strip()
        if not message:
            return

        # Run nanobot agent with the message
        safe_msg = message.replace("'", "'\\''")
        cmd = (
            f"docker exec {self.container} "
            f"nanobot agent -m '{safe_msg}' --no-markdown"
        )

        try:
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=120)
            output = stdout.decode("utf-8", errors="replace").strip()
            if not output and stderr:
                output = stderr.decode("utf-8", errors="replace").strip()
            if not output:
                output = "(no response)"
        except asyncio.TimeoutError:
            output = "Error: Command timed out after 120s"
        except Exception as e:
            output = f"Error: {e}"

        await self.send(text_data=json.dumps({"content": output}))


class LogConsumer(AsyncWebSocketConsumer):
    """Streams container logs in real-time."""

    async def connect(self):
        await self.accept()
        self._running = True

    async def disconnect(self, close_code):
        self._running = False

    async def receive(self, text_data=None, bytes_data=None):
        if not text_data:
            return
        try:
            data = json.loads(text_data)
        except json.JSONDecodeError:
            return

        container = data.get("container", "billybot-ros2")
        lines = min(data.get("lines", 50), 500)

        cmd = f"docker logs --tail {lines} {container}"
        try:
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=10)
            output = (stdout or b"").decode("utf-8", errors="replace")
            if not output:
                output = (stderr or b"").decode("utf-8", errors="replace")
            await self.send(text_data=json.dumps({"logs": output}))
        except Exception as e:
            await self.send(text_data=json.dumps({"error": str(e)}))
