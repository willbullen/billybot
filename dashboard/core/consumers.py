"""WebSocket consumers for the BillyBot dashboard."""

import asyncio
import json
import subprocess

from channels.generic.websocket import AsyncWebsocketConsumer
from django.conf import settings


class Ros2BridgeConsumer(AsyncWebsocketConsumer):
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
            await self._publish_string("/grunt1/arm_preset", data.get("preset", ""))
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


class TelemetryConsumer(AsyncWebsocketConsumer):
    """Streams telemetry data to the browser (motor_feedback from ROS 2)."""

    async def connect(self):
        await self.accept()
        self._running = True
        self._container = getattr(settings, "ROS2_CONTAINER_NAME", "billybot-ros2")
        asyncio.ensure_future(self._telemetry_loop())

    async def disconnect(self, close_code):
        self._running = False

    async def _telemetry_loop(self):
        """Poll /motor_feedback via docker exec ~1 Hz and push to browser."""
        while self._running:
            try:
                await asyncio.sleep(1)
                data = await self._poll_motor_feedback()
                if data is not None:
                    await self.send(text_data=json.dumps({"type": "motor_feedback", "data": data}))
            except asyncio.CancelledError:
                break
            except Exception:
                pass

    async def _poll_motor_feedback(self):
        """One-shot read of /motor_feedback (std_msgs/String JSON). Returns parsed dict or None."""
        cmd = (
            f"docker exec {self._container} bash -c "
            "'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; "
            "timeout 1 ros2 topic echo --once /motor_feedback std_msgs/String 2>/dev/null'"
        )
        try:
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=3)
            if not stdout:
                return None
            text = stdout.decode("utf-8", errors="replace")
            # ros2 topic echo prints "data: '<json>'" or "data: \"<json>\""
            for line in text.splitlines():
                line = line.strip()
                if line.startswith("data:"):
                    val = line[5:].strip().strip('"').strip("'")
                    return json.loads(val)
            return None
        except (asyncio.TimeoutError, json.JSONDecodeError, ValueError):
            return None

    async def receive(self, text_data=None, bytes_data=None):
        pass


class ChatConsumer(AsyncWebsocketConsumer):
    """Chat interface to nanobot via docker exec.

    Maintains a session ID per WebSocket connection so nanobot remembers
    conversation history across messages.
    """

    async def connect(self):
        await self.accept()
        self.container = settings.NANOBOT_CONTAINER_NAME
        # Generate a session ID for this WebSocket connection
        import uuid
        self.session_id = f"dashboard-{uuid.uuid4().hex[:12]}"
        self.model_override = None

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data=None, bytes_data=None):
        if not text_data:
            return
        try:
            data = json.loads(text_data)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "chat")

        # Handle model switch
        if msg_type == "set_model":
            self.model_override = data.get("model") or None
            await self.send(text_data=json.dumps({
                "type": "model_changed",
                "model": self.model_override or "(default)",
            }))
            return

        # Handle session reset
        if msg_type == "reset_session":
            import uuid
            self.session_id = f"dashboard-{uuid.uuid4().hex[:12]}"
            await self.send(text_data=json.dumps({
                "type": "session_reset",
                "session_id": self.session_id,
            }))
            return

        message = data.get("message", "").strip()
        if not message:
            return

        # Build nanobot command with session persistence
        safe_msg = message.replace("'", "'\\''")
        cmd_parts = [
            f"docker exec {self.container}",
            "nanobot agent",
            f"-m '{safe_msg}'",
            f"-s '{self.session_id}'",
            "--no-markdown",
        ]
        cmd = " ".join(cmd_parts)

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

        await self.send(text_data=json.dumps({"type": "chat", "content": output}))


class CameraConsumer(AsyncWebsocketConsumer):
    """Streams camera frames from ROS 2 to the browser via WebSocket.

    Polls /camera/color/compressed (sensor_msgs/CompressedImage) and sends
    base64-encoded JPEG frames to the browser at ~5 fps.
    """

    async def connect(self):
        await self.accept()
        self._running = True
        self._container = getattr(settings, "ROS2_CONTAINER_NAME", "billybot-ros2")
        self._fps = 5
        asyncio.ensure_future(self._camera_loop())

    async def disconnect(self, close_code):
        self._running = False

    async def receive(self, text_data=None, bytes_data=None):
        """Handle control messages from browser (fps adjustment, etc.)."""
        if not text_data:
            return
        try:
            data = json.loads(text_data)
        except json.JSONDecodeError:
            return
        if "fps" in data:
            self._fps = max(1, min(int(data["fps"]), 15))

    async def _camera_loop(self):
        """Poll compressed camera image and push base64 JPEG to browser."""
        import base64

        while self._running:
            try:
                delay = 1.0 / self._fps
                await asyncio.sleep(delay)
                frame_b64 = await self._grab_frame()
                if frame_b64:
                    await self.send(text_data=json.dumps({
                        "type": "frame",
                        "data": frame_b64,
                        "format": "jpeg",
                    }))
            except asyncio.CancelledError:
                break
            except Exception:
                await asyncio.sleep(1)

    async def _grab_frame(self):
        """Grab one compressed frame via docker exec and return base64 string."""
        import base64

        cmd = (
            f"docker exec {self._container} bash -c "
            "'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; "
            "python3 -c \""
            "import rclpy, base64, sys; "
            "from sensor_msgs.msg import CompressedImage; "
            "rclpy.init(); "
            "node = rclpy.create_node(\\\"_cam_grab\\\"); "
            "msg = [None]; "
            "sub = node.create_subscription(CompressedImage, \\\"/camera/color/compressed\\\", lambda m: (msg.__setitem__(0, m), rclpy.shutdown()), 1); "
            "import threading; t = threading.Timer(2.0, lambda: rclpy.try_shutdown()); t.start(); "
            "rclpy.spin(node); t.cancel(); node.destroy_node(); "
            "sys.stdout.buffer.write(base64.b64encode(msg[0].data) if msg[0] else b\\\"\\\"); "
            "\"'"
        )
        try:
            proc = await asyncio.create_subprocess_shell(
                cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=5)
            if stdout:
                return stdout.decode("utf-8", errors="replace").strip()
            return None
        except (asyncio.TimeoutError, Exception):
            return None


class LogConsumer(AsyncWebsocketConsumer):
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
