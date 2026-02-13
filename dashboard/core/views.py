"""Views for the BillyBot dashboard."""

import asyncio
import json
import os
import shlex
import subprocess

from django.conf import settings
from django.http import JsonResponse
from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_GET, require_POST


# ---------------------------------------------------------------------------
# Page views
# ---------------------------------------------------------------------------


def dashboard(request):
    return render(request, "core/dashboard.html")


def control(request):
    return render(request, "core/control.html")


def telemetry(request):
    return render(request, "core/telemetry.html")


def topics(request):
    return render(request, "core/topics.html")


def nodes(request):
    return render(request, "core/nodes.html")


def chat(request):
    return render(request, "core/chat.html")


def logs(request):
    return render(request, "core/logs.html")


def vision(request):
    return render(request, "core/vision.html")


def settings_view(request):
    return render(request, "core/settings.html", {
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ros2_bridge_ws": settings.ROS2_BRIDGE_WS,
    })


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

CONTAINER = os.environ.get("ROS2_CONTAINER_NAME", "billybot-ros2")
NANOBOT_CONTAINER = os.environ.get("NANOBOT_CONTAINER_NAME", "billybot-nanobot")


def _run_sync(cmd, timeout=30):
    """Run a shell command synchronously and return output."""
    import subprocess

    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        output = result.stdout.strip()
        if not output and result.stderr.strip():
            output = result.stderr.strip()
        return output or "(no output)"
    except subprocess.TimeoutExpired:
        return f"Error: timed out after {timeout}s"
    except Exception as e:
        return f"Error: {e}"


def _ros2_exec(cmd, timeout=30):
    """Run a ROS 2 command inside the container."""
    escaped = cmd.replace("'", "'\\''")
    full = (
        f"docker exec {shlex.quote(CONTAINER)} bash -c "
        f"'source /opt/ros/humble/setup.bash && "
        f"source /ros2_ws/install/setup.bash 2>/dev/null; "
        f"{escaped}'"
    )
    return _run_sync(full, timeout=timeout)


# ---------------------------------------------------------------------------
# API views: ROS 2
# ---------------------------------------------------------------------------


@require_GET
def api_ros2_topics(request):
    """List all ROS 2 topics."""
    output = _ros2_exec("ros2 topic list -v")
    topics = []
    for line in output.strip().split("\n"):
        line = line.strip()
        if line.startswith("/"):
            parts = line.split()
            topics.append({
                "name": parts[0],
                "type": parts[1] if len(parts) > 1 else "",
            })
    return JsonResponse({"topics": topics, "raw": output})


@require_GET
def api_ros2_nodes(request):
    """List all ROS 2 nodes."""
    output = _ros2_exec("ros2 node list")
    nodes = [n.strip() for n in output.strip().split("\n") if n.strip().startswith("/")]
    return JsonResponse({"nodes": nodes, "raw": output})


@require_GET
def api_ros2_topic_echo(request):
    """Echo messages from a topic or get topic info."""
    topic = request.GET.get("topic", "")
    if not topic:
        return JsonResponse({"error": "topic parameter required"}, status=400)

    info_mode = request.GET.get("info") == "true"
    if info_mode:
        output = _ros2_exec(f"ros2 topic info {shlex.quote(topic)} -v", timeout=10)
        # Parse publishers/subscribers counts
        info = {"type": "", "publishers": 0, "subscribers": 0}
        for line in output.split("\n"):
            if "Type:" in line:
                info["type"] = line.split("Type:")[-1].strip()
            elif "Publisher count:" in line:
                try:
                    info["publishers"] = int(line.split(":")[-1].strip())
                except ValueError:
                    pass
            elif "Subscription count:" in line:
                try:
                    info["subscribers"] = int(line.split(":")[-1].strip())
                except ValueError:
                    pass
        return JsonResponse({"info": info, "raw": output})

    count = min(int(request.GET.get("count", 1)), 10)
    output = _ros2_exec(
        f"ros2 topic echo {shlex.quote(topic)} --max-count {count}",
        timeout=15,
    )
    return JsonResponse({"output": output})


@require_GET
def api_ros2_node_info(request):
    """Get info about a ROS 2 node."""
    node = request.GET.get("node", "")
    if not node:
        return JsonResponse({"error": "node parameter required"}, status=400)

    output = _ros2_exec(f"ros2 node info {shlex.quote(node)}")

    # Also get parameters
    params_output = _ros2_exec(f"ros2 param list {shlex.quote(node)}")
    params = [p.strip() for p in params_output.split("\n") if p.strip() and not p.startswith("Error")]

    return JsonResponse({"output": output, "params": params})


@csrf_exempt
@require_POST
def api_ros2_exec(request):
    """Execute an arbitrary command inside the ROS 2 container."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    command = data.get("command", "")
    if not command:
        return JsonResponse({"error": "command required"}, status=400)

    output = _ros2_exec(command, timeout=60)
    return JsonResponse({"output": output})


@csrf_exempt
@require_POST
def api_ros2_param_set(request):
    """Set a parameter on a ROS 2 node."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    node = data.get("node", "")
    param = data.get("param", "")
    value = data.get("value", "")
    if not all([node, param, value]):
        return JsonResponse({"error": "node, param, and value required"}, status=400)

    output = _ros2_exec(
        f"ros2 param set {shlex.quote(node)} {shlex.quote(param)} {shlex.quote(value)}"
    )
    return JsonResponse({"result": output})


# ---------------------------------------------------------------------------
# API views: Docker
# ---------------------------------------------------------------------------


@require_GET
def api_docker_containers(request):
    """Get status of BillyBot containers."""
    output = _run_sync(
        "docker ps -a --filter name=billybot --format '{{.Names}}|{{.Status}}|{{.State}}'",
        timeout=10,
    )
    containers = []
    for line in output.strip().split("\n"):
        if "|" not in line:
            continue
        parts = line.split("|")
        name = parts[0].strip()
        status = parts[1].strip() if len(parts) > 1 else ""
        state = parts[2].strip() if len(parts) > 2 else ""
        label_map = {
            "billybot-ros2": "ROS 2",
            "billybot-nanobot": "Nanobot",
            "billybot-dashboard": "Dashboard",
            "billybot-redis": "Redis",
        }
        containers.append({
            "name": name,
            "label": label_map.get(name, name),
            "status": status,
            "running": state == "running",
        })
    return JsonResponse({"containers": containers})


@require_GET
def api_docker_logs(request):
    """Get container logs."""
    container = request.GET.get("container", "billybot-ros2")
    # Only allow billybot containers
    if not container.startswith("billybot-"):
        return JsonResponse({"error": "Invalid container name"}, status=400)

    lines = min(int(request.GET.get("lines", 100)), 2000)
    output = _run_sync(f"docker logs --tail {lines} {shlex.quote(container)}", timeout=10)
    return JsonResponse({"logs": output})


@csrf_exempt
@require_POST
def api_docker_restart(request):
    """Restart a BillyBot container."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    container = data.get("container", "")
    if not container.startswith("billybot-"):
        return JsonResponse({"error": "Invalid container name"}, status=400)

    output = _run_sync(f"docker restart {shlex.quote(container)}", timeout=30)
    return JsonResponse({"result": output})


# ---------------------------------------------------------------------------
# Helpers: Nanobot
# ---------------------------------------------------------------------------


def _nanobot_exec(cmd, timeout=30):
    """Run a command inside the nanobot container."""
    escaped = cmd.replace("'", "'\\''")
    full = f"docker exec {shlex.quote(NANOBOT_CONTAINER)} bash -c '{escaped}'"
    return _run_sync(full, timeout=timeout)


# ---------------------------------------------------------------------------
# API views: Nanobot
# ---------------------------------------------------------------------------


@require_GET
def api_nanobot_config(request):
    """Read nanobot config from the container."""
    output = _nanobot_exec("cat ~/.nanobot/config.json 2>/dev/null || echo '{}'")
    try:
        config = json.loads(output)
    except json.JSONDecodeError:
        config = {}
    return JsonResponse({"config": config, "raw": output})


@csrf_exempt
@require_POST
def api_nanobot_config_update(request):
    """Update nanobot config in the container."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    config_json = json.dumps(data.get("config", {}), indent=2)
    # Write config via heredoc
    safe_json = config_json.replace("'", "'\\''")
    result = _nanobot_exec(
        f"mkdir -p ~/.nanobot && echo '{safe_json}' > ~/.nanobot/config.json",
        timeout=10,
    )
    return JsonResponse({"result": "ok", "output": result})


@require_GET
def api_nanobot_status(request):
    """Get nanobot status (config summary, active model, provider keys)."""
    output = _nanobot_exec("nanobot status 2>&1", timeout=15)
    return JsonResponse({"output": output})


@require_GET
def api_nanobot_cron_list(request):
    """List nanobot cron jobs."""
    output = _nanobot_exec("nanobot cron list 2>&1", timeout=10)
    # Also try reading the raw JSON
    raw = _nanobot_exec("cat ~/.nanobot/cron/jobs.json 2>/dev/null || echo '[]'")
    try:
        jobs = json.loads(raw)
    except json.JSONDecodeError:
        jobs = []
    return JsonResponse({"output": output, "jobs": jobs})


@csrf_exempt
@require_POST
def api_nanobot_cron_manage(request):
    """Add/remove/enable/disable/run a cron job."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    action = data.get("action", "")
    if action not in ("add", "remove", "enable", "disable", "run"):
        return JsonResponse({"error": "Invalid action"}, status=400)

    if action == "add":
        name = data.get("name", "job")
        message = data.get("message", "")
        schedule_type = data.get("schedule_type", "every")
        schedule_value = data.get("schedule_value", "300")
        safe_name = shlex.quote(name)
        safe_msg = shlex.quote(message)
        if schedule_type == "cron":
            cmd = f'nanobot cron add -n {safe_name} -m {safe_msg} --cron {shlex.quote(schedule_value)}'
        else:
            cmd = f'nanobot cron add -n {safe_name} -m {safe_msg} --every {shlex.quote(str(schedule_value))}'
        output = _nanobot_exec(cmd, timeout=10)
    elif action == "remove":
        job_id = shlex.quote(data.get("id", ""))
        output = _nanobot_exec(f"nanobot cron remove {job_id}", timeout=10)
    elif action in ("enable", "disable"):
        job_id = shlex.quote(data.get("id", ""))
        output = _nanobot_exec(f"nanobot cron {action} {job_id}", timeout=10)
    elif action == "run":
        job_id = shlex.quote(data.get("id", ""))
        output = _nanobot_exec(f"nanobot cron run {job_id}", timeout=30)
    else:
        output = ""

    return JsonResponse({"result": output})


@csrf_exempt
@require_POST
def api_nanobot_gateway_restart(request):
    """Restart the nanobot gateway by restarting the container."""
    output = _run_sync(f"docker restart {shlex.quote(NANOBOT_CONTAINER)}", timeout=30)
    return JsonResponse({"result": output})


@require_GET
def api_nanobot_tools(request):
    """List available nanobot tools."""
    # Read workspace TOOLS.md if it exists
    tools_md = _nanobot_exec("cat ~/.nanobot/workspace/TOOLS.md 2>/dev/null || echo ''")
    # List skills
    skills = _nanobot_exec("ls ~/.nanobot/workspace/skills/ 2>/dev/null || echo ''")
    builtin_skills = _nanobot_exec("ls /app/nanobot/skills/ 2>/dev/null || echo ''")
    return JsonResponse({
        "tools_md": tools_md,
        "custom_skills": [s for s in skills.strip().split("\n") if s.strip()],
        "builtin_skills": [s for s in builtin_skills.strip().split("\n") if s.strip()],
    })


@require_GET
def api_nanobot_workspace_file(request):
    """Read a file from the nanobot workspace."""
    filename = request.GET.get("file", "")
    allowed = ("AGENTS.md", "SOUL.md", "USER.md", "TOOLS.md", "IDENTITY.md")
    if filename not in allowed:
        return JsonResponse({"error": f"File must be one of: {', '.join(allowed)}"}, status=400)
    content = _nanobot_exec(f"cat ~/.nanobot/workspace/{shlex.quote(filename)} 2>/dev/null || echo ''")
    return JsonResponse({"filename": filename, "content": content})


@csrf_exempt
@require_POST
def api_nanobot_workspace_file_update(request):
    """Write a file to the nanobot workspace."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    filename = data.get("file", "")
    allowed = ("AGENTS.md", "SOUL.md", "USER.md", "TOOLS.md", "IDENTITY.md")
    if filename not in allowed:
        return JsonResponse({"error": f"File must be one of: {', '.join(allowed)}"}, status=400)

    content = data.get("content", "")
    safe_content = content.replace("'", "'\\''")
    result = _nanobot_exec(
        f"mkdir -p ~/.nanobot/workspace && echo '{safe_content}' > ~/.nanobot/workspace/{filename}",
        timeout=10,
    )
    return JsonResponse({"result": "ok", "output": result})


@require_GET
def api_nanobot_memory(request):
    """Read nanobot memory files."""
    memory = _nanobot_exec("cat ~/.nanobot/workspace/memory/MEMORY.md 2>/dev/null || echo ''")
    # List daily files
    daily_files = _nanobot_exec("ls -1 ~/.nanobot/workspace/memory/*.md 2>/dev/null || echo ''")
    return JsonResponse({
        "memory": memory,
        "daily_files": [f for f in daily_files.strip().split("\n") if f.strip() and "MEMORY.md" not in f],
    })


# ---------------------------------------------------------------------------
# API views: Navigation
# ---------------------------------------------------------------------------


@require_GET
def api_nav_status(request):
    """Get navigation stack status (SLAM mode, active goal, map info)."""
    # Check if nav2 nodes are running
    nodes_output = _ros2_exec("ros2 node list 2>/dev/null", timeout=5)
    nav_nodes = [n.strip() for n in nodes_output.split("\n")
                 if any(k in n for k in ["slam", "controller", "planner", "bt_navigator", "camera"])]

    # Check if camera topic is active
    topics_output = _ros2_exec("ros2 topic list 2>/dev/null", timeout=5)
    camera_active = "/camera/color/compressed" in topics_output
    depth_active = "/camera/depth/image_rect_raw" in topics_output
    odom_active = "/odom" in topics_output
    map_active = "/map" in topics_output

    return JsonResponse({
        "nav_nodes": nav_nodes,
        "camera_active": camera_active,
        "depth_active": depth_active,
        "odom_active": odom_active,
        "map_active": map_active,
    })


@csrf_exempt
@require_POST
def api_nav_goal(request):
    """Send a navigation goal (PoseStamped) to nav2."""
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    yaw = float(data.get("yaw", 0))

    # Convert yaw to quaternion (rotation around Z)
    import math
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    output = _ros2_exec(
        f"ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
        f"'{{header: {{frame_id: \"map\"}}, pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}'",
        timeout=10,
    )
    return JsonResponse({"result": output})


@csrf_exempt
@require_POST
def api_nav_cancel(request):
    """Cancel the current navigation goal."""
    output = _ros2_exec(
        "ros2 topic pub --once /navigate_to_pose/_action/cancel_goal "
        "action_msgs/CancelGoal '{}'",
        timeout=5,
    )
    return JsonResponse({"result": output})


@require_GET
def api_camera_snapshot(request):
    """Get a single camera frame as base64 JPEG (for non-WebSocket clients)."""
    import base64

    cmd = (
        f"docker exec {shlex.quote(CONTAINER)} bash -c "
        "'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; "
        "python3 -c \""
        "import rclpy, base64, sys; "
        "from sensor_msgs.msg import CompressedImage; "
        "rclpy.init(); "
        "node = rclpy.create_node(\\\"_snap\\\"); "
        "msg = [None]; "
        "sub = node.create_subscription(CompressedImage, \\\"/camera/color/compressed\\\", "
        "lambda m: (msg.__setitem__(0, m), rclpy.shutdown()), 1); "
        "import threading; t = threading.Timer(2.0, lambda: rclpy.try_shutdown()); t.start(); "
        "rclpy.spin(node); t.cancel(); node.destroy_node(); "
        "sys.stdout.buffer.write(base64.b64encode(msg[0].data) if msg[0] else b\\\"\\\"); "
        "\"'"
    )
    output = _run_sync(cmd, timeout=5)
    if output and output != "(no output)" and not output.startswith("Error"):
        return JsonResponse({"image": output, "format": "jpeg"})
    return JsonResponse({"image": None, "error": "No camera frame available"})
