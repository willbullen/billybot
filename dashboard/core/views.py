"""Views for the BillyBot dashboard."""

import asyncio
import json
import os
import shlex

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


def settings_view(request):
    return render(request, "core/settings.html", {
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ros2_bridge_ws": settings.ROS2_BRIDGE_WS,
    })


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

CONTAINER = os.environ.get("ROS2_CONTAINER_NAME", "billybot-ros2")


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
