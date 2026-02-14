"""URL configuration for BillyBot dashboard."""

from django.urls import path

from core import views

urlpatterns = [
    path("", views.dashboard, name="dashboard"),
    path("control/", views.control, name="control"),
    path("telemetry/", views.telemetry, name="telemetry"),
    path("topics/", views.topics, name="topics"),
    path("nodes/", views.nodes, name="nodes"),
    path("chat/", views.chat, name="chat"),
    path("logs/", views.logs, name="logs"),
    path("vision/", views.vision, name="vision"),
    path("behaviors/", views.behaviors, name="behaviors"),
    path("arm/", views.arm, name="arm"),
    path("fleet/", views.fleet, name="fleet"),
    path("settings/", views.settings_view, name="settings"),
    # API endpoints
    path("api/ros2/exec/", views.api_ros2_exec, name="api_ros2_exec"),
    path("api/ros2/topics/", views.api_ros2_topics, name="api_ros2_topics"),
    path("api/ros2/nodes/", views.api_ros2_nodes, name="api_ros2_nodes"),
    path("api/ros2/topic-echo/", views.api_ros2_topic_echo, name="api_ros2_topic_echo"),
    path("api/ros2/node-info/", views.api_ros2_node_info, name="api_ros2_node_info"),
    path("api/ros2/param-set/", views.api_ros2_param_set, name="api_ros2_param_set"),
    path("api/docker/containers/", views.api_docker_containers, name="api_docker_containers"),
    path("api/docker/logs/", views.api_docker_logs, name="api_docker_logs"),
    path("api/docker/restart/", views.api_docker_restart, name="api_docker_restart"),
    # Nanobot API
    path("api/nanobot/config/", views.api_nanobot_config, name="api_nanobot_config"),
    path("api/nanobot/config/update/", views.api_nanobot_config_update, name="api_nanobot_config_update"),
    path("api/nanobot/status/", views.api_nanobot_status, name="api_nanobot_status"),
    path("api/nanobot/cron/", views.api_nanobot_cron_list, name="api_nanobot_cron_list"),
    path("api/nanobot/cron/manage/", views.api_nanobot_cron_manage, name="api_nanobot_cron_manage"),
    path("api/nanobot/gateway/restart/", views.api_nanobot_gateway_restart, name="api_nanobot_gateway_restart"),
    path("api/nanobot/tools/", views.api_nanobot_tools, name="api_nanobot_tools"),
    path("api/nanobot/workspace/", views.api_nanobot_workspace_file, name="api_nanobot_workspace_file"),
    path("api/nanobot/workspace/update/", views.api_nanobot_workspace_file_update, name="api_nanobot_workspace_file_update"),
    path("api/nanobot/memory/", views.api_nanobot_memory, name="api_nanobot_memory"),
    # Navigation API
    path("api/nav/status/", views.api_nav_status, name="api_nav_status"),
    path("api/nav/goal/", views.api_nav_goal, name="api_nav_goal"),
    path("api/nav/cancel/", views.api_nav_cancel, name="api_nav_cancel"),
    path("api/map/save/", views.api_map_save, name="api_map_save"),
    path("api/map/load/", views.api_map_load, name="api_map_load"),
    path("api/map/list/", views.api_map_list, name="api_map_list"),
    path("api/camera/snapshot/", views.api_camera_snapshot, name="api_camera_snapshot"),
    path("api/camera/scan/", views.api_camera_scan, name="api_camera_scan"),
    path("api/camera/status/", views.api_camera_status, name="api_camera_status"),
    # Health & Observability
    path("api/health/", views.api_health, name="api_health"),
    path("api/system/health/", views.api_system_health, name="api_system_health"),
    # Fleet API
    path("api/fleet/robots/", views.api_fleet_robots, name="api_fleet_robots"),
]
