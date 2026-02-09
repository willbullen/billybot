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
]
