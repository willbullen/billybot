"""WebSocket URL routing for the dashboard."""

from django.urls import path

from core import consumers

websocket_urlpatterns = [
    path("ws/ros2/", consumers.Ros2BridgeConsumer.as_asgi()),
    path("ws/telemetry/", consumers.TelemetryConsumer.as_asgi()),
    path("ws/chat/", consumers.ChatConsumer.as_asgi()),
    path("ws/camera/", consumers.CameraConsumer.as_asgi()),
    path("ws/map/", consumers.MapConsumer.as_asgi()),
    path("ws/behaviors/", consumers.BehaviorConsumer.as_asgi()),
    path("ws/logs/", consumers.LogConsumer.as_asgi()),
]
