"""Tests for BillyBot dashboard core app."""

from django.test import TestCase, RequestFactory
from django.urls import reverse, resolve


class URLTests(TestCase):
    """Verify all URL routes resolve correctly."""

    def test_dashboard_url(self):
        url = reverse("dashboard")
        self.assertEqual(url, "/")

    def test_control_url(self):
        url = reverse("control")
        self.assertEqual(url, "/control/")

    def test_telemetry_url(self):
        url = reverse("telemetry")
        self.assertEqual(url, "/telemetry/")

    def test_topics_url(self):
        url = reverse("topics")
        self.assertEqual(url, "/topics/")

    def test_nodes_url(self):
        url = reverse("nodes")
        self.assertEqual(url, "/nodes/")

    def test_chat_url(self):
        url = reverse("chat")
        self.assertEqual(url, "/chat/")

    def test_logs_url(self):
        url = reverse("logs")
        self.assertEqual(url, "/logs/")

    def test_vision_url(self):
        url = reverse("vision")
        self.assertEqual(url, "/vision/")

    def test_settings_url(self):
        url = reverse("settings")
        self.assertEqual(url, "/settings/")


class PageViewTests(TestCase):
    """Verify all page views render successfully."""

    def test_dashboard_page(self):
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertContains(response, "BillyBot")

    def test_control_page(self):
        response = self.client.get("/control/")
        self.assertEqual(response.status_code, 200)

    def test_telemetry_page(self):
        response = self.client.get("/telemetry/")
        self.assertEqual(response.status_code, 200)

    def test_topics_page(self):
        response = self.client.get("/topics/")
        self.assertEqual(response.status_code, 200)

    def test_nodes_page(self):
        response = self.client.get("/nodes/")
        self.assertEqual(response.status_code, 200)

    def test_chat_page(self):
        response = self.client.get("/chat/")
        self.assertEqual(response.status_code, 200)

    def test_logs_page(self):
        response = self.client.get("/logs/")
        self.assertEqual(response.status_code, 200)

    def test_vision_page(self):
        response = self.client.get("/vision/")
        self.assertEqual(response.status_code, 200)

    def test_settings_page(self):
        response = self.client.get("/settings/")
        self.assertEqual(response.status_code, 200)


class APIRouteTests(TestCase):
    """Verify API endpoints exist and return JSON."""

    def test_ros2_topics_api(self):
        response = self.client.get("/api/ros2/topics/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response["Content-Type"], "application/json")

    def test_ros2_nodes_api(self):
        response = self.client.get("/api/ros2/nodes/")
        self.assertEqual(response.status_code, 200)

    def test_docker_containers_api(self):
        response = self.client.get("/api/docker/containers/")
        self.assertEqual(response.status_code, 200)

    def test_nav_status_api(self):
        response = self.client.get("/api/nav/status/")
        self.assertEqual(response.status_code, 200)

    def test_camera_snapshot_api(self):
        response = self.client.get("/api/camera/snapshot/")
        self.assertEqual(response.status_code, 200)

    def test_topic_echo_requires_param(self):
        response = self.client.get("/api/ros2/topic-echo/")
        self.assertEqual(response.status_code, 400)

    def test_node_info_requires_param(self):
        response = self.client.get("/api/ros2/node-info/")
        self.assertEqual(response.status_code, 400)

    def test_post_endpoints_reject_get(self):
        """POST-only endpoints should reject GET requests."""
        post_urls = [
            "/api/ros2/exec/",
            "/api/ros2/param-set/",
            "/api/docker/restart/",
            "/api/nav/goal/",
            "/api/nav/cancel/",
        ]
        for url in post_urls:
            response = self.client.get(url)
            self.assertEqual(response.status_code, 405, f"GET {url} should be 405")


class HealthCheckTests(TestCase):
    """Test the health check endpoint."""

    def test_health_endpoint(self):
        response = self.client.get("/api/health/")
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data["status"], "ok")
        self.assertIn("uptime", data)
