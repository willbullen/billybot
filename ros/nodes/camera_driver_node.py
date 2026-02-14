#!/usr/bin/env python3
"""Camera driver node for BillyBot.

Supports Intel RealSense D435i with automatic simulation fallback.
Publishes color images, depth images, compressed color (MJPEG), and camera info.

Features:
  - Device discovery: scans USB for connected RealSense cameras
  - Hot-plug retry: periodically rescans if no camera found
  - /camera/scan service: trigger manual device scan from dashboard/CLI
  - /camera/status topic: publishes camera connection state

Usage:
  ros2 run by_your_command camera_driver_node
  ros2 run by_your_command camera_driver_node --ros-args -p simulate:=true
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
import time

# Optional imports - graceful fallback
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class CameraDriverNode(Node):
    """ROS 2 camera driver with RealSense support and simulation fallback."""

    def __init__(self):
        super().__init__("camera_driver_node")

        # Parameters
        self.declare_parameter("simulate", False)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("depth_frame_id", "camera_depth_frame")
        self.declare_parameter("scan_interval", 10.0)  # seconds between auto-scans

        self.simulate = self.get_parameter("simulate").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value
        self.enable_depth = self.get_parameter("enable_depth").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.depth_frame_id = self.get_parameter("depth_frame_id").value
        self.scan_interval = self.get_parameter("scan_interval").value

        # Publishers
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, "/camera/color/compressed", 10
        )
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.status_pub = self.create_publisher(String, "/camera/status", 10)

        if self.enable_depth:
            self.depth_pub = self.create_publisher(
                Image, "/camera/depth/image_rect_raw", 10
            )
        else:
            self.depth_pub = None

        # Scan service -- call with: ros2 service call /camera/scan std_srvs/srv/Trigger
        self.scan_srv = self.create_service(Trigger, "/camera/scan", self._scan_callback)

        # Camera state
        self.pipeline = None
        self.sim_frame_count = 0
        self._camera_connected = False
        self._device_info = {}
        self._consecutive_failures = 0
        self._max_failures = 30  # switch to sim after this many frame failures

        # Initial device scan
        if not self.simulate:
            self._scan_and_connect()

        if self.simulate:
            self.get_logger().info(
                f"Camera running in SIMULATION mode ({self.width}x{self.height})"
            )

        # Timer for publishing frames
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._publish_frames)
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz")

        # Timer for auto-rescan when disconnected (only if not forced simulate)
        if not self.get_parameter("simulate").value:
            self.scan_timer = self.create_timer(self.scan_interval, self._auto_scan)

        # Publish initial status
        self._publish_status()

    # ------------------------------------------------------------------
    # Device discovery
    # ------------------------------------------------------------------

    def _discover_devices(self):
        """Scan USB for connected RealSense devices. Returns list of device info dicts."""
        if not HAS_REALSENSE:
            return []

        devices = []
        try:
            ctx = rs.context()
            for dev in ctx.query_devices():
                info = {
                    "name": dev.get_info(rs.camera_info.name) if dev.supports(rs.camera_info.name) else "Unknown",
                    "serial": dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else "",
                    "firmware": dev.get_info(rs.camera_info.firmware_version) if dev.supports(rs.camera_info.firmware_version) else "",
                    "usb_type": dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "",
                    "product_id": dev.get_info(rs.camera_info.product_id) if dev.supports(rs.camera_info.product_id) else "",
                }
                devices.append(info)
                self.get_logger().info(
                    f"Found RealSense: {info['name']} (S/N: {info['serial']}, "
                    f"FW: {info['firmware']}, USB: {info['usb_type']})"
                )
        except Exception as e:
            self.get_logger().warn(f"Device discovery error: {e}")

        if not devices:
            self.get_logger().info("No RealSense devices found on USB bus")

        return devices

    def _scan_and_connect(self):
        """Discover devices and connect to the first available one."""
        if self._camera_connected and self.pipeline:
            self.get_logger().info("Camera already connected, skipping scan")
            return True

        devices = self._discover_devices()
        if not devices:
            if not self.simulate:
                self.get_logger().warn(
                    "No RealSense detected -- running in simulation mode until camera found"
                )
                self.simulate = True
            return False

        # Try to connect to the first device
        self._device_info = devices[0]
        try:
            self._init_realsense(serial=devices[0].get("serial"))
            self._camera_connected = True
            self.simulate = False
            self._consecutive_failures = 0
            self.get_logger().info(
                f"RealSense connected: {devices[0]['name']} "
                f"({self.width}x{self.height} @ {self.fps}fps)"
            )
            self._publish_status()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            self.simulate = True
            self._publish_status()
            return False

    def _auto_scan(self):
        """Periodically try to reconnect if camera is not connected."""
        if self._camera_connected:
            return
        self.get_logger().debug("Auto-scanning for RealSense camera...")
        self._scan_and_connect()

    def _scan_callback(self, request, response):
        """ROS 2 service callback: /camera/scan -- trigger manual device scan."""
        self.get_logger().info("Manual camera scan triggered")

        # If already connected, disconnect first to allow reconnection
        if self._camera_connected:
            self._disconnect()

        devices = self._discover_devices()
        if not devices:
            response.success = False
            response.message = (
                "No RealSense devices found. Check USB connection. "
                "Ensure /dev/bus/usb is mapped in docker-compose.yml."
            )
            self._publish_status()
            return response

        if self._scan_and_connect():
            response.success = True
            info = self._device_info
            response.message = (
                f"Connected to {info.get('name', 'RealSense')} "
                f"(S/N: {info.get('serial', '?')}, USB: {info.get('usb_type', '?')})"
            )
        else:
            response.success = False
            response.message = "Device found but failed to start pipeline"

        return response

    def _disconnect(self):
        """Stop the current pipeline and mark as disconnected."""
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        self._camera_connected = False
        self.simulate = True
        self._publish_status()

    # ------------------------------------------------------------------
    # RealSense initialization
    # ------------------------------------------------------------------

    def _init_realsense(self, serial=None):
        """Initialize Intel RealSense pipeline, optionally targeting a specific serial."""
        self.pipeline = rs.pipeline()
        config = rs.config()

        if serial:
            config.enable_device(serial)

        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
        )
        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
            )
        profile = self.pipeline.start(config)

        # Get camera intrinsics for CameraInfo
        color_profile = profile.get_stream(rs.stream.color)
        intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        self._intrinsics = intrinsics

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_status(self):
        """Publish camera status to /camera/status as JSON."""
        status = {
            "connected": self._camera_connected,
            "simulate": self.simulate,
            "resolution": f"{self.width}x{self.height}",
            "fps": self.fps,
            "publish_rate": self.publish_rate,
            "depth_enabled": self.enable_depth,
            "device": self._device_info if self._camera_connected else {},
            "realsense_sdk": HAS_REALSENSE,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Frame publishing
    # ------------------------------------------------------------------

    def _publish_frames(self):
        """Capture and publish camera frames."""
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame_id

        if self.simulate:
            color_data, depth_data = self._generate_sim_frame()
        else:
            color_data, depth_data = self._capture_realsense()
            if color_data is None:
                self._consecutive_failures += 1
                if self._consecutive_failures >= self._max_failures:
                    self.get_logger().error(
                        f"Camera lost after {self._max_failures} consecutive failures, "
                        "switching to simulation. Call /camera/scan to reconnect."
                    )
                    self._disconnect()
                return

            self._consecutive_failures = 0

        # Publish raw color image
        color_msg = Image()
        color_msg.header = header
        color_msg.height = self.height
        color_msg.width = self.width
        color_msg.encoding = "bgr8"
        color_msg.is_bigendian = False
        color_msg.step = self.width * 3
        color_msg.data = color_data.tobytes()
        self.color_pub.publish(color_msg)

        # Publish compressed JPEG
        if HAS_CV2:
            _, jpeg_data = cv2.imencode(
                ".jpg", color_data, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = "jpeg"
            comp_msg.data = jpeg_data.tobytes()
            self.compressed_pub.publish(comp_msg)

        # Publish depth image
        if self.depth_pub is not None and depth_data is not None:
            depth_header = Header()
            depth_header.stamp = stamp
            depth_header.frame_id = self.depth_frame_id
            depth_msg = Image()
            depth_msg.header = depth_header
            depth_msg.height = self.height
            depth_msg.width = self.width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = self.width * 2
            depth_msg.data = depth_data.tobytes()
            self.depth_pub.publish(depth_msg)

        # Publish camera info
        info_msg = self._build_camera_info(header)
        self.info_pub.publish(info_msg)

    def _capture_realsense(self):
        """Capture frames from RealSense camera."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except Exception as e:
            self.get_logger().warn(f"Frame capture failed: {e}")
            return None, None

        color_frame = frames.get_color_frame()
        if not color_frame:
            return None, None

        color_data = np.asanyarray(color_frame.get_data())
        depth_data = None
        if self.enable_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_data = np.asanyarray(depth_frame.get_data())

        return color_data, depth_data

    def _generate_sim_frame(self):
        """Generate simulated camera frames with test pattern."""
        self.sim_frame_count += 1
        t = self.sim_frame_count / self.publish_rate

        # Create a test pattern with moving elements
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Dark background with grid
        frame[:] = [20, 25, 30]  # Dark blue-gray (BGR)
        for y in range(0, self.height, 40):
            frame[y, :] = [30, 40, 45]
        for x in range(0, self.width, 40):
            frame[:, x] = [30, 40, 45]

        # Moving crosshair
        cx = int(self.width / 2 + 100 * np.sin(t * 0.5))
        cy = int(self.height / 2 + 60 * np.cos(t * 0.7))
        if HAS_CV2:
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 200, 210), 1)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 200, 210), 1)
            cv2.circle(frame, (cx, cy), 15, (0, 200, 210), 1)

            # HUD overlay text
            mode_text = "SIM MODE" if not self._camera_connected else "LIVE"
            mode_color = (0, 140, 150) if not self._camera_connected else (0, 200, 100)
            cv2.putText(
                frame, "BILLYBOT CAM", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 190), 1,
            )
            cv2.putText(
                frame, mode_text, (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, mode_color, 1,
            )

            # Scanning indicator when looking for camera
            if not self._camera_connected:
                scan_text = "SCANNING FOR CAMERA..."
                cv2.putText(
                    frame, scan_text, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 100, 180), 1,
                )

            cv2.putText(
                frame, f"FRAME {self.sim_frame_count}", (10, self.height - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 120, 130), 1,
            )
            ts = time.strftime("%H:%M:%S")
            cv2.putText(
                frame, ts, (self.width - 90, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 150), 1,
            )

            # Simulated depth indicator bar
            bar_y = self.height - 40
            bar_w = int(self.width * 0.6)
            bar_x = (self.width - bar_w) // 2
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 8), (30, 40, 45), -1)
            fill_w = int(bar_w * (0.5 + 0.3 * np.sin(t * 0.3)))
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + 8), (0, 180, 100), -1)

        # Simulated depth data (gradient with noise)
        depth_data = None
        if self.enable_depth:
            depth = np.zeros((self.height, self.width), dtype=np.uint16)
            # Gradient from near (500mm) to far (5000mm) top to bottom
            for row in range(self.height):
                base = int(500 + 4500 * (row / self.height))
                depth[row, :] = base
            # Add some noise
            noise = np.random.randint(-50, 50, (self.height, self.width), dtype=np.int32)
            depth = np.clip(depth.astype(np.int32) + noise, 300, 8000).astype(np.uint16)
            # Simulated object at crosshair
            y1, y2 = max(0, cy - 30), min(self.height, cy + 30)
            x1, x2 = max(0, cx - 30), min(self.width, cx + 30)
            depth[y1:y2, x1:x2] = 800  # Close object
            depth_data = depth

        return frame, depth_data

    def _build_camera_info(self, header):
        """Build CameraInfo message."""
        msg = CameraInfo()
        msg.header = header
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"

        if not self.simulate and hasattr(self, "_intrinsics"):
            i = self._intrinsics
            msg.k = [i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0]
            msg.d = list(i.coeffs)
            msg.p = [i.fx, 0.0, i.ppx, 0.0, 0.0, i.fy, i.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        else:
            # Simulated intrinsics (typical for 640x480)
            fx = fy = 615.0
            cx, cy_cam = self.width / 2.0, self.height / 2.0
            msg.k = [fx, 0.0, cx, 0.0, fy, cy_cam, 0.0, 0.0, 1.0]
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy_cam, 0.0, 0.0, 0.0, 1.0, 0.0]

        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return msg

    def destroy_node(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
