#!/usr/bin/env python3
"""Camera driver node for BillyBot.

Supports Intel RealSense D435i with automatic simulation fallback.
Publishes color images, depth images, compressed color (MJPEG), and camera info.

Usage:
  ros2 run by_your_command camera_driver_node
  ros2 run by_your_command camera_driver_node --ros-args -p simulate:=true
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header
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

        self.simulate = self.get_parameter("simulate").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value
        self.enable_depth = self.get_parameter("enable_depth").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.depth_frame_id = self.get_parameter("depth_frame_id").value

        # Publishers
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage, "/camera/color/compressed", 10
        )
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        if self.enable_depth:
            self.depth_pub = self.create_publisher(
                Image, "/camera/depth/image_rect_raw", 10
            )
        else:
            self.depth_pub = None

        # Initialize camera or simulation
        self.pipeline = None
        self.sim_frame_count = 0

        if not self.simulate and HAS_REALSENSE:
            try:
                self._init_realsense()
                self.get_logger().info(
                    f"RealSense camera initialized ({self.width}x{self.height} @ {self.fps}fps)"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"RealSense init failed ({e}), falling back to simulation"
                )
                self.simulate = True
        elif not self.simulate:
            self.get_logger().warn(
                "pyrealsense2 not installed, falling back to simulation"
            )
            self.simulate = True

        if self.simulate:
            self.get_logger().info(
                f"Camera running in SIMULATION mode ({self.width}x{self.height})"
            )

        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._publish_frames)
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz")

    def _init_realsense(self):
        """Initialize Intel RealSense pipeline."""
        self.pipeline = rs.pipeline()
        config = rs.config()
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
                return

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
            cv2.putText(
                frame, "BILLYBOT CAM", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 190), 1,
            )
            cv2.putText(
                frame, f"SIM MODE", (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 150), 1,
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
