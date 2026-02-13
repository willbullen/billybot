#!/usr/bin/env python3
"""
DDSM Driver Node: /cmd_vel -> motors -> /odom, /motor_feedback, /joint_states, TF.

Subscribes to geometry_msgs/Twist cmd_vel, applies skid-steer kinematics,
sends JSON to DDSM HAT via hardware/ddsm_serial. Publishes odometry (from
encoder feedback), motor_feedback (JSON String), joint_states (wheels), and
odom -> base_link TF. Watchdog stops motors after cmd_vel_timeout with no commands.
"""

import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Add package root so hardware can be imported
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware.ddsm_serial import DDSMSerial


class DDSMDriverNode(Node):
    def __init__(self):
        super().__init__("ddsm_driver_node")

        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("simulate", False)
        self.declare_parameter("wheel_separation", 0.30)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 3.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("feedback_rate", 20.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("encoder_counts_per_rev", 4096)
        self.declare_parameter("publish_tf", True)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        sim = self.get_parameter("simulate").value
        if isinstance(sim, str):
            sim = sim.strip().lower() in ("1", "true", "yes")
        self._ddsm = DDSMSerial(port=port, baudrate=baud, simulate=sim)
        if self._ddsm.simulate:
            self.get_logger().info("DDSM driver in simulation mode (no serial)")

        self._wheel_sep = self.get_parameter("wheel_separation").value
        self._wheel_radius = self.get_parameter("wheel_radius").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value
        self._cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self._feedback_rate = self.get_parameter("feedback_rate").value
        self._odom_frame = self.get_parameter("odom_frame_id").value
        self._base_frame = self.get_parameter("base_frame_id").value
        self._counts_per_rev = self.get_parameter("encoder_counts_per_rev").value
        self._publish_tf = self.get_parameter("publish_tf").value

        self._last_cmd_time = 0.0
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_left_encoder = 0
        self._prev_right_encoder = 0
        self._first_feedback = True

        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, qos_cmd)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._motor_feedback_pub = self.create_publisher(String, "/motor_feedback", 10)
        self._joint_states_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._feedback_timer = self.create_timer(1.0 / self._feedback_rate, self._feedback_cb)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info("DDSM driver node started")

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        lx = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        az = max(-self._max_angular, min(self._max_angular, msg.angular.z))
        # Skid-steer: left_vel = (v - omega*L/2)/r, right_vel = (v + omega*L/2)/r
        left_vel = (lx - az * self._wheel_sep / 2) / self._wheel_radius
        right_vel = (lx + az * self._wheel_sep / 2) / self._wheel_radius
        left_rpm = left_vel * 60.0 / (2.0 * math.pi)
        right_rpm = right_vel * 60.0 / (2.0 * math.pi)
        self._ddsm.send_speed(left_rpm, right_rpm)

    def _feedback_cb(self) -> None:
        fb = self._ddsm.read_feedback()
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        # Publish motor_feedback JSON
        payload = {
            "left_speed": fb["left_speed"],
            "right_speed": fb["right_speed"],
            "left_current": fb["left_current"],
            "right_current": fb["right_current"],
            "left_temp": fb["left_temp"],
            "right_temp": fb["right_temp"],
            "left_encoder": fb["left_encoder"],
            "right_encoder": fb["right_encoder"],
        }
        self._motor_feedback_pub.publish(String(data=json.dumps(payload)))

        # Joint states: 4 wheels (FL, FR, RL, RR) - left pair / right pair from diff drive
        counts_to_rad = 2.0 * math.pi / self._counts_per_rev
        left_pos = fb["left_encoder"] * counts_to_rad
        right_pos = fb["right_encoder"] * counts_to_rad
        left_vel = math.radians(fb["left_speed"] * 6)  # rpm -> rad/s approx
        right_vel = math.radians(fb["right_speed"] * 6)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.header.frame_id = ""
        js.name = ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"]
        js.position = [left_pos, right_pos, left_pos, right_pos]
        js.velocity = [left_vel, right_vel, left_vel, right_vel]
        self._joint_states_pub.publish(js)

        # Differential odometry
        if self._first_feedback:
            self._prev_left_encoder = fb["left_encoder"]
            self._prev_right_encoder = fb["right_encoder"]
            self._first_feedback = False
            return
        delta_left = (fb["left_encoder"] - self._prev_left_encoder) * counts_to_rad * self._wheel_radius
        delta_right = (fb["right_encoder"] - self._prev_right_encoder) * counts_to_rad * self._wheel_radius
        self._prev_left_encoder = fb["left_encoder"]
        self._prev_right_encoder = fb["right_encoder"]

        delta_s = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self._wheel_sep
        self._x += delta_s * math.cos(self._theta + delta_theta / 2.0)
        self._y += delta_s * math.sin(self._theta + delta_theta / 2.0)
        self._theta += delta_theta

        # Normalize theta
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        vx = (delta_left + delta_right) / 2.0 / (1.0 / self._feedback_rate) if self._feedback_rate > 0 else 0.0
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = delta_theta * self._feedback_rate if self._feedback_rate > 0 else 0.0
        self._odom_pub.publish(odom)

        if self._tf_broadcaster:
            ts = TransformStamped()
            ts.header.stamp = now.to_msg()
            ts.header.frame_id = self._odom_frame
            ts.child_frame_id = self._base_frame
            ts.transform.translation.x = self._x
            ts.transform.translation.y = self._y
            ts.transform.translation.z = 0.0
            ts.transform.rotation.x = 0.0
            ts.transform.rotation.y = 0.0
            ts.transform.rotation.z = qz
            ts.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(ts)

    def _watchdog_cb(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cmd_time > self._cmd_vel_timeout:
            self._ddsm.stop()

    def destroy_node(self) -> None:
        self._ddsm.stop()
        self._ddsm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DDSMDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
