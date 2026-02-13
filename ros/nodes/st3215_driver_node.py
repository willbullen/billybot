#!/usr/bin/env python3
"""
ST3215 Driver Node: /grunt1/arm_preset + /joint_command -> servos -> /joint_states.

Subscribes to /grunt1/arm_preset (std_msgs/String, format "preset" or "preset@bearing")
and /joint_command (sensor_msgs/JointState). Publishes /joint_states for arm_pan_joint
and arm_tilt_joint. Optional services: /calibrate (Trigger), /set_servo_mode (SetBool).
"""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

# Add package root so hardware can be imported
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware.st3215_serial import ST3215Serial, BEARING_PAN


class ST3215DriverNode(Node):
    def __init__(self):
        super().__init__("st3215_driver_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 1000000)
        self.declare_parameter("simulate", False)
        self.declare_parameter("servo_ids", [1, 2])
        self.declare_parameter("feedback_rate", 10.0)
        self.declare_parameter("preset_bumper", [2048, 3072])
        self.declare_parameter("preset_tenhut", [2048, 2048])
        self.declare_parameter("preset_lookup", [2048, 1024])
        self.declare_parameter("preset_lookout", [2048, 1536])
        self.declare_parameter("preset_reach", [2048, 512])

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        sim = self.get_parameter("simulate").value
        if isinstance(sim, str):
            sim = sim.strip().lower() in ("1", "true", "yes")
        presets = {
            "bumper": self.get_parameter("preset_bumper").value,
            "tenhut": self.get_parameter("preset_tenhut").value,
            "lookup": self.get_parameter("preset_lookup").value,
            "lookout": self.get_parameter("preset_lookout").value,
            "reach": self.get_parameter("preset_reach").value,
        }
        self._st3215 = ST3215Serial(port=port, baudrate=baud, simulate=sim, presets=presets)
        if self._st3215.simulate:
            self.get_logger().info("ST3215 driver in simulation mode (no serial)")

        self._servo_ids = self.get_parameter("servo_ids").value
        self._feedback_rate = self.get_parameter("feedback_rate").value
        self._joint_names = ["arm_pan_joint", "arm_tilt_joint"]

        self._arm_preset_sub = self.create_subscription(
            String, "/grunt1/arm_preset", self._arm_preset_cb, 10
        )
        self._joint_cmd_sub = self.create_subscription(
            JointState, "/joint_command", self._joint_command_cb, 10
        )
        self._joint_states_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._calibrate_srv = self.create_service(Trigger, "calibrate", self._calibrate_cb)
        self._set_servo_mode_srv = self.create_service(SetBool, "set_servo_mode", self._set_servo_mode_cb)

        self._feedback_timer = self.create_timer(1.0 / self._feedback_rate, self._feedback_cb)
        self.get_logger().info("ST3215 driver node started")

    def _parse_preset_bearing(self, data: str) -> tuple:
        """Parse 'preset' or 'preset@bearing' -> (preset_name, bearing or None)."""
        data = (data or "").strip().lower()
        if "@" in data:
            parts = data.split("@", 1)
            return parts[0].strip(), parts[1].strip() or None
        return data, None

    def _arm_preset_cb(self, msg: String) -> None:
        preset_name, bearing = self._parse_preset_bearing(msg.data)
        if not preset_name:
            return
        ok = self._st3215.set_preset(preset_name, bearing)
        if ok:
            self.get_logger().info(f"Arm preset: {preset_name}" + (f" @ {bearing}" if bearing else ""))
        else:
            self.get_logger().warn(f"Unknown preset: {preset_name}")

    def _joint_command_cb(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        for name, pos in zip(msg.name, msg.position):
            if name == "arm_pan_joint" and len(self._servo_ids) >= 1:
                # rad -> 0-4095 (center 2048, ±π -> ±1024)
                pos_raw = int(2048 + pos * 1024 / 3.14159)
                self._st3215.set_position(self._servo_ids[0], pos_raw)
            elif name == "arm_tilt_joint" and len(self._servo_ids) >= 2:
                pos_raw = int(2048 + pos * 1024 / 1.5708)
                self._st3215.set_position(self._servo_ids[1], pos_raw)

    def _feedback_cb(self) -> None:
        now = self.get_clock().now()
        js = JointState()
        js.header.stamp = now.to_msg()
        js.header.frame_id = ""
        js.name = list(self._joint_names)
        pos_pan = self._st3215.read_feedback(self._servo_ids[0])
        pos_tilt = self._st3215.read_feedback(self._servo_ids[1])
        if pos_pan is None:
            pos_pan = 2048
        if pos_tilt is None:
            pos_tilt = 2048
        # 0-4095 -> rad (center 2048)
        js.position = [
            (pos_pan - 2048) * 3.14159 / 1024,
            (pos_tilt - 2048) * 1.5708 / 1024,
        ]
        js.velocity = [0.0, 0.0]
        js.effort = []
        self._joint_states_pub.publish(js)

    def _calibrate_cb(self, request, response):
        """Home all servos to center."""
        for i, jname in enumerate(self._joint_names):
            if i < len(self._servo_ids):
                self._st3215.set_position(self._servo_ids[i], 2048)
        response.success = True
        response.message = "Calibrated to center"
        return response

    def _set_servo_mode_cb(self, request, response):
        """Placeholder: toggle servo/motor mode if hardware supports it."""
        response.success = True
        response.message = "set_servo_mode not implemented on hardware"
        return response

    def destroy_node(self) -> None:
        self._st3215.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ST3215DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
