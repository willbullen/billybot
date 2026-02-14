#!/usr/bin/env python3
"""
Command Processor Node

Listens for response_cmd messages (was command_transcript) from the AI agents 
and processes them into actionable commands or behavior requests for various 
robot subsystems.

Currently supports:
- Arm preset commands (e.g., "lookup", "tenhut@rightish")
- Behavior commands (e.g., "stop", "move@forward", "turn@left", "follow@target")
- Navigation commands (e.g., "goto@kitchen", "navigate@1.5,2.0", "patrol@waypoints")
- Compound commands with bearings

Author: Karim Virani
Version: 1.0
Date: January 2025
"""

import re
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from typing import Optional, Tuple


class CommandProcessor(Node):
    def __init__(self):
        super().__init__('command_processor')
        
        # Declare parameters for topic names - updated to new naming convention
        self.declare_parameter('command_transcript_topic', 'response_cmd')  # was command_transcript
        self.declare_parameter('arm_preset_topic', '/grunt1/arm_preset')  # Absolute path to arm_preset
        self.declare_parameter('behavior_command_topic', '/grunt1/behavior_command')  # Absolute path outside namespace
        
        # Get configured topic names
        command_topic = self.get_parameter('command_transcript_topic').value
        arm_topic = self.get_parameter('arm_preset_topic').value
        behavior_topic = self.get_parameter('behavior_command_topic').value
        
        # Create subscriber for command transcripts (relative - will be in node's namespace)
        self.command_sub = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10
        )
        
        # Create publishers
        # arm_preset needs to be at /grunt1/arm_preset (absolute)
        self.arm_preset_pub = self.create_publisher(
            String,
            arm_topic,
            10
        )
        
        # behavior_command needs to be at /grunt1/behavior_command (absolute)
        self.behavior_command_pub = self.create_publisher(
            String,
            behavior_topic,
            10
        )
        
        # Publisher for wake_cmd control (sleep/wake commands)
        self.wake_cmd_pub = self.create_publisher(
            Bool,
            'wake_cmd',  # Relative topic name for namespacing
            10
        )

        # Publisher for navigation goals
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Define valid command categories
        self.arm_presets = {'bumper', 'tenhut', 'lookup', 'lookout', 'reach', 'pan'}  # pan controls arm joint
        self.behavior_commands = {'stop', 'follow', 'track', 'sleep', 'wake', 'move', 'turn'}
        self.nav_commands = {'navigate', 'goto', 'go', 'nav', 'patrol'}
        self.bearing_presets = {
            'back-left', 'full-left', 'left', 'leftish', 'forward',
            'rightish', 'right', 'full-right', 'back-right', 'back'
        }
        
        # Named locations for voice-to-nav (configurable via parameter)
        self.named_locations = {
            'home': (0.0, 0.0, 0.0),
            'origin': (0.0, 0.0, 0.0),
            'kitchen': (3.0, 1.5, 0.0),
            'bedroom': (-2.0, 3.0, 1.57),
            'living': (2.0, -1.0, 0.0),
            'door': (4.0, 0.0, 3.14),
            'garage': (-3.0, -2.0, -1.57),
            'office': (1.0, 4.0, 0.78),
            'charging': (0.0, 0.0, 0.0),
        }

        # Patrol waypoints (cycle through in order)
        self._patrol_index = 0
        self._patrol_waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57),
        ]

        self.get_logger().info("Command Processor initialized")
        self.get_logger().info(f"  Listening on: {command_topic}")
        self.get_logger().info(f"  Publishing arm presets to: {arm_topic}")
        self.get_logger().info(f"  Publishing behavior commands to: {behavior_topic}")
        self.get_logger().info(f"  Publishing nav goals to: /goal_pose")
        self.get_logger().info(f"  Named locations: {list(self.named_locations.keys())}")
    
    def parse_command(self, command: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        """
        Parse a command string into its components.
        
        Returns:
            (command_type, base_command, modifier)
            - command_type: 'arm', 'behavior', or None
            - base_command: The primary command (e.g., 'lookup', 'move')
            - modifier: Optional modifier (e.g., 'rightish', 'forward', 'betty')
        """
        command = command.strip().lower()
        
        # Check for compound command with @ separator
        if '@' in command:
            base, modifier = command.split('@', 1)
            base = base.strip()
            modifier = modifier.strip()
        else:
            base = command
            modifier = None
        
        # Determine command type
        if base in self.arm_presets:
            # Pan command requires a modifier (bearing)
            if base == 'pan' and not modifier:
                self.get_logger().warn("Pan command requires a bearing modifier (e.g., pan@left)")
                return None, None, None
            return 'arm', base, modifier
        elif base in self.behavior_commands:
            # Some behavior commands require modifiers
            if base in {'follow', 'track', 'move', 'turn'} and not modifier:
                self.get_logger().warn(f"Behavior command '{base}' requires a modifier (e.g., {base}@forward)")
                return None, None, None
            return 'behavior', base, modifier
        elif base in self.nav_commands:
            # Nav commands: goto@kitchen, navigate@1.5,2.0, patrol
            if base == 'patrol':
                return 'nav', base, modifier  # modifier optional for patrol
            if not modifier:
                self.get_logger().warn(f"Nav command '{base}' requires a target (e.g., {base}@kitchen or {base}@1.5,2.0)")
                return None, None, None
            return 'nav', base, modifier
        elif base in self.bearing_presets:
            # Standalone bearing is interpreted as pan@bearing (arm command)
            return 'arm', 'pan', base
        else:
            self.get_logger().warn(f"Unrecognized command: '{command}'")
            return None, None, None
    
    def validate_modifier(self, command_type: str, base_command: str, modifier: str) -> bool:
        """Validate that a modifier is appropriate for the command."""
        if not modifier:
            return True
            
        if command_type == 'arm':
            # Arm commands can have bearing modifiers
            return modifier in self.bearing_presets or self._is_valid_angle(modifier)
        elif command_type == 'behavior':
            if base_command in {'move', 'turn'}:
                return modifier in self.bearing_presets or self._is_valid_angle(modifier)
            elif base_command in {'follow', 'track'}:
                return True
        elif command_type == 'nav':
            # Nav accepts named locations, coordinate pairs (x,y or x,y,yaw), or "waypoints"
            if modifier in self.named_locations:
                return True
            if modifier == 'waypoints':
                return True
            if self._is_valid_coordinates(modifier):
                return True
            return False

        return False
    
    def _is_valid_angle(self, value: str) -> bool:
        """Check if a string represents a valid angle value."""
        try:
            float(value)
            return True
        except ValueError:
            return False

    def _is_valid_coordinates(self, value: str) -> bool:
        """Check if a string is a valid coordinate pair: x,y or x,y,yaw."""
        parts = value.split(',')
        if len(parts) not in (2, 3):
            return False
        try:
            for p in parts:
                float(p.strip())
            return True
        except ValueError:
            return False

    def _parse_coordinates(self, value: str) -> Tuple[float, float, float]:
        """Parse coordinate string into (x, y, yaw)."""
        parts = [float(p.strip()) for p in value.split(',')]
        if len(parts) == 2:
            return parts[0], parts[1], 0.0
        return parts[0], parts[1], parts[2]
    
    def command_callback(self, msg: String):
        """Process incoming command transcript messages."""
        command = msg.data.strip()
        
        if not command:
            return
            
        self.get_logger().info(f"Received command: '{command}'")
        
        # Parse the command
        command_type, base_command, modifier = self.parse_command(command)
        
        if not command_type:
            return
        
        # Validate modifier if present
        if modifier and not self.validate_modifier(command_type, base_command, modifier):
            self.get_logger().warn(f"Invalid modifier '{modifier}' for command '{base_command}'")
            return
        
        # Route to appropriate handler
        if command_type == 'arm':
            self.handle_arm_command(base_command, modifier)
        elif command_type == 'behavior':
            self.handle_behavior_command(base_command, modifier)
        elif command_type == 'nav':
            self.handle_nav_command(base_command, modifier)
    
    def handle_arm_command(self, preset: str, bearing: Optional[str]):
        """Publish arm preset commands."""
        if bearing:
            command = f"{preset}@{bearing}"
        else:
            command = preset
            
        msg = String()
        msg.data = command
        self.arm_preset_pub.publish(msg)
        self.get_logger().info(f"Published arm preset: '{command}'")
    
    def handle_behavior_command(self, command: str, modifier: Optional[str]):
        """Publish behavior commands."""
        # Handle special sleep/wake commands for VAD control
        if command == 'sleep':
            # Send wake_cmd false to put VAD to sleep
            wake_msg = Bool()
            wake_msg.data = False
            self.wake_cmd_pub.publish(wake_msg)
            self.get_logger().info("Sleep command - putting voice detection to sleep")
        elif command == 'wake':
            # Send wake_cmd true to wake up VAD
            wake_msg = Bool()
            wake_msg.data = True
            self.wake_cmd_pub.publish(wake_msg)
            self.get_logger().info("Wake command - waking up voice detection")
        
        # Also publish the behavior command normally for other systems
        if modifier:
            full_command = f"{command}@{modifier}"
        else:
            full_command = command
            
        msg = String()
        msg.data = full_command
        self.behavior_command_pub.publish(msg)
        self.get_logger().info(f"Published behavior command: '{full_command}'")

    def handle_nav_command(self, command: str, modifier: Optional[str]):
        """Process navigation commands and publish PoseStamped goals."""
        if command == 'patrol':
            # Patrol cycles through predefined waypoints
            if modifier == 'waypoints' or not modifier:
                wp = self._patrol_waypoints[self._patrol_index % len(self._patrol_waypoints)]
                self._patrol_index += 1
                x, y, yaw = wp
                self.get_logger().info(f"Patrol waypoint {self._patrol_index}: ({x}, {y}, {yaw})")
            elif modifier in self.named_locations:
                x, y, yaw = self.named_locations[modifier]
            elif self._is_valid_coordinates(modifier):
                x, y, yaw = self._parse_coordinates(modifier)
            else:
                self.get_logger().warn(f"Invalid patrol target: '{modifier}'")
                return
        elif modifier in self.named_locations:
            x, y, yaw = self.named_locations[modifier]
            self.get_logger().info(f"Navigating to named location '{modifier}': ({x}, {y}, {yaw})")
        elif self._is_valid_coordinates(modifier):
            x, y, yaw = self._parse_coordinates(modifier)
            self.get_logger().info(f"Navigating to coordinates: ({x}, {y}, {yaw})")
        else:
            self.get_logger().warn(f"Unknown navigation target: '{modifier}'")
            return

        self._publish_nav_goal(x, y, yaw)

    def _publish_nav_goal(self, x: float, y: float, yaw: float):
        """Construct and publish a PoseStamped message to /goal_pose."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        # Convert yaw to quaternion (rotation around Z axis)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_goal_pub.publish(goal)
        self.get_logger().info(f"Published nav goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()