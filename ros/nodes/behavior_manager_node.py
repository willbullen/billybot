#!/usr/bin/env python3
"""
Behavior Manager Node

Implements high-level autonomous behaviors as finite state machines:
- Patrol: cycle through waypoints continuously
- Search: spiral/grid search pattern looking for objects
- Follow: track and follow a detected target
- Guard: monitor area, alert on changes

Subscribes to /grunt1/behavior_command for behavior triggers from
command_processor. Publishes nav goals to /goal_pose and manages
arm/camera pointing.

Author: BillyBot
Version: 1.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from typing import Optional


class BehaviorState:
    IDLE = 'idle'
    PATROL = 'patrol'
    SEARCH = 'search'
    FOLLOW = 'follow'
    GUARD = 'guard'
    MOVING = 'moving'


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # Parameters
        self.declare_parameter('patrol_loop', True)
        self.declare_parameter('patrol_speed', 0.3)
        self.declare_parameter('search_radius', 3.0)
        self.declare_parameter('search_step', 1.0)
        self.declare_parameter('follow_distance', 1.5)
        self.declare_parameter('guard_alert_threshold', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)

        self._patrol_loop = self.get_parameter('patrol_loop').value
        self._patrol_speed = self.get_parameter('patrol_speed').value
        self._search_radius = self.get_parameter('search_radius').value
        self._search_step = self.get_parameter('search_step').value
        self._follow_distance = self.get_parameter('follow_distance').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self._state = BehaviorState.IDLE
        self._prev_state = BehaviorState.IDLE
        self._state_start_time = time.time()

        # Patrol state
        self._patrol_waypoints = []
        self._patrol_index = 0

        # Search state
        self._search_origin = (0.0, 0.0)
        self._search_ring = 0
        self._search_angle = 0.0

        # Follow state
        self._follow_target_x = 0.0
        self._follow_target_y = 0.0
        self._follow_target_label = ''

        # Guard state
        self._guard_position = (0.0, 0.0, 0.0)
        self._guard_scan_index = 0
        self._guard_bearings = [0.0, 1.57, 3.14, -1.57]  # N, E, S, W

        # Robot pose (from odometry)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # Goal tracking
        self._current_goal = None
        self._goal_sent_time = 0.0

        # Subscribers
        self.create_subscription(
            String,
            '/grunt1/behavior_command',
            self._behavior_cmd_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        # Target detection (from vision pipeline)
        self.create_subscription(
            String,
            '/detected_target',
            self._target_callback,
            10
        )

        # Publishers
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._arm_pub = self.create_publisher(String, '/grunt1/arm_preset', 10)
        self._status_pub = self.create_publisher(String, '/behavior_status', 10)

        # State machine timer (5 Hz)
        self._timer = self.create_timer(0.2, self._tick)

        # Status publish timer (1 Hz)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # Default patrol waypoints (square pattern)
        self._default_waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, 1.57),
            (0.0, 2.0, 3.14),
            (0.0, 0.0, -1.57),
        ]

        self.get_logger().info("Behavior Manager initialized (state: IDLE)")

    def _behavior_cmd_callback(self, msg: String):
        """Handle behavior commands from command_processor."""
        cmd = msg.data.strip().lower()

        if cmd == 'stop':
            self._transition(BehaviorState.IDLE)
            self._stop_robot()
            return

        if '@' in cmd:
            base, modifier = cmd.split('@', 1)
        else:
            base = cmd
            modifier = None

        if base == 'patrol':
            self._start_patrol(modifier)
        elif base == 'search':
            self._start_search(modifier)
        elif base == 'follow':
            self._start_follow(modifier or 'person')
        elif base == 'guard':
            self._start_guard()
        else:
            self.get_logger().debug(f"Behavior manager ignoring non-behavior command: '{cmd}'")

    def _odom_callback(self, msg: Odometry):
        """Track robot position from odometry."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self._robot_yaw = math.atan2(siny, cosy)

    def _target_callback(self, msg: String):
        """Handle detected target updates (from vision pipeline)."""
        import json
        try:
            data = json.loads(msg.data)
            if data.get('label') == self._follow_target_label or self._state == BehaviorState.FOLLOW:
                self._follow_target_x = data.get('x', self._robot_x)
                self._follow_target_y = data.get('y', self._robot_y)
        except (json.JSONDecodeError, ValueError):
            pass

    # ------ State transitions ------

    def _transition(self, new_state: str):
        """Transition to a new behavior state."""
        if new_state != self._state:
            self.get_logger().info(f"Behavior: {self._state} -> {new_state}")
            self._prev_state = self._state
            self._state = new_state
            self._state_start_time = time.time()

    def _start_patrol(self, modifier: Optional[str]):
        """Start patrol behavior with default or custom waypoints."""
        self._patrol_waypoints = list(self._default_waypoints)
        self._patrol_index = 0
        self._transition(BehaviorState.PATROL)
        self.get_logger().info(f"Patrol started with {len(self._patrol_waypoints)} waypoints")
        # Point camera forward for patrol
        self._publish_arm_preset('lookout')

    def _start_search(self, modifier: Optional[str]):
        """Start spiral search from current position."""
        self._search_origin = (self._robot_x, self._robot_y)
        self._search_ring = 1
        self._search_angle = 0.0
        self._transition(BehaviorState.SEARCH)
        self.get_logger().info(f"Search started from ({self._robot_x:.1f}, {self._robot_y:.1f})")
        # Camera in lookup mode for searching
        self._publish_arm_preset('lookup')

    def _start_follow(self, target_label: str):
        """Start following a target."""
        self._follow_target_label = target_label
        self._follow_target_x = self._robot_x + 1.0  # Initial forward
        self._follow_target_y = self._robot_y
        self._transition(BehaviorState.FOLLOW)
        self.get_logger().info(f"Following target: '{target_label}'")
        # Camera forward for tracking
        self._publish_arm_preset('lookout')

    def _start_guard(self):
        """Start guard mode at current position."""
        self._guard_position = (self._robot_x, self._robot_y, self._robot_yaw)
        self._guard_scan_index = 0
        self._transition(BehaviorState.GUARD)
        self.get_logger().info(
            f"Guard mode at ({self._robot_x:.1f}, {self._robot_y:.1f})"
        )
        self._publish_arm_preset('lookup')

    # ------ State machine tick ------

    def _tick(self):
        """Main state machine tick (5 Hz)."""
        if self._state == BehaviorState.IDLE:
            return
        elif self._state == BehaviorState.PATROL:
            self._tick_patrol()
        elif self._state == BehaviorState.SEARCH:
            self._tick_search()
        elif self._state == BehaviorState.FOLLOW:
            self._tick_follow()
        elif self._state == BehaviorState.GUARD:
            self._tick_guard()

    def _tick_patrol(self):
        """Patrol: navigate to each waypoint in sequence."""
        if not self._patrol_waypoints:
            self._transition(BehaviorState.IDLE)
            return

        # Check if we reached the current waypoint
        wp = self._patrol_waypoints[self._patrol_index]
        dist = self._distance_to(wp[0], wp[1])

        if self._current_goal is None or dist < self._goal_tolerance:
            # Move to next waypoint
            if self._current_goal is not None:
                self._patrol_index += 1
                if self._patrol_index >= len(self._patrol_waypoints):
                    if self._patrol_loop:
                        self._patrol_index = 0
                        self.get_logger().info("Patrol: loop restart")
                    else:
                        self.get_logger().info("Patrol: complete")
                        self._transition(BehaviorState.IDLE)
                        return

            wp = self._patrol_waypoints[self._patrol_index]
            self.get_logger().info(
                f"Patrol: waypoint {self._patrol_index + 1}/{len(self._patrol_waypoints)} "
                f"-> ({wp[0]:.1f}, {wp[1]:.1f})"
            )
            self._send_nav_goal(wp[0], wp[1], wp[2])

        # Timeout check: if stuck at a waypoint for >30s, skip it
        if time.time() - self._goal_sent_time > 30.0 and self._current_goal is not None:
            self.get_logger().warn(f"Patrol: timeout at waypoint {self._patrol_index}, skipping")
            self._patrol_index += 1
            self._current_goal = None

    def _tick_search(self):
        """Search: expanding spiral pattern."""
        if self._search_ring * self._search_step > self._search_radius:
            self.get_logger().info("Search: area covered, returning to origin")
            self._send_nav_goal(
                self._search_origin[0], self._search_origin[1], 0.0
            )
            self._transition(BehaviorState.IDLE)
            return

        # Check if reached current search point
        if self._current_goal is None or self._at_goal():
            # Generate next search point on the spiral
            r = self._search_ring * self._search_step
            x = self._search_origin[0] + r * math.cos(self._search_angle)
            y = self._search_origin[1] + r * math.sin(self._search_angle)
            yaw = self._search_angle

            self.get_logger().info(
                f"Search: ring {self._search_ring}, "
                f"angle {math.degrees(self._search_angle):.0f}deg -> ({x:.1f}, {y:.1f})"
            )
            self._send_nav_goal(x, y, yaw)

            # Advance spiral: 4 points per ring
            self._search_angle += math.pi / 2
            if self._search_angle >= 2 * math.pi:
                self._search_angle = 0.0
                self._search_ring += 1

        # Timeout: skip if stuck
        if time.time() - self._goal_sent_time > 20.0 and self._current_goal is not None:
            self._current_goal = None

    def _tick_follow(self):
        """Follow: move toward the target, maintaining distance."""
        dx = self._follow_target_x - self._robot_x
        dy = self._follow_target_y - self._robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self._follow_distance:
            # Close enough, just rotate to face target
            target_yaw = math.atan2(dy, dx)
            twist = Twist()
            yaw_err = self._normalize_angle(target_yaw - self._robot_yaw)
            twist.angular.z = max(-1.0, min(1.0, yaw_err * 2.0))
            self._cmd_vel_pub.publish(twist)
        elif dist > self._follow_distance * 1.5:
            # Too far, navigate toward target
            target_yaw = math.atan2(dy, dx)
            goal_x = self._follow_target_x - self._follow_distance * math.cos(target_yaw)
            goal_y = self._follow_target_y - self._follow_distance * math.sin(target_yaw)
            self._send_nav_goal(goal_x, goal_y, target_yaw)

    def _tick_guard(self):
        """Guard: sit at position, scan surroundings periodically."""
        elapsed = time.time() - self._state_start_time

        # Every 10 seconds, look in a new direction
        scan_period = 10.0
        expected_index = int(elapsed / scan_period) % len(self._guard_bearings)

        if expected_index != self._guard_scan_index:
            self._guard_scan_index = expected_index
            bearing = self._guard_bearings[self._guard_scan_index]

            # Map bearing to arm pan preset name
            bearing_names = ['forward', 'right', 'back', 'left']
            bearing_name = bearing_names[self._guard_scan_index % len(bearing_names)]
            self._publish_arm_preset(f'pan@{bearing_name}')
            self.get_logger().info(f"Guard: scanning {bearing_name}")

        # Check if we've drifted from guard position
        dist = self._distance_to(self._guard_position[0], self._guard_position[1])
        if dist > 0.5:
            self.get_logger().info("Guard: returning to guard position")
            self._send_nav_goal(*self._guard_position)

    # ------ Helpers ------

    def _send_nav_goal(self, x: float, y: float, yaw: float):
        """Publish a nav goal to /goal_pose."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_pub.publish(goal)
        self._current_goal = (x, y, yaw)
        self._goal_sent_time = time.time()

    def _stop_robot(self):
        """Send zero velocity and clear current goal."""
        twist = Twist()
        self._cmd_vel_pub.publish(twist)
        self._current_goal = None
        self.get_logger().info("Behavior: STOP - robot halted")

    def _publish_arm_preset(self, preset: str):
        """Send an arm preset command."""
        msg = String()
        msg.data = preset
        self._arm_pub.publish(msg)

    def _distance_to(self, x: float, y: float) -> float:
        dx = x - self._robot_x
        dy = y - self._robot_y
        return math.sqrt(dx * dx + dy * dy)

    def _at_goal(self) -> bool:
        if self._current_goal is None:
            return True
        return self._distance_to(self._current_goal[0], self._current_goal[1]) < self._goal_tolerance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_status(self):
        """Publish current behavior status at 1 Hz."""
        import json
        status = {
            'state': self._state,
            'time_in_state': round(time.time() - self._state_start_time, 1),
            'robot_x': round(self._robot_x, 2),
            'robot_y': round(self._robot_y, 2),
            'robot_yaw': round(self._robot_yaw, 2),
        }

        if self._state == BehaviorState.PATROL:
            status['patrol_index'] = self._patrol_index
            status['patrol_total'] = len(self._patrol_waypoints)
        elif self._state == BehaviorState.SEARCH:
            status['search_ring'] = self._search_ring
        elif self._state == BehaviorState.FOLLOW:
            status['follow_target'] = self._follow_target_label
            status['follow_distance'] = round(
                self._distance_to(self._follow_target_x, self._follow_target_y), 2
            )
        elif self._state == BehaviorState.GUARD:
            status['guard_scan_direction'] = self._guard_scan_index

        if self._current_goal:
            status['current_goal'] = {
                'x': round(self._current_goal[0], 2),
                'y': round(self._current_goal[1], 2),
            }

        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
