#!/usr/bin/env python3
"""
Spatial Memory Node

Maintains a persistent spatial memory of objects and locations:
- "The keys are on the desk" -> stores object-location association
- "Where did I put the keys?" -> queries spatial memory
- "What's near the kitchen?" -> proximity queries
- Scene change detection between visits

Subscribes to:
- /spatial_memory/store (String JSON) - store new memory
- /spatial_memory/query (String) - query memory
- /odom (Odometry) - track robot position

Publishes:
- /spatial_memory/result (String JSON) - query results
- /spatial_memory/status (String JSON) - memory status

Author: BillyBot
Version: 1.0
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class SpatialMemory:
    """In-memory spatial database with JSON persistence."""

    def __init__(self, persistence_file: str = '/tmp/billybot_spatial_memory.json'):
        self._file = persistence_file
        self._objects = {}  # label -> list of observations
        self._locations = {}  # named_location -> (x, y, description)
        self._scenes = {}  # location_key -> list of scene snapshots
        self._load()

    def store_object(self, label: str, x: float, y: float, description: str = '', confidence: float = 1.0):
        """Store an object observation at a location."""
        if label not in self._objects:
            self._objects[label] = []

        observation = {
            'x': round(x, 2),
            'y': round(y, 2),
            'description': description,
            'confidence': confidence,
            'timestamp': time.time(),
        }

        # Update existing if same location (within 0.5m), else append
        updated = False
        for obs in self._objects[label]:
            if self._distance(obs['x'], obs['y'], x, y) < 0.5:
                obs.update(observation)
                updated = True
                break

        if not updated:
            self._objects[label].append(observation)

        self._save()
        return observation

    def store_location(self, name: str, x: float, y: float, description: str = ''):
        """Name a location for natural language reference."""
        self._locations[name] = {
            'x': round(x, 2),
            'y': round(y, 2),
            'description': description,
            'timestamp': time.time(),
        }
        self._save()

    def query_object(self, label: str):
        """Find where an object was last seen."""
        if label not in self._objects:
            return None
        observations = sorted(self._objects[label], key=lambda o: o['timestamp'], reverse=True)
        return observations[0] if observations else None

    def query_nearby(self, x: float, y: float, radius: float = 2.0):
        """Find all objects within radius of a point."""
        results = []
        for label, observations in self._objects.items():
            for obs in observations:
                dist = self._distance(x, y, obs['x'], obs['y'])
                if dist <= radius:
                    results.append({
                        'label': label,
                        'distance': round(dist, 2),
                        **obs,
                    })
        results.sort(key=lambda r: r['distance'])
        return results

    def query_location(self, name: str):
        """Lookup a named location."""
        return self._locations.get(name)

    def store_scene(self, location_key: str, objects_visible: list):
        """Store a scene snapshot for change detection."""
        snapshot = {
            'objects': objects_visible,
            'timestamp': time.time(),
        }
        if location_key not in self._scenes:
            self._scenes[location_key] = []
        self._scenes[location_key].append(snapshot)
        # Keep last 10 snapshots per location
        self._scenes[location_key] = self._scenes[location_key][-10:]
        self._save()

    def detect_scene_changes(self, location_key: str, current_objects: list):
        """Compare current scene with last snapshot, return changes."""
        if location_key not in self._scenes or not self._scenes[location_key]:
            return {'new': current_objects, 'missing': [], 'unchanged': []}

        last = self._scenes[location_key][-1]
        prev_labels = set(o.get('label', o) if isinstance(o, dict) else o for o in last['objects'])
        curr_labels = set(o.get('label', o) if isinstance(o, dict) else o for o in current_objects)

        return {
            'new': list(curr_labels - prev_labels),
            'missing': list(prev_labels - curr_labels),
            'unchanged': list(curr_labels & prev_labels),
            'time_since_last': round(time.time() - last['timestamp'], 1),
        }

    def get_all_objects(self):
        """Return summary of all known objects."""
        summary = {}
        for label, observations in self._objects.items():
            latest = max(observations, key=lambda o: o['timestamp'])
            summary[label] = {
                'count': len(observations),
                'latest': latest,
            }
        return summary

    def get_all_locations(self):
        """Return all named locations."""
        return dict(self._locations)

    def forget_object(self, label: str):
        """Remove an object from memory."""
        if label in self._objects:
            del self._objects[label]
            self._save()
            return True
        return False

    @staticmethod
    def _distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def _save(self):
        try:
            data = {
                'objects': self._objects,
                'locations': self._locations,
                'scenes': self._scenes,
            }
            with open(self._file, 'w') as f:
                json.dump(data, f, indent=2)
        except OSError:
            pass

    def _load(self):
        try:
            with open(self._file) as f:
                data = json.load(f)
                self._objects = data.get('objects', {})
                self._locations = data.get('locations', {})
                self._scenes = data.get('scenes', {})
        except (OSError, json.JSONDecodeError):
            pass


class SpatialMemoryNode(Node):
    def __init__(self):
        super().__init__('spatial_memory_node')

        self.declare_parameter('persistence_file', '/tmp/billybot_spatial_memory.json')
        self.declare_parameter('auto_name_radius', 1.0)

        persistence = self.get_parameter('persistence_file').value
        self._memory = SpatialMemory(persistence)
        self._auto_name_radius = self.get_parameter('auto_name_radius').value

        # Robot position
        self._robot_x = 0.0
        self._robot_y = 0.0

        # Subscribers
        self.create_subscription(String, '/spatial_memory/store', self._store_callback, 10)
        self.create_subscription(String, '/spatial_memory/query', self._query_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        # Publishers
        self._result_pub = self.create_publisher(String, '/spatial_memory/result', 10)
        self._status_pub = self.create_publisher(String, '/spatial_memory/status', 10)

        # Publish status every 5 seconds
        self.create_timer(5.0, self._publish_status)

        obj_count = len(self._memory.get_all_objects())
        loc_count = len(self._memory.get_all_locations())
        self.get_logger().info(
            f"Spatial Memory initialized: {obj_count} objects, {loc_count} locations"
        )

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _store_callback(self, msg: String):
        """Handle store requests.

        Expected JSON format:
        - {"type": "object", "label": "keys", "x": 1.5, "y": 2.0, "description": "on the desk"}
        - {"type": "location", "name": "kitchen", "x": 3.0, "y": 1.5, "description": "Kitchen area"}
        - {"type": "scene", "location": "kitchen", "objects": ["chair", "table", "mug"]}
        - {"type": "object", "label": "keys", "description": "on the desk"}  (uses robot position)
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in store request: {msg.data}")
            return

        store_type = data.get('type', 'object')
        x = data.get('x', self._robot_x)
        y = data.get('y', self._robot_y)

        if store_type == 'object':
            label = data.get('label', '')
            if not label:
                return
            obs = self._memory.store_object(
                label, x, y,
                description=data.get('description', ''),
                confidence=data.get('confidence', 1.0),
            )
            self.get_logger().info(f"Stored: '{label}' at ({obs['x']}, {obs['y']})")

        elif store_type == 'location':
            name = data.get('name', '')
            if not name:
                return
            self._memory.store_location(name, x, y, data.get('description', ''))
            self.get_logger().info(f"Named location: '{name}' at ({x:.1f}, {y:.1f})")

        elif store_type == 'scene':
            location = data.get('location', f"{x:.0f}_{y:.0f}")
            objects = data.get('objects', [])
            self._memory.store_scene(location, objects)
            self.get_logger().info(f"Scene snapshot at '{location}': {len(objects)} objects")

        elif store_type == 'forget':
            label = data.get('label', '')
            if self._memory.forget_object(label):
                self.get_logger().info(f"Forgot: '{label}'")

    def _query_callback(self, msg: String):
        """Handle query requests.

        Expected JSON format:
        - {"type": "find", "label": "keys"}
        - {"type": "nearby", "x": 1.5, "y": 2.0, "radius": 3.0}
        - {"type": "nearby"}  (uses robot position, default 2m radius)
        - {"type": "location", "name": "kitchen"}
        - {"type": "scene_changes", "location": "kitchen", "current_objects": [...]}
        - {"type": "all_objects"}
        - {"type": "all_locations"}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in query: {msg.data}")
            return

        query_type = data.get('type', 'find')
        result = {}

        if query_type == 'find':
            label = data.get('label', '')
            obs = self._memory.query_object(label)
            if obs:
                result = {'found': True, 'label': label, **obs}
            else:
                result = {'found': False, 'label': label}

        elif query_type == 'nearby':
            x = data.get('x', self._robot_x)
            y = data.get('y', self._robot_y)
            radius = data.get('radius', 2.0)
            nearby = self._memory.query_nearby(x, y, radius)
            result = {'nearby': nearby, 'center': {'x': round(x, 2), 'y': round(y, 2)}, 'radius': radius}

        elif query_type == 'location':
            name = data.get('name', '')
            loc = self._memory.query_location(name)
            if loc:
                result = {'found': True, 'name': name, **loc}
            else:
                result = {'found': False, 'name': name}

        elif query_type == 'scene_changes':
            location = data.get('location', '')
            current = data.get('current_objects', [])
            changes = self._memory.detect_scene_changes(location, current)
            result = {'location': location, 'changes': changes}

        elif query_type == 'all_objects':
            result = {'objects': self._memory.get_all_objects()}

        elif query_type == 'all_locations':
            result = {'locations': self._memory.get_all_locations()}

        # Publish result
        result_msg = String()
        result_msg.data = json.dumps(result)
        self._result_pub.publish(result_msg)

    def _publish_status(self):
        """Publish memory status periodically."""
        objects = self._memory.get_all_objects()
        locations = self._memory.get_all_locations()
        status = {
            'object_count': len(objects),
            'location_count': len(locations),
            'robot_x': round(self._robot_x, 2),
            'robot_y': round(self._robot_y, 2),
            'objects': list(objects.keys())[:20],  # Top 20 labels
            'locations': list(locations.keys()),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpatialMemoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
