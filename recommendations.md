# BillyBot - Project Analysis & Recommendations

## Project Overview

BillyBot is a voice-interactive robot platform combining two major systems:

1. **ByYourCommand** (ROS 2) - A real-time voice-controlled robot interaction system with AI-powered conversational agents (OpenAI Realtime, Google Gemini Live), VAD-based speech detection, vision integration, and dual-agent command extraction.

2. **Nanobot** - An ultra-lightweight personal AI assistant framework (~3,500 lines) supporting 10 messaging channels (Telegram, Slack, WhatsApp, Discord, etc.), extensible skills, and multi-provider LLM support. Includes a native ROS 2 tool with 15 actions for container management and development.

**Hardware target**: NVIDIA Jetson Orin NX/Nano running a 4WD skid-steer UGV with DDSM210 hub motors, ST3215 serial servos (arm/pan-tilt), depth cameras (RealSense D435i or ZED 2), and microphone/speaker for voice.

**Central concept**: Nanobot manages the entire ROS 2 ecosystem - development, testing, deployment, monitoring, and ongoing improvement - through its ROS 2 tool, skill system, cron scheduler, and multi-channel interface.

---

## Current State (Post Stage 1)

### What's Working

| Component | Status | Detail |
|-----------|--------|--------|
| **Docker Compose** | Running | Two-service stack: `billybot-ros2` + `billybot-nanobot` |
| **ROS 2 Container** | Running | Humble + audio_common from source, CycloneDDS, PulseAudio routed |
| **Nanobot Container** | Running | Gateway mode, Docker CLI included, Docker socket mounted |
| **Nanobot ROS 2 Tool** | Working | 15 actions (exec, topics, nodes, build, log, restart, read/write_file, etc.) |
| **Voice Pipeline** | Running | GStreamer capture → Silero VAD → WebSocket Bridge → AI Agent |
| **Audio Playback** | Running | PyAudio with interruption support and queue management |
| **Clap Detector** | Working | Standalone ZCR-based double-clap wake detection |
| **Dual-Agent System** | Working | Conversation + command extraction (OpenAI & Gemini) |
| **Smart Frame Forwarding** | Working | Vision frames to Gemini with rate limiting |
| **ALSA/PulseAudio** | Working | ALSA routed through PulseAudio via `/etc/asound.conf` in container |
| **Workspace Sharing** | Working | ROS source mounted R/W at `/app/ros2_workspace` in nanobot |

### What Was Fixed (Since Initial Assessment)

1. **Hardcoded paths** - Launch files now use `get_package_prefix()` instead of `/home/karim/ros2_ws/...`
2. **audio_common** - Built from source (no Humble apt package); GStreamer dependencies added
3. **PulseAudio routing** - Container ALSA config routes all audio through PulseAudio automatically
4. **Nanobot-ROS integration** - Native `ros2` tool with docker exec + shared file mount + safety guards
5. **Docker CLI in nanobot** - Static Docker binary installed for container management
6. **Environment mapping** - Proper pydantic_settings `__` delimiter for nanobot config

### Remaining Areas for Improvement

1. **~~No motor driver~~** - Stage 3 complete. DDSM driver node: cmd_vel → motors, odom, motor_feedback.
2. **~~No URDF~~** - Stage 3 complete. billybot.urdf.xml + robot_state_publisher, TF tree.
3. **~~No web dashboard~~** - Dashboard complete (Stage 2). Touch joysticks, telemetry, chat.
4. **No navigation** - No nav2, SLAM, or obstacle avoidance.
5. **No CI/CD** - No automated testing or container builds.
6. **No health monitoring** - No watchdog or auto-recovery for crashed nodes.

---

## Docker Compose Assessment

### What Will Work

| Component | Containerization | Notes |
|-----------|-----------------|-------|
| **Nanobot** | Running | Docker CLI + socket mount + ROS workspace mount. Gateway mode active. |
| **ROS 2 nodes** | Running | audio_common from source, CycloneDDS, PulseAudio. All nodes launching. |
| **GPU acceleration** | Ready | NVIDIA runtime sections in compose (uncomment for Jetson). |
| **AI API calls** | Working | OpenAI Realtime, Gemini Live outbound HTTPS/WSS. No container config needed. |
| **Django dashboard** | Ready to add | Standard web container, connects to ROS 2 via WebSocket bridge or docker exec. |

### What Needs Careful Handling

| Component | Challenge | Mitigation |
|-----------|-----------|------------|
| **USB cameras** | Device passthrough + vendor SDK | Mount `/dev/bus/usb`. Use vendor Docker base images. |
| **UART to DDSM HAT** | Serial device passthrough | Mount specific `/dev/ttyTHS1` or `/dev/ttyUSB0`. One owner per port. |
| **ST3215 servo bus** | TTL serial at 1Mbps | Dedicated device mount. Verify USB-serial driver support. |
| **Real-time latency** | Docker overhead on audio path | `SYS_NICE` capability already set. Pin CPU cores if needed. |

---

## Staged Development Plan

### Design Principle

Nanobot manages the ROS 2 ecosystem at every stage. Each stage produces:
- **Nanobot skills** that encode domain knowledge
- **Cron jobs** for ongoing monitoring and maintenance
- **Working capabilities** verified through the nanobot ROS 2 tool
- **Dashboard panels** (from Stage 2 onward) for visual monitoring

---

### Stage 1: Foundation [COMPLETE]

**Goal**: Establish nanobot as the operational manager of the ROS 2 system.

**Completed**:
- Docker Compose with ros2-byc and nanobot containers running
- Nanobot ROS 2 tool with 15 actions (exec, topics, nodes, build, log, restart, read/write_file, params, etc.)
- Docker socket mounted in nanobot for container management
- ROS workspace shared at `/app/ros2_workspace` for direct file access
- Nanobot gateway running with multi-channel support
- Audio pipeline containerized with PulseAudio routing
- audio_common built from source (no Humble apt package)
- Launch file hardcoded paths fixed
- Safety guards on destructive commands in ROS 2 tool
- Entrypoint PulseAudio validation

---

### Stage 2: Web Dashboard (Django + Touch Controls) [COMPLETE]

**Goal**: A modern, responsive web dashboard for real-time robot monitoring and control, with touch-friendly joystick interfaces for mobile/tablet operation.

**Completed**:
- Django 5 + Channels + Daphne ASGI in Docker container (billybot-dashboard)
- Redis channel layer for WebSocket support
- 8 pages: Dashboard, Control, Telemetry, Topics, Nodes, Chat, Logs, Settings
- 4 WebSocket consumers: ROS2 bridge, telemetry, chat, logs
- 9 REST API endpoints for ROS2/Docker operations
- Dual nipple.js touch joysticks (drive + pan/tilt) publishing cmd_vel at 10Hz
- E-STOP with visual feedback, arm presets, bearing presets, voice toggle
- Chart.js telemetry (motor speeds, temps, audio energy)
- Nanobot chat interface via docker exec
- Dark glass-morphism theme with Tailwind CSS + Alpine.js
- Container log viewer with severity filtering

#### 2.1 Dashboard Architecture

```
docker-compose.yml
  │
  ├── ros2-byc          (existing) ROS 2 + ByYourCommand
  │
  ├── nanobot           (existing) AI assistant gateway
  │
  └── dashboard          NEW: Django + Channels + Redis
       ├── Port 8000 (HTTP)
       ├── Port 8001 (WebSocket)
       ├── Connects to ros2-byc via:
       │   ├── ROS AI Bridge WebSocket (port 8765)
       │   ├── docker exec (for CLI commands)
       │   └── Shared /dev/shm (for DDS, if needed)
       └── Connects to nanobot via:
           └── HTTP API or message bus
```

#### 2.2 Technology Stack

| Component | Technology | Reason |
|-----------|-----------|--------|
| **Backend** | Django 5.x | Mature, batteries-included, admin panel |
| **Real-time** | Django Channels + WebSocket | Native async WebSocket support |
| **Channel layer** | Redis | Required for Channels group messaging |
| **Frontend** | Tailwind CSS + Alpine.js | Modern, lightweight, no build step needed |
| **Joystick** | nipple.js | Touch-optimized virtual joystick library |
| **Charts** | Chart.js or Lightweight Charts | Real-time telemetry graphs |
| **Camera feed** | MJPEG or WebRTC | Low-latency video streaming |
| **Container** | Python 3.12 slim | Minimal footprint |

#### 2.3 Dashboard Pages

**Main Dashboard (`/`)**
- System status overview (containers running, node count, uptime)
- Quick-glance cards: audio rate, AI agent status, motor temps, battery
- Live activity log (last 20 events)
- Container health indicators with restart buttons

**Robot Control (`/control`)**
- **Touch joystick** (left): Drive control - publishes to `/cmd_vel`
  - Outer ring: forward/backward/turn
  - Dead zone in center for safety
  - Speed limit slider
- **Touch joystick** (right): Camera pan/tilt - publishes to `/arm_preset` or `/joint_command`
  - Pan left/right, tilt up/down
  - Center button: return to home position
- **Preset buttons**: Quick arm positions (bumper, tenhut, lookup, lookout, reach)
- **Voice toggle**: Mute/unmute via `/voice_active`
- **Emergency stop**: Big red button, publishes zero `/cmd_vel` + motor disable
- Camera live feed with overlay controls

**Telemetry (`/telemetry`)**
- Real-time charts:
  - Motor speeds (4 wheels, line chart)
  - Motor temperatures (4 wheels, gauge)
  - Motor currents (4 wheels, bar chart)
  - Audio energy level (waveform)
  - VAD state (speech detected timeline)
- Odometry: position (x, y, theta) on 2D map
- IMU: orientation visualization (pitch, roll, yaw)

**Topics Explorer (`/topics`)**
- List all ROS 2 topics with types and subscriber counts
- Click to echo messages (live stream)
- Topic frequency (Hz) display
- Publish to topics (for testing)

**Nodes Manager (`/nodes`)**
- List all active nodes with status indicators
- Node info panel (subscriptions, publishers, services, parameters)
- Parameter editing (get/set via form)
- Lifecycle state controls (if lifecycle nodes)

**Nanobot Chat (`/chat`)**
- Chat interface to nanobot agent
- Send messages, view responses
- Skill status display
- Cron job management (list, add, enable/disable, run)

**Logs (`/logs`)**
- Container log viewer (ros2-byc, nanobot, dashboard)
- Filterable by severity (info, warn, error)
- Auto-scroll with pause
- Download log files

**Settings (`/settings`)**
- Environment variable viewer (masked secrets)
- Launch configuration selector
- ROS 2 domain ID
- Audio device selection

#### 2.4 Docker Compose Addition

```yaml
dashboard:
  build:
    context: ./dashboard
    dockerfile: Dockerfile
  container_name: billybot-dashboard
  ports:
    - "${DASHBOARD_PORT:-8000}:8000"
  environment:
    - DJANGO_SECRET_KEY=${DJANGO_SECRET_KEY:-change-me-in-production}
    - DJANGO_DEBUG=${DJANGO_DEBUG:-true}
    - ROS2_BRIDGE_WS=ws://localhost:8765
    - ROS2_CONTAINER_NAME=billybot-ros2
    - NANOBOT_URL=http://billybot-nanobot:18790
    - REDIS_URL=redis://redis:6379/0
  volumes:
    - /var/run/docker.sock:/var/run/docker.sock:ro
    - ./dashboard:/app:rw
  depends_on:
    - redis
    - ros2-byc
  restart: unless-stopped

redis:
  image: redis:7-alpine
  container_name: billybot-redis
  restart: unless-stopped
```

#### 2.5 Dashboard Dockerfile

```dockerfile
FROM python:3.12-slim

RUN pip install --no-cache-dir \
    django>=5.0 \
    channels>=4.0 \
    channels-redis>=4.0 \
    daphne>=4.0 \
    websockets>=12.0 \
    docker>=7.0 \
    whitenoise>=6.0

WORKDIR /app
COPY . .
RUN python manage.py collectstatic --noinput 2>/dev/null || true

EXPOSE 8000
CMD ["daphne", "-b", "0.0.0.0", "-p", "8000", "dashboard.asgi:application"]
```

#### 2.6 Joystick Implementation

The touch joystick uses **nipple.js** for mobile-friendly virtual joystick controls:

```javascript
// Drive joystick - publishes to /cmd_vel via WebSocket
const driveJoystick = nipplejs.create({
    zone: document.getElementById('drive-zone'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#3b82f6',
    size: 150,
    threshold: 0.1,
    fadeTime: 100,
});

driveJoystick.on('move', (evt, data) => {
    const linear = Math.cos(data.angle.radian) * data.force * maxSpeed;
    const angular = Math.sin(data.angle.radian) * data.force * maxTurn;
    ws.send(JSON.stringify({
        type: 'cmd_vel',
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
    }));
});

driveJoystick.on('end', () => {
    // Safety: send zero velocity on release
    ws.send(JSON.stringify({
        type: 'cmd_vel',
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
    }));
});
```

#### 2.7 Design Language

- **Color scheme**: Dark mode default (slate-900 background, blue-500 accents)
- **Typography**: Inter or system-ui, clean and readable
- **Layout**: Responsive grid, sidebar navigation, mobile-first
- **Cards**: Rounded corners, subtle shadows, glass-morphism panels
- **Indicators**: Pulsing green dots for active, red for error, amber for warning
- **Animations**: Smooth transitions, skeleton loading states
- **Touch targets**: Minimum 44x44px for all interactive elements

#### 2.8 WebSocket Communication

The dashboard connects to the ROS AI Bridge WebSocket (already running on port 8765) for real-time topic data. A Django Channels consumer bridges WebSocket connections from the browser to the ROS bridge:

```
Browser → Django Channels WS → ROS AI Bridge WS (8765) → ROS 2 Topics
Browser ← Django Channels WS ← ROS AI Bridge WS (8765) ← ROS 2 Topics
```

For commands that need docker exec (build, restart, param_set), the dashboard uses the Docker SDK:

```
Browser → Django REST API → Docker SDK → docker exec billybot-ros2 → ROS 2 CLI
```

#### 2.9 Nanobot Integration

**New skill**: `dashboard` - Dashboard deployment, configuration, and troubleshooting.

**New cron job**:
```bash
nanobot cron add --name "dashboard-health" \
  --message "Check if the dashboard container is running and responding on port 8000." \
  --every 300
```

**Deliverables**:
- Django project with Channels + WebSocket real-time transport
- Touch joystick controls (drive + camera pan/tilt)
- Live telemetry dashboard with charts
- Topics explorer and nodes manager
- Nanobot chat interface
- Container log viewer
- Dark mode, responsive, mobile-friendly design
- Redis for Channels layer
- Dashboard container in docker-compose

---

### Stage 3: Hardware Integration (Motors + URDF + Odometry) [COMPLETE]

**Goal**: Make the robot move. Nanobot drives development of hardware drivers.

#### 3.1 DDSM Driver Node

```
ddsm_driver_node
  Subscribes: /cmd_vel (geometry_msgs/Twist)
  Publishes:  /odom (nav_msgs/Odometry)
              /motor_feedback (custom: speed, current, temp per motor)
              /joint_states (sensor_msgs/JointState)
  Parameters: serial_port, baud_rate, wheel_separation, wheel_radius
  Protocol:   JSON over UART @ 115200 to DDSM Driver HAT
```

Skid-steer kinematics:
```
left_speed  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
right_speed = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
```

JSON command format (from ugv_base_ros reference):
```json
{"T": 1, "L": <left_rpm_x10>, "R": <right_rpm_x10>}
```

#### 3.2 Robot URDF

```
base_link
├── front_left_wheel_link
├── front_right_wheel_link
├── rear_left_wheel_link
├── rear_right_wheel_link
├── arm_base_link (ST3215 mount point)
│   ├── arm_shoulder_link
│   ├── arm_elbow_link
│   └── camera_mount_link
│       └── camera_link
└── imu_link
```

#### 3.3 Servo Driver Node (ST3215)

```
st3215_driver_node
  Subscribes: /arm_preset (std_msgs/String)
              /joint_command (sensor_msgs/JointState)
  Publishes:  /joint_states (sensor_msgs/JointState)
  Services:   /set_servo_mode, /calibrate
  Protocol:   SCServo TTL serial @ 1Mbps
```

#### 3.4 Dashboard Integration

- Motor telemetry panels on dashboard (speed, temp, current gauges)
- Joystick controls now actually drive the robot
- Odometry visualization on 2D map
- Arm preset buttons wired to real servos

#### 3.5 Nanobot Integration

**New skill**: `ddsm-driver` - DDSM210 protocol, JSON format, ugv_base_ros patterns.
**New skill**: `hardware-test` - Motor verification, encoder feedback testing.

**New cron jobs**:
```bash
nanobot cron add --name "motor-temp-watch" \
  --message "Use the ros2 tool to echo /motor_feedback. Alert if any motor exceeds 60C." \
  --every 60 --deliver --channel telegram --to <chat_id>
```

**Deliverables**:
- `ddsm_driver_node` with cmd_vel to UART and odometry
- `st3215_driver_node` for arm/pan-tilt
- Robot URDF with TF tree
- Dashboard telemetry panels for motors
- Robot moves via joystick on dashboard

---

### Stage 4: Vision & Navigation

**Goal**: Spatial awareness through camera, depth, and autonomous navigation.

#### 4.1 Camera Driver Container

```yaml
camera:
  image: ros2-realsense
  network_mode: host
  devices:
    - /dev/bus/usb:/dev/bus/usb
  environment:
    - ROS_DOMAIN_ID=0
```

Publishes: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu`, `/camera/pointcloud`

#### 4.2 Navigation Stack (nav2)

- SLAM: rtabmap_ros or slam_toolbox
- Planner: nav2 with DWB controller (suited for skid-steer)
- Costmap: 2D from depth projection
- Voice-to-nav: `"Go to the kitchen"` → Gemini → command_processor → `/nav2/goal_pose`

#### 4.3 Dashboard Integration

- Camera feed panel (MJPEG or WebRTC)
- 2D map with robot position, obstacles, and waypoints
- Click-to-navigate: tap on map to set goal
- SLAM map viewer with save/load controls

**Deliverables**:
- Camera driver container
- Gemini vision queries from camera feed
- nav2 basic navigation
- SLAM mapping with save/load
- Dashboard map and camera panels
- Voice-commanded navigation

---

### Stage 5: Operations & Production Hardening

**Goal**: Reliable unattended operation with remote management.

#### 5.1 CI/CD Pipeline

GitHub Actions:
- Build all Docker images (ros2-byc, nanobot, dashboard)
- Run linting (ament_lint, ruff)
- Run unit tests inside container
- Push images to registry on main

#### 5.2 Lifecycle Node Management

Convert critical nodes to lifecycle nodes:
- `ddsm_driver_node` - configure/activate/deactivate for safe motor control
- `ros_ai_bridge` - graceful WebSocket shutdown

Dashboard lifecycle controls panel.

#### 5.3 OTA Deployment

Nanobot deploy skill:
1. Build Docker images (locally or CI)
2. Push to container registry
3. SSH to Jetson, pull new images
4. `docker compose down && docker compose up -d`
5. Verify health via dashboard and cron checks
6. Auto-rollback if health checks fail

#### 5.4 Observability

- Structured JSON logging from all nodes
- Dashboard log aggregation with severity filtering
- Nanobot summarize skill for log analysis
- Hourly health reports via Telegram/Slack

**Deliverables**:
- CI/CD pipeline
- Lifecycle node management with dashboard controls
- OTA deployment via nanobot
- Automated health reporting

---

### Stage 6: Advanced Capabilities

**Goal**: Genuinely useful autonomous robot behaviors.

#### 6.1 Autonomous Behaviors

Behavior trees or state machines:
- **Patrol**: Navigate between waypoints on schedule
- **Search**: Systematically explore looking for objects
- **Follow**: Track and follow a person using depth + detection
- **Guard**: Monitor an area and alert on changes

Nanobot schedules via cron:
```bash
nanobot cron add --name "evening-patrol" \
  --message "Start patrol: kitchen -> hallway -> front_door. Report anomalies." \
  --cron "0 22 * * *" --deliver --channel telegram --to <chat_id>
```

#### 6.2 Manipulation

ST3215 arm with Gemini vision guidance:
- Pick-and-place with visual feedback
- MoveIt2 for path planning
- Dashboard arm control panel with 3D visualization

#### 6.3 Semantic Understanding

- **Spatial memory**: "The keys are on the desk"
- **Scene change detection**: "Something changed since last check"
- **Natural navigation**: "Go to where you last saw the cat"

#### 6.4 Multi-Robot Fleet

ByYourCommand namespace support:
- `/grunt1/` namespace for robot 1, `/grunt2/` for robot 2
- Nanobot manages fleet via separate sessions
- Dashboard fleet overview with per-robot panels

**Deliverables**:
- Autonomous behavior framework
- Manipulation with vision feedback
- Spatial memory in nanobot workspace
- Dashboard fleet management panel

---

## Nanobot-ROS Communication Architecture

Three integration paths, implemented progressively:

### Path 1: Docker Exec + File Mount [COMPLETE]

Nanobot's native `ros2` tool uses docker exec and shared workspace:

```
Nanobot → docker exec billybot-ros2 "ros2 topic list" → result
Nanobot → /app/ros2_workspace/audio/new_node.py → direct file write
```

15 actions available. Safety guards on destructive commands.

### Path 2: WebSocket Bridge (Stage 2-3)

Dashboard and nanobot connect to ROS AI Bridge WebSocket (port 8765):

```
Dashboard/Nanobot → WebSocket:8765 → ros_ai_bridge → ROS 2 topics
```

Real-time topic streaming, low latency, reuses existing infrastructure.

### Path 3: Custom Nanobot Channel (Stage 5-6)

Event-driven ROS 2 channel for nanobot:

```python
class ROS2Channel(BaseChannel):
    name = "ros2"
    async def start(self):
        # Subscribe to /system_alerts, /motor_feedback
    async def send(self, msg: OutboundMessage):
        # Publish to /cmd_vel, /nav2/goal_pose
```

Full bidirectional integration. Requires rclpy or lightweight bridge.

---

## Summary

| Role | How It's Done |
|------|---------------|
| **Development** | Nanobot ros2 tool writes/reads workspace files, builds packages |
| **Monitoring** | Dashboard telemetry + nanobot cron health checks |
| **Control** | Dashboard touch joysticks + voice commands + Telegram |
| **Build** | Nanobot ros2 tool: build action |
| **Deploy** | Nanobot deploy skill: Docker image push + restart |
| **Operate** | Dashboard + Telegram/Slack for remote management |
| **Learn** | Nanobot memory accumulates system behavior knowledge |

The combination of real-time voice AI (ByYourCommand), intelligent management (Nanobot), and a modern web dashboard creates a platform where the robot can be developed, monitored, and controlled through any interface - voice, chat, or browser.
