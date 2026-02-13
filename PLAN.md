# BillyBot - Master Project Plan

## Project Overview

BillyBot is a voice-interactive robot platform combining two major systems:

1. **ByYourCommand** (ROS 2) - A real-time voice-controlled robot interaction system with AI-powered conversational agents (OpenAI Realtime, Google Gemini Live), VAD-based speech detection, vision integration, and dual-agent command extraction.

2. **Nanobot** - An ultra-lightweight personal AI assistant framework (~3,500 lines) supporting 10 messaging channels (Telegram, Slack, WhatsApp, Discord, etc.), extensible skills, and multi-provider LLM support. Includes a native ROS 2 tool with 15 actions for container management and development.

**Hardware target**: NVIDIA Jetson Orin NX/Nano running a 4WD skid-steer UGV with DDSM210 hub motors, ST3215 serial servos (arm/pan-tilt), depth cameras (RealSense D435i or ZED 2), and microphone/speaker for voice.

**Central concept**: Nanobot manages the entire ROS 2 ecosystem - development, testing, deployment, monitoring, and ongoing improvement - through its ROS 2 tool, skill system, cron scheduler, and multi-channel interface.

---

## Progress Summary

| Stage | Name | Status |
|-------|------|--------|
| **1** | Foundation | COMPLETE |
| **2** | Web Dashboard | COMPLETE |
| **2.1** | HUD Design System | COMPLETE |
| **3** | Hardware Integration | COMPLETE |
| **4** | Vision & Navigation | NOT STARTED (Gemini vision partial) |
| **5** | Operations & Production Hardening | NOT STARTED |
| **6** | Advanced Capabilities | NOT STARTED |

---

## Stage 1: Foundation [COMPLETE]

**Goal**: Establish nanobot as the operational manager of the ROS 2 system.

- [x] Docker Compose with ros2-byc and nanobot containers running
- [x] Nanobot ROS 2 tool with 15 actions (exec, topics, nodes, build, log, restart, read/write_file, params, etc.)
- [x] Docker socket mounted in nanobot for container management
- [x] ROS workspace shared at `/app/ros2_workspace` for direct file access
- [x] Nanobot gateway running with multi-channel support
- [x] Audio pipeline containerized with PulseAudio routing
- [x] audio_common built from source (no Humble apt package)
- [x] Launch file hardcoded paths fixed
- [x] Safety guards on destructive commands in ROS 2 tool
- [x] Entrypoint PulseAudio validation

**ROS 2 Nodes Implemented**:
- Voice pipeline: GStreamer capture -> Silero VAD -> WebSocket Bridge -> AI Agent
- Audio playback: PyAudio with interruption support and queue management
- Clap detector: Standalone ZCR-based double-clap wake detection
- Dual-agent system: Conversation + command extraction (OpenAI & Gemini)
- Smart frame forwarding: Vision frames to Gemini with rate limiting
- Command processor: Voice commands -> ROS 2 topic publishing

**Launch Configurations (7)**:
- `oai_realtime.launch.py` - OpenAI voice-only
- `oai_dual_agent.launch.py` - OpenAI dual-agent
- `gemini_live.launch.py` - Gemini voice
- `gemini_dual_agent.launch.py` - Gemini full multimodal
- `gemini_vision.launch.py` - Vision-focused
- `byc.launch.py` - Audio pipeline only
- `gemini_single.launch.py` - Gemini single-agent

---

## Stage 2: Web Dashboard [COMPLETE]

**Goal**: A modern, responsive web dashboard for real-time robot monitoring and control, with touch-friendly joystick interfaces for mobile/tablet operation.

- [x] Django 5 + Channels + Daphne ASGI in Docker container (billybot-dashboard)
- [x] Redis channel layer for WebSocket support
- [x] 8 pages: Dashboard, Control, Telemetry, Topics, Nodes, Chat, Logs, Settings
- [x] 4 WebSocket consumers: ROS2 bridge, telemetry, chat, logs
- [x] 9 REST API endpoints for ROS2/Docker operations
- [x] Dual nipple.js touch joysticks (drive + pan/tilt) publishing cmd_vel at 10Hz
- [x] E-STOP with visual feedback, arm presets, bearing presets, voice toggle
- [x] Chart.js telemetry (motor speeds, temps, audio energy)
- [x] Nanobot chat interface via docker exec
- [x] Container log viewer with severity filtering
- [x] Dashboard + Redis containers in docker-compose.yml

### 2.1 HUD Design System [COMPLETE]

- [x] JetBrains Mono monospace font throughout (technical/instrument feel)
- [x] Dark theme: slate-950 body, slate-900 sidebar/header, cyan-500 accent family
- [x] Sidebar: cyan gradient logo icon, "BillyBot" + "Robot Dashboard" subtitle, left border-l-2 active nav accent
- [x] HUD header bar: gradient banner, pulsing cyan indicator, STATUS/MODE sub-bar
- [x] Glass-morphism panels (`.hud-panel`): backdrop-blur, cyan-tinted borders
- [x] Title case for readability; uppercase reserved for status readouts and section labels
- [x] Design documented in `dashboard/HUD_DESIGN_SYSTEM.md`

### Architecture

```
docker-compose.yml
  ├── ros2-byc          ROS 2 + ByYourCommand
  ├── nanobot           AI assistant gateway
  ├── dashboard         Django + Channels + Redis
  │    ├── Port 8000 (HTTP + WebSocket)
  │    ├── Connects to ros2-byc via:
  │    │   ├── ROS AI Bridge WebSocket (port 8765)
  │    │   ├── docker exec (for CLI commands)
  │    └── Connects to nanobot via docker exec
  └── redis             Channel layer for Django Channels
```

### Technology Stack

| Component | Technology |
|-----------|-----------|
| Backend | Django 5.x |
| Real-time | Django Channels + WebSocket |
| Channel layer | Redis |
| Frontend | Tailwind CSS + Alpine.js |
| Joystick | nipple.js |
| Charts | Chart.js |
| Container | Python 3.12 slim + Daphne |

### Dashboard Pages

| Page | Route | Key Components |
|------|-------|---------------|
| Dashboard | `/` | Status cards, nodes list, activity log, quick actions |
| Control | `/control` | Drive joystick, pan/tilt, arm presets, bearing presets, E-STOP |
| Telemetry | `/telemetry` | Motor speeds chart, temperatures, audio energy, VAD state |
| Topics | `/topics` | Topics list, filter, topic details, message echo |
| Nodes | `/nodes` | Nodes list, node info, parameters |
| Chat | `/chat` | Nanobot message thread, quick commands |
| Logs | `/logs` | Container selector, line count, filter, auto-scroll |
| Settings | `/settings` | Container status, environment vars, launch configs |

### WebSocket Consumers

| Consumer | Route | Purpose |
|----------|-------|---------|
| Ros2BridgeConsumer | `ws/ros2/` | cmd_vel, arm_preset, bearing, voice_active, estop, pantilt |
| TelemetryConsumer | `ws/telemetry/` | Streams telemetry data |
| ChatConsumer | `ws/chat/` | Routes chat to nanobot via docker exec |
| LogConsumer | `ws/logs/` | Streams container logs in real-time |

### REST API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/ros2/topics/` | GET | List ROS 2 topics |
| `/api/ros2/nodes/` | GET | List ROS 2 nodes |
| `/api/ros2/topic-echo/` | GET | Echo topic messages / topic info |
| `/api/ros2/node-info/` | GET | Node info + parameters |
| `/api/ros2/exec/` | POST | Arbitrary ROS 2 command execution |
| `/api/ros2/param-set/` | POST | Set ROS 2 parameters |
| `/api/docker/containers/` | GET | Container status |
| `/api/docker/logs/` | GET | Container logs |
| `/api/docker/restart/` | POST | Restart containers |

---

## Stage 3: Hardware Integration (Motors + URDF + Odometry) [COMPLETE]

**Goal**: Make the robot move. Bridge the gap between software commands and physical hardware.

**Hardware target**: 4WD skid-steer UGV with DDSM210 hub motors (JSON over UART @ 115200 via DDSM Driver HAT), ST3215 serial servos (TTL @ 1Mbps) for arm/pan-tilt, running on Jetson Orin NX/Nano.

**Key design decision**: All hardware drivers include **simulation mode** that activates automatically when serial ports aren't available (dev machine without hardware).

### New Files (8)

| File | Purpose | Status |
|------|---------|--------|
| `ros/hardware/__init__.py` | Python package for hardware serial modules | [x] |
| `ros/hardware/ddsm_serial.py` | DDSM210 UART JSON protocol (with simulation fallback) | [x] |
| `ros/hardware/st3215_serial.py` | ST3215 TTL servo protocol (with simulation fallback) | [x] |
| `ros/nodes/ddsm_driver_node.py` | ROS 2 node: `/cmd_vel` -> motors -> `/odom` + `/motor_feedback` + `/joint_states` | [x] |
| `ros/nodes/st3215_driver_node.py` | ROS 2 node: `/arm_preset` + `/joint_command` -> servos -> `/joint_states` | [x] |
| `ros/urdf/billybot.urdf.xml` | Robot description: base, 4 wheels, pan/tilt arm, camera, IMU frames | [x] |
| `ros/config/hardware.yaml` | Hardware parameters (serial ports, wheel dims, motor limits, servo presets) | [x] |
| `ros/bringup/hardware.launch.py` | Launch: robot_state_publisher + ddsm_driver + st3215_driver | [x] |

### Modified Files (10)

| File | Changes | Status |
|------|---------|--------|
| `ros/CMakeLists.txt` | Add find_package, ament_python_install_package(hardware), install nodes, urdf dirs | [x] |
| `ros/package.xml` | Add deps: geometry_msgs, nav_msgs, sensor_msgs, tf2_ros, robot_state_publisher | [x] |
| `ros/setup/requirements.txt` | Add `pyserial>=3.5` | [x] |
| `ros/Dockerfile` | Add apt: ros-humble-robot-state-publisher, ros-humble-tf2-ros, ros-humble-joint-state-publisher | [x] |
| `docker-compose.yml` | Keep UART device lines commented (dev mode), add Jetson notes | [x] |
| `dashboard/core/consumers.py` | TelemetryConsumer: poll `/motor_feedback`, stream to browser | [x] |
| `dashboard/core/templates/core/telemetry.html` | Wire real motor data to charts, gate demo mode | [x] |
| `README.md` | Mark Stage 3 COMPLETE, add hardware nodes to status table | [x] |
| `PLAN.md` | Mark Stage 3 COMPLETE | [x] |
| `ros/CLAUDE.md` | Document hardware nodes, `hardware.launch.py simulate:=true` | [x] |

### Implementation Steps

#### Step 1: Serial Protocol Modules (no ROS deps)

**`ros/hardware/ddsm_serial.py`** - DDSM HAT communication:
- `DDSMSerial(port, baudrate=115200, simulate=False)`
- Auto-enters simulation mode if port can't open or pyserial missing
- `send_speed(left_rpm, right_rpm)` - JSON `{"T":1, "L":<x10>, "R":<x10>}`
- `read_feedback()` - dict with left/right speed, current, temp, encoder
- Thread-safe via Lock, speed clamped to [-2100, 2100]

**`ros/hardware/st3215_serial.py`** - ST3215 servo communication:
- `ST3215Serial(port, baudrate=1000000, simulate=False)`
- Preset dictionary: bumper, tenhut, lookup, lookout, reach -> servo angle arrays (0-4095)
- Bearing-to-pan mapping: forward=2048, left=3072, right=1024, etc.
- `set_preset(name)`, `set_position(servo_id, position)`, `read_feedback(servo_id)`

#### Step 2: Config, URDF, Dependencies

**`ros/config/hardware.yaml`**:
```yaml
ddsm_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyTHS1"
    baud_rate: 115200
    simulate: false
    wheel_separation: 0.30
    wheel_radius: 0.05
    max_linear_speed: 1.0
    max_angular_speed: 3.0
    cmd_vel_timeout: 0.5
    feedback_rate: 20.0
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    encoder_counts_per_rev: 4096
    publish_tf: true

st3215_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 1000000
    simulate: false
    servo_ids: [1, 2]
    feedback_rate: 10.0
    preset_bumper:  [2048, 3072]
    preset_tenhut:  [2048, 2048]
    preset_lookup:  [2048, 1024]
    preset_lookout: [2048, 1536]
    preset_reach:   [2048, 512]
```

**`ros/urdf/billybot.urdf.xml`**:
- `base_footprint` -> `base_link` (chassis 0.30x0.25x0.10m)
- 4 wheel links + continuous joints (cylinder r=0.05, l=0.04)
- `arm_base_link` (fixed on base) -> `arm_pan_joint` (revolute, z) -> `arm_tilt_joint` (revolute, y)
- `camera_link` (fixed on tilt head), `imu_link` (fixed on base)

#### Step 3: DDSM Driver Node

**`ros/nodes/ddsm_driver_node.py`**:
- Subscribes: `/cmd_vel` (Twist)
- Publishes: `/odom` (Odometry), `/motor_feedback` (String JSON), `/joint_states` (JointState)
- TF: Broadcasts `odom` -> `base_link`
- Skid-steer kinematics, differential odometry from encoders
- Watchdog: stop motors after 0.5s cmd_vel timeout

#### Step 4: ST3215 Servo Driver Node

**`ros/nodes/st3215_driver_node.py`**:
- Subscribes: `/grunt1/arm_preset` (String), `/joint_command` (JointState)
- Publishes: `/joint_states` (JointState)
- Services: `/calibrate` (Trigger), `/set_servo_mode` (SetBool)
- Parses `"preset@bearing"` format, applies bearing pan offset

#### Step 5: Hardware Launch File

**`ros/bringup/hardware.launch.py`**:
- Reads URDF, passes to robot_state_publisher
- Launches ddsm_driver_node + st3215_driver_node with hardware.yaml params
- `simulate` launch argument (default: false)

#### Step 6: Dashboard Telemetry

- TelemetryConsumer: poll `/motor_feedback` via docker exec, send to browser
- Wire real motor data to Chart.js charts, gate demo data

#### Step 7: Docker Compose & Documentation

- Keep UART devices commented in docker-compose
- Update README, CLAUDE.md

### Architecture After Stage 3

```
Dashboard Joystick -> WebSocket -> docker exec -> ros2 topic pub /cmd_vel
                                                      |
                                      ddsm_driver_node (subscribes /cmd_vel)
                                        | skid-steer kinematics
                                      JSON {"T":1,"L":x,"R":y} -> UART -> DDSM HAT -> Motors
                                        | encoder feedback
                                      Publishes: /odom, /motor_feedback, /joint_states

Dashboard Arm Preset -> WebSocket -> docker exec -> ros2 topic pub /arm_preset
                                                      |
                                      command_processor -> /grunt1/arm_preset
                                        |
                                      st3215_driver_node (subscribes)
                                        | preset lookup
                                      TTL serial -> ST3215 servos -> arm moves
```

### Topic Map After Stage 3

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | Twist | Dashboard joystick | ddsm_driver_node |
| `/odom` | Odometry | ddsm_driver_node | (future: nav2) |
| `/motor_feedback` | String (JSON) | ddsm_driver_node | Dashboard telemetry |
| `/joint_states` | JointState | ddsm_driver + st3215_driver | robot_state_publisher |
| `/grunt1/arm_preset` | String | command_processor | st3215_driver_node |
| `/joint_command` | JointState | (direct control) | st3215_driver_node |
| `/tf` | TransformStamped | ddsm_driver + robot_state_publisher | (rviz, nav2) |

### Verification Checklist

- [ ] Build: `docker compose build ros2-byc` succeeds with new deps
- [ ] Launch (sim): `ros2 launch by_your_command hardware.launch.py simulate:=true`
- [ ] Nodes visible: ddsm_driver_node, st3215_driver_node, robot_state_publisher
- [ ] Topics visible: /cmd_vel, /odom, /motor_feedback, /joint_states, /tf
- [ ] Joystick -> Driver: dashboard joystick -> ddsm_driver_node logs cmd_vel
- [ ] Odometry: `ros2 topic echo /odom` shows pose data
- [ ] TF tree: odom -> base_link -> wheels + arm chain
- [ ] Telemetry: dashboard shows motor data (simulated)
- [ ] Arm presets: dashboard button -> st3215_driver_node logs preset
- [ ] Watchdog: stop joystick -> motors zeroed after 0.5s
- [ ] Graceful sim: no errors without physical serial ports

### Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| No serial hardware on dev machine | Simulation mode auto-activates; pyserial import guarded |
| pyserial not in ROS image | Added to requirements.txt; ImportError caught |
| robot_state_publisher missing | Added to Dockerfile apt install |
| Joint name collision (DDSM + ST3215) | Distinct names: wheel joints vs arm joints |
| DDSM JSON protocol mismatch | Matches ugv_base_ros reference; configurable via parameters |
| URDF dimensions wrong | Placeholder values; tune on real robot |
| 4WD odometry accuracy | 4WD skid-steer treated as 2WD diff-drive; good enough for basic nav |

---

## Stage 4: Vision & Navigation [ ]

**Goal**: Spatial awareness through camera, depth, and autonomous navigation.

### 4.1 Camera Driver Container

- [ ] Camera container (RealSense or ZED)
- [ ] Publishes: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/imu`, `/camera/pointcloud`

```yaml
camera:
  image: ros2-realsense
  network_mode: host
  devices:
    - /dev/bus/usb:/dev/bus/usb
  environment:
    - ROS_DOMAIN_ID=0
```

### 4.2 Navigation Stack (nav2)

- [ ] SLAM: rtabmap_ros or slam_toolbox
- [ ] Planner: nav2 with DWB controller (suited for skid-steer)
- [ ] Costmap: 2D from depth projection
- [ ] Voice-to-nav: "Go to the kitchen" -> Gemini -> command_processor -> `/nav2/goal_pose`

### 4.3 Dashboard Integration

- [ ] Camera feed panel (MJPEG or WebRTC)
- [ ] 2D map with robot position, obstacles, and waypoints
- [ ] Click-to-navigate: tap on map to set goal
- [ ] SLAM map viewer with save/load controls

### 4.4 Nanobot Integration

- [ ] Nanobot skill: camera diagnostics
- [ ] Cron jobs for map backup and camera health

**Partial progress**: Gemini vision integration exists (frame forwarding via `gemini_vision.launch.py`) but no ROS 2 camera driver or nav2 stack.

---

## Stage 5: Operations & Production Hardening [ ]

**Goal**: Reliable unattended operation with remote management.

### 5.1 CI/CD Pipeline

- [ ] GitHub Actions: build all Docker images
- [ ] Linting: ament_lint, ruff
- [ ] Unit tests inside container
- [ ] Push images to registry on main

### 5.2 Lifecycle Node Management

- [ ] Convert ddsm_driver_node to lifecycle node
- [ ] Convert ros_ai_bridge to lifecycle node
- [ ] Dashboard lifecycle controls panel

### 5.3 OTA Deployment

- [ ] Nanobot deploy skill
- [ ] Build -> push -> pull -> restart cycle
- [ ] Auto-rollback on health check failure

### 5.4 Observability

- [ ] Structured JSON logging from all nodes
- [ ] Dashboard log aggregation with severity filtering
- [ ] Nanobot summarize skill for log analysis
- [ ] Hourly health reports via Telegram/Slack

---

## Stage 6: Advanced Capabilities [ ]

**Goal**: Genuinely useful autonomous robot behaviors.

### 6.1 Autonomous Behaviors

- [ ] Behavior trees or state machines
- [ ] Patrol: navigate between waypoints on schedule
- [ ] Search: systematically explore looking for objects
- [ ] Follow: track and follow a person using depth + detection
- [ ] Guard: monitor an area and alert on changes

### 6.2 Manipulation

- [ ] ST3215 arm with Gemini vision guidance
- [ ] Pick-and-place with visual feedback
- [ ] MoveIt2 for path planning
- [ ] Dashboard arm control panel with 3D visualization

### 6.3 Semantic Understanding

- [ ] Spatial memory: "The keys are on the desk"
- [ ] Scene change detection: "Something changed since last check"
- [ ] Natural navigation: "Go to where you last saw the cat"

### 6.4 Multi-Robot Fleet

- [ ] ByYourCommand namespace support (`/grunt1/`, `/grunt2/`)
- [ ] Nanobot manages fleet via separate sessions
- [ ] Dashboard fleet overview with per-robot panels

---

## Nanobot-ROS Communication Architecture

Three integration paths, implemented progressively:

### Path 1: Docker Exec + File Mount [COMPLETE]

```
Nanobot -> docker exec billybot-ros2 "ros2 topic list" -> result
Nanobot -> /app/ros2_workspace/audio/new_node.py -> direct file write
```
15 actions available. Safety guards on destructive commands.

### Path 2: WebSocket Bridge [COMPLETE]

Dashboard and nanobot connect to ROS AI Bridge WebSocket (port 8765):
```
Dashboard/Nanobot -> WebSocket:8765 -> ros_ai_bridge -> ROS 2 Topics
```

### Path 3: Custom Nanobot Channel [Stage 5-6]

- [ ] Event-driven ROS 2 channel for nanobot
- [ ] Full bidirectional integration
- [ ] Requires rclpy or lightweight bridge

---

## Docker Compose Services

| Service | Container | Status |
|---------|-----------|--------|
| `ros2-byc` | billybot-ros2 | Running |
| `nanobot` | billybot-nanobot | Running |
| `dashboard` | billybot-dashboard | Running |
| `redis` | billybot-redis | Running |

### Hardware Considerations

| Component | Challenge | Mitigation |
|-----------|-----------|------------|
| USB cameras | Device passthrough + vendor SDK | Mount `/dev/bus/usb`. Vendor Docker base images. |
| UART to DDSM HAT | Serial device passthrough | Mount `/dev/ttyTHS1` or `/dev/ttyUSB0`. One owner per port. |
| ST3215 servo bus | TTL serial at 1Mbps | Dedicated device mount. Verify USB-serial driver support. |
| Real-time latency | Docker overhead on audio path | `SYS_NICE` capability set. Pin CPU cores if needed. |

---

## Role Summary

| Role | How It's Done |
|------|---------------|
| **Development** | Nanobot ros2 tool writes/reads workspace files, builds packages |
| **Monitoring** | Dashboard telemetry + nanobot cron health checks |
| **Control** | Dashboard touch joysticks + voice commands + Telegram |
| **Build** | Nanobot ros2 tool: build action |
| **Deploy** | Nanobot deploy skill: Docker image push + restart |
| **Operate** | Dashboard + Telegram/Slack for remote management |
| **Learn** | Nanobot memory accumulates system behavior knowledge |
