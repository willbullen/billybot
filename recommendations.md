# BillyBot - Project Analysis & Recommendations

## Project Overview

BillyBot is a voice-interactive robot platform combining two major systems:

1. **ByYourCommand** (ROS 2) - A real-time voice-controlled robot interaction system with AI-powered conversational agents (OpenAI Realtime, Google Gemini Live), VAD-based speech detection, vision integration, and dual-agent command extraction.

2. **Nanobot** - An ultra-lightweight personal AI assistant framework (~3,500 lines) supporting 10 messaging channels (Telegram, Slack, WhatsApp, Discord, etc.), extensible skills, and multi-provider LLM support.

**Hardware target**: NVIDIA Jetson Orin NX/Nano running a 4WD skid-steer UGV with DDSM210 hub motors, ST3215 serial servos (arm/pan-tilt), depth cameras (RealSense D435i or ZED 2), and microphone/speaker for voice.

**Central concept**: Nanobot manages the entire ROS 2 ecosystem - development, testing, deployment, monitoring, and ongoing improvement - through its skill system, cron scheduler, and multi-channel interface.

---

## Docker Compose Assessment

### What Will Work

| Component | Containerization | Notes |
|-----------|-----------------|-------|
| **Nanobot** | Works well | Already has a Dockerfile. Stateless web service, ideal for containers. |
| **ROS 2 nodes (non-hardware)** | Works well | ros_ai_bridge, agents (OpenAI/Gemini), command_processor, silero_vad_node all run as pure software. WebSocket bridge on port 8765 maps cleanly. |
| **GPU acceleration (CUDA/TensorRT)** | Works with NVIDIA runtime | Jetson containers are well-supported via `nvidia-container-runtime`. Silero VAD (PyTorch) and any vision inference benefit from GPU passthrough. |
| **Network-based AI APIs** | Works well | OpenAI Realtime, Gemini Live, and LiteLLM calls are outbound HTTPS/WSS - no special container config needed. |
| **Inter-container ROS 2 comms** | Works with shared network | ROS 2 DDS (FastDDS/CycloneDDS) works across containers on the same Docker network using multicast or shared-memory transport. |

### What Needs Careful Handling

| Component | Challenge | Mitigation |
|-----------|-----------|------------|
| **Audio capture/playback** | ALSA/PulseAudio device access inside containers requires host device passthrough | Mount `/dev/snd`, share PulseAudio socket, or use `--device` flags. The `audio_common` capturer and `simple_audio_player` both need direct audio hardware. |
| **USB cameras (RealSense/ZED)** | USB device passthrough needed; RealSense requires `librealsense2` udev rules; ZED needs the ZED SDK | Mount `/dev/bus/usb` or specific video devices. Install udev rules on host. Use vendor-provided Docker images as base where possible. |
| **UART to DDSM Driver HAT** | Serial device passthrough (`/dev/ttyTHS1` or `/dev/ttyUSB0`) | Mount the specific serial device into the container. Only one container should own a serial port at a time. |
| **ST3215 servo bus** | Separate TTL serial bus, same passthrough concern | Dedicated device mount; ensure baud rate (1Mbps) is supported by the USB-serial adapter driver in the container. |
| **Real-time audio latency** | Docker adds marginal overhead; VAD + streaming AI needs <100ms round-trip | Use `--privileged` or specific capabilities (`SYS_NICE`, `SYS_RAWIO`). Pin CPU cores if needed. On Jetson, the overhead is generally acceptable. |
| **DDS discovery** | Multi-container ROS 2 needs DDS config | Use `ROS_DOMAIN_ID` and set `RMW_IMPLEMENTATION` consistently. CycloneDDS with shared-memory is recommended for same-host multi-container. |

### What Won't Work (Without Workarounds)

| Component | Issue | Recommendation |
|-----------|-------|----------------|
| **GUI tools (rviz2, rqt)** | Need X11/Wayland display forwarding | Mount X11 socket or use VNC/noVNC sidecar container. Not critical for headless Jetson deployment. |
| **ESP32 flashing/OTA** | The DDSM Driver HAT ESP32 firmware updates can't happen from inside a container easily | Flash firmware from the host before containerizing. Keep a host-side flash script. |
| **Kernel-level GPIO** | Direct Jetson GPIO (for UART) may need `--privileged` | Use `/dev/ttyTHS*` device passthrough instead of raw GPIO. This is the standard approach. |

---

## Project Potential

### Strengths

1. **Dual AI agent architecture** - The conversational + command extraction split is well-designed. It allows natural conversation while reliably extracting structured robot commands.

2. **Multi-provider LLM support** - Supporting both OpenAI Realtime and Gemini Live gives flexibility on cost, latency, and capability. The agent abstraction is clean.

3. **Nanobot as the management brain** - Nanobot's skill system, cron scheduler, multi-channel interface, and exec tool make it a natural fit for managing the entire robot lifecycle. You can build, test, deploy, and monitor the ROS 2 system from Telegram.

4. **Cost-aware session management** - The intelligent session cycling on pauses (rather than keeping a persistent connection) shows good production thinking for API cost control.

5. **Modular audio pipeline** - VAD, echo suppression, clap detection, and chunking are well-separated nodes. Easy to swap or disable components.

6. **Hardware abstraction** - The DDSM HAT's JSON-over-UART protocol is simple to interface with from ROS 2. The ugv_base_ros reference implementation provides a clear path.

### Areas for Improvement

1. **No ROS 2 driver for the DDSM HAT yet** - The hardware docs reference `ugv_base_ros` but there's no actual `cmd_vel` to UART/JSON driver node in the codebase. This is the critical missing piece for the robot to actually move.

2. **No URDF or robot description** - There's no robot model, TF tree, or joint state publisher. This blocks nav2, visualization, and proper coordinate frame management.

3. **Hardcoded paths in launch files** - The `oai_realtime.launch.py` has a hardcoded path (`/home/karim/ros2_ws/...`). This breaks in containers or on other machines.

4. **No health monitoring or watchdog** - If the AI agent crashes, the audio pipeline, or a motor driver hangs, there's no automatic recovery.

5. **No navigation stack integration** - The depth cameras are listed in hardware but there's no nav2, SLAM, or obstacle avoidance code.

6. **No Nanobot-ROS integration** - Nanobot and ByYourCommand run side-by-side but don't communicate. The WebSocket bridge and exec tools provide the path to connect them.

7. **No CI/CD pipeline** - No GitHub Actions, automated tests, or container builds.

8. **Security: API keys in launch arguments** - Launch files pass API keys as arguments which may appear in process listings.

---

## Staged Development Plan

### Design Principle

Nanobot manages the ROS 2 ecosystem at every stage. Each stage produces:
- **Nanobot skills** that encode the knowledge for that domain
- **Cron jobs** for ongoing monitoring and maintenance
- **Workspace artifacts** (bootstrap files, reference docs) for the agent's memory
- **Working ROS 2 capabilities** verified through the pipeline

---

### Stage 1: Foundation (Nanobot + Docker + Workspace Bootstrap)

**Goal**: Establish nanobot as the operational manager of the ROS 2 system.

#### 1.1 Nanobot Workspace Setup

Configure the nanobot workspace with BillyBot-specific knowledge:

```
~/.nanobot/workspace/
├── AGENTS.md                    # Agent instructions: "You manage a ROS 2 robot called BillyBot"
├── SOUL.md                      # Personality: precise, safety-conscious, robotics-expert
├── USER.md                      # Developer preferences and context
├── TOOLS.md                     # Custom tool documentation
├── MEMORY.md                    # Persistent system knowledge
├── memory/                      # Daily notes and observations
├── skills/                      # Custom skills (see below)
└── references/                  # Hardware docs, ROS 2 guides
```

**AGENTS.md** should describe:
- The BillyBot hardware stack (Jetson, DDSM motors, cameras, servos)
- The docker-compose topology (ros2-byc + nanobot containers)
- The ROS 2 package structure and key nodes
- Safety rules (never force-push, always backup before destructive ops)
- The development workflow (edit in ros/ -> nanobot builds -> nanobot tests -> nanobot deploys)

#### 1.2 Core Nanobot Skills

**`ros2-workspace` skill** - Build and workspace management:
```markdown
---
name: ros2-workspace
description: Build, test, and manage the ByYourCommand ROS 2 workspace
---
# ROS 2 Workspace Management

## Build Commands
- colcon build --packages-select by_your_command --symlink-install
- Source: source /ros2_ws/install/setup.bash

## Test Commands
- ros2 run by_your_command test_clap_detection
- ros2 run by_your_command test_vad_mute_control
- ros2 run by_your_command test_sleep_clap_integration
- ros2 run by_your_command test_recorder_integration
- ros2 run by_your_command test_command_processor

## Common Issues
- Missing audio_common_msgs: sudo apt install ros-humble-audio-common-msgs
- Symlink-install stale: delete build/ and install/ then rebuild
```

**`ros2-diagnostics` skill** - Runtime monitoring:
```markdown
---
name: ros2-diagnostics
description: Monitor ROS 2 node health, topic rates, and system diagnostics
---
# Diagnostics

## Node Health
- ros2 node list -> expected: silero_vad_node, ros_ai_bridge, simple_audio_player, ...
- ros2 topic hz /audio -> should be ~31 Hz (512 samples @ 16kHz)
- ros2 topic hz /prompt_voice -> active during speech

## Container Health
- docker ps --filter name=billybot -> both containers "Up"
- docker logs billybot-ros2 --tail 50 -> check for errors
```

**`ros2-launch` skill** - Launch configuration management

**`docker-ops` skill** - Container build, deploy, and operations

#### 1.3 Initial Cron Jobs

```bash
# Container health check every 2 minutes
nanobot cron add --name "container-health" \
  --message "Run docker ps and verify billybot-ros2 and billybot-nanobot are running. If either is down, restart with docker compose up -d." \
  --every 120

# ROS 2 node check every 5 minutes
nanobot cron add --name "ros2-node-check" \
  --message "Run ros2 node list inside billybot-ros2 container. Report any missing nodes." \
  --every 300 --deliver --channel telegram --to <chat_id>
```

#### 1.4 Docker Environment Validation

- Verify docker-compose builds both images successfully
- Validate PulseAudio passthrough (audio capture + playback inside container)
- Verify ROS 2 launches inside the container
- Test nanobot can exec commands inside the ros2-byc container
- Fix hardcoded paths in launch files to work in container environment

**Deliverables**:
- Working docker-compose with both containers
- 4+ nanobot skills (ros2-workspace, ros2-diagnostics, ros2-launch, docker-ops)
- Nanobot workspace bootstrapped with BillyBot context
- Health monitoring cron jobs active
- All launch files containerization-compatible

---

### Stage 2: Hardware Integration (Motors + URDF + Odometry)

**Goal**: Make the robot move. Nanobot drives the development of hardware drivers.

#### 2.1 DDSM Driver Node

The critical missing piece. Nanobot develops this using its knowledge of:
- The DDSM210 UART protocol (from `hardware/TECHSTACK_HARDWARE.md`)
- The ugv_base_ros JSON command reference
- ROS 2 node development patterns (from `devrules/agentic_rules.md`)

**Node specification**:

```
ddsm_driver_node
  Subscribes: /cmd_vel (geometry_msgs/Twist)
  Publishes:  /odom (nav_msgs/Odometry)
              /motor_feedback (custom: speed, current, temp per motor)
              /joint_states (sensor_msgs/JointState)
  Parameters: serial_port, baud_rate, wheel_separation, wheel_radius
  Protocol:   JSON over UART @ 115200 to DDSM Driver HAT
```

**Skid-steer kinematics**:
```
left_speed  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
right_speed = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
```

**JSON command format** (from ugv_base_ros reference):
```json
{"T": 1, "L": <left_rpm_x10>, "R": <right_rpm_x10>}
```

**New nanobot skill**: `ddsm-driver` - encodes the DDSM210 protocol, JSON format, and ugv_base_ros patterns.

#### 2.2 Robot URDF

Basic skid-steer model:

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

#### 2.3 TF Tree and Joint State Publisher

- `robot_state_publisher` from URDF
- `joint_state_publisher` for arm servos
- Odometry TF: `odom -> base_link` from DDSM feedback

#### 2.4 Servo Driver Node (ST3215)

```
st3215_driver_node
  Subscribes: /arm_preset (std_msgs/String)
              /joint_command (sensor_msgs/JointState)
  Publishes:  /joint_states (sensor_msgs/JointState)
  Services:   /set_servo_mode, /calibrate
  Protocol:   SCServo TTL serial @ 1Mbps
```

#### 2.5 Nanobot Integration

**New skill**: `hardware-test` - motor control verification, servo response, encoder feedback testing.

**New cron jobs**:
```bash
# Motor temperature monitoring
nanobot cron add --name "motor-temp-watch" \
  --message "Check /motor_feedback temperatures. Alert if any motor exceeds 60C." \
  --every 60 --deliver --channel telegram --to <chat_id>

# Odometry drift check
nanobot cron add --name "odom-drift" \
  --message "Read /odom for 10 seconds while stationary. Drift should be < 0.01m." \
  --every 600
```

**Deliverables**:
- `ddsm_driver_node` with cmd_vel -> UART and odometry feedback
- `st3215_driver_node` for arm/pan-tilt servo control
- Robot URDF with TF tree
- Hardware test skill and motor monitoring cron jobs
- Robot can move via `ros2 topic pub /cmd_vel`

---

### Stage 3: Vision & Perception

**Goal**: Give the robot spatial awareness through camera, depth, and mapping.

#### 3.1 Camera Driver Container

Add a dedicated camera service to docker-compose:

```yaml
camera:
  image: ros2-realsense   # or stereolabs/zed
  network_mode: host
  devices:
    - /dev/bus/usb:/dev/bus/usb
  environment:
    - ROS_DOMAIN_ID=0
```

Publishes:
- `/camera/color/image_raw` - RGB frames
- `/camera/depth/image_rect_raw` - Depth frames
- `/camera/imu` - IMU data
- `/camera/pointcloud` - Point cloud

#### 3.2 Visual Integration with ByYourCommand

The Gemini Live agent already supports vision via compressed image topics. Connect the camera output to the bridge:

```yaml
# bridge_dual_agent.yaml
subscribed_topics:
  - topic: "/camera/color/image_raw/compressed"
    msg_type: "sensor_msgs/CompressedImage"
```

#### 3.3 Navigation Stack (nav2)

- Requires: depth camera + odometry + TF tree (from Stage 2)
- SLAM: rtabmap_ros or slam_toolbox
- Planner: nav2 with DWB controller (suited for skid-steer)
- Costmap: 2D from depth projection

Voice-to-navigation:
```
"Go to the kitchen" -> Gemini -> command_processor -> /nav2/goal_pose
```

#### 3.4 Nanobot Integration

**New skill**: `navigation` - nav2 launch, waypoint management, map saving/loading.

**Deliverables**:
- Camera driver container publishing RGB + depth + IMU
- Visual queries working through Gemini vision
- nav2 basic navigation with depth-based obstacle avoidance
- SLAM mapping with save/load
- Voice-commanded navigation

---

### Stage 4: Operations & Production Hardening

**Goal**: Reliable unattended operation with remote management.

#### 4.1 CI/CD Pipeline

GitHub Actions:
- Build ros2-byc and nanobot Docker images
- Run ROS 2 linting (ament_lint) and Python linting (ruff)
- Run unit tests inside container
- Push images to registry on main branch

Nanobot manages CI through its GitHub skill.

#### 4.2 Lifecycle Node Management

Convert critical nodes to ROS 2 lifecycle nodes:
- `ddsm_driver_node` - configure/activate/deactivate for safe motor control
- `ros_ai_bridge` - graceful WebSocket shutdown

Nanobot triggers transitions:
```bash
ros2 lifecycle set /ddsm_driver_node activate
ros2 lifecycle set /ddsm_driver_node deactivate  # emergency stop
```

#### 4.3 OTA Deployment

Nanobot deploy skill:
1. Build Docker images (locally or CI)
2. Push to container registry
3. SSH to Jetson, pull new images
4. `docker compose down && docker compose up -d`
5. Verify health checks pass
6. Roll back if health checks fail

#### 4.4 Observability

- Structured JSON logging from all nodes
- Log aggregation via Docker logging driver
- Nanobot summarize skill for log analysis
- Hourly health reports via Telegram/Slack

**Deliverables**:
- GitHub Actions CI/CD pipeline
- Lifecycle node management
- OTA deployment via nanobot
- Hourly automated health reports

---

### Stage 5: Advanced Capabilities

**Goal**: Genuinely useful autonomous robot behaviors.

#### 5.1 Autonomous Behaviors

Behavior trees or state machines for:
- **Patrol**: Navigate between waypoints on schedule
- **Search**: Systematically explore looking for objects
- **Follow**: Track and follow a person using depth + object detection
- **Guard**: Monitor an area and alert on changes

Nanobot schedules behaviors via cron:
```bash
nanobot cron add --name "evening-patrol" \
  --message "Start patrol: kitchen -> hallway -> front_door. Report anomalies." \
  --cron "0 22 * * *" --deliver --channel telegram --to <chat_id>
```

#### 5.2 Manipulation

ST3215 arm with Gemini vision guidance:
- Pick-and-place with visual feedback
- Extend arm presets (bumper, tenhut, lookup, lookout, reach)
- MoveIt2 for path planning

#### 5.3 Semantic Understanding

Extend the dual-agent system:
- **Spatial memory**: "The keys are on the desk"
- **Scene change detection**: "Something changed since last check"
- **Natural navigation**: "Go to where you last saw the cat"

Nanobot's memory system stores spatial knowledge persistently.

#### 5.4 Multi-Robot Fleet

ByYourCommand's namespace support enables multi-robot:
- `/grunt1/` namespace for robot 1
- `/grunt2/` namespace for robot 2
- Nanobot manages the fleet via separate sessions per robot

**Deliverables**:
- Autonomous behavior framework
- Basic manipulation with vision feedback
- Spatial memory in nanobot workspace
- Multi-robot namespace support validated

---

## Nanobot-ROS Communication Architecture

Three integration paths, in order of implementation:

### Path 1: Shell Exec (Stage 1, simplest)

Nanobot uses its `exec` tool to run ROS 2 CLI commands:

```
Nanobot -> exec("docker exec billybot-ros2 ros2 topic list") -> result
Nanobot -> exec("docker exec billybot-ros2 ros2 topic pub /cmd_vel ...") -> result
```

**Pros**: Zero additional code. Works immediately.
**Cons**: ~200ms latency per command, no streaming, CLI-only.

### Path 2: WebSocket Bridge (Stage 2-3, medium)

Nanobot connects to the existing ROS AI Bridge WebSocket (port 8765):

```
Nanobot -> WebSocket:8765 -> ros_ai_bridge -> ROS 2 topics
```

Reuses the bridge that AI agents already use.

**Pros**: Low latency, real-time topic streaming, reuses existing infrastructure.
**Cons**: Requires a nanobot WebSocket tool or custom skill with scripts.

### Path 3: Custom Nanobot Channel (Stage 4-5, full integration)

A ROS 2 channel for nanobot that subscribes to topics directly:

```python
class ROS2Channel(BaseChannel):
    name = "ros2"

    async def start(self):
        # Subscribe to /system_alerts, /motor_feedback, etc.
        # Publish InboundMessage to bus on relevant events

    async def send(self, msg: OutboundMessage):
        # Publish to ROS 2 topics (/cmd_vel, /nav2/goal_pose)
```

**Pros**: Full bidirectional, event-driven, native message types.
**Cons**: Requires rclpy in nanobot container or a lightweight bridge.

### Recommended Approach

Start with **Path 1** (shell exec) in Stage 1-2. It works immediately with zero code changes. Move to **Path 2** (WebSocket) in Stage 3 when real-time monitoring is needed. Consider **Path 3** (custom channel) in Stage 4-5 for proactive event-driven integration.

---

## Summary

BillyBot has a solid foundation with its dual-AI voice interaction system and clean ROS 2 architecture. The key insight is using nanobot not just as a chatbot but as the intelligent operations layer:

| Role | How Nanobot Does It |
|------|-------------------|
| **Development** | Skills encode ROS 2 patterns; agent writes and tests code |
| **Build** | Exec tool runs colcon builds, reports errors |
| **Deploy** | Deploy skill pushes Docker images, restarts containers |
| **Monitor** | Cron jobs watch node health, motor temps, topic rates |
| **Operate** | Developers interact via Telegram/Slack to control the robot |
| **Learn** | Memory system accumulates knowledge about system behavior |

The combination of real-time conversational AI (ByYourCommand) with an intelligent management layer (Nanobot) creates a platform where the robot can be developed, operated, and improved through natural language - both locally through voice and remotely through messaging.
