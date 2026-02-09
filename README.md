# BillyBot

A voice-interactive robot platform where an AI assistant framework (Nanobot) manages the development, deployment, and operation of a ROS 2 robotic ecosystem (ByYourCommand). The robot speaks, sees, moves, and learns - controlled through natural conversation or remote messaging channels.

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                         BILLYBOT PLATFORM                            │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │                    NANOBOT (Project Manager)                     │ │
│  │                                                                  │ │
│  │  Channels: Telegram | Slack | Discord | WhatsApp | Email | CLI  │ │
│  │                              │                                   │ │
│  │              ┌───────────────┴───────────────┐                  │ │
│  │              │         Agent Loop            │                  │ │
│  │              │   LLM + Tools + Memory        │                  │ │
│  │              └───────────┬───────────────────┘                  │ │
│  │                          │                                       │ │
│  │    ┌─────────┬───────────┼───────────┬──────────┐               │ │
│  │    │         │           │           │          │               │ │
│  │  Skills    Cron      Subagents   Heartbeat   Memory            │ │
│  │  (ROS2,   (health,   (parallel   (watchdog)  (logs,           │ │
│  │   build,   deploy,    tasks)                   state)          │ │
│  │   test)    monitor)                                             │ │
│  └─────────────────────────────┬───────────────────────────────────┘ │
│                                │ WebSocket / exec / topics            │
│  ┌─────────────────────────────▼───────────────────────────────────┐ │
│  │                  BY YOUR COMMAND (ROS 2)                         │ │
│  │                                                                  │ │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────┐  ┌──────────────┐ │ │
│  │  │ Audio    │  │ Silero   │  │ ROS AI     │  │ LLM Agents   │ │ │
│  │  │ Capture  │→ │ VAD      │→ │ Bridge     │→ │ (OpenAI /    │ │ │
│  │  │          │  │          │  │ (WS:8765)  │  │  Gemini)     │ │ │
│  │  └──────────┘  └──────────┘  └────────────┘  └──────────────┘ │ │
│  │                                    │                            │ │
│  │  ┌──────────┐  ┌──────────┐  ┌────▼───────┐  ┌──────────────┐ │ │
│  │  │ Audio    │  │ Echo     │  │ Command    │  │ Camera /     │ │ │
│  │  │ Player   │  │ Suppress │  │ Processor  │  │ Vision       │ │ │
│  │  └──────────┘  └──────────┘  └────────────┘  └──────────────┘ │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                │                                     │
│  ┌─────────────────────────────▼───────────────────────────────────┐ │
│  │                       HARDWARE                                   │ │
│  │                                                                  │ │
│  │  Jetson Orin NX/Nano  │  DDSM210 Motors  │  ST3215 Servos      │ │
│  │  RealSense D435i/ZED2 │  DDSM Driver HAT │  Mic + Speaker      │ │
│  └─────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

## Components

### Nanobot - AI Project Manager

Nanobot is an ultra-lightweight (~3,500 lines) personal AI assistant framework that serves as the operational brain of the BillyBot platform. It manages the ROS 2 ecosystem through:

- **Skills**: Domain-specific knowledge modules (ROS 2 build/test/deploy, hardware drivers, diagnostics)
- **Cron**: Scheduled tasks for health monitoring, log rotation, metrics collection
- **Subagents**: Background workers for long-running operations (builds, tests, deployments)
- **Heartbeat**: Periodic system health checks and auto-recovery
- **Memory**: Persistent knowledge of system state, issues, and solutions
- **Channels**: Remote access via Telegram, Slack, Discord, WhatsApp, Email, and 5 more platforms

**Location**: `nanobot/`

| Feature | Detail |
|---------|--------|
| Version | 0.1.3.post5 |
| Python | 3.11+ |
| LLM Providers | OpenRouter, Anthropic, OpenAI, Gemini, DeepSeek, Groq, vLLM, and more via LiteLLM |
| Built-in Skills | GitHub, weather, cron, summarize, tmux, skill-creator |
| Tools | read, write, edit, exec, search, fetch, message, spawn, cron |
| Gateway Port | 18790 |

### ByYourCommand - ROS 2 Robot Control

ByYourCommand is the ROS 2 package that handles real-time voice interaction, vision processing, and robot control. It provides the physical interface between AI and hardware.

**Location**: `ros/`

| Feature | Detail |
|---------|--------|
| ROS Version | Humble |
| Build System | ament_cmake + Python |
| AI Providers | OpenAI Realtime API, Google Gemini Live |
| Architecture | Dual-agent (conversational + command extraction) |
| Audio | 16kHz capture, Silero VAD, echo suppression, clap wake |
| Vision | Camera integration with smart frame forwarding |
| Bridge | WebSocket server on port 8765 |

#### ROS 2 Nodes

| Node | Purpose |
|------|---------|
| `silero_vad_node` | Voice activity detection with utterance chunking |
| `simple_audio_player` | Real-time audio playback with interruption support |
| `echo_suppressor` | Feedback loop prevention during playback |
| `clap_detector_node` | Double-clap wake detection (ZCR-based) |
| `ros_ai_bridge` | WebSocket bridge between ROS topics and AI agents |
| `oai_realtime_agent` | OpenAI Realtime API agent |
| `gemini_live_agent` | Google Gemini Live agent with vision |
| `command_processor` | Command extraction and routing to robot subsystems |
| `voice_chunk_recorder` | Debug recording of voice utterances |

#### Topic Map

```
Voice Input:  /audio → /audio_filtered → /prompt_voice → [Bridge] → [Agent] → LLM API
Text Input:   /prompt_text → [Bridge] → [Agent] → LLM API
Voice Output: LLM API → [Agent] → [Bridge] → /response_voice → Speaker
Text Output:  LLM API → [Agent] → [Bridge] → /response_text
Commands:     LLM API → [Agent] → [Bridge] → /response_cmd → /arm_preset, /behavior_command
Control:      /voice_active (mute), /wake_cmd (clap), /interruption_signal (stop playback)
Vision:       /cam_live/color/image_raw/compressed → [Bridge] → [Agent] → Gemini
```

### Hardware

| Component | Model | Interface | Role |
|-----------|-------|-----------|------|
| Compute | Jetson Orin NX / Nano | - | Host: ROS 2, AI, control |
| Motors | Waveshare DDSM210 x4 | UART 115200 via HAT | 4WD skid-steer drive |
| Motor Driver | DDSM Driver HAT (A) | JSON over UART/USB | ESP32 lower computer |
| Servos | Waveshare ST3215 | TTL serial 1Mbps | Arm/pan-tilt joints |
| Depth Camera | RealSense D435i or ZED 2 | USB | RGB + depth + IMU |
| Voice | Onboard mic + speaker | ALSA/PulseAudio | Voice I/O |

Full hardware specifications: `hardware/TECHSTACK_HARDWARE.md`

## Docker Compose Setup

The system runs as two containers with hardware passthrough:

```
docker-compose.yml
  │
  ├── ros2-byc          ROS 2 + ByYourCommand
  │                     network_mode: host (DDS + audio)
  │                     Mounts: /dev/snd, PulseAudio, /dev/shm
  │                     Optional: UART, USB cameras, NVIDIA GPU
  │
  └── nanobot           AI assistant gateway
                        Port: 18790
                        Volumes: config, workspace
```

### Quick Start

```bash
# 1. Clone and configure
cd billybot
cp .env.example .env
# Edit .env with your API keys

# 2. Build and start
docker compose up -d

# 3. Check status
docker compose logs -f ros2-byc
docker compose logs -f nanobot

# 4. Interact with nanobot
docker compose exec nanobot nanobot agent -m "What is the system status?"
```

### Environment Variables

| Variable | Service | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | ros2-byc | OpenAI Realtime API key |
| `GOOGLE_API_KEY` | ros2-byc | Google Gemini API key |
| `NANOBOT_LLM_API_KEY` | nanobot | LLM provider API key |
| `NANOBOT_LLM_MODEL` | nanobot | Default LLM model |
| `TELEGRAM_BOT_TOKEN` | nanobot | Telegram bot token |
| `SLACK_BOT_TOKEN` | nanobot | Slack bot token |
| `DISCORD_BOT_TOKEN` | nanobot | Discord bot token |
| `ROS_DOMAIN_ID` | ros2-byc | ROS 2 domain (default: 0) |

### Jetson GPU Deployment

Uncomment the NVIDIA runtime section in `docker-compose.yml`:

```yaml
ros2-byc:
  runtime: nvidia
  deploy:
    resources:
      reservations:
        devices:
          - driver: nvidia
            count: all
            capabilities: [gpu]
```

For Jetson, replace the base image in `ros/Dockerfile`:
```dockerfile
FROM dustynv/ros:humble-desktop-l4t-r36.4.0
```

### Hardware Passthrough

Uncomment device lines in `docker-compose.yml` as needed:

```yaml
devices:
  - /dev/snd:/dev/snd                  # Audio (always needed)
  - /dev/ttyTHS1:/dev/ttyTHS1          # UART to DDSM Driver HAT
  - /dev/ttyUSB0:/dev/ttyUSB0          # USB-serial adapter
  - /dev/bus/usb:/dev/bus/usb          # USB cameras
```

## How Nanobot Manages the ROS 2 Ecosystem

Nanobot acts as the intelligent operations layer. It doesn't replace ROS 2 - it orchestrates, monitors, and develops it.

### Management Model

```
Developer ←→ Nanobot (via Telegram/Slack/CLI) ←→ ROS 2 System
                │
                ├── "Build the workspace"         → exec: colcon build
                ├── "Run the tests"               → exec: ros2 run ... test_*
                ├── "Launch the dual agent"        → exec: ros2 launch ...
                ├── "What nodes are running?"      → exec: ros2 node list
                ├── "Check motor health"           → exec: ros2 topic echo /motor_feedback
                ├── "Deploy to the Jetson"         → skill: deploy
                ├── "Create a DDSM driver node"    → skill: ros2-dev + write + edit
                ├── "Review the latest logs"       → read: /tmp/prompt_voice/*.wav
                └── "Schedule hourly health check" → cron: ros2 node list + topic hz
```

### Nanobot Skills for ROS 2

Custom skills are YAML-frontmatter Markdown files placed in the nanobot workspace:

```
~/.nanobot/workspace/skills/
├── ros2-build/SKILL.md          # Build, test, install workflows
├── ros2-launch/SKILL.md         # Launch file management
├── ros2-diagnostics/SKILL.md    # Node health, topic monitoring
├── ddsm-driver/SKILL.md         # Motor driver development reference
├── deploy/SKILL.md              # Docker build + push + Jetson deploy
└── byc-dev/SKILL.md             # ByYourCommand development patterns
```

Each skill bundles instructions, reference docs, and executable scripts that the agent loads on-demand.

### Cron Jobs for Operations

```bash
# Health check every 5 minutes
nanobot cron add --name "ros2-health" \
  --message "Run ros2 node list and ros2 topic hz /audio. Report any missing nodes or zero-rate topics." \
  --every 300 --deliver --channel telegram --to <chat_id>

# Daily build verification
nanobot cron add --name "daily-build" \
  --message "Run colcon build in /ros2_ws. Report success or failure with error details." \
  --cron "0 6 * * *" --deliver --channel slack --to <channel_id>

# Motor temperature monitoring
nanobot cron add --name "motor-temp" \
  --message "Check DDSM motor temperatures via /motor_feedback. Alert if any exceed 60C." \
  --every 60 --deliver --channel telegram --to <chat_id>
```

### Heartbeat for Watchdog

Create `~/.nanobot/workspace/HEARTBEAT.md`:

```markdown
# Heartbeat Tasks

- [ ] Verify ros2-byc container is running (docker ps)
- [ ] Check ROS 2 node count matches expected (ros2 node list)
- [ ] Verify audio pipeline is active (ros2 topic hz /audio > 0)
- [ ] Check WebSocket bridge connectivity (ws://localhost:8765)
- [ ] Monitor disk space on /tmp/prompt_voice
```

Nanobot's heartbeat service wakes every 30 minutes to execute these checks.

### Remote Control via Messaging

Once nanobot gateway is running with channels enabled:

```
You (Telegram): "Launch the Gemini vision system"
Nanobot: Running `ros2 launch by_your_command gemini_dual_agent.launch.py`...
         Launched. 8 nodes active. Camera publishing at 30fps.

You (Telegram): "What does the robot see?"
Nanobot: Querying Gemini vision agent...
         Scene: Indoor room, desk with laptop, chair, door to the left.

You (Telegram): "Tell it to look right"
Nanobot: Publishing `pan@right` to /response_cmd...
         Command acknowledged. Camera panning right.

You (Slack): "Run the test suite"
Nanobot: Executing tests...
         ✓ test_clap_detection: PASSED
         ✓ test_vad_mute_control: PASSED
         ✗ test_recorder_integration: FAILED (timeout on audio callback)
         2/3 passed. See full output: [link]
```

## Project Structure

```
billybot/
├── docker-compose.yml              # Container orchestration
├── .env.example                    # Environment variable template
├── recommendations.md              # Analysis and development roadmap
│
├── ros/                            # ByYourCommand ROS 2 package
│   ├── Dockerfile                  # ROS 2 container definition
│   ├── docker-entrypoint.sh        # Container startup script
│   ├── package.xml                 # ROS 2 package manifest
│   ├── CMakeLists.txt              # Build configuration
│   ├── CLAUDE.md                   # Development reference
│   ├── README.md                   # ROS 2 package documentation
│   ├── agents/                     # AI agent implementations
│   │   ├── common/                 # Shared: websocket, context, prompts
│   │   ├── oai_realtime/           # OpenAI Realtime API agent
│   │   └── gemini_live/            # Google Gemini Live agent
│   ├── audio/                      # Audio pipeline nodes
│   ├── bringup/                    # Launch files (7 configurations)
│   ├── config/                     # YAML configs (10 files)
│   ├── msg/                        # Custom ROS messages
│   ├── nodes/                      # command_processor
│   ├── ros_ai_bridge/              # WebSocket bridge
│   ├── tests/                      # 40+ test utilities
│   ├── setup/                      # requirements.txt, setup.sh
│   ├── devrules/                   # Development guidelines
│   └── specs/                      # Technical specifications
│
├── nanobot/                        # AI assistant framework
│   ├── Dockerfile                  # Nanobot container definition
│   ├── pyproject.toml              # Python package config
│   ├── nanobot/                    # Core framework
│   │   ├── agent/                  # Agent loop, context, memory, tools
│   │   ├── channels/              # 10 messaging platforms
│   │   ├── providers/             # LLM provider abstraction
│   │   ├── skills/                # Extensible skill system
│   │   ├── cli/                   # Command-line interface
│   │   ├── bus/                   # Async message bus
│   │   ├── cron/                  # Task scheduling
│   │   ├── heartbeat/             # Health monitoring
│   │   └── session/               # Conversation history
│   ├── bridge/                    # WhatsApp bridge (Node.js)
│   └── workspace/                 # Runtime workspace + memory
│
├── hardware/                       # Hardware specifications
│   └── TECHSTACK_HARDWARE.md       # Complete hardware BOM
│
└── archive/                        # Archived utilities
```

## Launch Configurations

| Launch File | Provider | Agents | Vision | Use Case |
|-------------|----------|--------|--------|----------|
| `oai_realtime.launch.py` | OpenAI | Single | No | Voice-only conversation |
| `oai_dual_agent.launch.py` | OpenAI | Dual | No | Voice + command extraction |
| `gemini_live.launch.py` | Gemini | Single | Optional | Voice with optional video |
| `gemini_dual_agent.launch.py` | Gemini | Dual | Yes | Full multimodal interaction |
| `gemini_vision.launch.py` | Gemini | Single | Yes | Vision-focused queries |
| `byc.launch.py` | None | None | No | Audio pipeline only (testing) |

## Development

### Building ByYourCommand

```bash
# Inside the ros2-byc container (or local ROS 2 workspace)
cd /ros2_ws
colcon build --packages-select by_your_command --symlink-install
source install/setup.bash
```

### Running Tests

```bash
ros2 run by_your_command test_clap_detection
ros2 run by_your_command test_vad_mute_control
ros2 run by_your_command test_sleep_clap_integration
ros2 run by_your_command test_recorder_integration
ros2 run by_your_command test_command_processor
```

### Nanobot CLI

```bash
nanobot onboard              # Initialize config and workspace
nanobot status               # Show configuration status
nanobot agent                # Interactive chat
nanobot agent -m "message"   # Single message
nanobot gateway              # Start multi-channel gateway
nanobot cron list            # List scheduled tasks
nanobot cron add ...         # Add scheduled task
nanobot channels status      # Show channel status
```

### Adding Nanobot Skills

Create a skill directory in the workspace:

```bash
mkdir -p ~/.nanobot/workspace/skills/my-skill
```

Write `SKILL.md` with YAML frontmatter:

```markdown
---
name: my-skill
description: Short description of what this skill does
---

# My Skill

Instructions for the agent when this skill is loaded...
```

Skills are discovered automatically and loaded on-demand when the agent determines they're relevant.

## Configuration Files

| File | Purpose |
|------|---------|
| `ros/config/prompts.yaml` | System prompts with recursive macro expansion |
| `ros/config/config.yaml` | VAD and clap detection parameters |
| `ros/config/oai_realtime_agent.yaml` | OpenAI agent settings |
| `ros/config/gemini_live_agent.yaml` | Gemini agent settings |
| `ros/config/bridge_dual_agent.yaml` | WebSocket bridge + topic routing |
| `~/.nanobot/config.json` | Nanobot agent, provider, and channel config |
| `.env` | API keys and environment variables |

## Staged Development Plan

See `recommendations.md` for the complete analysis and phased development roadmap covering:

- **Stage 1**: Foundation - Docker environment, nanobot skills, workspace bootstrap
- **Stage 2**: Hardware Integration - DDSM motor driver, URDF, odometry
- **Stage 3**: Intelligence - Navigation, SLAM, autonomous behaviors
- **Stage 4**: Operations - CI/CD, monitoring, remote deployment, fleet management
- **Stage 5**: Advanced Capabilities - Manipulation, semantic mapping, multi-robot

## Status

**Pre-alpha** - Under active development.

- Voice pipeline: Working (OpenAI + Gemini)
- Vision pipeline: Working (Gemini)
- Dual-agent system: Working
- Docker setup: Initial
- Motor driver: Not started
- Navigation: Not started
- Nanobot-ROS integration: Not started

## License

- ByYourCommand: Apache License 2.0
- Nanobot: MIT License
