# BillyBot

A voice-interactive robot platform where an AI assistant framework (Nanobot) manages the development, deployment, and operation of a ROS 2 robotic ecosystem (ByYourCommand). The robot speaks, sees, moves, and learns - controlled through natural conversation, remote messaging channels, or a web dashboard.

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
│  │  Skills    Cron       ROS 2      Heartbeat   Memory            │ │
│  │  (ROS2,   (health,   Tool       (watchdog)  (logs,            │ │
│  │   build,   deploy,   (15 actions)            state)            │ │
│  │   test)    monitor)                                             │ │
│  └─────────────────────────────┬───────────────────────────────────┘ │
│                                │ docker exec / file mount             │
│  ┌─────────────────────────────▼───────────────────────────────────┐ │
│  │                  BY YOUR COMMAND (ROS 2)                         │ │
│  │                                                                  │ │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────┐  ┌──────────────┐ │ │
│  │  │ Audio    │  │ Silero   │  │ ROS AI     │  │ LLM Agents   │ │ │
│  │  │ Capture  │→ │ VAD      │→ │ Bridge     │→ │ (OpenAI /    │ │ │
│  │  │(GStream) │  │          │  │ (WS:8765)  │  │  Gemini)     │ │ │
│  │  └──────────┘  └──────────┘  └────────────┘  └──────────────┘ │ │
│  │                                    │                            │ │
│  │  ┌──────────┐  ┌──────────┐  ┌────▼───────┐  ┌──────────────┐ │ │
│  │  │ Audio    │  │ Clap     │  │ Command    │  │ Camera /     │ │ │
│  │  │ Player   │  │ Detector │  │ Processor  │  │ Vision       │ │ │
│  │  │(PyAudio) │  │ (ZCR)    │  │            │  │              │ │ │
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

Nanobot is an ultra-lightweight (~3,500 lines) personal AI assistant framework that serves as the operational brain of the BillyBot platform. It manages the ROS 2 ecosystem through a dedicated **ROS 2 tool** with 15 actions, plus skills, cron scheduling, and multi-channel remote access.

**Location**: `nanobot/`

| Feature | Detail |
|---------|--------|
| Version | 0.1.3.post5 |
| Python | 3.11+ |
| LLM Providers | OpenRouter, Anthropic, OpenAI, Gemini, DeepSeek, Groq, vLLM, and more via LiteLLM |
| Built-in Skills | GitHub, weather, cron, summarize, tmux, skill-creator |
| Tools | read, write, edit, exec, search, fetch, message, spawn, cron, **ros2** |
| Gateway Port | 18790 |

#### ROS 2 Tool (Built-in)

Nanobot includes a native `ros2` tool that interacts with the ByYourCommand container via `docker exec` and shared file mounts. Available actions:

| Action | Description |
|--------|-------------|
| `exec` | Run arbitrary commands inside the ROS 2 container |
| `topics` | List all ROS 2 topics (verbose: show types + counts) |
| `nodes` | List all active ROS 2 nodes |
| `services` | List all ROS 2 services |
| `topic_echo` | Echo messages from a topic (with count limit) |
| `topic_info` | Detailed info about a topic |
| `node_info` | Detailed info about a node |
| `params` | List parameters for a node |
| `param_get` | Get a parameter value |
| `param_set` | Set a parameter value |
| `build` | Run `colcon build` (specific packages or all) |
| `log` | Show container logs (configurable line count) |
| `restart` | Restart the ROS 2 container |
| `read_file` | Read files from the shared ROS workspace |
| `write_file` | Create/write files in the shared ROS workspace |

The tool includes safety guards that block destructive commands (rm -rf /, dd, format, shutdown, fork bombs).

### ByYourCommand - ROS 2 Robot Control

ByYourCommand is the ROS 2 package that handles real-time voice interaction, vision processing, and robot control. It provides the physical interface between AI and hardware.

**Location**: `ros/`

| Feature | Detail |
|---------|--------|
| ROS Version | Humble |
| Build System | ament_cmake + Python |
| AI Providers | OpenAI Realtime API, Google Gemini Live |
| Architecture | Dual-agent (conversational + command extraction) |
| Audio | 16kHz GStreamer capture, Silero VAD, PyAudio playback, clap wake |
| Vision | Camera integration with smart frame forwarding |
| Bridge | WebSocket server on port 8765 |

#### ROS 2 Nodes

| Node | Purpose |
|------|---------|
| `silero_vad_node` | Voice activity detection with utterance chunking, sleep/wake control |
| `simple_audio_player` | PyAudio-based playback with interruption support and queue management |
| `clap_detector_node` | Standalone ZCR-based double-clap wake detection |
| `ros_ai_bridge` | WebSocket bridge between ROS topics and AI agents |
| `oai_realtime_agent` | OpenAI Realtime API agent |
| `gemini_live_agent` | Google Gemini Live agent with vision |
| `command_processor` | Command extraction and routing to robot subsystems |
| `voice_chunk_recorder` | Debug recording of voice utterances |
| `echo_suppressor` | Audio feedback loop prevention |

#### Topic Map

```
Voice Input:  /audio → /prompt_voice → [Bridge:8765] → [Agent] → LLM API
Text Input:   /prompt_text → [Bridge:8765] → [Agent] → LLM API
Voice Output: LLM API → [Agent] → [Bridge] → /response_voice → PyAudio Speaker
Text Output:  LLM API → [Agent] → [Bridge] → /response_text
Commands:     LLM API → [Agent] → [Bridge] → /response_cmd → /arm_preset, /behavior_command
Control:      /voice_active (mute), /wake_cmd (clap wake), /interruption_signal (stop playback)
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
| Voice | Onboard mic + speaker | PulseAudio (ALSA routed) | Voice I/O |

Full hardware specifications: `hardware/TECHSTACK_HARDWARE.md`

## Docker Compose Setup

The system runs as containerized services with hardware passthrough:

```
docker-compose.yml
  │
  ├── ros2-byc          ROS 2 + ByYourCommand
  │                     network_mode: host (DDS + audio)
  │                     Mounts: /dev/snd, PulseAudio, /dev/shm
  │                     Builds: audio_common from source + by_your_command
  │                     ALSA routed through PulseAudio via asound.conf
  │
  ├── nanobot           AI assistant gateway
  │                     Port: 18790
  │                     Docker socket mounted (for ros2 tool)
  │                     ROS workspace mounted at /app/ros2_workspace
  │                     Includes Docker CLI binary
  │
  ├── dashboard         Django + Channels + Redis
  │                     Port: 8000 (Daphne ASGI)
  │                     Touch joystick controls, live telemetry
  │                     8 pages, 4 WebSocket consumers, 9 REST APIs
  │
  └── redis             Channel layer for Django Channels
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

# 4. Interact with nanobot via CLI
docker compose exec nanobot nanobot agent -m "What ROS 2 nodes are running?"

# 5. Interact via the ros2 tool directly
docker compose exec nanobot nanobot agent -m "Use the ros2 tool to list all topics"
```

### Environment Variables

| Variable | Service | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | ros2-byc | OpenAI Realtime API key |
| `GOOGLE_API_KEY` | ros2-byc | Google Gemini API key |
| `NANOBOT_LLM_API_KEY` | nanobot | LLM provider API key (mapped to OpenAI provider) |
| `NANOBOT_LLM_MODEL` | nanobot | Default LLM model (default: gpt-4o) |
| `TELEGRAM_BOT_TOKEN` | nanobot | Telegram bot token (auto-enables channel) |
| `SLACK_BOT_TOKEN` | nanobot | Slack bot token |
| `DISCORD_BOT_TOKEN` | nanobot | Discord bot token |
| `ROS_DOMAIN_ID` | ros2-byc | ROS 2 domain (default: 0) |

Nanobot environment variables use pydantic_settings nested delimiter (`__`):
```
NANOBOT_PROVIDERS__OPENAI__API_KEY  →  config.providers.openai.apiKey
NANOBOT_AGENTS__DEFAULTS__MODEL     →  config.agents.defaults.model
NANOBOT_CHANNELS__TELEGRAM__TOKEN   →  config.channels.telegram.token
```

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

Nanobot manages the ROS 2 system through its native `ros2` tool, which uses `docker exec` into the ROS container and direct file access via the shared workspace mount.

### Integration Architecture

```
Nanobot Container                         ROS 2 Container
┌─────────────────────┐                  ┌──────────────────────┐
│  Agent Loop         │                  │  /ros2_ws/           │
│    │                │                  │    src/               │
│    ├── ros2 tool ───┼── docker exec ──→│      by_your_command/ │
│    │   (15 actions) │                  │    install/           │
│    │                │                  │                       │
│    └── file I/O ────┼── shared mount ─→│  (source code R/W)   │
│                     │                  │                       │
│  /var/run/docker ───┼── docker API    │                       │
│  .sock (read-only)  │                  └──────────────────────┘
└─────────────────────┘
```

### Management Examples

```
Developer ←→ Nanobot (via Telegram/Slack/CLI) ←→ ROS 2 System
                │
                ├── "Build the workspace"     → ros2 tool: build
                ├── "What nodes are running?" → ros2 tool: nodes
                ├── "List all topics"         → ros2 tool: topics (verbose)
                ├── "Echo /audio for 3 msgs"  → ros2 tool: topic_echo
                ├── "Check VAD parameters"    → ros2 tool: params (node)
                ├── "Set threshold to 0.6"    → ros2 tool: param_set
                ├── "Show last 50 log lines"  → ros2 tool: log
                ├── "Restart the container"   → ros2 tool: restart
                ├── "Read silero_vad_node.py" → ros2 tool: read_file
                ├── "Write a new driver node" → ros2 tool: write_file
                └── "Schedule health check"   → cron tool
```

### Cron Jobs for Operations

```bash
# Health check every 5 minutes
nanobot cron add --name "ros2-health" \
  --message "Use the ros2 tool to list nodes and topics. Report any missing nodes or zero-rate topics." \
  --every 300 --deliver --channel telegram --to <chat_id>

# Daily build verification
nanobot cron add --name "daily-build" \
  --message "Use the ros2 tool to build the workspace. Report success or failure." \
  --cron "0 6 * * *" --deliver --channel slack --to <channel_id>
```

### Heartbeat for Watchdog

Create `~/.nanobot/workspace/HEARTBEAT.md`:

```markdown
# Heartbeat Tasks

- [ ] Verify ros2-byc container is running (docker ps)
- [ ] Check ROS 2 node count matches expected (ros2 tool: nodes)
- [ ] Verify audio pipeline is active (ros2 tool: topic_echo /audio)
- [ ] Check WebSocket bridge connectivity
- [ ] Monitor disk space on /tmp/prompt_voice
```

### Remote Control via Messaging

Once nanobot gateway is running with channels enabled:

```
You (Telegram): "What nodes are running?"
Nanobot: [uses ros2 tool: nodes]
         6 nodes active:
         /audio_capture_node, /silero_vad_node, /simple_audio_player,
         /clap_detector_node, /ros_ai_bridge, /oai_realtime_agent

You (Telegram): "Check the audio topic rate"
Nanobot: [uses ros2 tool: exec "ros2 topic hz /audio --window 5"]
         average rate: 31.25 Hz, min: 0.031s, max: 0.033s

You (Slack): "Build the workspace"
Nanobot: [uses ros2 tool: build packages="by_your_command"]
         Starting build... colcon build completed successfully.
         Packages built: by_your_command
```

## Project Structure

```
billybot/
├── docker-compose.yml              # Container orchestration (ros2-byc + nanobot)
├── .env.example                    # Environment variable template
├── recommendations.md              # Analysis and development roadmap
├── README.md                       # This file
│
├── ros/                            # ByYourCommand ROS 2 package
│   ├── Dockerfile                  # ROS 2 Humble + audio_common from source
│   ├── docker-entrypoint.sh        # PulseAudio validation + ROS env sourcing
│   ├── package.xml                 # ROS 2 package manifest
│   ├── CMakeLists.txt              # Build configuration
│   ├── agents/                     # AI agent implementations
│   │   ├── common/                 # Shared: websocket, context, prompts, base_serializer
│   │   ├── oai_realtime/           # OpenAI Realtime API agent
│   │   └── gemini_live/            # Google Gemini Live agent
│   ├── audio/                      # Audio pipeline nodes
│   │   ├── silero_vad_node.py      # VAD with sleep/wake, topic: prompt_voice
│   │   ├── simple_audio_player.py  # PyAudio playback with interruption
│   │   ├── clap_detector_node.py   # Standalone ZCR double-clap detection
│   │   ├── echo_suppressor.py      # Feedback prevention
│   │   └── voice_chunk_recorder.py # Debug recording
│   ├── bringup/                    # Launch files (7 configurations)
│   ├── config/                     # YAML configs (10 files)
│   ├── msg/                        # Custom ROS messages
│   ├── nodes/                      # command_processor
│   ├── ros_ai_bridge/              # WebSocket bridge
│   ├── tests/                      # 40+ test utilities
│   └── setup/                      # requirements.txt
│
├── nanobot/                        # AI assistant framework
│   ├── Dockerfile                  # Python 3.12 + Node.js 20 + Docker CLI
│   ├── pyproject.toml              # Python package config
│   ├── nanobot/                    # Core framework
│   │   ├── agent/                  # Agent loop, context, memory
│   │   │   └── tools/              # Built-in tools
│   │   │       ├── ros2.py         # ROS 2 tool (15 actions, docker exec)
│   │   │       ├── filesystem.py   # Read, write, edit, list
│   │   │       ├── shell.py        # Shell execution
│   │   │       └── ...             # web, message, spawn, cron
│   │   ├── channels/              # 10 messaging platforms
│   │   ├── providers/             # LLM provider abstraction (LiteLLM)
│   │   ├── skills/                # Extensible skill system
│   │   ├── bus/                   # Async message bus
│   │   ├── cron/                  # Task scheduling
│   │   └── session/               # Conversation history
│   └── bridge/                    # WhatsApp bridge (Node.js)
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
# Via nanobot (recommended)
docker compose exec nanobot nanobot agent -m "Build the by_your_command package"

# Or directly inside the ROS 2 container
docker compose exec ros2-byc bash -c \
  "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select by_your_command --symlink-install"
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
nanobot gateway              # Start multi-channel gateway (default mode in compose)
nanobot cron list            # List scheduled tasks
nanobot cron add ...         # Add scheduled task
nanobot channels status      # Show channel status
```

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

See `recommendations.md` for the complete analysis and phased development roadmap:

- **Stage 1**: Foundation - Docker environment, nanobot ROS 2 tool, workspace bootstrap **[COMPLETE]**
- **Stage 2**: Web Dashboard - Django dashboard with touch joystick controls, live telemetry **[COMPLETE]**
- **Stage 3**: Hardware Integration - DDSM motor driver, URDF, odometry
- **Stage 4**: Vision & Navigation - Camera driver, nav2, SLAM
- **Stage 5**: Operations - CI/CD, lifecycle nodes, OTA deployment
- **Stage 6**: Advanced Capabilities - Autonomous behaviors, manipulation, fleet management

## Status

**Pre-alpha** - Under active development.

| Component | Status |
|-----------|--------|
| Docker Compose | Running (ros2-byc + nanobot) |
| Voice pipeline | Running (OpenAI + Gemini, PulseAudio routed) |
| Vision pipeline | Working (Gemini with smart frame forwarding) |
| Dual-agent system | Working (conversation + command extraction) |
| Nanobot ROS 2 tool | Working (15 actions via docker exec + file mount) |
| Nanobot gateway | Running (multi-channel, cron, heartbeat) |
| Clap detector | Working (standalone ZCR-based) |
| Web dashboard | Running (8 pages, touch joysticks, telemetry, chat) |
| Motor driver | Not started |
| Navigation | Not started |

## License

- ByYourCommand: Apache License 2.0
- Nanobot: MIT License
