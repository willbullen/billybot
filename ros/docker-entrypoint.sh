#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Verify PulseAudio connectivity (non-fatal)
if [ -n "$PULSE_SERVER" ]; then
    if pactl info >/dev/null 2>&1; then
        echo "[entrypoint] ✅ PulseAudio connected via $PULSE_SERVER"
        pactl info 2>/dev/null | grep -E "Default (Sink|Source):" || true
    else
        echo "[entrypoint] ⚠️  PulseAudio not reachable at $PULSE_SERVER"
        echo "[entrypoint]    Audio may not work. Check that PulseAudio is running on the host"
        echo "[entrypoint]    and /run/user/1000/pulse is mounted correctly."
    fi
fi

exec "$@"
