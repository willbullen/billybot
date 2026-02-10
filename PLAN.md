# Stage 3: Hardware Integration (Motors + URDF + Odometry)

## Context

BillyBot Stages 1 (Foundation) and 2 (Dashboard) are complete. The dashboard sends `cmd_vel` messages via joystick controls, but nothing subscribes to them - the robot can't move. Stage 3 bridges the gap between software commands and physical hardware by adding motor/servo driver nodes, a robot URDF for the TF tree, odometry computation, and wiring real telemetry data into the dashboard.

**Hardware target**: 4WD skid-steer UGV with DDSM210 hub motors (JSON over UART @ 115200 via DDSM Driver HAT), ST3215 serial servos (TTL @ 1Mbps) for arm/pan-tilt, running on Jetson Orin NX/Nano.

**Key design decision**: All hardware drivers include **simulation mode** that activates automatically when serial ports aren't available (dev machine without hardware). This keeps the full stack testable in Docker on any machine.

---

## New Files (8)

| File | Purpose |
|------|---------|
| `ros/hardware/__init__.py` | Python package for hardware serial modules |
| `ros/hardware/ddsm_serial.py` | DDSM210 UART JSON protocol (with simulation fallback) |
| `ros/hardware/st3215_serial.py` | ST3215 TTL servo protocol (with simulation fallback) |
| `ros/nodes/ddsm_driver_node.py` | ROS 2 node: `/cmd_vel` -> motors -> `/odom` + `/motor_feedback` + `/joint_states` |
| `ros/nodes/st3215_driver_node.py` | ROS 2 node: `/arm_preset` + `/joint_command` -> servos -> `/joint_states` |
| `ros/urdf/billybot.urdf.xml` | Robot description: base, 4 wheels, pan/tilt arm, camera, IMU frames |
| `ros/config/hardware.yaml` | Hardware parameters (serial ports, wheel dims, motor limits, servo presets) |
| `ros/bringup/hardware.launch.py` | Launch: robot_state_publisher + ddsm_driver + st3215_driver |

## Modified Files (10)

| File | Changes |
|------|---------|
| `ros/CMakeLists.txt` | Add `find_package` (geometry_msgs, nav_msgs, sensor_msgs, tf2_ros), `ament_python_install_package(hardware)`, install 2 new nodes, add `urdf` to install dirs |
| `ros/package.xml` | Add deps: geometry_msgs, nav_msgs, sensor_msgs, tf2_ros, robot_state_publisher |
| `ros/setup/requirements.txt` | Add `pyserial>=3.5` |
| `ros/Dockerfile` | Add apt packages: ros-humble-robot-state-publisher, ros-humble-tf2-ros, ros-humble-joint-state-publisher |
| `docker-compose.yml` | Keep UART device lines commented (dev mode), add notes for Jetson |
| `dashboard/core/consumers.py` | Implement TelemetryConsumer: poll `/motor_feedback` via docker exec, stream to browser |
| `dashboard/core/templates/core/telemetry.html` | Wire real motor data to Chart.js charts/gauges, gate demo mode |
| `README.md` | Mark Stage 3 COMPLETE, add hardware nodes to status table |
| `recommendations.md` | Mark Stage 3 COMPLETE, update remaining areas |
| `ros/CLAUDE.md` | Document hardware nodes, `hardware.launch.py simulate:=true` |

---

## Implementation Order

### Step 1: Serial Protocol Modules (no ROS deps)

**`ros/hardware/__init__.py`** - empty init

**`ros/hardware/ddsm_serial.py`** - DDSM HAT communication:
- `DDSMSerial(port, baudrate=115200, simulate=False)`
- Auto-enters simulation mode if port can't open or pyserial missing
- `send_speed(left_rpm, right_rpm)` - JSON `{"T":1, "L":<x10>, "R":<x10>}`
- `read_feedback()` - dict with left/right speed, current, temp, encoder
- `stop()`, `close()`
- Thread-safe via Lock
- Speed commands clamped to [-2100, 2100] (0.1 rpm units, +-210 RPM range)

**`ros/hardware/st3215_serial.py`** - ST3215 servo communication:
- `ST3215Serial(port, baudrate=1000000, simulate=False)`
- Preset dictionary: bumper, tenhut, lookup, lookout, reach - servo angle arrays (0-4095 positions)
- Bearing-to-pan mapping: forward=2048, left=3072, right=1024, etc.
- `set_preset(name)`, `set_position(servo_id, position)`, `read_feedback(servo_id)`

### Step 2: Config, URDF, Dependencies

**`ros/config/hardware.yaml`** - parameters for both driver nodes:
```yaml
ddsm_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyTHS1"
    baud_rate: 115200
    simulate: false
    wheel_separation: 0.30      # meters
    wheel_radius: 0.05          # meters (DDSM210 hub wheel)
    max_linear_speed: 1.0       # m/s
    max_angular_speed: 3.0      # rad/s
    cmd_vel_timeout: 0.5        # seconds
    feedback_rate: 20.0         # Hz
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    encoder_counts_per_rev: 4096
    publish_tf: true

st3215_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 1000000
    simulate: false
    servo_ids: [1, 2]           # pan, tilt
    feedback_rate: 10.0         # Hz
    preset_bumper:  [2048, 3072]
    preset_tenhut:  [2048, 2048]
    preset_lookup:  [2048, 1024]
    preset_lookout: [2048, 1536]
    preset_reach:   [2048, 512]
```

**`ros/urdf/billybot.urdf.xml`** - URDF with:
- `base_footprint` -> `base_link` (chassis box 0.30x0.25x0.10m)
- 4 wheel links + continuous joints (cylinder r=0.05, l=0.04)
  - front_left:  xyz="0.10 0.15 -0.025"
  - front_right: xyz="0.10 -0.15 -0.025"
  - rear_left:   xyz="-0.10 0.15 -0.025"
  - rear_right:  xyz="-0.10 -0.15 -0.025"
- `arm_base_link` (fixed on top of base)
- `arm_pan_joint` (revolute, z-axis, +-3.14 rad)
- `arm_tilt_joint` (revolute, y-axis, +-1.57 rad)
- `camera_link` (fixed on tilt head)
- `imu_link` (fixed on base)

**Package dependency updates:**
- `ros/package.xml` - add geometry_msgs, nav_msgs, sensor_msgs, tf2_ros, robot_state_publisher
- `ros/setup/requirements.txt` - add pyserial>=3.5
- `ros/Dockerfile` - add ros-humble-robot-state-publisher, ros-humble-tf2-ros, ros-humble-joint-state-publisher
- `ros/CMakeLists.txt` - add find_package calls, ament_python_install_package(hardware), install nodes, add urdf to install dirs

### Step 3: DDSM Driver Node

**`ros/nodes/ddsm_driver_node.py`** - the core deliverable:

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist)

**Publishers:**
- `/odom` (nav_msgs/Odometry) - differential odometry from wheel encoders
- `/motor_feedback` (std_msgs/String) - JSON: {left_speed, right_speed, left_current, right_current, left_temp, right_temp, left_encoder, right_encoder}
- `/joint_states` (sensor_msgs/JointState) - wheel positions/velocities

**TF:**
- Broadcasts `odom` -> `base_link` via tf2_ros.TransformBroadcaster

**Logic:**
1. **cmd_vel callback**: Clamp to limits, apply skid-steer kinematics:
   ```
   left_vel  = (linear.x - angular.z * wheel_separation / 2) / wheel_radius
   right_vel = (linear.x + angular.z * wheel_separation / 2) / wheel_radius
   left_rpm  = left_vel * 60 / (2 * pi)
   right_rpm = right_vel * 60 / (2 * pi)
   ```
   Send via DDSMSerial.send_speed()

2. **Feedback timer** (20Hz): DDSMSerial.read_feedback() -> publish motor_feedback (JSON String), joint_states, compute differential odometry:
   ```
   delta_left  = (left_encoder - prev_left) / counts_per_rev * 2 * pi * wheel_radius
   delta_right = (right_encoder - prev_right) / counts_per_rev * 2 * pi * wheel_radius
   delta_s = (delta_left + delta_right) / 2
   delta_theta = (delta_right - delta_left) / wheel_separation
   x += delta_s * cos(theta + delta_theta/2)
   y += delta_s * sin(theta + delta_theta/2)
   theta += delta_theta
   ```
   Publish Odometry + TF transform

3. **Watchdog** (10Hz): If no cmd_vel for 0.5s, call DDSMSerial.stop()

4. **Shutdown**: Stop motors, close serial

### Step 4: ST3215 Servo Driver Node

**`ros/nodes/st3215_driver_node.py`**:

**Subscriptions:**
- `/grunt1/arm_preset` (std_msgs/String) - from command_processor
- `/joint_command` (sensor_msgs/JointState) - direct angle control

**Publishers:**
- `/joint_states` (sensor_msgs/JointState) - arm_pan_joint, arm_tilt_joint

**Services:**
- `/calibrate` (std_srvs/Trigger) - home all servos
- `/set_servo_mode` (std_srvs/SetBool) - toggle servo/motor mode

**Logic:**
1. **arm_preset callback**: Parse `"preset@bearing"` format, look up preset -> servo positions, apply bearing pan offset
2. **joint_command callback**: Direct servo position control from JointState message
3. **Feedback timer** (10Hz): Read servo positions -> publish joint_states

**Bearing-to-pan mapping:**
```
forward=2048, leftish=2560, left=3072, full-left=3584, back-left=4096
rightish=1536, right=1024, full-right=512, back-right=0, back=2048(flip)
```

### Step 5: Hardware Launch File

**`ros/bringup/hardware.launch.py`**:
- Reads URDF from share dir, passes as `robot_description` parameter to robot_state_publisher
- Launches ddsm_driver_node with hardware.yaml params
- Launches st3215_driver_node with hardware.yaml params
- `simulate` launch argument (default: false) - overrides simulate param on both drivers
- Follows existing pattern: DeclareLaunchArgument, Node, GroupAction, PushRosNamespace
- LogInfo startup message listing active hardware

### Step 6: Dashboard Telemetry

**`dashboard/core/consumers.py`** - TelemetryConsumer:
- Poll `/motor_feedback` topic via `docker exec ros2 topic echo --once`
- Parse JSON from String msg data field
- Send to browser WebSocket as `{"type": "motor_feedback", "data": {...}}`
- ~1Hz poll rate
- Handle timeout/errors gracefully

**`dashboard/core/templates/core/telemetry.html`**:
- Wire `motor_feedback` data to existing Chart.js speed charts (4 wheels) and temperature gauges
- Update speed chart datasets from feedback left/right speeds
- Update temperature gauges from feedback left/right temps
- Gate demo data generation (only when no real data flowing)

### Step 7: Docker Compose & Documentation

**`docker-compose.yml`** - keep UART commented with clear Jetson deployment notes
**`README.md`** - Stage 3 COMPLETE, hardware launch docs, status table update, new node list
**`recommendations.md`** - Stage 3 COMPLETE, update remaining areas
**`ros/CLAUDE.md`** - Document hardware package, nodes, launch command

---

## Architecture After Stage 3

```
Dashboard Joystick -> WebSocket -> docker exec -> ros2 topic pub /cmd_vel
                                                      |
                                      ddsm_driver_node (subscribes /cmd_vel)
                                        | skid-steer kinematics
                                      JSON {"T":1,"L":x,"R":y} -> UART -> DDSM HAT -> Motors
                                        | encoder feedback
                                      Publishes: /odom, /motor_feedback, /joint_states
                                        |
                                      Dashboard telemetry: real speed/temp charts

Dashboard Arm Preset -> WebSocket -> docker exec -> ros2 topic pub /arm_preset
                                                      |
                                      command_processor -> /grunt1/arm_preset
                                        |
                                      st3215_driver_node (subscribes /grunt1/arm_preset)
                                        | preset lookup
                                      TTL serial -> ST3215 servos -> arm moves
                                        | position feedback
                                      Publishes: /joint_states -> robot_state_publisher -> TF tree
```

---

## Topic Map After Stage 3

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | Twist | Dashboard joystick | ddsm_driver_node |
| `/odom` | Odometry | ddsm_driver_node | (future: nav2) |
| `/motor_feedback` | String (JSON) | ddsm_driver_node | Dashboard telemetry |
| `/joint_states` | JointState | ddsm_driver + st3215_driver | robot_state_publisher |
| `/grunt1/arm_preset` | String | command_processor | st3215_driver_node |
| `/joint_command` | JointState | (direct control) | st3215_driver_node |
| `/tf` | TransformStamped | ddsm_driver + robot_state_publisher | (rviz, nav2) |

---

## Verification Checklist

1. **Build**: `docker compose build ros2-byc` succeeds with new deps
2. **Launch (sim)**: `ros2 launch by_your_command hardware.launch.py simulate:=true`
   - `ros2 node list` shows ddsm_driver_node, st3215_driver_node, robot_state_publisher
   - `ros2 topic list` shows /cmd_vel, /odom, /motor_feedback, /joint_states, /tf
3. **Joystick -> Driver**: Move dashboard joystick -> ddsm_driver_node logs cmd_vel receipt
4. **Odometry**: `ros2 topic echo /odom` shows pose data (zeros in sim)
5. **TF tree**: `ros2 run tf2_tools view_frames` shows odom -> base_link -> wheels + arm chain
6. **Telemetry**: Dashboard telemetry page shows motor data (simulated values)
7. **Arm presets**: Dashboard "bumper" button -> st3215_driver_node logs preset
8. **Watchdog**: Stop joystick -> motors zeroed after 0.5s (check logs)
9. **Graceful sim**: No errors when running without physical serial ports

---

## Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| No serial hardware on dev machine | Simulation mode auto-activates; pyserial import guarded |
| pyserial not in ROS image | Added to requirements.txt; ImportError caught |
| robot_state_publisher missing | Added to Dockerfile apt install |
| Joint name collision (DDSM + ST3215) | Distinct names: wheel joints vs arm joints |
| DDSM JSON protocol mismatch | Matches ugv_base_ros reference; configurable via parameters |
| URDF dimensions wrong | Placeholder values; functional TF tree; tune on real robot |
| 4WD odometry accuracy | 4WD skid-steer treated as 2WD diff-drive (left pair = right pair); good enough for basic nav |
| motor_feedback as String not custom msg | Pragmatic: avoids .msg rebuild cycle; JSON easy to parse; upgrade later |
