# Hardware Tech Stack

Hardware used in this project for the voice-controlled robot platform (By Your Command). Covers main compute, drive system, hub motors, vision/depth cameras, serial servos, and voice hardware.

---

## Main compute

### NVIDIA Jetson Orin NX or Jetson Orin Nano

- **Role:** Host / upper computer for ROS 2, AI (LLM, vision), and high-level control.
- **Relevance:** Runs `by_your_command` (Silero VAD, OpenAI Realtime / Gemini Live), strategy, and sends motion/JSON commands to the lower computer over UART.
- **Notes:**
  - Orin NX: higher performance, more power; Orin Nano: smaller, lower power.
  - Both support ROS 2 and the audio/vision stack described in CLAUDE.md.
  - Jetson is explicitly supported as host in Waveshare UGV docs (GPIO UART to driver HAT).

---

## Drive system (lower computer + HAT)

### Waveshare DDSM Driver HAT (A)

- **Role:** Driver board for DDSM hub motors; lower computer interface for motion and telemetry.
- **Links:**
  - [Waveshare Wiki – DDSM Driver HAT (A)](http://www.waveshare.com/wiki/DDSM_Driver_HAT_(A))
- **Summary:**
  - 4× DDSM115 motor interfaces, 4× DDSM210 interfaces (use one motor type at a time per bus).
  - Onboard ESP32; JSON command interface over UART/USB @ 115200.
  - Power: DC5525 or XT60; 9–28 V (DDSM115: 12–24 V, DDSM210: 11–22 V). Can power Raspberry Pi via onboard 5 V buck.
  - Control: JSON over UART (e.g. from Jetson GPIO UART or USB), or Web UI at `192.168.4.1` (ESP32-AP, default password `12345678`).
- **Relevance:** Sits between Jetson (ROS 2 / by_your_command) and the DDSM210 hub motors; accepts JSON motion commands and returns motor feedback (speed, current, temperature, etc.).

### Waveshare UGV Base ROS (lower-computer firmware / ROS driver)

- **Role:** ESP32 program and ROS driver for Waveshare UGV platforms; reference for integrating DDSM HAT with ROS.
- **Links:**
  - [GitHub – waveshareteam/ugv_base_ros](https://github.com/waveshareteam/ugv_base_ros)
- **Summary:**
  - ESP32 lower computer for UGV Rover, UGV Beast, RaspRover, UGV02 (ROS Driver variant).
  - Host (Jetson Orin / Raspberry Pi) talks to ESP32 via **GPIO UART** with **JSON commands**.
  - Features: closed-loop speed control (PID), Web app, IMU, OLED, LED control, camera PT, RoArm-M2 support, ESP-NOW.
  - Robot type config example: `{"T":900,"main":2,"module":2}` (e.g. main=2 → UGV Rover, module=2 → Camera PT).
- **Relevance:** Shows how to run the lower computer and how ROS/host talks to the DDSM ecosystem (JSON over UART); can inform a ROS 2 driver or bridge for this project.

---

## Hub motors (drive wheels)

### Waveshare DDSM210 – Direct drive servo hub motor

- **Role:** Wheel drive units (e.g. 4WD UGV); speed/position/current feedback.
- **Links:**
  - [Waveshare Wiki – DDSM210](http://www.waveshare.com/wiki/DDSM210)
- **Specifications:**
  - **No-load speed:** 210 ± 11 rpm  
  - **Rated:** 98 rpm, 0.25 Nm, 0.5 A  
  - **Locked-rotor:** 0.85 Nm, ≤ 3 A  
  - **Voltage:** 11–22 V DC  
  - **Encoder:** 4096 counts/rev (relative precision 1024)  
  - **Modes:** Open loop (DDSM210 only), speed loop, position loop (0–360°). Default: speed loop.  
  - **Communication:** UART, 115200, 8N1, 10-byte frames, CRC-8/MAXIM; one question–one answer, up to 500 Hz.  
  - **Interface:** ZH1.5 4P (UART Tx/Rx, GND, VCC).  
  - **Environment:** -25 °C to 45 °C; over-temperature protection at 80 °C.  
  - **Single-wheel load:** 3 kg (design reference).
- **Relevance:** Primary drive motor for the platform; controlled via DDSM Driver HAT (A) using JSON (e.g. `CMD_DDSM_CTRL` for speed, mode switch, heartbeat). Speed loop: cmd in 0.1 rpm (-2100–2100 ↔ -210–210 rpm). Position loop: 0–32767 ↔ 0–360°.

---

## Serial bus servos (arm / pan-tilt / joints)

### Waveshare ST3215 serial bus servo

- **Role:** Joints for robot arm, pan-tilt, or other articulated parts (not wheel drive).
- **Links:**
  - [Waveshare Wiki – ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo)  
  - [Waveshare ST3215 product](https://www.waveshare.com/st3215-servo.htm)
- **Specifications:**
  - **Voltage:** 6–12.6 V (12 V recommended).  
  - **Torque:** 30 kg·cm @ 12 V; 19.5 kg·cm @ 7.4 V.  
  - **Rotation:** 360° in servo mode (absolute angle); continuous in motor/stepper mode.  
  - **Encoder:** 360° magnetic, 4096 steps (360°/4096 resolution).  
  - **Baud rate:** 1 Mbps.  
  - **Idle speed:** ~0.222 s/60° (≈ 45 RPM) @ 12 V.  
  - **ID range:** 0–253; daisy-chain, up to 253 servos on one bus (with adequate power).  
  - **Feedback:** Position, load, speed, input voltage.  
  - **Modes:** Servo (angle) or motor/stepper (continuous); programmable middle position; built-in acceleration.  
  - **Dimensions:** 45.22 × 35 × 24.72 mm.  
  - **Variant:** ST3215-HS – 20 kg·cm, 106 RPM (faster).
- **Relevance:** For arms or camera PT; typically used with a separate servo driver (e.g. [Servo Driver with ESP32](https://www.waveshare.com/servo-driver-with-esp32.htm)) and SCServo library. ugv_base_ros supports “Camera PT” and “RoArm-M2”; ST3215 fits similar roles. Control is TTL serial (not the DDSM JSON protocol).

---

## Vision / depth cameras

Use **one of** the following for depth, RGB, and (where applicable) IMU input. Both have strong ROS 2 support and suit robotic navigation, object recognition, and vision-based LLM features (e.g. Gemini Live "what do you see?").

### Intel RealSense D435i

- **Role:** Stereo depth camera with onboard IMU for depth, RGB, and motion-aware perception.
- **Links:**
  - [RealSense D435i – Depth Camera](https://www.realsenseai.com/products/depth-camera-d435i/)
- **Specifications:**
  - **Depth:** Stereoscopic, global shutter; ideal range **0.3 m–3 m**; min-Z ~28 cm at max resolution. Accuracy **<2% at 2 m**. FOV **87° × 58°**. Up to **1280×720 @ 90 fps**.
  - **RGB:** 1920×1080 @ 30 fps, rolling shutter; FOV 69° × 42°, 2 MP.
  - **IMU:** 6DoF (gyro + accel); timestamps aligned with depth in RealSense SDK 2.0. Supports SLAM, tracking, handheld scanning, robotics.
  - **Physical:** 90 × 25 × 25 mm; USB-C 3.1 Gen 1; 1/4‑20 UNC + two M3 mounting points.
  - **Environment:** Indoor/outdoor.
- **Relevance:** Compact, well-supported on Linux/Jetson; `ros2_intel_realsense` and RealSense SDK 2.0 provide depth, RGB, and IMU for navigation, mapping, or feeding vision to By Your Command (e.g. Gemini Live).

### Stereolabs ZED 2

- **Role:** AI stereo camera with wide FOV, neural depth, and built-in sensors; full SDK (positional tracking, mapping, object/body detection).
- **Links:**
  - [Stereolabs Documentation](https://www.stereolabs.com/docs)  
  - [ZED 2 – AI Stereo Camera](https://stereolabs.com/products/zed-2)
- **Specifications:**
  - **FOV:** **110° (H) × 70° (V)** (diagonal ~120°); 8-element all-glass lens, optically corrected.
  - **Sensors:** Dual 1/3" 4 MP CMOS (2688×1520); output up to 2× 2208×1242 @ 15 fps.
  - **Depth:** Neural stereo matching; range on same-family 2i: **0.3 m–20 m** (ZED 2 in similar range). Depth accuracy <1% up to 3 m, <5% up to 15 m (2i reference); baseline 12 cm.
  - **Sensors:** Built-in **IMU, barometer, magnetometer**; f/1.8, improved low-light.
  - **SDK:** Depth, positional tracking, spatial mapping, object detection, body tracking, ROS 2 wrapper, Isaac ROS, Docker, GStreamer, etc.
- **Relevance:** Wide FOV and neural depth suit robotic navigation and "what do you see?" style vision; [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and [Stereolabs ROS 2 docs](https://www.stereolabs.com/docs/ros2/) integrate with By Your Command's vision pipeline (e.g. Gemini).

---

## Voice hardware (optional / alternative)

### Yahboom intelligent voice speech recognition module (CSK4002)

- **Role:** Off-the-shelf voice recognition and broadcast; optional alternative or complement to by_your_command’s microphone + Silero VAD + cloud LLM.
- **Links:**
  - [Yahboom – Voice interaction module](https://category.yahboom.net/collections/ros-accessories/products/voice-interaction)
- **Summary:**
  - **Chip:** CSK4002; voice recognition compute up to 128 GOPS.  
  - **Wake word:** Fixed (e.g. “Hi Yahboom”; not user-configurable).  
  - **Range:** ~5 m, 360°; wake rate ~95% in various environments.  
  - **Commands:** 85 built-in commands (no user-defined wake word; limited custom command registration per product notes).  
  - **Interfaces:** USB and serial; Type-C and Dupont.  
  - **Audio:** Onboard mics; speaker for playback.  
  - **Typical use:** Smart car / robotic arm voice control over serial or USB.
- **Relevance:** If the project uses a simpler, on-device voice pipeline instead of (or in addition to) by_your_command, this module fits ROS accessories and can drive discrete commands. By Your Command remains the path for open-ended conversation and custom wake/phrasing via Silero VAD + LLM.

---

## Integration summary

| Layer            | Hardware                      | Interface to project                          |
|-----------------|-------------------------------|-----------------------------------------------|
| Host            | Jetson Orin NX / Orin Nano    | Runs ROS 2 + by_your_command                 |
| Vision (one of) | RealSense D435i or ZED 2      | USB; depth + RGB + IMU → ROS 2 / Gemini      |
| Lower computer  | DDSM Driver HAT (A) + ugv_base_ros | JSON over UART (GPIO or USB 115200)   |
| Drive wheels    | DDSM210 × N                   | Via DDSM HAT (speed/position/current)        |
| Arm / PT        | ST3215 (optional)             | Separate TTL bus + driver (e.g. ESP32)       |
| Voice (optional)| Yahboom CSK4002               | Serial/USB for fixed-command voice          |

---

## References

- DDSM Driver HAT (A): [Waveshare Wiki](http://www.waveshare.com/wiki/DDSM_Driver_HAT_(A))  
- UGV ROS driver: [ugv_base_ros](https://github.com/waveshareteam/ugv_base_ros)  
- DDSM210: [Waveshare Wiki – DDSM210](http://www.waveshare.com/wiki/DDSM210)  
- ST3215: [Waveshare Wiki – ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo)  
- RealSense D435i: [RealSense D435i](https://www.realsenseai.com/products/depth-camera-d435i/)  
- ZED 2: [Stereolabs Docs](https://www.stereolabs.com/docs), [ZED 2 product](https://stereolabs.com/products/zed-2)  
- Yahboom voice: [Voice interaction module](https://category.yahboom.net/collections/ros-accessories/products/voice-interaction)
