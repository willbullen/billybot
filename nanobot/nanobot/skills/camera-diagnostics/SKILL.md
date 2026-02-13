---
name: camera-diagnostics
description: Diagnose BillyBot camera system - check RealSense D435i node health, topic publish rates, depth stream, parameters, and image quality.
---

# Camera Diagnostics

Use the `ros2` tool to inspect and troubleshoot the BillyBot camera system.

## Quick Health Check

1. Verify the camera node is running:
```
ros2(action="nodes")
```
Look for `/camera_driver_node` in the list.

2. Check all camera topics are publishing:
```
ros2(action="exec", command="ros2 topic hz /camera/color/compressed --window 5 & ros2 topic hz /camera/depth/image_rect_raw --window 5 & sleep 3 && kill %1 %2 2>/dev/null")
```

3. Get node details:
```
ros2(action="node_info", node="/camera_driver_node")
```

## Topics to Check

| Topic | Type | Expected Rate |
|-------|------|---------------|
| `/camera/color/image_raw` | sensor_msgs/Image | ~15 Hz |
| `/camera/color/compressed` | sensor_msgs/CompressedImage | ~15 Hz |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | ~15 Hz |
| `/camera/camera_info` | sensor_msgs/CameraInfo | ~15 Hz |

## Parameter Inspection

Check current camera configuration:
```
ros2(action="params", node="/camera_driver_node")
```

Key parameters:
```
ros2(action="param_get", node="/camera_driver_node", param="simulate")
ros2(action="param_get", node="/camera_driver_node", param="publish_rate")
ros2(action="param_get", node="/camera_driver_node", param="width")
ros2(action="param_get", node="/camera_driver_node", param="height")
ros2(action="param_get", node="/camera_driver_node", param="fps")
ros2(action="param_get", node="/camera_driver_node", param="enable_depth")
ros2(action="param_get", node="/camera_driver_node", param="jpeg_quality")
```

## Common Issues

### No camera topics publishing
- Check if node is running: `ros2(action="nodes")`
- Check container logs: `ros2(action="log", lines=50)`
- If `simulate` is false and no RealSense connected, the node may have failed to start

### Low frame rate
- Check `publish_rate` parameter (default 15.0 Hz)
- Check CPU usage: `ros2(action="exec", command="top -bn1 | head -20")`
- Reduce resolution or JPEG quality if needed

### Depth stream not working
- Verify `enable_depth` is true: `ros2(action="param_get", node="/camera_driver_node", param="enable_depth")`
- Check depth topic: `ros2(action="topic_info", topic="/camera/depth/image_rect_raw")`

### Image quality issues
- Check JPEG quality (default 80): `ros2(action="param_get", node="/camera_driver_node", param="jpeg_quality")`
- Adjust quality: `ros2(action="param_set", node="/camera_driver_node", param="jpeg_quality", value="90")`

## Snapshot Test

Echo a single compressed image message to verify data flow:
```
ros2(action="topic_echo", topic="/camera/color/compressed", count=1)
```
