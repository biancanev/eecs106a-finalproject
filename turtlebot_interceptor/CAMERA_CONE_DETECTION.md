# Camera-Based Yellow Cone Detection

## Overview

This system uses computer vision to detect yellow cones when the robot is close (< 1.5m). It provides high-accuracy obstacle positions for the MPC controller.

## Architecture

```
Camera Feed → Color Detection (HSV) → 3D Estimation → Map Transform → MPC
```

## Features

- **Color-based detection**: Uses HSV color space to detect yellow cones
- **Automatic depth estimation**: Estimates 3D position from pixel coordinates
- **Map frame integration**: Transforms detections to map frame for MPC
- **Priority system**: Camera detections have highest priority when close
- **Debug visualization**: Shows detections in RViz

## Setup

### 1. Install Dependencies

```bash
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-v4l2-camera
pip3 install opencv-python
```

### 2. Connect Camera

For USB camera (e.g., Logitech):
```bash
ros2 run v4l2_camera v4l2_camera_node \
    --ros-args \
    -p device:=/dev/video0 \
    -p image_size:="640x480"
```

### 3. Launch Camera Detector

```bash
ros2 run turtlebot_interceptor camera_cone_detector
```

### 4. Launch Full System

The camera detector integrates automatically with MPC. Just launch your navigation system:
```bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py
```

## Tuning HSV Values

The default yellow range is:
- **Lower**: HSV(20, 100, 100)
- **Upper**: HSV(30, 255, 255)

To tune for your specific yellow cones:

1. View camera feed:
```bash
ros2 run rqt_image_view rqt_image_view
# Subscribe to /camera/image_raw
```

2. View debug image with detections:
```bash
# Subscribe to /camera_cones_debug
```

3. Adjust values in `camera_cone_detector.py`:
```python
self.lower_yellow = np.array([H_min, S_min, V_min])
self.upper_yellow = np.array([H_max, S_max, V_max])
```

## Topics

### Subscribed
- `/camera/image_raw` - Camera feed
- `/camera/camera_info` - Camera intrinsic parameters
- `/amcl_pose` - Robot pose for coordinate transforms

### Published
- `/camera_cone_positions` - `PointStamped` messages (for MPC)
- `/camera_cones` - `MarkerArray` for visualization
- `/camera_cones_debug` - Debug image with detections overlaid

## Integration with MPC

The MPC node automatically subscribes to `/camera_cone_positions` and uses camera detections with **highest priority** when:
- Robot is within 1.5m of detected cones
- Camera detections are recent (< 0.5 seconds old)

Priority order:
1. **Camera** (when close)
2. Fast Local Grid
3. Scan-Matched Points
4. Raw LIDAR

## Upgrading to YOLO

To use YOLO for better accuracy:

1. Install YOLO:
```bash
pip3 install ultralytics
```

2. Train or download a model:
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # or your trained model
```

3. Replace `detect_yellow_cones()` in `camera_cone_detector.py`:
```python
def detect_yellow_cones(self, cv_image):
    results = model(cv_image)
    cones = []
    for result in results:
        for box in result.boxes:
            if box.cls == 'cone':  # or your class ID
                x1, y1, x2, y2 = box.xyxy[0]
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                w = x2 - x1
                h = y2 - y1
                cones.append((cx, cy, w, h, w*h))
    return cones
```

## Training YOLO on Yellow Cones

1. Collect dataset:
   - Record video of yellow cones
   - Extract frames
   - Label with bounding boxes (use LabelImg or similar)

2. Train model:
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.train(data='path/to/dataset.yaml', epochs=100)
```

3. Use trained model:
```python
model = YOLO('best.pt')  # Your trained model
```

## Troubleshooting

### No detections
- Check camera is publishing: `ros2 topic echo /camera/image_raw`
- Adjust HSV values for your lighting conditions
- Check minimum cone area (default: 500 pixels)

### Wrong positions
- Verify camera_info is being published
- Check TF tree: `ros2 run tf2_tools view_frames`
- Ensure camera_link → base_link transform exists

### Poor accuracy
- Use stereo camera or depth camera for better depth estimation
- Upgrade to YOLO for better detection
- Train YOLO on your specific cone types

## Future Improvements

- [ ] Stereo vision for accurate depth
- [ ] YOLO integration
- [ ] Multi-cone tracking
- [ ] Cone size estimation
- [ ] Occlusion handling

