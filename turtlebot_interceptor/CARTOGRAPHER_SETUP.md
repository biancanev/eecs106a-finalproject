# Cartographer SLAM + EKF Setup Guide

## Overview

The navigation stack now uses **Google Cartographer** for SLAM instead of custom log-odds mapping. This provides:

- **Superior mapping quality** (loop closure, sub-mapping)
- **Better obstacle detection** (especially for small objects like cones)
- **Robust sensor fusion** (LIDAR + IMU + Odometry)
- **Production-grade reliability**

## Architecture

```
┌─────────────┐     ┌──────────────┐
│   /imu      │────▶│ Cartographer │
└─────────────┘     │     SLAM     │
                    │              │
┌─────────────┐     │  (Google's   │     ┌──────────┐
│   /scan     │────▶│  production  │────▶│   /map   │
└─────────────┘     │  SLAM)       │     └──────────┘
                    │              │
┌─────────────┐     │              │     ┌──────────────┐
│   /odom     │────▶│              │────▶│ /tracked_pose│
└─────────────┘     └──────────────┘     └──────────────┘
                                                  │
                                                  ▼
                                         ┌──────────────┐
                                         │ Pose Bridge  │
                                         │  (optional)  │
                                         └──────┬───────┘
                                                │
                                                ▼
                                         ┌──────────────┐
                                         │ /amcl_pose   │
                                         └──────────────┘
```

## Two State Estimation Options

### Option 1: Cartographer Pose (Recommended)

**What:** Use Cartographer's built-in pose tracking (`/tracked_pose`)

**Pros:**
- Already fused with SLAM (consistent map & pose)
- Loop closure corrections automatically applied
- Production-tested on TurtleBot3
- No additional drift from separate estimator

**Cons:**
- No magnetometer fusion (Cartographer uses LIDAR + IMU + Odom)

**How:** The `cartographer_pose_bridge` republishes `/tracked_pose` as `/amcl_pose` for MPC compatibility.

### Option 2: EKF Pose Estimator (For Advanced Users)

**What:** Custom EKF fusing IMU + Magnetometer + Wheel Encoders

**Pros:**
- Magnetometer provides absolute heading reference (no gyro drift)
- Gravity-compensated acceleration for better velocity estimates
- Independent of SLAM (can work without map)

**Cons:**
- Separate from SLAM (pose may not align perfectly with map)
- Requires magnetometer calibration (robot must rotate full circle at startup)
- More complex to tune

**How:** The `ekf_pose_estimator` node subscribes to `/imu`, `/magnetic_field`, `/odom` and publishes `/amcl_pose`.

## Launch File Configuration

Currently configured for **Option 1 (Cartographer Pose)**.

To switch to **Option 2 (EKF)**:

1. Comment out `cartographer_pose_bridge` node
2. Uncomment `ekf_pose_estimator` node

```python
# In single_robot_navigation.launch.py

# OPTION 1: Cartographer Pose (CURRENT)
Node(
    package='turtlebot_interceptor',
    executable='cartographer_pose_bridge',
    ...
),

# OPTION 2: EKF (UNCOMMENT TO USE)
# Node(
#     package='turtlebot_interceptor',
#     executable='ekf_pose_estimator',
#     ...
# ),
```

## Topics

### Cartographer Publishes

- `/map` - `nav_msgs/OccupancyGrid` (2cm resolution)
- `/tracked_pose` - `geometry_msgs/PoseStamped` (robot pose in map frame)
- `/submap_list` - Cartographer's internal submaps
- `/scan_matched_points2` - Point cloud after scan matching

### Cartographer Subscribes

- `/scan` - `sensor_msgs/LaserScan` (LIDAR data)
- `/imu` - `sensor_msgs/Imu` (accelerometer + gyroscope)
- `/odom` - `nav_msgs/Odometry` (wheel encoders)

### EKF Publishes

- `/amcl_pose` - `geometry_msgs/PoseWithCovarianceStamped` (fused pose estimate)

### EKF Subscribes

- `/imu` - `sensor_msgs/Imu` (accelerometer + gyroscope)
- `/magnetic_field` - `sensor_msgs/MagneticField` (magnetometer)
- `/odom` - `nav_msgs/Odometry` (wheel encoders)

## Build and Run

```bash
# Build
cd ~/eecs106a-finalproject
colcon build --packages-select turtlebot_interceptor
source install/setup.bash

# Launch (with Cartographer)
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=1.5
```

## Verification

### Check Cartographer is Running

```bash
# Check nodes
ros2 node list | grep cartographer
# Should see:
#   /cartographer_node
#   /cartographer_occupancy_grid_node
#   /cartographer_pose_bridge

# Check topics
ros2 topic list | grep -E "(map|tracked_pose)"
# Should see:
#   /map
#   /tracked_pose

# Echo map (should have data)
ros2 topic echo /map --once

# Echo pose
ros2 topic echo /tracked_pose --once
```

### Check EKF is Running (if using Option 2)

```bash
# Check node
ros2 node list | grep ekf
# Should see: /ekf_pose_estimator

# Check magnetometer calibration
ros2 topic echo /magnetic_field --once

# Echo pose
ros2 topic echo /amcl_pose --once
```

## Troubleshooting

### Cartographer Not Starting

**Error:** `Cannot find configuration file`

**Fix:** Install TurtleBot3 Cartographer package:
```bash
sudo apt install ros-humble-turtlebot3-cartographer
```

### Poor Map Quality

**Issue:** Map is noisy or inconsistent

**Fixes:**
1. Ensure LIDAR is clean and unobstructed
2. Move robot slowly (< 0.3 m/s) during initial mapping
3. Complete at least one loop closure (return to start position)
4. Check IMU data is valid: `ros2 topic echo /imu --once`

### EKF Magnetometer Issues

**Issue:** Heading drifting or jumping

**Fixes:**
1. Ensure magnetometer calibration completed (first 100 samples)
2. Keep away from metal/magnetic interference
3. Rotate robot in full circle slowly at startup
4. Check: `ros2 topic echo /magnetic_field --once`

### Pose Not Published

**Issue:** `/amcl_pose` has no data

**Fixes:**
1. Check which option is enabled (bridge vs EKF)
2. For bridge: Verify `/tracked_pose` exists
3. For EKF: Verify `/imu`, `/magnetic_field`, `/odom` exist
4. Check node logs: `ros2 node info <node_name>`

## Performance

### Cartographer (Recommended)

- **Map update rate:** ~1-2 Hz (adaptive)
- **Pose update rate:** ~10 Hz
- **CPU usage:** ~30% on TurtleBot3 (RaspberryPi 4)
- **Memory:** ~200 MB

### EKF

- **Pose update rate:** 100 Hz
- **CPU usage:** ~5%
- **Memory:** ~20 MB

## Notes

- **Magnetometer calibration:** EKF requires robot to rotate slowly in place for first 10 seconds
- **Loop closure:** Cartographer automatically corrects drift when revisiting locations
- **Static obstacles:** Cartographer is designed for static environments (perfect for cone courses)
- **Target tracking:** Moving targets are handled by UKF node (separate from SLAM)
- **Map clearing:** Cartographer does NOT clear obstacles (they're permanent) - exactly what we need!

## Switching Between Options

You can hot-swap between options without rebuilding:

```bash
# Stop current launch
Ctrl+C

# Edit launch file
nano ~/eecs106a-finalproject/turtlebot_interceptor/launch/single_robot_navigation.launch.py

# Toggle comments on cartographer_pose_bridge vs ekf_pose_estimator

# Relaunch (no rebuild needed if only changing which node runs)
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py
```


