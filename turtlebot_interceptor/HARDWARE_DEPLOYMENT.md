# Hardware Deployment Guide - Step-by-Step Instructions

Complete guide for deploying the TurtleBot Interceptor system on real hardware.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [System Overview](#system-overview)
3. [Installation](#installation)
4. [Hardware Setup](#hardware-setup)
5. [Configuration](#configuration)
6. [Launching the System](#launching-the-system)
7. [Verification & Testing](#verification--testing)
8. [Troubleshooting](#troubleshooting)
9. [Advanced Configuration](#advanced-configuration)

---

## Prerequisites

### Required Hardware
- **TurtleBot3** (Burger, Waffle, or Waffle Pi)
- **LIDAR sensor** (e.g., RPLIDAR A1/A2, HLS-LFCD2)
- **Computer** running ROS2 (Ubuntu 20.04/22.04)
- **Target tracking system** (camera + object detection OR external tracking)

### Required Software
- **ROS2** (Humble or Foxy)
- **Python 3.8+**
- **Required Python packages:**
  ```bash
  sudo apt-get update
  sudo apt-get install -y \
    python3-pip \
    python3-cvxpy \
    python3-transforms3d \
    python3-matplotlib \
    python3-numpy \
    python3-rosdep
  ```

### ROS2 Workspace Setup
```bash
# Create workspace (if not exists)
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone this repository
git clone <your-repo-url> turtlebot_interceptor

# Install dependencies
cd ~/turtlebot3_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## System Overview

### Node Architecture

```
┌─────────────────┐
│  LIDAR Sensor   │
└────────┬────────┘
         │ /scan
         ▼
┌─────────────────┐     ┌──────────────┐
│ LIDAR Processor │────▶│  SLAM Node   │
│     Node        │     │  (Mapping)   │
└─────────────────┘     └──────┬───────┘
                                │ /map
         ┌──────────────────────┼──────────────────────┐
         │                      │                      │
         ▼                      ▼                      ▼
┌─────────────────┐     ┌──────────────┐     ┌──────────────┐
│   MCL Node      │     │  MPC Node    │     │  UKF Node    │
│ (Localization)  │     │  (Control)   │     │ (Tracking)   │
└────────┬────────┘     └──────┬───────┘     └──────┬───────┘
         │                      │                      │
         │ /amcl_pose           │ /cmd_vel             │ /target_estimate
         │                      │                      │
         └──────────────────────┼──────────────────────┘
                                │
                                ▼
                         ┌──────────────┐
                         │ Robot Base   │
                         │  Controller │
                         └──────────────┘
```

### Topic Flow

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR driver | LIDAR processor, SLAM, MCL | Raw LIDAR data |
| `/scan_processed` | `sensor_msgs/LaserScan` | LIDAR processor | (optional) | Filtered LIDAR |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM node | MCL, MPC | Occupancy grid map |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | MCL node | SLAM, MPC | Robot pose estimate |
| `/target_pose_measurement` | `geometry_msgs/PoseStamped` | Target measurement node | UKF node | Target measurements (artificial or LIDAR) |
| `/target_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | (Optional) External source | Target measurement node | External target estimate |
| `/target_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | UKF node | MPC node | Target state estimate |
| `/cmd_vel` | `geometry_msgs/Twist` | MPC node | MCL, Robot base | Velocity commands |

---

## Installation

### Step 1: Build the Package

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

**Verify installation:**
```bash
# Check nodes are available
ros2 pkg executables turtlebot_interceptor

# Should show:
# turtlebot_interceptor lidar_processor_node
# turtlebot_interceptor slam_node
# turtlebot_interceptor mcl_node
# turtlebot_interceptor target_measurement_node
# turtlebot_interceptor ukf_node
# turtlebot_interceptor mpc_node
```

### Step 2: Verify Dependencies

```bash
# Test Python imports
python3 -c "import cvxpy; import transforms3d; import numpy; print('All dependencies OK')"

# Test ROS2 topics
ros2 topic list  # Should work without errors
```

---

## Hardware Setup

### Step 1: Connect LIDAR

1. **Physical connection:**
   - Connect LIDAR to robot's USB port
   - Power on LIDAR (check LED indicators)

2. **Verify LIDAR driver:**
   ```bash
   # Check LIDAR is detected
   lsusb | grep -i lidar
   
   # Launch LIDAR driver (example for RPLIDAR)
   ros2 launch rplidar_ros rplidar.launch.py
   
   # In another terminal, verify data
   ros2 topic echo /scan --once
   ```

3. **Expected output:**
   - Topic `/scan` should publish `sensor_msgs/LaserScan` messages
   - Check `range_min`, `range_max`, `ranges` array is populated

### Step 2: Setup Robot Base Controller

1. **Verify robot base controller:**
   ```bash
   # Check if robot base controller is running
   ros2 topic list | grep cmd_vel
   
   # Test robot movement (CAREFUL - robot will move!)
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
   ```

2. **Stop robot:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
   ```

### Step 3: Setup Target Tracking

**The `target_measurement_node` handles target measurements automatically!**

**Option A: With External Source (Camera/GPS/etc.)**
```bash
# Your external tracking system should publish to /target_estimate
# The target_measurement_node will use it in artificial mode
ros2 topic echo /target_estimate
```

**Option B: LIDAR-Only Mode (No External Source)**
```bash
# The node will work with just LIDAR
# It will use LIDAR detection when target is within range (terminal homing)
# No additional setup needed!
```

**Option C: Manual Testing (for development)**
```bash
# Publish test target to external source topic
ros2 topic pub -r 10 /target_estimate geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, \
   pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, \
                 orientation: {w: 1.0}}}}"
```

**Note:** The `target_measurement_node` automatically:
- Publishes artificial measurements at configurable rate (default: 10 Hz)
- Monitors LIDAR for target detection
- Switches to terminal homing when target is within 2.0m
- Uses LIDAR-based detection for precise final approach

---

## Configuration

### Step 1: Configure Launch File Parameters

Edit `launch/hardware.launch.py` or create a custom config:

```python
# Example: Create custom_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_interceptor',
            executable='slam_node',
            name='slam_node',
            parameters=[{
                'map_width': 200,        # Larger map for bigger environment
                'map_height': 200,
                'resolution': 0.05,      # 5cm resolution
                'origin_x': -5.0,        # Adjust for your environment
                'origin_y': -5.0,
            }],
        ),
        # ... other nodes
    ])
```

### Step 2: Tune MCL Parameters

For better localization, adjust in launch file:
```python
Node(
    package='turtlebot_interceptor',
    executable='mcl_node',
    parameters=[{
        'num_particles': 500,           # More particles = better accuracy
        'motion_noise_x': 0.02,
        'motion_noise_y': 0.02,
        'motion_noise_theta': 0.01,
    }],
)
```

### Step 2: Tune Target Measurement Node Parameters

For terminal homing behavior:
```python
Node(
    package='turtlebot_interceptor',
    executable='target_measurement_node',
    parameters=[{
        'artificial_rate': 10.0,        # Hz - measurement rate in artificial mode
        'lidar_detection_range': 2.0,   # m - distance to activate terminal homing
        'target_radius': 0.15,          # m - physical target radius for LIDAR detection
        'use_external_source': True,     # Use external source if available
    }],
)
```

### Step 3: Tune MPC Parameters

For faster/safer control:
```python
Node(
    package='turtlebot_interceptor',
    executable='mpc_node',
    parameters=[{
        'mpc_horizon': 20,              # Longer horizon = better planning
        'v_max_base': 0.4,              # Adjust for your robot
        'omega_max': 1.0,               # Adjust for your robot
    }],
)
```

---

## Launching the System

### Method 1: Full System Launch (Recommended)

**Step 1: Open terminal and source workspace:**
```bash
cd ~/turtlebot3_ws
source install/setup.bash
```

**Step 2: Launch all nodes:**
```bash
ros2 launch turtlebot_interceptor hardware.launch.py
```

**Expected output:**
```
[INFO] [slam_node]: SLAM node initialized - log-odds occupancy grid mapping
[INFO] [mcl_node]: MCL initialized with 300 particles
[INFO] [target_measurement_node]: Target measurement node initialized - artificial rate: 10.0Hz, terminal homing range: 2.0m
[INFO] [ukf_node]: UKF node initialized - ready for hardware
[INFO] [mpc_node]: MPC node initialized (lab8 control patterns)
```

### Method 2: Individual Nodes (For Debugging)

**Terminal 1 - LIDAR Processor:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor lidar_processor_node
```

**Terminal 2 - SLAM:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor slam_node
```

**Terminal 3 - MCL:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor mcl_node
```

**Terminal 4 - Target Measurement Node:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor target_measurement_node
```

**Terminal 5 - UKF:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor ukf_node
```

**Terminal 6 - MPC:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot_interceptor mpc_node
```

---

## Verification & Testing

### Step 1: Verify Topics Are Publishing

```bash
# Check all topics are active
ros2 topic list

# Should see:
# /scan
# /map
# /amcl_pose
# /target_estimate
# /cmd_vel
```

### Step 2: Check LIDAR Data

```bash
# View LIDAR scan
ros2 topic echo /scan --once

# Check scan frequency
ros2 topic hz /scan
# Should be ~10-30 Hz depending on LIDAR
```

### Step 3: Verify Map Building

```bash
# Check map is being published
ros2 topic echo /map --once

# View map in RViz
rviz2
# Add: Map display, subscribe to /map
```

### Step 4: Verify MCL Localization

```bash
# Check pose estimate
ros2 topic echo /amcl_pose

# Should see:
# - pose.pose.position (x, y, z)
# - pose.pose.orientation (quaternion)
# - pose.covariance (6x6 matrix)
```

**Check convergence:**
- Covariance should decrease over time
- Position should match actual robot location

### Step 5: Verify UKF Target Tracking

```bash
# Check target estimate
ros2 topic echo /target_estimate

# Should see:
# - pose.pose.position (target x, y)
# - pose.covariance (uncertainty)
```

**Verify tracking:**
- Estimate should follow target movements
- Uncertainty should be reasonable (< 0.5m)

### Step 6: Verify MPC Control

```bash
# Check control commands
ros2 topic echo /cmd_vel

# Should see:
# - linear.x (forward velocity)
# - angular.z (turn rate)
```

**Safety check:**
- Commands should be within limits
- Robot should move smoothly toward target

### Step 7: Monitor System Health

```bash
# Check node status
ros2 node list

# Check topic rates
ros2 topic hz /scan
ros2 topic hz /amcl_pose
ros2 topic hz /target_estimate
ros2 topic hz /cmd_vel

# All should be publishing at expected rates
```

---

## Troubleshooting

### Problem: LIDAR Not Publishing

**Symptoms:**
- `/scan` topic not found
- No LIDAR data

**Solutions:**
1. Check LIDAR hardware connection
2. Verify LIDAR driver is running:
   ```bash
   ros2 node list | grep lidar
   ```
3. Check USB permissions:
   ```bash
   ls -l /dev/ttyUSB*  # Check permissions
   sudo chmod 666 /dev/ttyUSB0  # If needed
   ```
4. Restart LIDAR driver

### Problem: Map Not Building

**Symptoms:**
- `/map` topic exists but map is empty
- No obstacles detected

**Solutions:**
1. Check `/amcl_pose` is publishing (MCL must initialize first)
2. Verify LIDAR data is valid:
   ```bash
   ros2 topic echo /scan --once | grep ranges
   ```
3. Check SLAM node parameters (map size, resolution)
4. Ensure robot is moving (static robot = no map updates)

### Problem: MCL Not Converging

**Symptoms:**
- High covariance in `/amcl_pose`
- Robot pose estimate is wrong

**Solutions:**
1. Increase particle count:
   ```python
   'num_particles': 500  # In launch file
   ```
2. Check `/map` is valid and has obstacles
3. Verify LIDAR data quality
4. Check motion noise parameters (may be too high)

### Problem: Target Measurement Node Not Publishing

**Symptoms:**
- `/target_pose_measurement` not found or not updating
- No terminal homing activation

**Solutions:**
1. Check node is running:
   ```bash
   ros2 node list | grep target_measurement
   ```
2. Verify LIDAR is publishing:
   ```bash
   ros2 topic echo /scan --once
   ```
3. Check robot pose is available:
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
4. Verify terminal homing activation:
   - Check logs for "TERMINAL HOMING ACTIVATED" message
   - Ensure target is within `lidar_detection_range` (default: 2.0m)
5. Adjust detection range if needed:
   ```python
   'lidar_detection_range': 3.0  # Increase for earlier activation
   ```

### Problem: UKF Not Tracking

**Symptoms:**
- `/target_estimate` not updating
- High uncertainty

**Solutions:**
1. Verify `/target_pose_measurement` is publishing (from target_measurement_node):
   ```bash
   ros2 topic echo /target_pose_measurement
   ```
2. Check measurement frequency (should be > 1 Hz)
3. Verify measurements are in 'map' frame
4. Check measurement noise (may be too high)
5. Ensure target_measurement_node is running

### Problem: MPC Not Publishing Commands

**Symptoms:**
- `/cmd_vel` not publishing
- Robot not moving

**Solutions:**
1. Check all required topics are available:
   ```bash
   ros2 topic list | grep -E "(amcl_pose|target_estimate|map)"
   ```
2. Verify MPC node is running:
   ```bash
   ros2 node list | grep mpc
   ```
3. Check MPC logs for errors:
   ```bash
   ros2 run turtlebot_interceptor mpc_node
   # Look for error messages
   ```
4. Verify MPC parameters (horizon, timestep)

### Problem: Robot Moving Erratically

**Symptoms:**
- Robot oscillates
- Commands change rapidly

**Solutions:**
1. Reduce MPC horizon (shorter = more reactive but less stable)
2. Increase control weights in MPC
3. Check MCL convergence (bad localization = bad control)
4. Verify target estimate is stable

### Problem: Robot Not Avoiding Obstacles

**Symptoms:**
- Robot hits obstacles
- MPC trajectory goes through obstacles

**Solutions:**
1. Verify map is being built correctly:
   ```bash
   rviz2  # Visualize /map topic
   ```
2. Check obstacle inflation parameters
3. Verify MPC obstacle constraints are enabled
4. Check map resolution (too coarse = missed obstacles)

---

## Advanced Configuration

### Custom Map Parameters

Edit `launch/hardware.launch.py`:

```python
Node(
    package='turtlebot_interceptor',
    executable='slam_node',
    parameters=[{
        'map_width': 400,           # 20m x 20m at 5cm resolution
        'map_height': 400,
        'resolution': 0.05,
        'origin_x': -10.0,          # Center map at origin
        'origin_y': -10.0,
        'log_odds_free': -0.6,      # Tune for faster/slower mapping
        'log_odds_occupied': 0.8,
        'occupancy_threshold': 0.25,
    }],
)
```

### Performance Tuning

**For faster response:**
- Reduce MPC horizon: `mpc_horizon: 10`
- Increase control frequency: `dt: 0.05`
- Reduce MCL particles: `num_particles: 200`

**For better accuracy:**
- Increase MPC horizon: `mpc_horizon: 25`
- More MCL particles: `num_particles: 500`
- Higher map resolution: `resolution: 0.02`

### Frame Configuration

Ensure all frames are consistent:
- Map frame: `'map'`
- Base frame: `'base_footprint'` or `'base_link'`
- LIDAR frame: `'laser_frame'` or `'scan'`

Check TF tree:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf  # View TF tree
```

---

## Safety Checklist

Before running on hardware:

- [ ] Robot is in safe, open area
- [ ] Emergency stop accessible
- [ ] LIDAR is working and calibrated
- [ ] Robot base controller responds to `/cmd_vel`
- [ ] All nodes are publishing correctly
- [ ] MCL has converged before starting interception
- [ ] Map is being built correctly
- [ ] Target tracking is working
- [ ] MPC commands are reasonable
- [ ] Test with slow speeds first

---

## Next Steps

1. **Tune parameters** for your specific environment
2. **Calibrate sensors** (LIDAR, odometry)
3. **Test in controlled environment** before full deployment
4. **Monitor performance** and adjust as needed
5. **Document your configuration** for reproducibility

---

## Support

For issues or questions:
1. Check logs: `ros2 topic echo /rosout`
2. Review node output in terminals
3. Check this troubleshooting guide
4. Verify all prerequisites are met

---

## Quick Reference

**Essential Commands:**
```bash
# Launch system
ros2 launch turtlebot_interceptor hardware.launch.py

# Check topics
ros2 topic list
ros2 topic echo /<topic_name>

# Check nodes
ros2 node list
ros2 node info /<node_name>

# Monitor rates
ros2 topic hz /<topic_name>

# Emergency stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

**Important Topics:**
- `/scan` - LIDAR data
- `/map` - Occupancy grid
- `/amcl_pose` - Robot pose
- `/target_pose_measurement` - Target measurements (from target_measurement_node)
- `/target_estimate` - Target position (from UKF)
- `/cmd_vel` - Control commands

---

*Last updated: 2025*
