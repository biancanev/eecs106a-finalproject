# Hardware Deployment - Quick Start Guide

**5-minute setup guide for experienced users. For detailed instructions, see [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md).**

## Prerequisites Check

```bash
# Verify ROS2
ros2 --version

# Verify Python packages
python3 -c "import cvxpy, transforms3d, numpy, matplotlib; print('OK')"

# Verify workspace
cd ~/turtlebot3_ws && source install/setup.bash
```

## Step 1: Build Package

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

## Step 2: Verify Hardware

```bash
# Check LIDAR
ros2 topic echo /scan --once

# Check robot base
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Step 3: Setup Target Tracking

**Option A: Manual test (development)**
```bash
ros2 topic pub -r 10 /target_pose_measurement geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"
```

**Option B: Your camera/tracking system**
- Ensure it publishes to `/target_pose_measurement`

## Step 4: Launch System

```bash
ros2 launch turtlebot_interceptor hardware.launch.py
```

## Step 5: Verify Operation

```bash
# Terminal 1: Check topics
ros2 topic list

# Terminal 2: Monitor robot pose
ros2 topic echo /amcl_pose

# Terminal 3: Monitor target
ros2 topic echo /target_estimate

# Terminal 4: Monitor commands
ros2 topic echo /cmd_vel
```

## Emergency Stop

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Common Issues

| Problem | Quick Fix |
|---------|-----------|
| No `/scan` | Check LIDAR driver is running |
| No `/map` | Wait for MCL to initialize (check `/amcl_pose`) |
| MCL not converging | Increase particles: `num_particles: 500` |
| UKF not tracking | Check `/target_pose_measurement` is publishing |
| MPC not working | Verify all topics: `/amcl_pose`, `/target_estimate`, `/map` |

## Next Steps

1. Tune parameters in `launch/hardware.launch.py`
2. Monitor performance with `ros2 topic hz <topic>`
3. Visualize in RViz2: `rviz2`
4. See [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) for detailed troubleshooting

---

**For first-time setup, see the full [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) guide.**

