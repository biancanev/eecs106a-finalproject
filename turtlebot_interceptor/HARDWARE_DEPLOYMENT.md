# Hardware Deployment Guide

This guide explains how to deploy the TurtleBot Interceptor system on real hardware.

## System Architecture

The hardware deployment consists of the following ROS2 nodes:

1. **LIDAR Processor Node** (`lidar_processor_node`)
   - Subscribes to `/scan` (raw LIDAR from hardware)
   - Filters noise, removes invalid readings
   - Publishes `/scan_processed` (optional, for other nodes)

2. **SLAM Node** (`slam_node`)
   - Subscribes to `/scan` (LIDAR data)
   - Subscribes to `/amcl_pose` (robot pose for mapping)
   - Implements log-odds occupancy grid mapping
   - Publishes `/map` (occupancy grid)

3. **MCL Node** (`mcl_node`)
   - Subscribes to `/scan` (LIDAR data)
   - Subscribes to `/map` (occupancy grid from SLAM)
   - Subscribes to `/cmd_vel` (for motion model)
   - Implements Monte Carlo Localization
   - Publishes `/amcl_pose` (robot pose estimate)

4. **UKF Node** (`ukf_node`)
   - Subscribes to `/target_pose_measurement` (external target measurements)
   - Implements Unscented Kalman Filter for target tracking
   - Publishes `/target_estimate` (target state estimate)

5. **MPC Node** (`mpc_node`)
   - Subscribes to `/amcl_pose` (robot pose from MCL)
   - Subscribes to `/target_estimate` (target estimate from UKF)
   - Subscribes to `/map` (for obstacle avoidance)
   - Implements Model Predictive Control
   - Publishes `/cmd_vel` (velocity commands)

## Topic Structure

```
/scan (sensor_msgs/LaserScan)
  └─> lidar_processor_node
  └─> slam_node
  └─> mcl_node

/amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
  └─> slam_node (for mapping)
  └─> mpc_node (for control)

/map (nav_msgs/OccupancyGrid)
  └─> mcl_node (for localization)
  └─> mpc_node (for obstacle avoidance)

/target_pose_measurement (geometry_msgs/PoseStamped)
  └─> ukf_node

/target_estimate (geometry_msgs/PoseWithCovarianceStamped)
  └─> mpc_node

/cmd_vel (geometry_msgs/Twist)
  └─> mcl_node (for motion model)
  └─> [robot base controller]
```

## Launching on Hardware

### Full System Launch

```bash
ros2 launch turtlebot_interceptor hardware.launch.py
```

This launches all nodes in the correct order:
1. LIDAR processor
2. SLAM (mapping)
3. MCL (localization)
4. UKF (target tracking)
5. MPC (control)

### Individual Nodes

You can also run nodes individually for debugging:

```bash
# Terminal 1: LIDAR processor
ros2 run turtlebot_interceptor lidar_processor_node

# Terminal 2: SLAM
ros2 run turtlebot_interceptor slam_node

# Terminal 3: MCL
ros2 run turtlebot_interceptor mcl_node

# Terminal 4: UKF
ros2 run turtlebot_interceptor ukf_node

# Terminal 5: MPC
ros2 run turtlebot_interceptor mpc_node
```

## External Dependencies

### Target Measurement Source

The UKF node requires target measurements on `/target_pose_measurement`. You need to provide this from:
- Vision system (camera + object detection)
- External tracking system
- Manual publishing for testing:

```bash
ros2 topic pub -r 10 /target_pose_measurement geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"
```

### Robot Base Controller

The MPC node publishes `/cmd_vel` commands. Ensure your robot base controller subscribes to this topic.

## Configuration

### SLAM Node Parameters

The SLAM node uses log-odds occupancy grid mapping with these default parameters:
- Map size: 100x100 cells (5m x 5m at 5cm resolution)
- Log-odds free: -0.6
- Log-odds occupied: 0.5
- Log-odds bounds: [-3.0, 3.0]

### MCL Node Parameters

- Particles: 300
- Motion noise: [0.02, 0.02, 0.01] (x, y, theta)

### MPC Node Parameters

- Horizon: 15 steps
- Time step: 0.1s
- Max velocity: 1.2 m/s
- Max angular velocity: 1.5 rad/s

## Testing

1. **Verify LIDAR is publishing:**
   ```bash
   ros2 topic echo /scan --once
   ```

2. **Check map is being built:**
   ```bash
   ros2 topic echo /map --once
   ```

3. **Verify MCL is localizing:**
   ```bash
   ros2 topic echo /amcl_pose
   ```

4. **Check target tracking:**
   ```bash
   ros2 topic echo /target_estimate
   ```

5. **Monitor control commands:**
   ```bash
   ros2 topic echo /cmd_vel
   ```

## Troubleshooting

- **No map updates:** Ensure `/amcl_pose` is being published (MCL needs to initialize)
- **MCL not converging:** Check `/map` is valid and `/scan` data is good
- **MPC not publishing:** Ensure both `/amcl_pose` and `/target_estimate` are available
- **UKF unstable:** Check target measurements are reasonable and not too noisy

