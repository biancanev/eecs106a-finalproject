# Launch Instructions for TurtleBot Navigation

## Architecture Overview

The system has TWO separate launch files that work together:

1. **`turtlebot3_cartographer`** (run separately) - SLAM mapping
2. **`single_robot_navigation.launch.py`** (our package) - EKF + MPC control

## Step-by-Step Launch

### Step 1: Launch Cartographer (On TurtleBot)

```bash
# On the TurtleBot machine
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

**What it does:**
- Subscribes to `/scan` (LIDAR) and `/odom` (wheel encoders)
- Publishes `/map` (OccupancyGrid)
- Provides SLAM with loop closure

**Expected topics:**
```
/map                  - OccupancyGrid (the map we need!)
/scan_matched_points2 - Point cloud
/submap_list          - Cartographer submaps
/trajectory_node_list - SLAM graph
```

### Step 2: Launch Navigation System (On TurtleBot)

```bash
# On the TurtleBot machine (in a new terminal)
cd ~/eecs106a-finalproject
source install/setup.bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=1.5
```

**What it does:**
- Launches **EKF** → Fuses IMU + Mag + Encoders → Publishes `/amcl_pose`
- Launches **MPC** → Subscribes to `/amcl_pose` and `/map` → Publishes `/cmd_vel`
- Launches **Visualizer** → RViz markers

**Expected behavior:**
1. EKF calibrates magnetometer (rotate robot slowly for 10 seconds)
2. MPC receives map from Cartographer
3. MPC logs "MAP RECEIVED" with occupied cell count
4. Robot starts navigating toward goal, avoiding obstacles

## Topic Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    CARTOGRAPHER LAUNCH                      │
│  (ros2 launch turtlebot3_cartographer cartographer.launch) │
└────────────────────────┬────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
        ▼                ▼                ▼
    /scan            /odom            /imu
     (LIDAR)      (encoders)     (accel+gyro)
        │                │                │
        └────────────────┼────────────────┘
                         │
                         ▼
                  ┌─────────────┐
                  │ Cartographer│
                  │    SLAM     │
                  └──────┬──────┘
                         │
                         ▼
                      /map ━━━━━━━━━━━━━━┓
                                          ┃
┌─────────────────────────────────────────┃───────────────────┐
│            NAVIGATION LAUNCH            ┃                   │
│  (ros2 launch ... single_robot_navigation.launch.py)       │
└─────────────────────────────────────────┃───────────────────┘
                                          ┃
        ┌─────────────────────────────────┃───────────┐
        │                                 ┃           │
        ▼                                 ┃           ▼
    /imu, /magnetic_field, /odom         ┃       /map
        │                                 ┃           │
        ▼                                 ┃           │
   ┌─────────┐                            ┃           │
   │   EKF   │                            ┃           │
   └────┬────┘                            ┃           │
        │                                 ┃           │
        ▼                                 ┃           │
    /amcl_pose                            ┃           │
        │                                 ┃           │
        └─────────────────┬───────────────┛           │
                          │                           │
                          ▼                           │
                     ┌─────────┐                      │
                     │   MPC   │◀─────────────────────┘
                     └────┬────┘
                          │
                          ▼
                      /cmd_vel
                          │
                          ▼
                    ┌──────────┐
                    │  Robot   │
                    │ Hardware │
                    └──────────┘
```

## Verification Checklist

### Check Cartographer is Running

```bash
# On TurtleBot (or from remote)
ros2 node list | grep cartographer
# Expected: /cartographer_node, /cartographer_occupancy_grid_node

ros2 topic list | grep map
# Expected: /map

ros2 topic hz /map
# Expected: ~1-2 Hz
```

### Check Navigation is Running

```bash
ros2 node list | grep -E "(ekf|mpc)"
# Expected: /ekf_pose_estimator, /mpc_node

ros2 topic echo /amcl_pose --once
# Expected: PoseWithCovarianceStamped message

ros2 topic echo /cmd_vel
# Expected: Twist messages (v, omega)
```

### Check MPC is Receiving Map

Look at MPC node logs:
```
[mpc_node]: MAP RECEIVED: 200x200 cells, resolution=0.020m, occupied=XXX, frame=map
```

If you don't see this, check:
1. Is Cartographer publishing `/map`? → `ros2 topic hz /map`
2. Are obstacles in the map? → Look at Cartographer in RViz
3. Is MPC subscribed? → `ros2 node info /mpc_node`

## Common Issues

### Issue 1: "No map received"

**Cause:** Cartographer not running or not publishing

**Fix:**
```bash
# Check if Cartographer is running
ros2 node list | grep cartographer

# If not, launch it:
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### Issue 2: "Robot spinning in place"

**Cause:** Magnetometer not calibrated

**Fix:** 
- Wait 10 seconds at startup
- Slowly rotate robot 360° to calibrate magnetometer
- Check EKF logs for "Magnetometer calibrated!"

### Issue 3: "Robot hitting obstacles"

**Cause:** Map not updated or MPC not using it

**Fix:**
1. Check map has obstacles: Look in RViz `/map` topic
2. Check MPC logs for "OBSTACLE CELLS: Found X occupied cells"
3. If 0 cells found, Cartographer map might be empty or resolution mismatch

### Issue 4: "Two Cartographer nodes conflict"

**Cause:** Launched Cartographer twice (once separately, once in our launch file)

**Fix:** 
- Our launch file no longer starts Cartographer
- Only run `turtlebot3_cartographer` separately
- Single robot navigation will use its `/map` topic

## Launch Order Summary

```
1. Launch turtlebot3_cartographer     (SLAM mapping)
   ↓
2. Wait for /map topic to appear      (verify with: ros2 topic list)
   ↓
3. Launch single_robot_navigation     (EKF + MPC control)
   ↓
4. Wait for magnetometer calibration  (10 seconds, rotate slowly)
   ↓
5. Robot navigates to goal!           (avoiding all obstacles)
```

## RViz Setup

Add these displays to visualize everything:

1. **Map** → Topic: `/map`, Color scheme: `map`
2. **LaserScan** → Topic: `/scan`
3. **Pose** → Topic: `/amcl_pose`
4. **Path** → Topic: `/trajectory_node_list` (Cartographer path)
5. **Markers** → Topic: `/visualization_marker` (goal, obstacles)

## Advanced: If You Want Cartographer to Use EKF Pose

If you want Cartographer to use EKF's refined pose instead of raw odometry:

1. Modify Cartographer launch to remap:
   ```python
   remappings=[('/odom', '/odom_ekf')]
   ```

2. Make sure EKF is publishing `/odom_ekf` (it already does)

3. This gives Cartographer better pose estimates for mapping

But for now, **keeping them separate is simpler and works fine!**



