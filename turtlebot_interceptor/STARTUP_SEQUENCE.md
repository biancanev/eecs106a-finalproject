# TurtleBot3 Navigation Startup Sequence

## Quick Start (2 Commands)

### Terminal 1: Launch Cartographer
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

**Wait until you see:**
- `[cartographer_node]: Node created`
- `/map` topic is publishing (check with `ros2 topic hz /map`)

### Terminal 2: Launch Navigation Stack
```bash
cd ~/eecs106a-finalproject
source install/setup.bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=0.0
```

**Wait 60 seconds** for initialization, then robot will navigate!

---

## Detailed Steps

### Step 1: Launch Cartographer SLAM

```bash
# Terminal 1 (on TurtleBot)
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

**What this does:**
- Starts Cartographer SLAM
- Subscribes to `/scan` (LIDAR), `/odom` (encoders), `/imu`
- Publishes `/map` (OccupancyGrid)
- Provides loop closure and accurate mapping

**Verify it's working:**
```bash
# In another terminal
ros2 topic list | grep map
# Should show: /map

ros2 topic hz /map
# Should show: ~1-2 Hz

ros2 topic echo /map --once | head -20
# Should show map data
```

### Step 2: Launch Navigation System

```bash
# Terminal 2 (on TurtleBot)
cd ~/eecs106a-finalproject
source install/setup.bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=0.0
```

**What this does:**
- Launches `simple_pose_publisher` (republishes `/odom` as `/amcl_pose`)
- Launches `mpc_node` (subscribes to `/amcl_pose` and `/map`, publishes `/cmd_vel`)
- Launches `navigation_visualizer` (RViz markers)
- Launches `rviz2` (visualization)

**Startup sequence (60 seconds):**
1. `[0-60s]` MPC waits for sensors to stabilize
2. `[60s]` MPC logs "MAP RECEIVED: X occupied cells"
3. `[60s+]` MPC starts controlling robot toward goal

### Step 3: Monitor Progress

```bash
# Check MPC is receiving map
ros2 topic echo /amcl_pose --once
# Should show robot pose

# Check MPC is sending commands
ros2 topic echo /cmd_vel
# Should show velocity commands

# Check map has obstacles
# (in RViz, add Map display with topic /map)
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│              TERMINAL 1: Cartographer                   │
├─────────────────────────────────────────────────────────┤
│  ros2 launch turtlebot3_cartographer cartographer...    │
│                                                          │
│  ┌────────────┐                                         │
│  │ /scan      │────┐                                    │
│  │ /odom      │────┤                                    │
│  │ /imu       │────┴──▶ Cartographer ──▶ /map          │
│  └────────────┘                                         │
└──────────────────────────────┬──────────────────────────┘
                               │
                               │ /map (OccupancyGrid)
                               │
┌──────────────────────────────┼──────────────────────────┐
│          TERMINAL 2: Navigation                         │
├──────────────────────────────┼──────────────────────────┤
│  ros2 launch turtlebot_interceptor single_robot_nav...  │
│                              │                           │
│  ┌──────────┐                │    ┌─────────────────┐  │
│  │  /odom   │─▶ simple_pose  │    │ /map            │  │
│  └──────────┘   publisher    │    │ (from Terminal 1)│  │
│                      │        │    └────────┬────────┘  │
│                      ▼        │             │           │
│                 /amcl_pose    │             │           │
│                      │        │             │           │
│                      └────────┴─────────────┤           │
│                                              ▼           │
│                                          MPC Node       │
│                                              │           │
│                                              ▼           │
│                                          /cmd_vel       │
│                                              │           │
│                                              ▼           │
│                                          Robot          │
└──────────────────────────────────────────────────────────┘
```

---

## Topic Flow

| Topic | Publisher | Subscriber | Description |
|-------|-----------|------------|-------------|
| `/scan` | LIDAR driver | Cartographer | LIDAR data |
| `/odom` | Robot driver | Cartographer, simple_pose_publisher | Wheel encoders |
| `/imu` | Robot driver | Cartographer | IMU data |
| `/map` | **Cartographer** | **MPC** | **Obstacle map** |
| `/amcl_pose` | simple_pose_publisher | MPC | Robot pose |
| `/cmd_vel` | MPC | Robot driver | Velocity commands |

---

## Verification Checklist

### ✅ Before Starting

```bash
# Check robot drivers are running
ros2 topic list | grep -E "(scan|odom|imu)"
# Should show: /scan, /odom, /imu

# Check topics are publishing
ros2 topic hz /scan    # Should be ~5-10 Hz
ros2 topic hz /odom    # Should be ~20-50 Hz
ros2 topic hz /imu     # Should be ~50-200 Hz
```

### ✅ After Cartographer Launch (Terminal 1)

```bash
# Check Cartographer is running
ros2 node list | grep cartographer
# Should show: /cartographer_node, /cartographer_occupancy_grid_node

# Check map is publishing
ros2 topic hz /map
# Should show: ~1-2 Hz

# Check map has data
ros2 topic echo /map --once | grep "width\|height\|resolution"
# Should show dimensions and resolution
```

### ✅ After Navigation Launch (Terminal 2)

```bash
# Check navigation nodes are running
ros2 node list | grep -E "(simple_pose|mpc)"
# Should show: /simple_pose_publisher, /mpc_node

# Check MPC is receiving pose
ros2 topic hz /amcl_pose
# Should show: ~10 Hz

# Check MPC logs
# Look for: "[mpc_node]: MAP RECEIVED: X occupied cells"

# Check robot is moving
ros2 topic echo /cmd_vel
# Should show non-zero velocities after 60s
```

---

## Expected Behavior

### Phase 1: Initialization (0-60 seconds)

```
[simple_pose_publisher]: Using /odom for pose
[mpc_node]: MPC node initialized - Waiting 60.0s...
[mpc_node]: MPC startup delay: 40.0s remaining...
[mpc_node]: MPC startup delay: 20.0s remaining...
```

**Robot:** Stationary

### Phase 2: Map Reception (~60 seconds)

```
[mpc_node]: MAP RECEIVED: 200x200 cells, resolution=0.050m, occupied=152, frame=map
[mpc_node]: OBSTACLE CELLS: Found 8 occupied cells within 1.5m
```

**Robot:** Still stationary, waiting for MPC

### Phase 3: Navigation (60+ seconds)

```
[mpc_node]: MPC controlling: dist=2.12m, angle_err=-45.3°, v=0.25, ω=-0.85
[mpc_node]: OBSTACLE CELLS: Found 3 occupied cells within 1.5m
```

**Robot:** Moving toward goal, avoiding obstacles

### Phase 4: Goal Reached

```
[mpc_node]: *** GOAL REACHED! *** Distance to goal: 0.08m
```

**Robot:** Stops at goal

---

## Common Issues

### Issue 1: "Cartographer crashed" or "Check failure"

**Cause:** Missing sensor topics

**Fix:**
```bash
# Check all sensors are publishing
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu

# If any are missing, restart robot drivers
```

### Issue 2: "MAP not received" or "occupied=0"

**Cause:** No obstacles in map OR MPC started before Cartographer

**Fix:**
1. Make sure Cartographer launched first
2. Wait for map to have data: `ros2 topic echo /map --once`
3. Put obstacles in front of robot so Cartographer sees them
4. Check RViz `/map` display shows obstacles

### Issue 3: "Robot not moving"

**Cause:** Still in 60s startup delay

**Fix:** Wait! Look for log message:
```
[mpc_node]: MPC startup delay: X.Xs remaining...
```

When it reaches 0, robot will start moving.

### Issue 4: "Robot spinning/wrong direction"

**Cause:** Goal position behind robot

**Fix:** Set goal in front of robot:
```bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=0.0
```

Adjust `goal_x` and `goal_y` to match where you want robot to go.

---

## Stopping the System

### Emergency Stop

```bash
# Press Ctrl+C in Terminal 2 (navigation)
# This stops MPC and robot movement
```

### Clean Shutdown

```bash
# Terminal 2: Ctrl+C (stops navigation)
# Wait for clean exit

# Terminal 1: Ctrl+C (stops Cartographer)
# Wait for clean exit
```

---

## Summary

**Two terminal launch sequence:**

```bash
# Terminal 1: SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py

# Terminal 2: Navigation (wait for Cartographer to start)
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py
```

**That's it!** Wait 60 seconds and the robot navigates to the goal while avoiding obstacles.

