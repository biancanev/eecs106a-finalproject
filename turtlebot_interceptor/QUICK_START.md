# Quick Start - Testing Simulation

## Prerequisites Check

```bash
# Check ROS2
ros2 --help

# Check Python packages
python3 -c "import numpy; import cvxpy; print('OK')"
```

## Step 1: Build the Package

```bash
# If you're in the package directory, go to workspace root
cd ~/ros2_ws  # or wherever your workspace is

# If package is not in src/, copy it:
# cp -r ~/eecs106a-finalproject/turtlebot_interceptor src/

# Build
colcon build --packages-select turtlebot_interceptor

# Source
source install/setup.bash
```

## Step 2: Run Simulation

### Option A: Full Launch (Recommended)

```bash
ros2 launch turtlebot_interceptor simulation.launch.py
```

### Option B: Step by Step (for debugging)

**Terminal 1 - Map:**
```bash
ros2 run turtlebot_interceptor simulation_node --map-only
```

**Terminal 2 - Check Map:**
```bash
ros2 topic echo /map --once
```

**Terminal 3 - Fake LIDAR:**
```bash
# First, publish a robot pose (or use a simulator)
ros2 topic pub -r 10 /robot_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"

# Then run fake lidar
ros2 run turtlebot_interceptor simulation_node --lidar-only
```

**Terminal 4 - Check LIDAR:**
```bash
ros2 topic echo /scan --once
```

**Terminal 5 - MCL:**
```bash
ros2 run turtlebot_interceptor mcl_node --sim
```

**Terminal 6 - Check MCL Output:**
```bash
ros2 topic echo /amcl_pose
```

## Step 3: Verify Topics

```bash
ros2 topic list
```

Should see:
- `/map`
- `/scan`
- `/amcl_pose`
- `/target_estimate`
- `/cmd_vel`

## Step 4: Publish Target Measurements (for KF)

```bash
ros2 topic pub -r 1 /target_pose_measurement geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"
```

## Step 5: Run MPC Node

```bash
ros2 run turtlebot_interceptor mpc_node
```

## Expected Behavior

1. **Map node**: Publishes occupancy grid with walls and obstacles
2. **Fake LIDAR**: Generates scan data based on map and robot pose
3. **MCL**: Localizes robot using LIDAR scans
4. **Target KF**: Tracks target position (needs measurements)
5. **MPC**: Computes control commands to intercept target

## Troubleshooting

### "Package turtlebot_interceptor not found"
```bash
source ~/ros2_ws/install/setup.bash
```

### "No module named 'rclpy'"
```bash
source /opt/ros/humble/setup.bash  # Adjust for your distro
```

### "MPC solver fails"
```bash
pip3 install cvxpy osqp
```

### "No map received"
- Check map node is running: `ros2 node list | grep map`
- Check topic: `ros2 topic hz /map`

## Visualizing (if RViz is available)

```bash
rviz2
# Add displays:
# - Map: /map
# - LaserScan: /scan
# - Pose: /amcl_pose
# - Twist: /cmd_vel
```

