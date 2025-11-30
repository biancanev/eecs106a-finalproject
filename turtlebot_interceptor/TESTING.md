# Testing the Simulation

## Prerequisites

1. **ROS2 Installation**: Make sure ROS2 (Humble/Iron/Rolling) is installed and sourced
2. **Dependencies**:
   ```bash
   sudo apt-get install python3-cvxpy python3-transforms3d python3-matplotlib python3-numpy
   pip3 install cvxpy  # if not available via apt
   ```

## Building the Package

```bash
# Navigate to your ROS2 workspace (or create one)
cd ~/ros2_ws  # or your workspace directory
mkdir -p src
cd src

# If not already there, copy/link the package
# cd ~/eecs106a-finalproject
# cp -r turtlebot_interceptor ~/ros2_ws/src/

# Build
cd ~/ros2_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

## Running the Simulation

### Option 1: Full Simulation (All Nodes)

```bash
ros2 launch turtlebot_interceptor simulation.launch.py
```

This launches:
- Voxel grid map node (publishes `/map`)
- Fake LIDAR node (publishes `/scan`)
- MCL node (localization)
- Target KF node (target tracking)
- MPC node (control)

### Option 2: Individual Nodes

```bash
# Terminal 1: Map node
ros2 run turtlebot_interceptor simulation_node --map-only

# Terminal 2: Fake LIDAR (requires map and robot pose)
ros2 run turtlebot_interceptor simulation_node --lidar-only

# Terminal 3: MCL
ros2 run turtlebot_interceptor mcl_node --sim

# Terminal 4: Target KF
ros2 run turtlebot_interceptor target_kf_node

# Terminal 5: MPC
ros2 run turtlebot_interceptor mpc_node
```

## Testing Individual Components

### Test MPC Solver (Standalone)

```python
import numpy as np
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC

# Create MPC
mpc = SimpleUnicycleMPC(horizon=10, dt=0.1)

# Test state: [px, py, theta, v]
x0 = [0.0, 0.0, 0.0, 0.5]
target = [1.0, 1.0]  # Target position

# Solve
a, omega = mpc.solve(x0, target)
print(f"Acceleration: {a}, Angular velocity: {omega}")
```

### Test Map Generation

```bash
# Publish a test map
ros2 run turtlebot_interceptor simulation_node --map-only

# In another terminal, check the map
ros2 topic echo /map --once
```

### Test Fake LIDAR

```bash
# Terminal 1: Map
ros2 run turtlebot_interceptor simulation_node --map-only

# Terminal 2: Publish robot pose (you'll need to do this manually or use a simulator)
ros2 topic pub /robot_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"

# Terminal 3: Fake LIDAR
ros2 run turtlebot_interceptor simulation_node --lidar-only

# Terminal 4: Check LIDAR output
ros2 topic echo /scan
```

## Expected Topics

After launching the simulation, you should see:

```bash
ros2 topic list
```

Should include:
- `/map` (nav_msgs/OccupancyGrid)
- `/scan` (sensor_msgs/LaserScan)
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped)
- `/target_estimate` (geometry_msgs/PoseWithCovarianceStamped)
- `/cmd_vel` (geometry_msgs/Twist)
- `/target_pose_measurement` (geometry_msgs/PoseStamped) - needs to be published

## Troubleshooting

### "Package not found"
- Make sure you've built: `colcon build --packages-select turtlebot_interceptor`
- Source the workspace: `source install/setup.bash`

### "Module not found: rclpy"
- Source ROS2: `source /opt/ros/humble/setup.bash` (or your ROS2 distro)

### "MPC solver fails"
- Check cvxpy: `python3 -c "import cvxpy; print(cvxpy.__version__)"`
- Install OSQP solver: `pip3 install osqp`

### "No map received"
- Make sure map node is running: `ros2 run turtlebot_interceptor simulation_node --map-only`
- Check: `ros2 topic echo /map --once`

### "MCL not initializing"
- Ensure map is published
- Check particle initialization range covers robot pose
- Verify LIDAR data is available

## Quick Test Script

Create a file `quick_test.sh`:

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 distro
cd ~/ros2_ws
source install/setup.bash

echo "Testing simulation..."
ros2 launch turtlebot_interceptor simulation.launch.py
```

Make it executable: `chmod +x quick_test.sh`

