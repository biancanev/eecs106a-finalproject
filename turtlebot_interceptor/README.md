# TurtleBot Interceptor

ROS2 package for intercepting a target TurtleBot using Monte Carlo Localization (MCL), Kalman Filtering (KF), and Model Predictive Control (MPC) in unknown cluttered environments.

## Overview

This package implements the system described in the paper "Turtlebot Seeker-Target Interception in Unknown Cluttered Environments with Monte Carlo Localization and Model Predictive Control". The system consists of:

1. **LIDAR Processing**: Noise filtering and scan preprocessing
2. **SLAM (Log-Odds Occupancy Grid)**: Simultaneous localization and mapping from LIDAR
3. **Monte Carlo Localization (MCL)**: Particle filter for robot pose estimation using LIDAR
4. **Target UKF**: Unscented Kalman Filter for tracking target position
5. **Model Predictive Control (MPC)**: Receding-horizon optimal control for interception with obstacle avoidance

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   MCL Node  │────▶│  Pose Est.  │────▶│             │
│  (LIDAR)    │     │  + Cov      │     │             │
└─────────────┘     └─────────────┘     │   MPC Node  │
                                         │             │
┌─────────────┐     ┌─────────────┐     │             │
│  Target KF  │────▶│ Target Est. │────▶│             │
│  (Vision)    │     │  + Cov      │     │             │
└─────────────┘     └─────────────┘     └──────┬──────┘
                                                │
                                         ┌──────▼──────┐
                                         │  /cmd_vel   │
                                         └─────────────┘
```

## Installation

### Dependencies

```bash
sudo apt-get install python3-cvxpy python3-transforms3d python3-matplotlib python3-numpy
```

### Build

```bash
cd ~/your_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

## Usage

### Standalone Simulation (No ROS2 Required) ⭐ NEW

**Easiest way to test!** Run the entire system without ROS2:

```bash
cd turtlebot_interceptor
python3 run_standalone_sim.py
```

Or:
```bash
python3 -m turtlebot_interceptor.standalone_sim
```

This runs:
- Map generation
- Fake LIDAR simulation
- MCL localization
- Target KF tracking
- MPC control
- Real-time visualization

See `STANDALONE_SIM.md` for details.

### ROS2 Simulation Mode

For testing with ROS2, use the simulation launch file which provides:
- Fake LIDAR scans
- Voxel grid / occupancy grid map
- Simulated robot pose

```bash
ros2 launch turtlebot_interceptor simulation.launch.py
```

### Hardware Mode

For real hardware deployment:

```bash
ros2 launch turtlebot_interceptor hardware.launch.py
```

This launches all nodes:
- LIDAR processor (filters raw LIDAR data)
- SLAM node (log-odds occupancy grid mapping)
- MCL node (localization)
- UKF node (target tracking)
- MPC node (control)

See `HARDWARE_DEPLOYMENT.md` for detailed hardware setup instructions.

### Individual Nodes

You can also run nodes individually:

```bash
# LIDAR processor
ros2 run turtlebot_interceptor lidar_processor_node

# SLAM node (log-odds mapping)
ros2 run turtlebot_interceptor slam_node

# MCL node
ros2 run turtlebot_interceptor mcl_node

# UKF node (target tracking)
ros2 run turtlebot_interceptor ukf_node

# MPC node
ros2 run turtlebot_interceptor mpc_node

# Standalone simulation (no ROS2)
python3 -m turtlebot_interceptor.animated_sim
```

## Topics

### Subscribed Topics

- `/scan` (sensor_msgs/LaserScan): LIDAR scan data
- `/map` (nav_msgs/OccupancyGrid): Map/occupancy grid
- `/target_pose_measurement` (geometry_msgs/PoseStamped): Target position measurements
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): Seeker pose estimate (from MCL)
- `/target_estimate` (geometry_msgs/PoseWithCovarianceStamped): Target state estimate (from KF)

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for seeker robot
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): MCL pose estimate
- `/target_estimate` (geometry_msgs/PoseWithCovarianceStamped): KF target estimate

## Parameters

### MCL Node
- `N`: Number of particles (default: 300)
- `motion_noise`: Motion model noise [x, y, theta] (default: [0.02, 0.02, 0.01])

### MPC Node
- `N`: Prediction horizon (default: 15)
- `dt`: Control timestep (default: 0.1s)
- `v_max`: Maximum velocity (default: 0.6 m/s)
- `omega_max`: Maximum angular velocity (default: 1.5 rad/s)

### SLAM Node
- Map size: 100x100 cells (5m x 5m at 5cm resolution)
- Log-odds free: -0.6
- Log-odds occupied: 0.5
- Log-odds bounds: [-3.0, 3.0]

### UKF Node (Target Tracking)
- `dt`: Timestep (default: 0.1s)
- `Q`: Process noise covariance (default: 0.02 * I)
- `R`: Measurement noise covariance (default: 0.03 * I)
- UKF parameters: alpha=0.001, beta=2.0, kappa=0.0

## Features

### Uncertainty-Aware Planning

The system incorporates uncertainty from both MCL and KF:

1. **Obstacle Inflation**: Obstacle radii are inflated based on seeker pose uncertainty
2. **Speed Limits**: Maximum velocity is reduced when uncertainty is high
3. **Interception Region**: Catching radius is adjusted based on combined uncertainty

### Offline-Online Hybrid Planning

- Offline: Probabilistic trajectory generation with uncertainty propagation
- Online: Real-time MPC re-planning based on updated beliefs

## Implementation Details

### MCL Algorithm

- Particle filter with 300 particles
- Motion model: Unicycle with Gaussian noise
- Measurement model: LIDAR range likelihood
- Resampling: Systematic resampling

### Target KF

- State: [px, py, vx, vy]
- Dynamics: Constant velocity model
- Observation: Position only

### MPC

- State: [px, py, theta, v]
- Control: [acceleration, angular_velocity]
- Constraints: Actuator limits, obstacle avoidance
- Cost: Quadratic tracking + control effort

## Testing

Run the test suite:

```bash
colcon test --packages-select turtlebot_interceptor
```

## Troubleshooting

### MPC solver fails
- Check that cvxpy is installed: `pip3 install cvxpy`
- Verify OSQP solver is available
- Check MPC parameters (horizon, timestep)

### MCL not converging
- Ensure map is published on `/map` topic
- Check LIDAR data is available on `/scan`
- Verify particle initialization range covers actual robot pose

### Target KF not tracking
- Ensure target measurements are published on `/target_pose_measurement`
- Check measurement noise parameters (R matrix)
- Verify target is moving (KF assumes constant velocity)

## Authors

Tim, Gabe, Thomas, Ryan, Kush

## License

TODO: Add license

