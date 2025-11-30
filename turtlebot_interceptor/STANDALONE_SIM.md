# Standalone Simulation (No ROS2 Required)

## Overview

The standalone simulation allows you to test the entire system without needing ROS2 installed. It simulates all the components using pure Python.

## Quick Start

```bash
# From the package directory
cd turtlebot_interceptor
python3 -m turtlebot_interceptor.standalone_sim

# Or if installed:
python3 standalone_sim.py
```

## What It Does

The standalone simulation includes:

1. **Map Generator**: Creates a test environment with walls and obstacles
2. **Fake LIDAR**: Simulates LIDAR scans using raycasting
3. **MCL**: Monte Carlo Localization with particle filter
4. **Target KF**: Kalman Filter for target tracking
5. **MPC**: Model Predictive Control for interception
6. **Visualization**: Real-time matplotlib visualization

## Features

- ✅ No ROS2 required
- ✅ Pure Python implementation
- ✅ Real-time visualization
- ✅ Same algorithms as ROS2 version
- ✅ Easy to modify and debug

## Running the Simulation

### Basic Run

```bash
python3 -m turtlebot_interceptor.standalone_sim
```

This will:
- Run 100 simulation steps (10 seconds)
- Print progress every second
- Show final visualization

### Custom Run

Modify the `main()` function in `standalone_sim.py`:

```python
# Change number of steps
num_steps = 200  # 20 seconds

# Change time step
dt = 0.05  # 50ms instead of 100ms

# Change initial conditions
sim.robot_pose = Pose(x=0.5, y=0.5, theta=0.0)
sim.target_position = np.array([2.0, 2.0])
```

## Visualization

The simulation shows:
- **Black rectangles**: Obstacles and walls
- **Blue dot with arrow**: Robot position and heading
- **Cyan dots**: MCL particles
- **Green dot**: MCL pose estimate
- **Red dot**: Target position
- **Red lines**: LIDAR scan rays

## Output

During simulation, you'll see:
```
Step   0: Robot=(0.00, 0.00), Target=(1.50, 1.50), Dist=2.12m
Step  10: Robot=(0.05, 0.00), Target=(1.50, 1.50), Dist=2.07m
...
```

## Dependencies

Required:
- `numpy`
- `matplotlib`
- `cvxpy` (for MPC)

Install:
```bash
pip3 install numpy matplotlib cvxpy
```

## Comparison with ROS2 Version

| Feature | Standalone | ROS2 |
|---------|-----------|------|
| Map | ✅ | ✅ |
| LIDAR | ✅ | ✅ |
| MCL | ✅ | ✅ |
| Target KF | ✅ | ✅ |
| MPC | ✅ | ✅ |
| Visualization | Matplotlib | RViz |
| Real Hardware | ❌ | ✅ |
| Multiple Robots | ❌ | ✅ |
| ROS2 Topics | ❌ | ✅ |

## Advantages

1. **No Setup**: Works immediately without ROS2 installation
2. **Fast Iteration**: Quick to test algorithm changes
3. **Easy Debugging**: Can add print statements, breakpoints easily
4. **Portable**: Works on any system with Python
5. **Educational**: Clear code structure, easy to understand

## Limitations

1. **No Real Hardware**: Can't control actual robots
2. **Simplified**: Some features simplified compared to ROS2 version
3. **Single Threaded**: Runs sequentially (though fast enough)
4. **No Network**: Can't communicate with other ROS2 nodes

## Example Output

```
==================================================
Standalone Simulation (No ROS2 Required)
==================================================

Running simulation for 100 steps (10.0 seconds)...
Press Ctrl+C to stop early

Step   0: Robot=(0.00, 0.00), Target=(1.50, 1.50), Dist=2.12m
Step  10: Robot=(0.05, 0.00), Target=(1.50, 1.50), Dist=2.07m
Step  20: Robot=(0.12, 0.01), Target=(1.50, 1.50), Dist=1.98m
...
Step 100: Robot=(1.45, 1.48), Target=(1.50, 1.50), Dist=0.05m

Simulation complete!

Generating visualization...
```

## Tips

1. **Adjust Parameters**: Modify MCL particle count, MPC horizon, etc.
2. **Add Logging**: Print intermediate values for debugging
3. **Save Results**: Add code to save plots or data
4. **Experiment**: Try different maps, targets, initial conditions

## Next Steps

Once the standalone simulation works, you can:
1. Verify algorithms are correct
2. Tune parameters
3. Then test with ROS2 for hardware integration

