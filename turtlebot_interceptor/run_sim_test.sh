#!/bin/bash
# Quick test script for simulation

echo "=========================================="
echo "TurtleBot Interceptor Simulation Test"
echo "=========================================="

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Please source ROS2:"
    echo "  source /opt/ros/humble/setup.bash  # or your ROS2 distro"
    exit 1
fi

echo "✓ ROS2 found"

# Check if package is built
if [ ! -d "install/turtlebot_interceptor" ]; then
    echo "⚠ Package not built. Building now..."
    if [ -d "../.." ] && [ -f "../../install/setup.bash" ]; then
        cd ../..
        colcon build --packages-select turtlebot_interceptor
        source install/setup.bash
        cd src/turtlebot_interceptor
    else
        echo "ERROR: Please build the package first:"
        echo "  cd ~/ros2_ws"
        echo "  colcon build --packages-select turtlebot_interceptor"
        echo "  source install/setup.bash"
        exit 1
    fi
fi

echo "✓ Package built"

# Check topics
echo ""
echo "Starting simulation nodes..."
echo "Press Ctrl+C to stop"
echo ""

# Run simulation launch
ros2 launch turtlebot_interceptor simulation.launch.py

