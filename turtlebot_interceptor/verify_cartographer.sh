#!/bin/bash
# Verify Cartographer is running and publishing map

echo "=== Checking Cartographer Nodes ==="
ros2 node list | grep cartographer
echo ""

echo "=== Checking Map Topic ==="
ros2 topic list | grep map
echo ""

echo "=== Checking Map Info ==="
timeout 2 ros2 topic echo /map --once | head -20
echo ""

echo "=== Checking Map Publish Rate ==="
timeout 5 ros2 topic hz /map
echo ""

echo "=== Checking MPC Node Map Subscription ==="
ros2 node info /mpc_node | grep -A 5 "Subscribers:"


