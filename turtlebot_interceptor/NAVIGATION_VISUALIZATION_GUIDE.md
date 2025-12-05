# Navigation Visualization Guide

## Overview

The `navigation_visualizer_node` publishes RViz markers to visualize robot navigation progress overlaid on the voxel grid map.

## What It Shows

1. **Robot Pose** (Green Arrow)
   - Current robot position and orientation
   - Updates in real-time

2. **Goal Point** (Red Sphere)
   - Target destination
   - Configurable via `goal_x` and `goal_y` parameters

3. **Path History** (Blue Line)
   - Trajectory the robot has traveled
   - Shows last N poses (configurable)

4. **Distance Line** (Yellow Line)
   - Direct line from robot to goal
   - Shows straight-line distance

5. **Text Overlay** (White Text)
   - Goal coordinates
   - Current distance to goal
   - Positioned above robot

## Usage

### Launch with Navigation System

The visualizer is automatically included in `single_robot_navigation.launch.py`:

```bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py goal_x:=1.5 goal_y:=1.5
```

### Launch Standalone

```bash
ros2 run turtlebot_interceptor navigation_visualizer --ros-args -p goal_x:=1.5 -p goal_y:=1.5
```

## RViz Configuration

### Add Visualization Markers

1. **Open RViz** (or it should open automatically with launch file)

2. **Add MarkerArray Display:**
   - Click "Add" button (bottom left)
   - Select "By topic" tab
   - Find `/navigation_visualization` → `MarkerArray`
   - Click "OK"

3. **Configure Display:**
   - In left panel, expand "MarkerArray"
   - All markers should be visible by default
   - You can toggle individual marker namespaces:
     - `robot` - Robot pose arrow
     - `goal` - Goal point sphere
     - `path` - Path history line
     - `distance` - Distance line
     - `text` - Text overlay

4. **Set Fixed Frame:**
   - Global Options → Fixed Frame → Set to `map`
   - This ensures markers align with the voxel grid

### Recommended View Settings

- **View Type**: "TopDownOrtho" (2D top-down view)
- **Zoom**: Adjust to see full map and navigation
- **Follow**: None (or robot if you want camera to follow)

## What You'll See

### In RViz:

1. **Voxel Grid Map** (from SLAM)
   - Black cells = Obstacles (cones)
   - White cells = Free space
   - Gray cells = Unknown

2. **Overlaid Navigation Visualization:**
   - **Green arrow** = Robot (points in direction of travel)
   - **Red sphere** = Goal point
   - **Blue line** = Path history (where robot has been)
   - **Yellow line** = Direct distance to goal
   - **White text** = Distance and goal info

### Example Scene:

```
                    [Goal - Red Sphere]
                           |
                           | (Yellow line)
                           |
    [Path History] -----> [Robot - Green Arrow]
    (Blue line)              |
                             |
                         [Text: Distance]
```

## Parameters

### Node Parameters

- `goal_x` (double, default: 1.5) - Goal X position in meters
- `goal_y` (double, default: 1.5) - Goal Y position in meters
- `robot_radius` (double, default: 0.15) - Robot footprint radius for arrow size
- `path_history_length` (int, default: 200) - Number of poses to keep in path history

### Example:

```bash
ros2 run turtlebot_interceptor navigation_visualizer \
  --ros-args \
  -p goal_x:=2.0 \
  -p goal_y:=1.0 \
  -p path_history_length:=500
```

## Troubleshooting

### Issue: Markers Not Showing

**Check:**
1. Visualizer node is running: `ros2 node list | grep visualizer`
2. Markers are publishing: `ros2 topic echo /navigation_visualization --once`
3. RViz is subscribed: Check MarkerArray display shows topic `/navigation_visualization`
4. Fixed frame is correct: Should be `map`

**Fix:**
- Restart visualizer node
- Re-add MarkerArray display in RViz
- Check topic name matches

### Issue: Markers in Wrong Position

**Check:**
1. Fixed frame matches: Should be `map`
2. Robot pose is publishing: `ros2 topic echo /amcl_pose --once`
3. Goal parameters are correct: `ros2 param get /navigation_visualizer goal_x`

**Fix:**
- Set Fixed Frame to `map` in RViz
- Ensure pose is being published
- Verify goal coordinates

### Issue: Path History Not Updating

**Check:**
1. Robot is moving: Check `/cmd_vel` topic
2. Pose is updating: `ros2 topic hz /amcl_pose`
3. Path history length: `ros2 param get /navigation_visualizer path_history_length`

**Fix:**
- Increase `path_history_length` if path is too short
- Ensure robot is actually moving
- Check pose updates are coming through

## Advanced: Custom Markers

You can extend the visualizer to show additional information:

1. **MPC Trajectory** - Planned path from MPC
2. **Obstacle Markers** - Highlight detected cones
3. **Velocity Vector** - Show robot velocity direction
4. **Uncertainty Ellipse** - Show pose uncertainty from MCL

See `navigation_visualizer_node.py` for implementation details.

## Quick Reference

```bash
# Check visualizer is running
ros2 node list | grep visualizer

# Check markers are publishing
ros2 topic hz /navigation_visualization
ros2 topic echo /navigation_visualization --once

# Check parameters
ros2 param list /navigation_visualizer
ros2 param get /navigation_visualizer goal_x
ros2 param get /navigation_visualizer goal_y

# View node info
ros2 node info /navigation_visualizer
```

