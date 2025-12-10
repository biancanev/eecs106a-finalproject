# LIDAR Frame Alignment Test Procedure

## Problem
The LIDAR frame and Cartographer map frame are misaligned. Need to find correct rotation offset.

## Quick Test Procedure

### Step 1: Place Obstacle
Place an obstacle **DIRECTLY IN FRONT** of the robot (not behind, not to side).

### Step 2: Launch System
```bash
cd ~/eecs106a-finalproject
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
ros2 launch turtlebot_interceptor single_robot_navigation.launch.py
```

### Step 3: Check RViz
Add these displays:
- Map → `/local_map` (our fast grid)
- LaserScan → `/scan` (LIDAR rays)

### Step 4: Observe Obstacle Location

Check where the obstacle appears in `/local_map`:

```
Current setting: π (180°)
```

| Obstacle appears | Current | What to do | New value |
|-----------------|---------|------------|-----------|
| **In front** ✓ | π | **CORRECT!** Keep it | `np.pi` |
| **Behind** | π | Add 180° | `0.0` or `2*np.pi` |
| **To left** | π | Add 90° | `π + π/2 = 3π/2` |
| **To right** | π | Subtract 90° | `π - π/2 = π/2` |

### Step 5: Adjust if Needed

Edit **BOTH files** (must match!):

**File 1: `fast_local_grid.py` (line ~66)**
```python
self.lidar_angle_offset = np.pi  # ← Change this
```

**File 2: `mpc_node.py` (line ~116)**
```python
self.lidar_angle_offset = np.pi  # ← Change this (same value!)
```

### Step 6: Rebuild and Test
```bash
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
# Launch again and check
```

## Common Values

```python
# 0° - No rotation
self.lidar_angle_offset = 0.0

# 90° counter-clockwise (left)
self.lidar_angle_offset = np.pi/2

# 180° (backwards) ← CURRENT
self.lidar_angle_offset = np.pi

# 270° counter-clockwise = 90° clockwise (right)
self.lidar_angle_offset = 3*np.pi/2
# OR equivalently:
self.lidar_angle_offset = -np.pi/2
```

## Visual Guide

```
        ↑ Robot Forward
        |
        | Obstacle here
        * 
        |
   ┌────┴────┐
   │  Robot  │
   └─────────┘
```

**If obstacle appears correctly (in front):** ✓ Done!

**If obstacle appears behind:**
```
   ┌─────────┐
   │  Robot  │
   └────┬────┘
        |
        * 
        | Obstacle here (WRONG!)
        |
        ↑ Robot Forward
```
→ Add π (180°)

**If obstacle appears to left/right:** Adjust by ±π/2

## Current Status

**Both files set to:** `np.pi` (180°)

This assumes LIDAR is mounted **backwards** on the robot.

## Verification

Once correct, both `/map` (Cartographer) and `/local_map` (ours) should show obstacle in **same location**.

## If Still Wrong

Check:
1. Is Cartographer also showing obstacle correctly?
2. Is `/scan` topic showing obstacle in front? (`ros2 topic echo /scan --once`)
3. Are both files using **same offset value**?
4. Did you rebuild after changing?

## Notes

- TurtleBot3 Burger: LIDAR often mounted backwards (need π)
- TurtleBot3 Waffle: LIDAR orientation varies
- Custom mounts: Depends on mounting angle



