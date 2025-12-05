# MPC Debugging Guide - Why MPC Fails

## The "Strict Inequalities" Error Explained

**Error**: `Strict inequalities are not allowed`

**What it means:**
- CVXPY (the optimization library) doesn't allow strict inequalities (`<` or `>`) in constraints
- Only non-strict inequalities (`<=` or `>=`) are allowed
- This error occurs when CVXPY detects a strict inequality somewhere

## Root Cause

The issue was in the **obstacle cost calculation**:
1. We were using Python `if` statements with CVXPY expressions
2. When rebuilding the problem, CVXPY was interpreting these as strict inequalities
3. Example: `if dist_sq < safety_radius_sq` - CVXPY sees this as a strict comparison

## The Fix

**Changed from:**
```python
if dist_sq < safety_radius_sq * 2.25:
    obstacle_cost += repulsion_weight * np.exp(...)
```

**To:**
```python
# Simple smooth penalty: 1/(distance^2)
# No conditionals - always computed, CVXPY-compatible
obstacle_cost += repulsion_weight / (dist_sq + safety_radius_sq * 0.1)
```

**Why this works:**
- No Python `if` statements
- Smooth function that's always computed
- CVXPY-compatible (no strict inequalities)
- Still provides obstacle avoidance (large penalty when close)

## Why Robot Moves Wrong Direction

If MPC is failing and using fallback control, check:

1. **Robot pose (theta) might be wrong:**
   - Check MCL estimate: `ros2 topic echo /amcl_pose`
   - The arrow direction in RViz should match robot's actual heading

2. **Goal coordinates might be wrong:**
   - Check goal: `ros2 param get /mpc_node goal_x`
   - Should be in `map` frame

3. **Sign error in fallback control:**
   - Fallback computes: `angle_err = angle_to_target - theta`
   - If theta is wrong, angle_err will be wrong
   - Check debug logs: `ros2 topic echo /rosout | grep Fallback`

## Debugging Steps

### 1. Check if MPC is Actually Running

```bash
# Check for MPC errors
ros2 topic echo /rosout | grep "MPC solve failed"
```

**If you see errors:**
- MPC is failing → using fallback control
- Check error message for details

### 2. Check Robot Pose

```bash
# Check MCL pose estimate
ros2 topic echo /amcl_pose --once

# Check true pose (from odometry)
ros2 topic echo /odom --once
```

**Compare:**
- MCL estimate should be close to true pose
- If very different, MCL is wrong → robot will go wrong direction

### 3. Check Goal Coordinates

```bash
# Check goal parameters
ros2 param get /mpc_node goal_x
ros2 param get /mpc_node goal_y

# Check in RViz
# Red sphere should be at goal location
```

### 4. Check Fallback Control Logs

The fallback control now logs every 2 seconds:
```
Fallback: robot=(x, y, theta°), goal=(x, y), dist=Xm, angle_to_target=X°, angle_err=X°
```

**What to look for:**
- `angle_err` should be small (robot pointing towards goal)
- If `angle_err` is large, robot needs to turn more
- If `angle_err` has wrong sign, there's a coordinate issue

### 5. Verify Coordinate Frames

**All should be in `map` frame:**
- Robot pose (`/amcl_pose`) → `frame_id: map`
- Goal coordinates → in `map` frame
- Map (`/map`) → `frame_id: map`

**Check:**
```bash
ros2 topic echo /amcl_pose --once | grep frame_id
ros2 topic echo /map --once | grep frame_id
```

## Common Issues

### Issue: Robot Goes Opposite Direction

**Cause:** Sign error in angle calculation or wrong coordinate frame

**Fix:**
- Check if `theta` (robot heading) is correct
- Check if goal is in correct frame
- Verify `angle_err = angle_to_target - theta` is correct

### Issue: MPC Always Fails

**Cause:** Obstacle cost or problem rebuilding issue

**Fix:**
- The new smooth penalty should fix this
- If still failing, check CVXPY version: `pip show cvxpy`
- Try without obstacles: set `obstacles=None` in MPC call

### Issue: Robot Doesn't Move

**Cause:** Fallback control might have wrong gains or robot pose is wrong

**Fix:**
- Check fallback control gains: `ros2 param get /mpc_node Kp_v`
- Check if robot pose is updating: `ros2 topic hz /amcl_pose`
- Check if commands are publishing: `ros2 topic echo /cmd_vel`

## Quick Test: Disable Obstacles

To test if obstacles are causing MPC to fail:

**Temporarily modify `mpc_node.py`:**
```python
# In timer_callback, change:
obstacles = self.compute_obstacles()
# To:
obstacles = None  # Disable obstacles for testing
```

**If MPC works without obstacles:**
- The obstacle cost was the issue (now fixed)
- Rebuild and test again

**If MPC still fails:**
- There's another issue (check CVXPY, problem structure, etc.)

## Expected Behavior

**After fix:**
1. ✅ MPC should solve successfully (no "Strict inequalities" errors)
2. ✅ Robot should move towards goal
3. ✅ Fallback control should work correctly if MPC fails
4. ✅ Debug logs should show correct angles and distances

**If still having issues:**
- Check the debug logs from fallback control
- Verify robot pose is correct
- Verify goal coordinates are correct
- Check coordinate frames match

