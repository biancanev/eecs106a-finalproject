# Debugging Guide - LIDAR and Map Visualization

Quick troubleshooting guide for when you're not seeing LIDAR scans or map data.

## Step 1: Rebuild Package (CRITICAL)

**The parameter fix requires rebuilding:**

```bash
cd ~/turtlebot3_ws  # or your workspace
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

**If you see `ParameterAlreadyDeclaredException`, the package wasn't rebuilt!**

## Step 2: Check Nodes Are Running

**After launching, check nodes:**
```bash
ros2 node list
```

**Expected nodes:**
- `/lidar_processor_node`
- `/slam_node`
- `/mcl_node` (or `/simple_pose_publisher` for validation)
- `/mpc_node` (for navigation)

**If nodes are missing, check logs for errors.**

## Step 3: Check Topics Are Publishing

### Check LIDAR Topic
```bash
ros2 topic list | grep scan
```

**Should see:**
- `/scan` (raw LIDAR)
- `/scan_processed` (optional, from lidar_processor)

**Check if LIDAR is publishing:**
```bash
ros2 topic echo /scan --once
```

**Expected:**
- `sensor_msgs/LaserScan` message
- `ranges` array with values (not all NaN)
- `range_min` and `range_max` set

**If no data:**
- LIDAR driver not running
- LIDAR not connected
- Check: `ros2 topic hz /scan` (should show rate)

### Check Map Topic
```bash
ros2 topic list | grep map
```

**Should see:**
- `/map` (from slam_node)

**Check if map is publishing:**
```bash
ros2 topic echo /map --once
```

**Expected:**
- `nav_msgs/OccupancyGrid` message
- `data` array (may be all -1 initially)
- `info.resolution` = 0.05
- `info.width` = 100, `info.height` = 100

**Check map update rate:**
```bash
ros2 topic hz /map
```

**Should update at ~2Hz (every 0.5 seconds)**

### Check Pose Topic
```bash
ros2 topic echo /amcl_pose --once
```

**Expected:**
- `geometry_msgs/PoseWithCovarianceStamped` message
- Pose position and orientation set

**If no pose:**
- MCL not initialized (needs map first)
- Or `simple_pose_publisher` not running
- Check: `ros2 node info /simple_pose_publisher` or `/mcl_node`

## Step 4: Check SLAM Node Status

**Check if SLAM is receiving data:**
```bash
ros2 node info /slam_node
```

**Should show:**
- Subscription to `/scan` (LaserScan)
- Subscription to `/amcl_pose` (PoseWithCovarianceStamped)
- Publisher to `/map` (OccupancyGrid)

**Check SLAM logs:**
Look for messages like:
- "SLAM node initialized"
- "Updating map from LIDAR scan"
- Any error messages

**If SLAM says "current_pose is None":**
- Pose publisher not running
- Or pose topic not publishing
- Check: `ros2 topic echo /amcl_pose --once`

## Step 5: RViz Configuration

### If Nothing Shows in RViz:

1. **Check Fixed Frame:**
   - Set to `map` (or `odom` if `map` doesn't exist)
   - If frame doesn't exist, data won't display

2. **Add Displays:**
   - **LIDAR Scan:**
     - Add → By topic → `/scan` → `LaserScan`
     - Set color: Red
     - Set size: 0.05
   - **Map:**
     - Add → By topic → `/map` → `Map`
     - Set topic: `/map`
     - Set color scheme: "costmap"

3. **Check Topic Names:**
   - RViz should show topic names in the display list
   - If topic shows "No messages received", check topic is publishing

4. **Check Timestamps:**
   - RViz may filter old messages
   - Check: "Use Timestamp" is enabled in display settings

## Step 6: Common Issues

### Issue: Nodes Crash Immediately

**Symptom:** Nodes start then immediately die

**Check:**
```bash
# Look for error messages in launch output
# Common errors:
# - ParameterAlreadyDeclaredException → Rebuild package!
# - ImportError → Check dependencies
# - ModuleNotFoundError → Install missing packages
```

**Solution:**
- Rebuild package: `colcon build --packages-select turtlebot_interceptor`
- Check Python path: `python3 -c "import turtlebot_interceptor"`

### Issue: No LIDAR Data

**Symptom:** `/scan` topic exists but no data

**Check:**
```bash
ros2 topic info /scan
ros2 topic echo /scan --once
```

**Solutions:**
- Start LIDAR driver: `ros2 launch rplidar_ros rplidar.launch.py`
- Check LIDAR connection: `lsusb | grep -i lidar`
- Check LIDAR permissions (may need sudo or udev rules)

### Issue: Map is All Gray (Unknown)

**Symptom:** Map displays but all cells are gray (-1)

**Check:**
```bash
ros2 topic echo /map --once | grep -A 5 "data:"
```

**If all -1:**
- SLAM not receiving LIDAR scans
- Or SLAM not receiving pose
- Check: `ros2 node info /slam_node`

**Solutions:**
- Ensure pose is publishing: `ros2 topic echo /amcl_pose --once`
- Ensure LIDAR is publishing: `ros2 topic echo /scan --once`
- Move robot slowly to allow mapping
- Check SLAM node logs for errors

### Issue: Voxels Not Appearing

**Symptom:** LIDAR shows obstacles but map doesn't

**Check:**
- SLAM parameters: `ros2 param list /slam_node`
- Occupancy threshold: `ros2 param get /slam_node occupancy_threshold`
- Log-odds parameters: `ros2 param get /slam_node log_odds_occupied`

**Solutions:**
- Lower `occupancy_threshold` (e.g., 0.2)
- Increase `log_odds_occupied` (e.g., 1.0)
- Move robot closer to obstacles
- Check LIDAR range is within limits

### Issue: Map Updates But No Obstacles

**Symptom:** Map updates but stays white (no black cells)

**Check:**
- LIDAR is detecting obstacles (red points in scan)
- Robot is close enough to obstacles (within LIDAR range)
- SLAM parameters are reasonable

**Solutions:**
- Place robot near a cone/obstacle
- Check LIDAR scan shows obstacles
- Adjust SLAM parameters (see above)

## Step 7: Quick Diagnostic Commands

**Full system check:**
```bash
# Check all topics
ros2 topic list

# Check all nodes
ros2 node list

# Check LIDAR
ros2 topic hz /scan
ros2 topic echo /scan --once | head -20

# Check map
ros2 topic hz /map
ros2 topic echo /map --once | grep -A 10 "info:"

# Check pose
ros2 topic echo /amcl_pose --once

# Check node connections
ros2 node info /slam_node
ros2 node info /mcl_node
```

## Step 8: Start with LIDAR Validation

**If nothing works, start simple:**

```bash
# Launch just LIDAR validation (no MCL, no MPC)
ros2 launch turtlebot_interceptor lidar_validation.launch.py
```

**This launches:**
- `simple_pose_publisher` (provides pose)
- `lidar_processor_node` (processes LIDAR)
- `slam_node` (creates map)
- `rviz2` (visualization)

**Then configure RViz:**
- Add `/scan` → LaserScan
- Add `/map` → Map
- Set Fixed Frame to `map`

**Once this works, add MCL and MPC back.**

## Step 9: Verify Package Installation

**Check package is installed:**
```bash
ros2 pkg list | grep turtlebot_interceptor
```

**Check executables exist:**
```bash
ros2 pkg executables turtlebot_interceptor
```

**Should see:**
- `lidar_processor_node`
- `slam_node`
- `mcl_node`
- `mpc_node`
- `simple_pose_publisher`

**If missing, rebuild:**
```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

## Still Not Working?

1. **Check ROS2 installation:**
   ```bash
   ros2 --version
   source /opt/ros/humble/setup.bash
   ```

2. **Check workspace setup:**
   ```bash
   echo $ROS_DOMAIN_ID
   source ~/turtlebot3_ws/install/setup.bash
   ```

3. **Check for conflicting nodes:**
   ```bash
   ros2 node list
   # Kill any old nodes if needed
   ```

4. **Check logs:**
   ```bash
   # Logs are in ~/.ros/log/
   ls -la ~/.ros/log/
   ```

5. **Restart everything:**
   - Kill all nodes: `pkill -f ros2`
   - Rebuild: `colcon build --packages-select turtlebot_interceptor`
   - Source: `source install/setup.bash`
   - Launch again

