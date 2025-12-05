# LIDAR Validation and Map Visualization Guide

Step-by-step guide for validating LIDAR obstacle detection and voxel grid generation using RViz.

## Step 1: LIDAR Validation - Obstacle Detection

### Prerequisites
- TurtleBot3 with LIDAR connected and running
- ROS2 Humble installed
- RViz2 installed

### Step 1.1: Launch LIDAR Validation Nodes

**Terminal 1 - Source workspace and launch:**
```bash
cd ~/turtlebot3_ws  # or your workspace
source install/setup.bash
ros2 launch turtlebot_interceptor lidar_validation.launch.py
```

This launches:
- `simple_pose_publisher` - Provides robot pose (from /odom or static)
- `lidar_processor_node` - Processes raw LIDAR data
- `slam_node` - Creates voxel grid map from LIDAR scans
- `rviz2` - Visualization tool

**Note:** If you don't have `/odom` topic, the pose publisher will use a static pose at origin.

### Step 1.2: Verify LIDAR Data is Publishing

**Terminal 2 - Check LIDAR topic:**
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 topic echo /scan --once
```

**Expected output:**
- Should see `sensor_msgs/LaserScan` message
- Check `ranges` array has values (not all NaN or Inf)
- Check `range_min` and `range_max` are reasonable (e.g., 0.25m to 3.5m)
- Check `angle_min`, `angle_max`, `angle_increment` are set

**If no data:**
- Check LIDAR is connected: `lsusb | grep -i lidar`
- Check LIDAR driver is running (e.g., `ros2 launch rplidar_ros rplidar.launch.py`)
- Check topic exists: `ros2 topic list | grep scan`
- Check topic info: `ros2 topic info /scan`

**Verify robot pose is available:**
```bash
ros2 topic echo /amcl_pose --once
```
Should see `PoseWithCovarianceStamped` message (from simple_pose_publisher or MCL)

### Step 1.3: Visualize LIDAR in RViz

**In RViz2 (should open automatically):**

1. **Add LIDAR Scan Display:**
   - Click "Add" button (bottom left)
   - Select "By topic" tab
   - Find `/scan` → `LaserScan`
   - Click "OK"

2. **Configure LIDAR Display:**
   - In the left panel, expand "LaserScan"
   - Set **Size (m)**: 0.05 (point size)
   - Set **Color**: Red or Yellow (for visibility)
   - Check **"Use Timestamp"** is enabled

3. **What to Look For:**
   - **Red/Yellow points** should appear around obstacles (cones)
   - Points should form **circular patterns** around cones
   - Points should **update in real-time** as robot moves
   - Points should **disappear** when obstacle is out of range

### Step 1.4: Verify Processed LIDAR (Optional)

**Check processed scan topic:**
```bash
ros2 topic echo /scan_processed --once
```

This shows the filtered/processed LIDAR data (noise removed, invalid readings filtered).

---

## Step 2: Voxel Grid Map Visualization

### Step 2.1: Verify Map is Being Generated

**Terminal 2 - Check map topic:**
```bash
ros2 topic echo /map --once
```

**Expected output:**
- Should see `nav_msgs/OccupancyGrid` message
- Check `data` array has values (not all -1)
- Check `info.resolution` is 0.05 (5cm)
- Check `info.width` and `info.height` are 100

**Check map update rate:**
```bash
ros2 topic hz /map
```
Should update at ~10Hz (depends on LIDAR rate)

### Step 2.2: Visualize Map in RViz

**In RViz2:**

1. **Add Map Display:**
   - Click "Add" button
   - Select "By topic" tab
   - Find `/map` → `Map`
   - Click "OK"

2. **Configure Map Display:**
   - In left panel, expand "Map"
   - Set **Topic**: `/map`
   - Set **Color Scheme**: "costmap" or "map" (for better visibility)
   - **Alpha**: 1.0 (fully opaque)

3. **Set Fixed Frame:**
   - In left panel, set **Fixed Frame** to `map`
   - If `map` frame doesn't exist, use `odom` or `base_link`

4. **What to Look For:**
   - **Black cells** = Occupied (obstacles detected)
   - **White cells** = Free space
   - **Gray cells** = Unknown (not yet scanned)
   - **Voxels should appear** as robot scans obstacles
   - **Cones should appear** as black circular clusters

### Step 2.3: Validate Obstacle Detection

**Move robot around cones and observe:**

1. **Place robot near a cone:**
   - LIDAR should detect cone (red points in scan)
   - Map should show black voxels at cone location
   - Voxels should form circular pattern

2. **Move robot around:**
   - Voxels should accumulate (more scans = better detection)
   - Multiple cones should all appear in map
   - Free space should be marked as white

3. **Check voxel quality:**
   - Voxels should be **small** (5cm = resolution)
   - Voxels should be **tightly clustered** around cones
   - Voxels should **not be huge** or scattered

### Step 2.4: Debug Voxel Grid Issues

**If voxels are not appearing:**

1. **Check LIDAR is publishing:**
   ```bash
   ros2 topic echo /scan --once | head -20
   ```

2. **Check SLAM node is receiving scans:**
   ```bash
   ros2 node info /slam_node
   ```
   Should show subscription to `/scan`

3. **Check map is being published:**
   ```bash
   ros2 topic echo /map --once | grep -A 5 "data:"
   ```
   Should show non-zero values for occupied cells

4. **Check SLAM node logs:**
   Look for messages like:
   - "SLAM node initialized"
   - "Updating map from LIDAR scan"
   - Any error messages

**If voxels look weird/huge:**

1. **Check map resolution:**
   ```bash
   ros2 param get /slam_node resolution
   ```
   Should be 0.05 (5cm)

2. **Check log-odds parameters:**
   ```bash
   ros2 param list /slam_node | grep log_odds
   ros2 param get /slam_node log_odds_occupied
   ros2 param get /slam_node occupancy_threshold
   ```

3. **Adjust parameters if needed:**
   - Lower `occupancy_threshold` (e.g., 0.2) = more sensitive
   - Increase `log_odds_occupied` (e.g., 1.0) = stronger evidence

---

## Step 3: Validate All 5 Cones Are Detected

### Step 3.1: Manual Cone Detection Test

**Place robot at different positions and verify:**

1. **Position 1 - Near cone 1:**
   - Move robot to (0.5, 0.25) - should see cone 1
   - Check map shows black voxels at cone location

2. **Position 2 - Near cone 2:**
   - Move robot to (0.25, 0.75) - should see cone 2
   - Check map shows black voxels

3. **Repeat for all 5 cones:**
   - Cone 3: (0.9, 0.9)
   - Cone 4: (1.25, 0.5)
   - Cone 5: (0.6, 1.4)

### Step 3.2: Check Obstacle Extraction

**Terminal 3 - Monitor obstacle extraction:**
```bash
ros2 topic echo /obstacles  # If this topic exists
```

Or check SLAM node logs for obstacle count.

**Expected:**
- All 5 cones should be detected
- Each cone should appear as circular cluster of voxels
- Cones should be extracted as obstacles (not walls)

---

## RViz Configuration Tips

### Recommended RViz Setup for LIDAR Validation

1. **Add these displays:**
   - `/scan` → LaserScan (red points)
   - `/map` → Map (black/white/gray grid)
   - `/tf` → TF (coordinate frames)
   - `/robot_model` → RobotModel (if available)

2. **View settings:**
   - **View Type**: TopDownOrtho (2D top-down view)
   - **Follow**: None (or robot base_link)
   - **Zoom**: Adjust to see full map

3. **Color scheme:**
   - **LIDAR**: Red (high visibility)
   - **Map**: Costmap scheme (black=obstacle, white=free, gray=unknown)

### Save RViz Configuration

1. Configure RViz as above
2. File → Save Config As
3. Save to: `~/turtlebot3_ws/src/turtlebot_interceptor/config/lidar_validation.rviz`

Then update launch file to use saved config:
```python
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', get_package_share_directory('turtlebot_interceptor') + '/config/lidar_validation.rviz'],
    output='screen'
),
```

---

## Troubleshooting

### Problem: No LIDAR data in RViz

**Solutions:**
1. Check LIDAR is publishing: `ros2 topic list | grep scan`
2. Check topic name matches: RViz should subscribe to `/scan`
3. Check QoS compatibility (should be BEST_EFFORT)
4. Restart LIDAR driver if needed

### Problem: Map is all gray (unknown)

**Solutions:**
1. Check robot pose is available: `ros2 topic echo /amcl_pose --once`
2. Check SLAM node is receiving pose: `ros2 node info /slam_node`
3. Move robot slowly to allow mapping
4. Check SLAM node parameters are correct

### Problem: Voxels not appearing around obstacles

**Solutions:**
1. Check LIDAR is detecting obstacles (red points in scan)
2. Check log-odds parameters (may need tuning)
3. Check occupancy threshold (may be too high)
4. Verify robot is close enough to obstacles (within LIDAR range)

### Problem: Voxels are huge/weird

**Solutions:**
1. Check map resolution: should be 0.05 (5cm)
2. Check RViz display settings (point size vs actual voxel size)
3. Verify log-odds parameters are reasonable
4. Check for multiple map updates causing expansion

---

## Next Steps

Once LIDAR validation is complete:
1. ✅ LIDAR detects all obstacles correctly
2. ✅ Voxel grid shows obstacles as black cells
3. ✅ All 5 cones are visible in map
4. ✅ Map updates in real-time

**Then proceed to:**
- Step 2: Object permanence (obstacles persist when out of view)
- Step 3: MCL localization
- Step 4: MPC navigation

---

## Quick Reference Commands

```bash
# Check LIDAR is publishing
ros2 topic echo /scan --once

# Check map is publishing
ros2 topic echo /map --once

# Monitor map update rate
ros2 topic hz /map

# Check node status
ros2 node list
ros2 node info /slam_node

# View parameters
ros2 param list /slam_node
ros2 param get /slam_node resolution

# View topics
ros2 topic list
ros2 topic info /scan
ros2 topic info /map
```

