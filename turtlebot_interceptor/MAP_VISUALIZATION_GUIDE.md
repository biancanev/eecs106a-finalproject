# Map Visualization Guide - Seeing the Voxel Grid

## Problem: Map Not Showing / No Voxel Grid

The map (voxel grid) should show as:
- **Black cells** = Occupied (obstacles detected)
- **White cells** = Free space
- **Gray cells** = Unknown (not yet scanned)

## Step 1: Rebuild Package

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot_interceptor
source install/setup.bash
```

## Step 2: Check Map is Publishing

**Terminal 1 - Check map topic:**
```bash
ros2 topic echo /map --once
```

**Should see:**
- `nav_msgs/OccupancyGrid` message
- `data` array with values (not all -1)
- `info.resolution` = 0.05
- `info.width` = 100, `info.height` = 100

**Check map update rate:**
```bash
ros2 topic hz /map
```
Should update at ~2Hz (every 0.5 seconds)

**Check if map has occupied cells:**
```bash
ros2 topic echo /map --once | grep -A 100 "data:" | head -20
```
Look for values > 0 (occupied cells) or 0 (free space). If all -1, map hasn't been updated yet.

## Step 3: Check Pose is Publishing

**SLAM needs pose to update map!**

```bash
ros2 topic echo /amcl_pose --once
```

**Should see:**
- `geometry_msgs/PoseWithCovarianceStamped` message
- Pose position and orientation set

**If no pose:**
- Check `simple_pose_publisher` is running: `ros2 node list | grep pose`
- Check `/odom` topic exists: `ros2 topic list | grep odom`
- If no `/odom`, the pose publisher will use static pose

**Check pose update rate:**
```bash
ros2 topic hz /amcl_pose
```

## Step 4: Check SLAM Node Logs

**Look for messages like:**
- "SLAM node initialized"
- "Map published: X occupied, Y free, Z unknown, pose=True"
- "SLAM: Waiting for pose" (if pose is missing)

**If you see "Waiting for pose":**
- Check `/amcl_pose` topic is publishing
- Check `simple_pose_publisher` node is running
- Check `/odom` topic exists (if using odometry)

## Step 5: Configure RViz for Map Display

### Fix Frame Issue

The map uses `map` frame, but RViz fixed frame might be `base_scan`. You need a TF transform.

**The launch file now includes a static TF publisher** that links `map` → `base_scan`.

**In RViz:**
1. **Set Fixed Frame:**
   - Global Options → Fixed Frame → Set to `map` (for map display)
   - OR keep `base_scan` if TF transform is working

2. **Add Map Display:**
   - Click "Add" → "By topic" → `/map` → `Map` → OK
   - In Map display settings:
     - **Topic**: `/map`
     - **Color Scheme**: "costmap" (best for seeing voxels)
     - **Alpha**: 1.0 (fully opaque)
     - **Draw Behind**: Unchecked (so map is visible)

3. **Configure Map Colors:**
   - **Color Scheme**: "costmap" shows:
     - Black = Occupied (obstacles)
     - White = Free space
     - Gray = Unknown
   - OR use "map" scheme (similar)

### View Settings

1. **View Type**: "TopDownOrtho" (2D top-down view)
2. **Zoom**: Adjust to see full map (5m x 5m = 100 cells at 0.05m resolution)
3. **Target Frame**: `<Fixed Frame>` (map)

## Step 6: Verify Map is Updating

**Move robot slowly** (or if using static pose, ensure LIDAR is scanning obstacles):

1. **Watch SLAM logs:**
   ```bash
   # Should see periodic messages:
   # "Map published: X occupied, Y free, Z unknown, pose=True"
   ```

2. **Check map data changes:**
   ```bash
   ros2 topic echo /map --once | grep -A 5 "data:" | head -10
   ```
   Values should change from all -1 (unknown) to mix of 0 (free), 100 (occupied), and -1 (unknown)

3. **In RViz:**
   - Map should show black cells appearing around obstacles
   - White cells appearing in free space
   - Gray cells in unscanned areas

## Step 7: Troubleshooting

### Issue: Map shows "No map received"

**Check:**
1. Map topic exists: `ros2 topic list | grep map`
2. Map is publishing: `ros2 topic hz /map`
3. RViz is subscribed: Check Map display shows topic `/map`

**Fix:**
- Restart RViz
- Re-add Map display
- Check topic name matches

### Issue: Map is all gray (unknown)

**Check:**
1. Pose is publishing: `ros2 topic echo /amcl_pose --once`
2. SLAM is receiving pose: Check SLAM logs for "pose=True"
3. LIDAR is scanning obstacles: Check LIDAR scan shows obstacles (red points)

**Fix:**
- Ensure `simple_pose_publisher` is running
- Move robot slowly to allow mapping
- Check LIDAR is detecting obstacles

### Issue: Map has no occupied cells (all white/gray)

**Check:**
1. LIDAR is detecting obstacles: Red points in scan around obstacles
2. SLAM parameters: `ros2 param get /slam_node occupancy_threshold`
3. Log-odds parameters: `ros2 param get /slam_node log_odds_occupied`

**Fix:**
- Lower `occupancy_threshold` (e.g., 0.2)
- Increase `log_odds_occupied` (e.g., 1.0)
- Move robot closer to obstacles
- Check LIDAR range is within limits

### Issue: Frame error in RViz

**Error**: "message filter dropping message: frame 'map'"

**Fix:**
- The launch file now includes static TF publisher
- Set Fixed Frame to `map` in RViz
- OR set Fixed Frame to `base_scan` and ensure TF transform exists

**Check TF:**
```bash
ros2 run tf2_ros tf2_echo map base_scan
```

Should show transform (or error if missing).

## Step 8: Understanding the Voxel Grid

**Map Resolution**: 0.05m (5cm per cell)
**Map Size**: 100 x 100 cells = 5m x 5m
**Map Origin**: (-2.5, -2.5) in map frame

**Occupancy Values:**
- `-1` = Unknown (gray)
- `0` = Free space (white)
- `1-100` = Occupied (black, higher = more certain)

**Log-Odds to Occupancy:**
- Log-odds > threshold → Occupied (black)
- Log-odds < -threshold → Free (white)
- Otherwise → Unknown (gray)

## Quick Diagnostic Commands

```bash
# Check all topics
ros2 topic list

# Check map is publishing
ros2 topic hz /map
ros2 topic echo /map --once | head -30

# Check pose is publishing
ros2 topic hz /amcl_pose
ros2 topic echo /amcl_pose --once

# Check nodes
ros2 node list
ros2 node info /slam_node

# Check parameters
ros2 param list /slam_node
ros2 param get /slam_node occupancy_threshold
ros2 param get /slam_node log_odds_occupied

# Check TF
ros2 run tf2_ros tf2_echo map base_scan
```

## Expected Behavior

**After launching:**
1. ✅ `simple_pose_publisher` publishes pose (check logs)
2. ✅ `slam_node` receives pose and scans
3. ✅ Map updates every 0.5 seconds
4. ✅ Map shows occupied cells (black) around obstacles
5. ✅ RViz displays map with proper colors

**If all working:**
- Map should show black voxels (occupied cells) around obstacles
- White voxels (free space) in open areas
- Gray voxels (unknown) in unscanned areas
- Map updates in real-time as robot scans

