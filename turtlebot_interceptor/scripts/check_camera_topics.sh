#!/bin/bash
# Quick diagnostic script to check camera topics

echo "=== CAMERA TOPIC DIAGNOSTICS ==="
echo ""

echo "1. All camera-related topics:"
ros2 topic list | grep -i camera || echo "   ❌ No camera topics found!"
echo ""

echo "2. All image topics:"
ros2 topic list | grep -i image || echo "   ❌ No image topics found!"
echo ""

echo "3. Checking /camera/image_raw:"
if ros2 topic list | grep -q "/camera/image_raw"; then
    echo "   ✅ Topic exists"
    ros2 topic hz /camera/image_raw --window 5 2>/dev/null || echo "   ⚠️  Not publishing (or slow)"
else
    echo "   ❌ Topic does not exist"
fi
echo ""

echo "4. Checking /camera/camera_info:"
if ros2 topic list | grep -q "/camera/camera_info"; then
    echo "   ✅ Topic exists"
    ros2 topic echo /camera/camera_info --once 2>/dev/null | head -20 || echo "   ⚠️  Not publishing"
else
    echo "   ❌ Topic does not exist"
fi
echo ""

echo "5. Checking /amcl_pose:"
if ros2 topic list | grep -q "/amcl_pose"; then
    echo "   ✅ Topic exists"
    ros2 topic echo /amcl_pose --once 2>/dev/null | head -10 || echo "   ⚠️  Not publishing"
else
    echo "   ❌ Topic does not exist"
fi
echo ""

echo "6. Camera detector node status:"
if ros2 node list | grep -q "camera_cone_detector"; then
    echo "   ✅ Node is running"
    echo "   Node info:"
    ros2 node info /camera_cone_detector 2>/dev/null | grep -A 5 "Subscribers:" || echo "   ⚠️  Could not get node info"
else
    echo "   ❌ Node is not running"
fi
echo ""

echo "=== END DIAGNOSTICS ==="

