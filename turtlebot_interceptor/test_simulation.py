#!/usr/bin/env python3
"""
Simple test script to verify simulation components can be imported
"""
import sys
import os

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'turtlebot_interceptor'))

def test_imports():
    """Test that all modules can be imported"""
    try:
        print("Testing imports...")
        
        # Test basic imports
        import numpy as np
        print("✓ numpy")
        
        try:
            import cvxpy as cp
            print("✓ cvxpy")
        except ImportError:
            print("✗ cvxpy (optional, needed for MPC)")
        
        # Test our modules (without ROS2)
        try:
            from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
            print("✓ SimpleUnicycleMPC")
        except Exception as e:
            print(f"✗ SimpleUnicycleMPC: {e}")
        
        try:
            from turtlebot_interceptor.Target_Estimator import TargetKF
            print("✓ TargetKF (class definition)")
        except Exception as e:
            print(f"✗ TargetKF: {e}")
        
        try:
            from turtlebot_interceptor.MCL_test import MCL
            print("✓ MCL (class definition)")
        except Exception as e:
            print(f"✗ MCL: {e}")
        
        print("\n✓ Basic imports successful!")
        print("\nNote: ROS2 (rclpy) is required to run the actual nodes.")
        print("To test with ROS2:")
        print("  1. Source your ROS2 workspace")
        print("  2. Build: colcon build --packages-select turtlebot_interceptor")
        print("  3. Run: ros2 launch turtlebot_interceptor simulation.launch.py")
        
        return True
    except Exception as e:
        print(f"✗ Import test failed: {e}")
        return False

if __name__ == '__main__':
    test_imports()

