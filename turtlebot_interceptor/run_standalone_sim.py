#!/usr/bin/env python3
"""
Simple runner for standalone simulation
Can be run directly without ROS2
"""
import sys
import os

# Add package to path
package_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, package_dir)

if __name__ == '__main__':
    from turtlebot_interceptor.standalone_sim import main
    main()

