#!/usr/bin/env python3
"""
Runner for animated simulation
Shows MCL convergence, KF tracking, and MPC in real-time
"""
import sys
import os

package_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, package_dir)

if __name__ == '__main__':
    from turtlebot_interceptor.animated_sim import main
    main()

