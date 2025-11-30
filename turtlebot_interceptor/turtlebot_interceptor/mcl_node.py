#!/usr/bin/env python3
"""
MCL Node for Monte Carlo Localization
Based on the paper implementation
"""
import rclpy
from rclpy.node import Node
from turtlebot_interceptor.MCL_test import MCL


def main(args=None):
    rclpy.init(args=args)
    
    # Check if we're in simulation
    use_sim = False
    import sys
    if '--sim' in sys.argv or 'use_sim_time:=true' in ' '.join(sys.argv):
        use_sim = True
    
    node = MCL(N=300, motion_noise=[0.02, 0.02, 0.01], use_sim=use_sim)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

