#!/usr/bin/env python3
"""
Test script to verify robot movement in MuJoCo simulation.

This script publishes joint trajectory commands to test if the robot responds.

Usage:
    # Make sure both simulator and controller_manager are running, then:
    python examples/test_robot_movement.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class RobotMovementTester(Node):
    """Simple test node to command robot movement."""
    
    def __init__(self):
        super().__init__('robot_movement_tester')
        
        # Publisher for joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info("Robot Movement Tester Started")
        self.get_logger().info("Will publish test trajectories...")
    
    def send_test_trajectory(self):
        """Send a simple test trajectory."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7'
        ]
        
        # Point 1: Move to a slightly different position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Home position
        point1.velocities = [0.0] * 7
        point1.time_from_start = Duration(sec=2, nanosec=0)
        
        # Point 2: Move joint 1
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        point2.velocities = [0.0] * 7
        point2.time_from_start = Duration(sec=4, nanosec=0)
        
        # Point 3: Return to home
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        point3.velocities = [0.0] * 7
        point3.time_from_start = Duration(sec=6, nanosec=0)
        
        msg.points = [point1, point2, point3]
        
        self.get_logger().info("="*60)
        self.get_logger().info("Publishing test trajectory:")
        self.get_logger().info(f"  Point 1 (2s): {point1.positions}")
        self.get_logger().info(f"  Point 2 (4s): {point2.positions}")
        self.get_logger().info(f"  Point 3 (6s): {point3.positions}")
        self.get_logger().info("="*60)
        
        # Publish
        self.publisher.publish(msg)
        self.get_logger().info("✓ Trajectory published!")
        self.get_logger().info("")
        self.get_logger().info("Watch the MuJoCo visualization to see if robot moves.")
        self.get_logger().info("If robot doesn't move, check:")
        self.get_logger().info("  1. Is joint_trajectory_controller active?")
        self.get_logger().info("  2. Are both simulator and controller_manager running?")
        self.get_logger().info("  3. Check console for errors")


def main(args=None):
    rclpy.init(args=args)
    
    tester = RobotMovementTester()
    
    # Wait a bit for connections
    time.sleep(1.0)
    
    try:
        # Send test trajectory
        tester.send_test_trajectory()
        
        # Keep alive for a bit to ensure message is sent
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
