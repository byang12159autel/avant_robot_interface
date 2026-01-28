#!/usr/bin/env python3
"""
ros2_control_example.py

Example demonstrating ROS2 integration with the multi-rate control loop.

This example shows:
1. Running control loop with ROS2 enabled
2. Subscribing to joint commands from ROS2
3. Publishing robot state to ROS2

To run this example:
    python examples/ros2_control_example.py

In another terminal, you can:
- Monitor joint states: ros2 topic echo /joint_states
- Send commands: ros2 topic pub /joint_command sensor_msgs/msg/JointState ...
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from avant_robot_interface.core.simple_control_loop import MultiRateControlLoop


def main():
    """Main function demonstrating ROS2-enabled control loop."""
    
    print("=" * 70)
    print("ROS2 Control Loop Example")
    print("=" * 70)
    print()
    print("This example runs a multi-rate control loop with ROS2 integration:")
    print("  • Main thread runs at 1000 Hz")
    print("  • ROS2 runs in separate thread")
    print("  • Subscribes to: /joint_command")
    print("  • Publishes to: /joint_states")
    print()
    print("To send commands in another terminal:")
    print("  ros2 topic pub /joint_command sensor_msgs/msg/JointState \\")
    print("    '{position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]}'")
    print()
    print("To monitor state:")
    print("  ros2 topic echo /joint_states")
    print()
    print("=" * 70)
    print()
    
    # Define joint names (7-DOF robot)
    joint_names = [
        'joint_1',
        'joint_2', 
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6',
        'joint_7'
    ]
    
    # Create control loop with ROS2 enabled
    control_loop = MultiRateControlLoop(
        enable_ros2=True,
        joint_names=joint_names
    )
    
    # Run for 30 seconds (or until Ctrl+C)
    try:
        control_loop.run(duration_s=30.0)
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    print("\n✓ Example completed")


if __name__ == '__main__':
    main()
