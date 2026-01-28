#!/usr/bin/env python3
"""
ros2_control_example.py

Example demonstrating ROS2 integration with the multi-rate control loop.

This example shows:
1. How to extend BaseMultiRateControlLoop for custom control logic
2. Subscribing to joint commands from ROS2
3. Publishing robot state to ROS2
4. Multi-rate task scheduling (1000 Hz, 500 Hz, 100 Hz)

To run this example:
    python examples/ros2_control_example.py

In another terminal, you can:
- Monitor joint states: ros2 topic echo /joint_states
- Send commands: ros2 topic pub /joint_command sensor_msgs/msg/JointState ...
"""

import sys
import os
import time
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from avant_robot_interface.core.simple_control_loop import BaseMultiRateControlLoop


class ROS2ControlLoop(BaseMultiRateControlLoop):
    """
    Custom multi-rate control loop demonstrating ROS2 integration.
    
    This class extends BaseMultiRateControlLoop to implement a robot
    control system with ROS2 communication:
    - Task 1 (1000 Hz): Read sensors and check for ROS2 commands
    - Task 2 (500 Hz): Perform control computation
    - Task 3 (100 Hz): Publish robot state to ROS2 and log status
    """
    
    def __init__(self, joint_names):
        """
        Initialize the ROS2-enabled control loop.
        
        Args:
            joint_names: List of joint names for the robot
        """
        # Initialize base class with ROS2 enabled
        super().__init__(
            base_frequency_hz=1000.0,      # Main loop at 1000 Hz
            task1_frequency_hz=1000.0,     # Fast task: sensor reading
            task2_frequency_hz=500.0,      # Medium task: control computation
            task3_frequency_hz=100.0,      # Slow task: ROS2 publishing
            enable_ros2=True,              # Enable ROS2 integration
            joint_names=joint_names        # Joint names for ROS2 messages
        )
        
        # Robot configuration (7-DOF robot)
        self.num_joints = len(joint_names)
        self.joint_names = joint_names
        
        # Robot state
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_efforts = np.zeros(self.num_joints)
        
        # Control parameters
        self.trajectory_amplitude = 0.1  # Amplitude of sine wave trajectory
        self.trajectory_frequency = 0.5  # Hz
        
        print(f"✓ ROS2ControlLoop initialized with {self.num_joints} joints")
    
    def task1_fast(self):
        """
        Fast task - runs at 1000 Hz.
        
        This task handles high-frequency operations:
        - Read sensor data (simulated here)
        - Check for incoming ROS2 commands
        - Update robot state based on commands
        
        Returns:
            Sensor data (simulated noise)
        """
        # ═══════════════════════════════════════════════
        # 1. Read sensor data (simulated)
        # ═══════════════════════════════════════════════
        sensor_noise = np.random.randn(3) * 0.001
        
        # ═══════════════════════════════════════════════
        # 2. Check for ROS2 joint commands
        # ═══════════════════════════════════════════════
        if self.enable_ros2 and self.ros2_node is not None:
            ros2_cmd = self.ros2_node.get_joint_command(max_age_s=1.0)
            if ros2_cmd is not None:
                # Update target positions based on ROS2 command
                self.joint_positions = ros2_cmd.copy()
                
                # Uncomment to see when commands are received:
                # print(f"  [ROS2] Received command: {ros2_cmd[:3]}...")
        
        return sensor_noise
    
    def task2_medium(self):
        """
        Medium task - runs at 500 Hz.
        
        This task handles control computation:
        - Calculate desired trajectory
        - Compute control commands
        - Update robot state (simulated dynamics)
        
        Returns:
            Target offset from trajectory
        """
        # ═══════════════════════════════════════════════
        # 1. Generate reference trajectory (sine wave)
        # ═══════════════════════════════════════════════
        t = time.time()
        target_offset = np.sin(2 * np.pi * self.trajectory_frequency * t) * self.trajectory_amplitude
        
        # ═══════════════════════════════════════════════
        # 2. Simulate control computation and dynamics
        # ═══════════════════════════════════════════════
        # In a real system, this would be actual control computation
        # For simulation, we add small random changes
        self.joint_positions += np.random.randn(self.num_joints) * 0.01
        self.joint_velocities = np.random.randn(self.num_joints) * 0.05
        
        # Simulate joint efforts (torques)
        self.joint_efforts = np.random.randn(self.num_joints) * 0.1
        
        return target_offset
    
    def task3_slow(self):
        """
        Slow task - runs at 100 Hz.
        
        This task handles low-frequency operations:
        - Publish robot state to ROS2
        - Log status information
        - Monitor system health
        
        Returns:
            Status data for monitoring
        """
        # ═══════════════════════════════════════════════
        # 1. Publish robot state to ROS2
        # ═══════════════════════════════════════════════
        if self.enable_ros2 and self.ros2_node is not None:
            self.ros2_node.publish_robot_state(
                positions=self.joint_positions,
                velocities=self.joint_velocities,
                efforts=self.joint_efforts
            )
        
        # ═══════════════════════════════════════════════
        # 2. Periodic logging (every 1 second)
        # ═══════════════════════════════════════════════
        if self.task3_counter % 100 == 0:
            print(f"  [Status] Iteration: {self.iteration_counter}, "
                  f"Joint pos[0]: {self.joint_positions[0]:.3f}, "
                  f"Joint vel[0]: {self.joint_velocities[0]:.3f}, "
                  f"ROS2: {'ON' if self.enable_ros2 else 'OFF'}")
        
        # Return status data for monitoring
        status_data = {
            'iteration': self.iteration_counter,
            'positions': self.joint_positions.copy(),
            'velocities': self.joint_velocities.copy()
        }
        return status_data


def main():
    """Main function demonstrating ROS2-enabled control loop."""
    
    print("=" * 70)
    print("ROS2 Control Loop Example")
    print("=" * 70)
    print()
    print("This example demonstrates:")
    print("  • Custom control loop extending BaseMultiRateControlLoop")
    print("  • Multi-rate task scheduling:")
    print("    - Task 1: 1000 Hz (sensor reading + ROS2 commands)")
    print("    - Task 2: 500 Hz (control computation)")
    print("    - Task 3: 100 Hz (ROS2 publishing + logging)")
    print("  • ROS2 integration with separate thread")
    print()
    print("ROS2 Topics:")
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
    
    # Create custom ROS2-enabled control loop
    control_loop = ROS2ControlLoop(joint_names=joint_names)
    
    # Run for 30 seconds (or until Ctrl+C)
    try:
        control_loop.run(duration_s=30.0)
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    
    print("\n✓ Example completed")


if __name__ == '__main__':
    main()
