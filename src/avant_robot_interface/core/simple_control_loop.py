"""
simple_control_loop.py
Minimal working example of multi-rate control loop with optional ROS2 integration.

Demonstrates:
- Task 1: 1000 Hz (every iteration) - Read sensors & ROS2 commands
- Task 2: 500 Hz  (every 2 iterations) - Control computation
- Task 3: 100 Hz  (every 10 iterations) - Publish to ROS2
"""

import time
import numpy as np
from typing import Optional

class MultiRateControlLoop:
    """
    Simple multi-rate control loop with optional ROS2 integration.
    Main loop runs at fastest rate, slower tasks are decimated.
    
    ROS2 Architecture:
    - Main thread: Control loop (1000 Hz)
    - ROS2 thread: rclpy.spin() for pub/sub
    - Thread-safe handlers: Exchange data between threads
    """

    def __init__(self, enable_ros2: bool = False, joint_names: list = None):
        """
        Args:
            enable_ros2: Enable ROS2 integration
            joint_names: List of joint names for ROS2 publishing
        """
        # Timing configuration
        self.base_frequency_hz = 1000.0  # Fastest rate
        self.base_dt = 1.0 / self.base_frequency_hz  # 1ms

        # Task frequencies
        self.task1_frequency = 1000.0  # Hz
        self.task2_frequency = 500.0   # Hz
        self.task3_frequency = 100.0   # Hz

        # Calculate decimation factors
        self.task1_decimation = int(self.base_frequency_hz / self.task1_frequency)  # = 1
        self.task2_decimation = int(self.base_frequency_hz / self.task2_frequency)  # = 2
        self.task3_decimation = int(self.base_frequency_hz / self.task3_frequency)  # = 10

        # Counters
        self.iteration_counter = 0
        self.task1_counter = 0
        self.task2_counter = 0
        self.task3_counter = 0
        
        # Robot state (simulated)
        self.num_joints = 7  # Example: 7-DOF robot
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_efforts = np.zeros(self.num_joints)
        
        # ROS2 integration
        self.enable_ros2 = enable_ros2
        self.ros2_node: Optional['ROS2Node'] = None
        
        if enable_ros2:
            try:
                from ..ros2 import create_ros2_node
                self.ros2_node = create_ros2_node(
                    node_name='simple_control_loop',
                    joint_names=joint_names
                )
                print(f"✓ ROS2 integration enabled")
            except ImportError as e:
                print(f"⚠️ ROS2 not available: {e}")
                self.enable_ros2 = False
            except Exception as e:
                print(f"✗ Failed to initialize ROS2: {e}")
                self.enable_ros2 = False

        print(f"\nControl Loop Configuration:")
        print(f"  Base frequency: {self.base_frequency_hz} Hz ({self.base_dt*1000:.2f} ms)")
        print(f"  Task 1: {self.task1_frequency} Hz (every {self.task1_decimation} iteration)")
        print(f"  Task 2: {self.task2_frequency} Hz (every {self.task2_decimation} iterations)")
        print(f"  Task 3: {self.task3_frequency} Hz (every {self.task3_decimation} iterations)")
        print(f"  ROS2 enabled: {self.enable_ros2}")
        print()

    def task1_fast(self):
        """
        Fast task - runs at 1000 Hz.
        Reads sensors and ROS2 commands.
        """
        # ═══════════════════════════════════════════════
        # 1. Read sensor data (simulated)
        # ═══════════════════════════════════════════════
        sensor_noise = np.random.randn(3) * 0.001
        
        # ═══════════════════════════════════════════════
        # 2. Check for ROS2 commands
        # ═══════════════════════════════════════════════
        if self.enable_ros2 and self.ros2_node is not None:
            ros2_cmd = self.ros2_node.get_joint_command(max_age_s=1.0)
            if ros2_cmd is not None:
                # Use ROS2 command
                self.joint_positions = ros2_cmd.copy()
                # print(f"  [ROS2] Received command: {ros2_cmd[:3]}...")
        
        self.task1_counter += 1
        return sensor_noise

    def task2_medium(self):
        """
        Medium task - runs at 500 Hz.
        Control computation.
        """
        # ═══════════════════════════════════════════════
        # Simulate control computation
        # ═══════════════════════════════════════════════
        
        # Example: Simple trajectory following with sine wave
        t = time.time()
        target_offset = np.sin(t) * 0.1
        
        # Update joint positions (simple integration)
        self.joint_positions += np.random.randn(self.num_joints) * 0.01
        self.joint_velocities = np.random.randn(self.num_joints) * 0.05
        self.joint_efforts = np.random.randn(self.num_joints) * 0.1
        
        self.task2_counter += 1
        return target_offset

    def task3_slow(self):
        """
        Slow task - runs at 100 Hz.
        Publish to ROS2 and logging.
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
        # 2. Logging / monitoring
        # ═══════════════════════════════════════════════
        self.task3_counter += 1
        
        # Print status every second (100 Hz / 100 = 1 Hz)
        if self.task3_counter % 100 == 0:
            print(f"  [Status] Iteration: {self.iteration_counter}, "
                  f"Joint pos[0]: {self.joint_positions[0]:.3f}, "
                  f"ROS2: {'ON' if self.enable_ros2 else 'OFF'}")
        
        result = np.random.randn(10)
        return result

    def run(self, duration_s: float = 5.0):
        """
        Run the multi-rate control loop.

        Args:
            duration_s: Duration to run in seconds
        """
        print(f"Starting control loop for {duration_s}s...")
        print("=" * 60)

        start_time = time.time()
        self.iteration_counter = 0

        try:
            while True:
                loop_start = time.time()

                # Check if duration exceeded
                elapsed = loop_start - start_time
                if elapsed >= duration_s:
                    break

                # ═══════════════════════════════════════════════
                # TASK 1: Fast (1000 Hz) - runs EVERY iteration
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task1_decimation == 0:
                    result1 = self.task1_fast()
                    # Use result1...

                # ═══════════════════════════════════════════════
                # TASK 2: Medium (500 Hz) - runs every 2 iterations
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task2_decimation == 0:
                    result2 = self.task2_medium()
                    # Use result2...

                # ═══════════════════════════════════════════════
                # TASK 3: Slow (100 Hz) - runs every 10 iterations
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task3_decimation == 0:
                    result3 = self.task3_slow()
                    # Use result3...

                # Increment counter
                self.iteration_counter += 1

                # Sleep to maintain timing (simple rate limiting)
                loop_elapsed = time.time() - loop_start
                sleep_time = self.base_dt - loop_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n⚠️ Interrupted by user")
        finally:
            # Cleanup
            self.shutdown()

        # Print statistics
        elapsed = time.time() - start_time
        self.print_statistics(elapsed)

    def shutdown(self):
        """Shutdown resources (including ROS2)."""
        if self.enable_ros2 and self.ros2_node is not None:
            self.ros2_node.shutdown()

    def print_statistics(self, elapsed: float):
        """Print execution statistics."""
        print("=" * 60)
        print("Execution Statistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total iterations: {self.iteration_counter}")
        print(f"  Average frequency: {self.iteration_counter/elapsed:.1f} Hz")
        print()
        print(f"  Task 1 calls: {self.task1_counter} "
              f"(expected: {int(self.task1_frequency * elapsed)})")
        print(f"  Task 2 calls: {self.task2_counter} "
              f"(expected: {int(self.task2_frequency * elapsed)})")
        print(f"  Task 3 calls: {self.task3_counter} "
              f"(expected: {int(self.task3_frequency * elapsed)})")
        print("=" * 60)


if __name__ == '__main__':
    # Example 1: Run without ROS2
    print("Example 1: Without ROS2")
    print("-" * 60)
    control_loop = MultiRateControlLoop(enable_ros2=False)
    control_loop.run(duration_s=2.0)
    
    print("\n\n")
    
    # Example 2: Run with ROS2
    print("Example 2: With ROS2")
    print("-" * 60)
    joint_names = [f'joint_{i+1}' for i in range(7)]
    control_loop_ros2 = MultiRateControlLoop(enable_ros2=True, joint_names=joint_names)
    control_loop_ros2.run(duration_s=5.0)
