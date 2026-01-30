#!/usr/bin/env python3
"""
Quick test script for MuJoCo simulator services.

Tests Init and Step services to verify communication.

Usage:
    # Terminal 1: Start simulator
    python examples/mujoco_sim_service_node.py --visualize
    
    # Terminal 2: Run this test
    python examples/test_mujoco_service.py
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    from crisp_mujoco_sim_msgs.srv import Init, Step
except ImportError:
    print("ERROR: crisp_mujoco_sim_msgs not found!")
    print("Please build and source the workspace first:")
    print("  cd ~/crisp_controllers_demos")
    print("  colcon build --packages-select crisp_mujoco_sim_msgs")
    print("  source install/setup.bash")
    sys.exit(1)


class MuJoCoServiceTester(Node):
    """Test client for MuJoCo simulator services."""
    
    def __init__(self):
        super().__init__('mujoco_service_tester')
        
        # Create service clients
        self.init_client = self.create_client(Init, 'mujoco_init')
        self.step_client = self.create_client(Step, 'mujoco_step')
        
        self.get_logger().info("MuJoCo Service Tester initialized")
    
    def test_init_service(self, model_path: str) -> bool:
        """
        Test Init service.
        
        Args:
            model_path: Path to MuJoCo XML model
            
        Returns:
            True if successful
        """
        self.get_logger().info("="*60)
        self.get_logger().info("Testing Init Service")
        self.get_logger().info("="*60)
        
        # Wait for service
        self.get_logger().info("Waiting for /mujoco_init service...")
        if not self.init_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Init service not available!")
            return False
        
        self.get_logger().info("✓ Service available")
        
        # Create request
        request = Init.Request()
        request.model_path = model_path
        
        self.get_logger().info(f"Calling Init service with model: {model_path}")
        
        # Call service
        future = self.init_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("Init service call timeout!")
            return False
        
        response = future.result()
        
        if response.success:
            self.get_logger().info("✓ Init service succeeded")
            self.get_logger().info(f"  DOF: {len(response.position)}")
            self.get_logger().info(f"  Initial positions: {response.position}")
            self.get_logger().info(f"  Initial velocities: {response.velocity}")
            return True
        else:
            self.get_logger().error("✗ Init service failed")
            return False
    
    def test_step_service(self, num_steps: int = 10) -> bool:
        """
        Test Step service.
        
        Args:
            num_steps: Number of steps to test
            
        Returns:
            True if successful
        """
        self.get_logger().info("")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Testing Step Service ({num_steps} steps)")
        self.get_logger().info("="*60)
        
        # Wait for service
        if not self.step_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Step service not available!")
            return False
        
        self.get_logger().info("✓ Service available")
        
        # Test multiple steps
        successes = 0
        total_time = 0.0
        
        for i in range(num_steps):
            # Create request (zero effort commands for 7 DOF)
            request = Step.Request()
            request.effort_commands = [0.0] * 7
            
            # Call service
            start_time = time.time()
            future = self.step_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            elapsed = time.time() - start_time
            total_time += elapsed
            
            if future.done():
                response = future.result()
                successes += 1
                
                if i == 0:
                    self.get_logger().info(f"  Step {i+1}: Success (latency: {elapsed*1000:.2f} ms)")
                    self.get_logger().info(f"    Position: {response.position}")
            else:
                self.get_logger().warn(f"  Step {i+1}: Timeout!")
        
        # Report results
        avg_latency = (total_time / num_steps) * 1000
        success_rate = (successes / num_steps) * 100
        
        self.get_logger().info("")
        self.get_logger().info(f"Results:")
        self.get_logger().info(f"  Success rate: {success_rate:.1f}% ({successes}/{num_steps})")
        self.get_logger().info(f"  Average latency: {avg_latency:.2f} ms")
        
        return successes == num_steps


def main():
    """Main entry point."""
    # Default model path
    home = Path.home()
    default_model = home / "crisp_controllers_demos" / "crisp_controllers_robot_demos" / "config" / "fr3" / "scene.xml"
    
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        model_path = str(default_model)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create tester
    tester = MuJoCoServiceTester()
    
    try:
        print("\n" + "="*60)
        print("MuJoCo Simulator Service Test")
        print("="*60)
        print(f"Model: {model_path}")
        print("="*60 + "\n")
        
        # Test Init service
        init_success = tester.test_init_service(model_path)
        
        if not init_success:
            print("\n✗ Init test failed - stopping")
            return 1
        
        # Test Step service
        step_success = tester.test_step_service(num_steps=10)
        
        if step_success:
            print("\n" + "="*60)
            print("✓ All tests passed!")
            print("="*60)
            return 0
        else:
            print("\n" + "="*60)
            print("✗ Some tests failed")
            print("="*60)
            return 1
            
    except KeyboardInterrupt:
        print("\nTest interrupted")
        return 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
