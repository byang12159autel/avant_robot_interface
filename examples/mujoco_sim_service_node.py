#!/usr/bin/env python3
"""
Python MuJoCo Simulator Node with ROS2 Service Interface

Provides Init and Step services for crisp_mujoco_sim System Interface.
Runs MuJoCo simulation in Python with optional visualization.

This implements Approach 1 from the architecture diagrams: Service-based
communication between C++ System Interface and Python MuJoCo simulator.

Services:
- /mujoco_init: Initialize simulation with model path
- /mujoco_step: Step simulation with effort commands, return state

Architecture:
- Process-isolated from controller_manager
- Uses crisp_mujoco_sim_msgs service definitions
- Real-time wall-clock synchronization
- Optional visualization using MuJoCoVisualizer

Usage:
    # Basic usage with visualization
    python examples/mujoco_sim_service_node.py \\
        --model /path/to/scene.xml \\
        --visualize
    
    # Specify simulation frequency
    python examples/mujoco_sim_service_node.py \\
        --model /path/to/scene.xml \\
        --freq 1000 \\
        --visualize
    
    # Wait for Init service call (default)
    python examples/mujoco_sim_service_node.py --visualize
    
    # Auto-start with model
    python examples/mujoco_sim_service_node.py \\
        --model crisp_controllers_robot_demos/config/fr3/scene.xml \\
        --auto-start \\
        --visualize

Example with controller_manager:
    # Terminal 1: Start Python simulator
    python examples/mujoco_sim_service_node.py --visualize
    
    # Terminal 2: Launch controller_manager (with modified System Interface)
    ros2 launch crisp_controllers_robot_demos franka.launch.py

Author: Generated for crisp_controllers_demos migration
Date: 2026-01-30
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import mujoco

# Import service definitions
try:
    from crisp_mujoco_sim_msgs.srv import Init, Step
except ImportError:
    print("ERROR: crisp_mujoco_sim_msgs not found!")
    print("Please build the crisp_mujoco_sim_msgs package first:")
    print("  cd /path/to/crisp_controllers_demos")
    print("  colcon build --packages-select crisp_mujoco_sim_msgs")
    print("  source install/setup.bash")
    sys.exit(1)

# Import visualization (optional)
try:
    from avant_robot_interface.visualization.mujoco.mujuco_viewer import MuJoCoVisualizer
    VISUALIZER_AVAILABLE = True
except ImportError:
    VISUALIZER_AVAILABLE = False
    print("Warning: MuJoCoVisualizer not available, --visualize will be ignored")


class MuJoCoSimulatorService(Node):
    """
    ROS2 Service Node for MuJoCo Simulation.
    
    Provides Init and Step services for hardware interface integration.
    Runs simulation with real-time wall-clock synchronization.
    """
    
    def __init__(self, args):
        """
        Initialize the MuJoCo simulator service node.
        
        Args:
            args: Command-line arguments
        """
        super().__init__('mujoco_simulator_service')
        
        self.args = args
        
        # Simulation state
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.sim_time = 0.0
        self.start_wall_time = None
        self.initialized = False
        
        # Timing
        self.sim_freq_hz = args.freq
        self.dt = 1.0 / self.sim_freq_hz
        
        # Visualization
        self.visualizer: Optional[MuJoCoVisualizer] = None
        self.viz_counter = 0
        self.viz_decimation = max(1, int(self.sim_freq_hz / args.viz_freq))
        
        # Create services
        self.init_service = self.create_service(
            Init,
            'mujoco_init',
            self.handle_init_request
        )
        
        self.step_service = self.create_service(
            Step,
            'mujoco_step',
            self.handle_step_request
        )
        
        self.get_logger().info("="*60)
        self.get_logger().info("MuJoCo Simulator Service Node Started")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Simulation frequency: {self.sim_freq_hz} Hz")
        self.get_logger().info(f"Timestep: {self.dt*1000:.3f} ms")
        self.get_logger().info(f"Visualization: {'Enabled' if args.visualize else 'Disabled'}")
        if args.visualize:
            self.get_logger().info(f"Visualization frequency: {args.viz_freq} Hz")
        self.get_logger().info("")
        self.get_logger().info("Services:")
        self.get_logger().info("  - /mujoco_init (crisp_mujoco_sim_msgs/srv/Init)")
        self.get_logger().info("  - /mujoco_step (crisp_mujoco_sim_msgs/srv/Step)")
        self.get_logger().info("="*60)
        
        # Auto-start if model provided
        if args.auto_start and args.model:
            self.get_logger().info(f"\nAuto-starting with model: {args.model}")
            self._load_model(args.model)
            self.get_logger().info("Ready for Step service calls\n")
        else:
            self.get_logger().info("\nWaiting for Init service call...\n")
    
    def handle_init_request(self, request: Init.Request, response: Init.Response) -> Init.Response:
        """
        Handle initialization service request.
        
        Loads the MuJoCo model and initializes simulation state.
        
        Args:
            request: Init service request with model_path
            response: Init service response (modified in place)
            
        Returns:
            Init.Response with initial state
        """
        self.get_logger().info("="*60)
        self.get_logger().info("Init Service Called")
        self.get_logger().info(f"Model path: {request.model_path}")
        
        try:
            # Load model
            self._load_model(request.model_path)
            
            # Extract initial state
            response.position = self.data.qpos[:self.model.nu].tolist()
            response.velocity = self.data.qvel[:self.model.nu].tolist()
            response.effort = self.data.qfrc_actuator[:self.model.nu].tolist()
            response.success = True
            
            self.get_logger().info(f"✓ Initialization successful")
            self.get_logger().info(f"  DOF: {self.model.nu}")
            self.get_logger().info(f"  Initial position: {response.position}")
            self.get_logger().info("="*60)
            
        except Exception as e:
            self.get_logger().error(f"✗ Initialization failed: {e}")
            import traceback
            traceback.print_exc()
            response.success = False
        
        return response
    
    def handle_step_request(self, request: Step.Request, response: Step.Response) -> Step.Response:
        """
        Handle simulation step service request.
        
        Applies effort commands, steps simulation, and returns new state.
        Uses wall-clock synchronization for real-time execution.
        
        Args:
            request: Step service request with effort_commands
            response: Step service response (modified in place)
            
        Returns:
            Step.Response with updated state
        """
        if not self.initialized:
            self.get_logger().warn("Step called before initialization!")
            return response
        
        try:
            # Apply effort commands to actuators
            if len(request.effort_commands) != self.model.nu:
                # Pad with zeros for missing actuators (e.g., gripper)
                if len(request.effort_commands) < self.model.nu:
                    padded_commands = list(request.effort_commands) + [0.0] * (self.model.nu - len(request.effort_commands))
                    self.data.ctrl[:] = padded_commands
                else:
                    # Truncate if too many commands
                    self.data.ctrl[:] = request.effort_commands[:self.model.nu]
            else:
                # Perfect match - use commands directly
                self.data.ctrl[:] = request.effort_commands
            
            # Step simulation
            mujoco.mj_step(self.model, self.data)
            self.sim_time += self.dt
            
            # Real-time synchronization (wall-clock based)
            if self.start_wall_time is not None:
                target_wall_time = self.start_wall_time + self.sim_time
                while time.time() < target_wall_time:
                    time.sleep(0.0001)  # 100 microseconds
            
            # Extract state
            response.position = self.data.qpos[:self.model.nu].tolist()
            response.velocity = self.data.qvel[:self.model.nu].tolist()
            response.effort = self.data.qfrc_actuator[:self.model.nu].tolist()
            
            # Update visualization (decimated)
            if self.args.visualize and self.visualizer is not None:
                self.viz_counter += 1
                if self.viz_counter >= self.viz_decimation:
                    self.visualizer.update()
                    self.viz_counter = 0
            
        except Exception as e:
            self.get_logger().error(f"Step failed: {e}")
            import traceback
            traceback.print_exc()
        
        return response
    
    def _load_model(self, model_path: str):
        """
        Load MuJoCo model and initialize simulation.
        
        Args:
            model_path: Path to MuJoCo XML model file
        
        Raises:
            FileNotFoundError: If model file doesn't exist
            Exception: If model loading fails
        """
        # Resolve path
        model_path_obj = Path(model_path)
        if not model_path_obj.is_absolute():
            # Try relative to current directory
            if not model_path_obj.exists():
                # Try relative to crisp_controllers_demos
                alt_path = Path.home() / "crisp_controllers_demos" / model_path
                if alt_path.exists():
                    model_path_obj = alt_path
                else:
                    raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.get_logger().info(f"Loading model from: {model_path_obj}")
        
        # Load model
        self.model = mujoco.MjModel.from_xml_path(str(model_path_obj))
        self.data = mujoco.MjData(self.model)
        
        # Set initial state from keyframe (if available)
        if self.model.nkey > 0:
            self.data.qpos[:] = self.model.key_qpos[0]
            self.get_logger().info("Initial state set from keyframe 0")
        
        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)
        
        # Initialize timing
        self.sim_time = 0.0
        self.start_wall_time = time.time()
        self.initialized = True
        
        # Setup visualization
        if self.args.visualize and VISUALIZER_AVAILABLE:
            self._setup_visualization()
        
        self.get_logger().info(f"Model loaded successfully:")
        self.get_logger().info(f"  nq (positions): {self.model.nq}")
        self.get_logger().info(f"  nv (velocities): {self.model.nv}")
        self.get_logger().info(f"  nu (actuators): {self.model.nu}")
    
    def _setup_visualization(self):
        """Setup MuJoCo visualization."""
        try:
            # Camera configuration for FR3 (adjust as needed)
            camera_config = {
                'distance': 2.0,
                'elevation': -20,
                'azimuth': 135,
                'lookat': [0.3, 0.0, 0.4]
            }
            
            self.visualizer = MuJoCoVisualizer(
                self.model,
                self.data,
                camera_config
            )
            
            if self.visualizer.initialize():
                self.get_logger().info("✓ Visualization initialized")
            else:
                self.get_logger().warn("✗ Visualization initialization failed")
                self.visualizer = None
                
        except Exception as e:
            self.get_logger().warn(f"Visualization setup failed: {e}")
            self.visualizer = None
    
    def shutdown(self):
        """Shutdown the simulator and cleanup resources."""
        self.get_logger().info("\nShutting down MuJoCo simulator...")
        
        if self.visualizer is not None:
            self.visualizer.shutdown()
        
        self.get_logger().info("Shutdown complete")


def main(args=None):
    """Main entry point."""
    # Parse arguments
    parser = argparse.ArgumentParser(
        description='MuJoCo Simulator Service Node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--model',
        type=str,
        default=None,
        help='Path to MuJoCo XML model (can be relative or absolute)'
    )
    parser.add_argument(
        '--auto-start',
        action='store_true',
        help='Auto-start simulation if --model provided (otherwise waits for Init service)'
    )
    parser.add_argument(
        '--freq',
        type=float,
        default=1000.0,
        help='Simulation frequency in Hz (default: 1000.0)'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Enable MuJoCo visualization'
    )
    parser.add_argument(
        '--viz-freq',
        type=float,
        default=60.0,
        help='Visualization update frequency in Hz (default: 60.0)'
    )
    
    parsed_args = parser.parse_args()
    
    # Validate arguments
    if parsed_args.auto_start and not parsed_args.model:
        parser.error("--auto-start requires --model to be specified")
    
    if parsed_args.visualize and not VISUALIZER_AVAILABLE:
        print("Warning: Visualization requested but MuJoCoVisualizer not available")
        print("Continuing without visualization...")
        parsed_args.visualize = False
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    simulator_node = MuJoCoSimulatorService(parsed_args)
    
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(simulator_node)
    
    try:
        # Spin
        print("\n" + "="*60)
        print("MuJoCo Simulator Service Running")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        # Cleanup
        simulator_node.shutdown()
        simulator_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
