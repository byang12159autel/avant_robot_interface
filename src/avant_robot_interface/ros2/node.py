"""
node.py
Main ROS2 Node wrapper.

Manages ROS2 node lifecycle in a separate thread.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .handlers import JointCommandHandler, RobotStatePublisher
from .subscribers import create_subscribers
from .publishers import create_publishers


class ROS2Node:
    """
    ROS2 Node wrapper that runs in a separate thread.
    
    Architecture:
    - Main thread: Control loop (1000 Hz)
    - ROS2 thread: rclpy.spin() for subscribers/publishers
    - Thread-safe handlers: Exchange data between threads
    """
    
    def __init__(self, node_name: str = 'robot_control_node', config: dict = None):
        """
        Args:
            node_name: Name of the ROS2 node
            config: Optional configuration dictionary
        """
        self.node_name = node_name
        self.config = config or {}
        
        # Thread management
        self._thread = None
        self._executor = None
        self._shutdown_requested = False
        
        # ROS2 components (created in thread)
        self._node = None
        self._subscribers = {}
        self._publishers = {}
        
        # Thread-safe handlers
        self.handlers = {
            'joint_cmd': JointCommandHandler(),
            'robot_state': RobotStatePublisher(),
        }
        
        print(f"✓ ROS2Node created: {node_name}")
    
    def start(self) -> bool:
        """
        Start the ROS2 node in a separate thread.
        
        Returns:
            True if started successfully
        """
        if self._thread is not None and self._thread.is_alive():
            print("⚠️ ROS2 node already running")
            return False
        
        # Initialize rclpy if not already done
        try:
            if not rclpy.ok():
                rclpy.init()
        except Exception as e:
            print(f"✗ Failed to initialize rclpy: {e}")
            return False
        
        # Start ROS2 thread
        self._shutdown_requested = False
        self._thread = threading.Thread(target=self._run_node, daemon=True)
        self._thread.start()
        
        print(f"✓ ROS2 node started in separate thread")
        return True
    
    def _run_node(self):
        """Run ROS2 node (executed in separate thread)."""
        try:
            # Create node
            self._node = Node(self.node_name)
            self._node.get_logger().info(f"Node '{self.node_name}' initialized")
            
            # Create subscribers
            self._subscribers = create_subscribers(self._node, self.handlers)
            
            # Create publishers
            self._publishers = create_publishers(self._node, self.handlers, self.config)
            
            # Create timer for publishing (optional - can also publish from control loop)
            self._create_publish_timer()
            
            # Create executor and spin
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            
            self._node.get_logger().info("Spinning node...")
            
            # Spin until shutdown requested
            while not self._shutdown_requested and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
            
        except Exception as e:
            print(f"✗ Error in ROS2 node: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._cleanup()
    
    def _create_publish_timer(self):
        """Create timer to publish robot state periodically."""
        # Publish at 100 Hz (adjust as needed)
        publish_period = 0.01  # 10ms = 100Hz
        
        def timer_callback():
            # Publish robot state if new data available
            if 'robot_state' in self._publishers:
                self._publishers['robot_state'].publish()
        
        self._publish_timer = self._node.create_timer(publish_period, timer_callback)
    
    def publish_now(self):
        """
        Manually trigger publishing (called from control loop).
        Thread-safe.
        """
        if 'robot_state' in self._publishers:
            self._publishers['robot_state'].publish()
    
    def shutdown(self):
        """Shutdown the ROS2 node and thread."""
        if self._shutdown_requested:
            return
        
        print("\n📦 Shutting down ROS2 node...")
        self._shutdown_requested = True
        
        # Wait for thread to finish
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                print("⚠️ ROS2 thread did not terminate cleanly")
        
        print("✓ ROS2 node shutdown complete")
    
    def _cleanup(self):
        """Clean up ROS2 resources."""
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    def is_running(self) -> bool:
        """Check if ROS2 node is running."""
        return self._thread is not None and self._thread.is_alive()
    
    # ═══════════════════════════════════════════════════════════
    # Convenience methods for control loop
    # ═══════════════════════════════════════════════════════════
    
    def get_joint_command(self, max_age_s: float = 1.0):
        """
        Get latest joint command from ROS2.
        Called from control loop.
        
        Returns:
            Joint positions as numpy array, or None
        """
        return self.handlers['joint_cmd'].get_joint_positions(max_age_s)
    
    def publish_robot_state(self, positions, velocities=None, efforts=None):
        """
        Publish robot state to ROS2.
        Called from control loop.
        
        Args:
            positions: Joint positions
            velocities: Joint velocities (optional)
            efforts: Joint efforts/torques (optional)
        """
        self.handlers['robot_state'].update_state(positions, velocities, efforts)


# ═══════════════════════════════════════════════════════════
# Utility function for easy initialization
# ═══════════════════════════════════════════════════════════

def create_ros2_node(node_name: str = 'robot_control_node', 
                     joint_names: list = None) -> ROS2Node:
    """
    Create and start a ROS2 node.
    
    Args:
        node_name: Name of the ROS2 node
        joint_names: List of joint names for publishing
    
    Returns:
        ROS2Node instance (already started)
    """
    config = {'joint_names': joint_names} if joint_names else {}
    node = ROS2Node(node_name=node_name, config=config)
    node.start()
    return node
