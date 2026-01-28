"""
publishers.py
ROS2 publisher definitions.

Creates publishers and publishes data from thread-safe handlers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time

from .handlers import RobotStatePublisher


class JointStatePublisherNode:
    """
    Publishes robot joint state to ROS2.
    
    Reads from thread-safe handler and publishes JointState messages.
    """
    
    def __init__(self, node: Node, handler: RobotStatePublisher,
                 topic: str = '/joint_states',
                 joint_names: list = None):
        """
        Args:
            node: ROS2 node
            handler: Thread-safe handler containing robot state
            topic: Topic name to publish to
            joint_names: List of joint names (optional)
        """
        self.node = node
        self.handler = handler
        self.topic = topic
        self.joint_names = joint_names or []
        
        # Create publisher
        self.publisher = node.create_publisher(
            JointState,
            topic,
            10  # QoS history depth
        )
        
        node.get_logger().info(f"Created publisher: {topic}")
    
    def publish(self) -> bool:
        """
        Publish latest state if available.
        
        Returns:
            True if published, False if no new data
        """
        # Check if new data is available
        if not self.handler.has_new_data():
            return False
        
        # Get state from handler
        state = self.handler.get_state()
        if state is None:
            return False
        
        positions, velocities, efforts, timestamp = state
        
        # Create JointState message
        msg = JointState()
        
        # Set timestamp
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Set joint names
        if self.joint_names:
            msg.name = self.joint_names
        else:
            # Generate default names
            msg.name = [f'joint_{i}' for i in range(len(positions))]
        
        # Set positions
        if positions is not None:
            msg.position = positions.tolist()
        
        # Set velocities
        if velocities is not None:
            msg.velocity = velocities.tolist()
        else:
            msg.velocity = []
        
        # Set efforts
        if efforts is not None:
            msg.effort = efforts.tolist()
        else:
            msg.effort = []
        
        # Publish
        self.publisher.publish(msg)
        return True


def create_publishers(node: Node, handlers: dict, config: dict = None) -> dict:
    """
    Factory function to create all publishers.
    
    Args:
        node: ROS2 node
        handlers: Dictionary of handlers (e.g., {'robot_state': RobotStatePublisher()})
        config: Optional configuration dictionary
    
    Returns:
        Dictionary of created publishers
    """
    publishers = {}
    config = config or {}
    
    # Create joint state publisher if handler exists
    if 'robot_state' in handlers:
        joint_names = config.get('joint_names', None)
        publishers['robot_state'] = JointStatePublisherNode(
            node=node,
            handler=handlers['robot_state'],
            topic='/joint_states',
            joint_names=joint_names
        )
    
    # Add more publishers here as needed
    # Example:
    # if 'end_effector_pose' in handlers:
    #     publishers['ee_pose'] = EndEffectorPosePublisher(
    #         node=node,
    #         handler=handlers['end_effector_pose'],
    #         topic='/end_effector_pose'
    #     )
    
    return publishers
