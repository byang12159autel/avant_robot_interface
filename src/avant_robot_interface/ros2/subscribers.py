"""
subscribers.py
ROS2 subscriber definitions.

Creates subscribers and connects them to thread-safe handlers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose

from .handlers import JointCommandHandler, EEPoseCommandHandler


class JointCommandSubscriber:
    """
    Subscribes to joint command topics.
    
    Supports multiple message types:
    - sensor_msgs/JointState
    - std_msgs/Float64MultiArray
    """
    
    def __init__(self, node: Node, handler: JointCommandHandler, 
                 topic: str = '/joint_command', 
                 msg_type: str = 'JointState'):
        """
        Args:
            node: ROS2 node
            handler: Thread-safe handler to store received messages
            topic: Topic name to subscribe to
            msg_type: Message type ('JointState' or 'Float64MultiArray')
        """
        self.node = node
        self.handler = handler
        self.topic = topic
        
        # Create subscriber based on message type
        if msg_type == 'JointState':
            self.subscriber = node.create_subscription(
                JointState,
                topic,
                self._callback,
                10  # QoS history depth
            )
        elif msg_type == 'Float64MultiArray':
            self.subscriber = node.create_subscription(
                Float64MultiArray,
                topic,
                self._callback,
                10
            )
        else:
            raise ValueError(f"Unsupported message type: {msg_type}")
        
        node.get_logger().info(f"Created subscriber: {topic} ({msg_type})")
    
    def _callback(self, msg):
        """
        Callback for received messages.
        Stores message in thread-safe handler.
        """
        self.handler.update(msg)
        # Optional: log at low frequency
        # self.node.get_logger().info(f"Received command on {self.topic}")


class EEPoseSubscriber:
    """
    Subscribes to end-effector pose command topics.
    
    Supports multiple message types:
    - geometry_msgs/PoseStamped
    - geometry_msgs/Pose
    """
    
    def __init__(self, node: Node, handler: EEPoseCommandHandler, 
                 topic: str = '/ee_command', 
                 msg_type: str = 'PoseStamped'):
        """
        Args:
            node: ROS2 node
            handler: Thread-safe handler to store received messages
            topic: Topic name to subscribe to
            msg_type: Message type ('PoseStamped' or 'Pose')
        """
        self.node = node
        self.handler = handler
        self.topic = topic
        
        # Create subscriber based on message type
        if msg_type == 'PoseStamped':
            self.subscriber = node.create_subscription(
                PoseStamped,
                topic,
                self._callback,
                10  # QoS history depth
            )
        elif msg_type == 'Pose':
            self.subscriber = node.create_subscription(
                Pose,
                topic,
                self._callback,
                10
            )
        else:
            raise ValueError(f"Unsupported message type: {msg_type}")
        
        node.get_logger().info(f"Created EE pose subscriber: {topic} ({msg_type})")
    
    def _callback(self, msg):
        """
        Callback for received messages.
        Stores message in thread-safe handler.
        """
        self.handler.update(msg)
        # Optional: log received commands
        # self.node.get_logger().info(f"Received EE command on {self.topic}")


def create_subscribers(node: Node, handlers: dict) -> dict:
    """
    Factory function to create all subscribers.
    
    Args:
        node: ROS2 node
        handlers: Dictionary of handlers (e.g., {'joint_cmd': JointCommandHandler()})
    
    Returns:
        Dictionary of created subscribers
    """
    subscribers = {}
    
    # Create joint command subscriber if handler exists
    if 'joint_cmd' in handlers:
        subscribers['joint_cmd'] = JointCommandSubscriber(
            node=node,
            handler=handlers['joint_cmd'],
            topic='/joint_command',
            msg_type='JointState'
        )
    
    # Create end-effector pose subscriber if handler exists
    if 'ee_pose' in handlers:
        subscribers['ee_pose'] = EEPoseSubscriber(
            node=node,
            handler=handlers['ee_pose'],
            topic='/ee_command',
            msg_type='PoseStamped'
        )
    
    # Add more subscribers here as needed
    # Example:
    # if 'trajectory' in handlers:
    #     subscribers['trajectory'] = TrajectorySubscriber(
    #         node=node,
    #         handler=handlers['trajectory'],
    #         topic='/trajectory'
    #     )
    
    return subscribers
