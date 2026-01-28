"""
handlers.py
Thread-safe data handlers for ROS2 communication.

These handlers safely exchange data between:
- ROS2 callbacks (running in ROS2 thread)
- Control loop (running in main thread at 1000 Hz)
"""

import threading
import time
from typing import Optional, Any
import numpy as np


class JointCommandHandler:
    """
    Thread-safe handler for receiving joint commands from ROS2.
    
    Used by control loop to get latest command from ROS2 subscriber.
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._latest_msg: Optional[Any] = None
        self._timestamp: float = 0.0
    
    def update(self, msg: Any) -> None:
        """
        Called by ROS2 subscriber callback to store new message.
        
        Args:
            msg: ROS2 message (e.g., sensor_msgs.msg.JointState)
        """
        with self._lock:
            self._latest_msg = msg
            self._timestamp = time.time()
    
    def get_latest(self, max_age_s: float = 1.0) -> Optional[Any]:
        """
        Called by control loop to get latest message.
        
        Args:
            max_age_s: Maximum age of message to return (seconds)
            
        Returns:
            Latest message if available and not stale, None otherwise
        """
        with self._lock:
            if self._latest_msg is None:
                return None
            
            age = time.time() - self._timestamp
            if age > max_age_s:
                # Message is stale
                return None
            
            return self._latest_msg
    
    def get_joint_positions(self, max_age_s: float = 1.0) -> Optional[np.ndarray]:
        """
        Convenience method to extract joint positions as numpy array.
        
        Returns:
            Joint positions as numpy array, or None if no valid message
        """
        msg = self.get_latest(max_age_s)
        if msg is None:
            return None
        
        # Handle different message types
        if hasattr(msg, 'position'):
            return np.array(msg.position)
        elif hasattr(msg, 'data'):
            return np.array(msg.data)
        else:
            return None


class RobotStatePublisher:
    """
    Thread-safe handler for publishing robot state to ROS2.
    
    Control loop writes state here, ROS2 publisher reads and publishes.
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._joint_positions: Optional[np.ndarray] = None
        self._joint_velocities: Optional[np.ndarray] = None
        self._joint_efforts: Optional[np.ndarray] = None
        self._timestamp: float = 0.0
        self._new_data_available = False
    
    def update_state(self,
                     positions: np.ndarray,
                     velocities: Optional[np.ndarray] = None,
                     efforts: Optional[np.ndarray] = None) -> None:
        """
        Called by control loop to store robot state.
        
        Args:
            positions: Joint positions
            velocities: Joint velocities (optional)
            efforts: Joint efforts/torques (optional)
        """
        with self._lock:
            self._joint_positions = positions.copy()
            if velocities is not None:
                self._joint_velocities = velocities.copy()
            if efforts is not None:
                self._joint_efforts = efforts.copy()
            self._timestamp = time.time()
            self._new_data_available = True
    
    def get_state(self) -> Optional[tuple]:
        """
        Called by ROS2 publisher to get latest state.
        
        Returns:
            Tuple of (positions, velocities, efforts, timestamp) or None
        """
        with self._lock:
            if not self._new_data_available:
                return None
            
            self._new_data_available = False
            return (
                self._joint_positions.copy() if self._joint_positions is not None else None,
                self._joint_velocities.copy() if self._joint_velocities is not None else None,
                self._joint_efforts.copy() if self._joint_efforts is not None else None,
                self._timestamp
            )
    
    def has_new_data(self) -> bool:
        """Check if new data is available for publishing."""
        with self._lock:
            return self._new_data_available
