from __future__ import annotations
from typing import Protocol, Tuple, Optional
import numpy as np
from .contracts import RobotState, TaskSpaceReference, JointCommand

class TaskPlanner(Protocol):
    """Protocol for task-space trajectory planners."""
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]: ...

class PositionController(Protocol):
    """Protocol for position controllers that compute joint commands."""
    def set_reference(self, ref: TaskSpaceReference) -> None: ...
    def step(self, state: RobotState) -> JointCommand: ...

class RobotInterface(Protocol):
    """
    Protocol for robot interface wrappers.
    
    Standardizes how robot interfaces should interact with the control system,
    regardless of the underlying hardware/simulation framework (crisp_py, ROS, etc.).
    """
    def get_state(self) -> RobotState:
        """
        Get current robot state.
        
        Returns:
            RobotState with current joint positions, velocities, etc.
        """
        ...
    
    def send_command(self, cmd: JointCommand) -> None:
        """
        Send joint command to robot.
        
        Args:
            cmd: Joint command with desired positions/velocities/torques
        """
        ...
    
    def get_initial_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get initial end-effector pose for trajectory planning.
        
        Returns:
            Tuple of (position, quaternion) where:
                - position: 3D position vector [x, y, z]
                - quaternion: orientation as [x, y, z, w]
        """
        ...
    
    def shutdown(self) -> None:
        """Shutdown robot connection and cleanup resources."""
        ...
