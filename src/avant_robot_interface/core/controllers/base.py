"""
Abstract base class for IK-based position controllers.

This module defines the interface that all IK controllers must implement,
ensuring consistency across different IK solvers (mink, pinocchio, KDL, etc.).
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

from ..contracts import RobotState, TaskSpaceReference, JointCommand


class BaseIKController(ABC):
    """
    Abstract base class for inverse kinematics position controllers.
    
    IK controllers receive task-space references (end-effector poses) and
    compute joint-space commands to achieve those poses. They handle:
    - HOLD mode: Maintain current position
    - TRACK mode: Track a moving target
    
    This class implements the PositionController Protocol defined in ports.py.
    
    Example usage:
        controller = MinkIKController(model, dt=0.001, ee_link='ee_site')
        controller.set_reference(task_space_ref)
        joint_cmd = controller.step(robot_state)
    """
    
    def __init__(
        self,
        control_dt: float,
        ee_link: str,
        pos_threshold: float = 0.001,
        ori_threshold: float = 0.01,
        max_iters: int = 10,
    ):
        """
        Initialize base IK controller.
        
        Args:
            control_dt: Controller timestep for IK integration (seconds)
            ee_link: End-effector link/site name in the model
            pos_threshold: Position convergence threshold (meters)
            ori_threshold: Orientation convergence threshold (radians)
            max_iters: Maximum IK solver iterations per step
        """
        self.dt = control_dt
        self.ee_link = ee_link
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.max_iters = max_iters
        
        # Current reference (set by set_reference)
        self.current_ref: Optional[TaskSpaceReference] = None
    
    @abstractmethod
    def set_reference(self, ref: TaskSpaceReference) -> None:
        """
        Update the task-space reference from planner/bridge.
        
        Args:
            ref: Task-space reference containing target pose and mode
        """
        pass
    
    @abstractmethod
    def step(self, state: RobotState) -> JointCommand:
        """
        Compute joint command based on current reference and state.
        
        Args:
            state: Current robot state (joint positions, velocities)
            
        Returns:
            JointCommand with desired joint positions/velocities/torques
        """
        pass
    
    @abstractmethod
    def initialize_posture_target(self) -> None:
        """
        Set posture task target to current configuration.
        
        Called after initialization to set the nullspace posture target.
        This helps maintain a preferred arm configuration while tracking.
        """
        pass
    
    @abstractmethod
    def update_configuration(self, q: np.ndarray) -> None:
        """
        Update the IK solver's internal configuration state.
        
        Args:
            q: Joint positions to update configuration with
        """
        pass
    
    def get_current_reference(self) -> Optional[TaskSpaceReference]:
        """
        Get the current task-space reference.
        
        Returns:
            Current reference or None if not set
        """
        return self.current_ref
