"""
Abstract base class for task-space planners.

This module defines the interface that all planners must implement,
ensuring consistency across different planning strategies.
"""

from abc import ABC, abstractmethod
from typing import Optional
import time

from ..contracts import RobotState, TaskSpaceReference


class BasePlanner(ABC):
    """
    Abstract base class for task-space trajectory planners.
    
    Planners generate TaskSpaceReference objects that describe desired
    end-effector poses. They implement the TaskPlanner Protocol defined
    in ports.py.
    
    Planners can be:
    - Trajectory generators (circular, linear, spline paths)
    - Position holders (maintain current position)
    - External command adapters (ROS2, teleoperation)
    
    Example usage:
        planner = CircularTrajectoryPlanner(
            initial_position=pos,
            initial_orientation=quat,
            ee_link='ee_site',
        )
        
        # In control loop:
        ref = planner.update(robot_state)
        if ref is not None:
            bridge.planner_tick(ref)
    """
    
    def __init__(
        self,
        ee_link: str,
        horizon_s: float = 0.1,
        pos_tol_m: float = 0.002,
        rot_tol_rad: float = 0.02,
    ):
        """
        Initialize base planner.
        
        Args:
            ee_link: End-effector link/site name
            horizon_s: Reference validity horizon (seconds)
            pos_tol_m: Position tolerance for Cartesian target (meters)
            rot_tol_rad: Rotation tolerance for Cartesian target (radians)
        """
        self.ee_link = ee_link
        self.horizon_s = horizon_s
        self.pos_tol_m = pos_tol_m
        self.rot_tol_rad = rot_tol_rad
        self.t0 = time.monotonic()
    
    @abstractmethod
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]:
        """
        Generate task-space reference at planner rate.
        
        This is the main planning method. It should return a TaskSpaceReference
        describing the desired end-effector pose, or None to indicate no
        reference (which causes HOLD mode in the Bridge).
        
        Args:
            state: Current robot state (may be used for feedback planning)
            
        Returns:
            TaskSpaceReference with target pose, or None for HOLD mode
        """
        pass
    
    def reset(self) -> None:
        """
        Reset planner state.
        
        Resets the planner's internal time reference and any other state.
        Override in subclasses if additional reset logic is needed.
        """
        self.t0 = time.monotonic()
    
    def get_elapsed_time(self) -> float:
        """
        Get elapsed time since planner start/reset.
        
        Returns:
            Elapsed time in seconds
        """
        return time.monotonic() - self.t0
