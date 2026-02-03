"""
Circular trajectory planner for task-space motion.

Generates a circular trajectory in the XY plane around an initial position.
"""

from typing import Optional
import time
import numpy as np

from .base import BasePlanner
from ..contracts import (
    RobotState, TaskSpaceReference, SE3, CartesianTarget,
    TimeStamp, RefMode, Frame
)


class CircularTrajectoryPlanner(BasePlanner):
    """
    Task-space trajectory planner using circular trajectory.

    Generates CartesianTarget references that trace a circle in the XY plane
    centered on the initial end-effector position. Orientation is held constant.
    
    Optionally supports planner dropout simulation for testing controller
    behavior when the planner stops providing references.
    
    Example:
        planner = CircularTrajectoryPlanner(
            initial_position=np.array([0.5, 0.0, 0.4]),
            initial_orientation=np.array([0, 0, 0, 1]),
            ee_link='ee_site',
            amplitude=0.10,  # 10cm radius
            frequency=0.2,   # 5 second period
        )
        
        # In control loop:
        ref = planner.update(robot_state)
    """

    def __init__(
        self,
        initial_position: np.ndarray,
        initial_orientation: np.ndarray,
        ee_link: str,
        horizon_s: float = 0.1,
        pos_tol_m: float = 0.002,
        rot_tol_rad: float = 0.02,
        amplitude: float = 0.10,
        frequency: float = 0.2,
        alive_s: Optional[float] = None,
    ):
        """
        Initialize circular trajectory planner.
        
        Args:
            initial_position: Starting XYZ position for trajectory center
            initial_orientation: Starting quaternion (x,y,z,w) for orientation
            ee_link: End-effector link name
            horizon_s: Reference validity horizon (seconds)
            pos_tol_m: Position tolerance for Cartesian target
            rot_tol_rad: Rotation tolerance for Cartesian target
            amplitude: Radius of circular trajectory (meters)
            frequency: Frequency of circular motion (Hz)
            alive_s: Planner lifetime before dropout (None = run forever)
        """
        super().__init__(
            ee_link=ee_link,
            horizon_s=horizon_s,
            pos_tol_m=pos_tol_m,
            rot_tol_rad=rot_tol_rad,
        )
        
        self.initial_position = np.array(initial_position)
        self.initial_orientation = np.array(initial_orientation)
        self.amplitude = amplitude
        self.frequency = frequency
        self.alive_s = alive_s
        self._dropout_logged = False
        
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]:
        """
        Generate task-space reference at planner rate.
        
        Args:
            state: Current robot state (not used by this planner)
            
        Returns:
            TaskSpaceReference with circular trajectory position, or None after dropout
        """
        elapsed = self.get_elapsed_time()
        
        # Simulate planner dropout if configured
        if self.alive_s is not None and elapsed > self.alive_s:
            if not self._dropout_logged:
                print(f"[PLANNER] Simulating dropout at t={elapsed:.2f}s")
                self._dropout_logged = True
            return None
        
        # Compute circular trajectory offset in XY plane
        offset = np.array([
            self.amplitude * np.cos(2 * np.pi * self.frequency * elapsed),
            self.amplitude * np.sin(2 * np.pi * self.frequency * elapsed),
            0.0,
        ])
        
        # Create target pose
        target_position = self.initial_position + offset
        target_SE3 = SE3(
            p=target_position,
            q=self.initial_orientation,
        )
        
        # Create Cartesian target
        target = CartesianTarget(
            link_name=self.ee_link,
            in_frame=Frame.WORLD,
            T=target_SE3,
            v=None,
            weight=1.0,
            pos_tol_m=self.pos_tol_m,
            rot_tol_rad=self.rot_tol_rad,
        )
        
        return TaskSpaceReference(
            stamp=TimeStamp(time.monotonic()),
            horizon_s=self.horizon_s,
            mode=RefMode.TRACK,
            target=target,
        )
    
    def reset(self) -> None:
        """Reset planner state including dropout flag."""
        super().reset()
        self._dropout_logged = False
    
    def set_initial_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray
    ) -> None:
        """
        Update the initial/center position for the trajectory.
        
        Args:
            position: New center XYZ position
            orientation: New orientation quaternion (x,y,z,w)
        """
        self.initial_position = np.array(position)
        self.initial_orientation = np.array(orientation)
