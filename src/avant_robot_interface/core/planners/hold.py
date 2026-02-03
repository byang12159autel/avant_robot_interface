"""
Hold planner that keeps robot in HOLD mode.

This planner returns None, causing the controller to maintain current position.
Useful for teleoperation scenarios where external commands drive the robot.
"""

from typing import Optional

from .base import BasePlanner
from ..contracts import RobotState, TaskSpaceReference


class HoldPlanner(BasePlanner):
    """
    Simple hold planner that keeps robot in HOLD mode.
    
    Does not generate trajectories - returns None from update() which causes
    the Bridge to put the controller in HOLD mode. This is useful for:
    - Teleoperation where external commands drive the robot
    - Waiting for external triggers before starting motion
    - Safe default behavior when no trajectory is needed
    
    Example:
        planner = HoldPlanner(ee_link='ee_site')
        
        # In control loop - robot stays at current position
        ref = planner.update(robot_state)  # Returns None
        if ref is not None:  # False, so no planner_tick
            bridge.planner_tick(ref)
    """

    def __init__(
        self,
        ee_link: str,
        horizon_s: float = 0.1,
        pos_tol_m: float = 0.002,
        rot_tol_rad: float = 0.02,
        log_on_first_call: bool = True,
    ):
        """
        Initialize hold planner.
        
        Args:
            ee_link: End-effector link name (for compatibility)
            horizon_s: Reference validity horizon (for compatibility)
            pos_tol_m: Position tolerance (for compatibility)
            rot_tol_rad: Rotation tolerance (for compatibility)
            log_on_first_call: Whether to print a message on first update call
        """
        super().__init__(
            ee_link=ee_link,
            horizon_s=horizon_s,
            pos_tol_m=pos_tol_m,
            rot_tol_rad=rot_tol_rad,
        )
        self.log_on_first_call = log_on_first_call
        self._first_call = True
        
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]:
        """
        Always returns None to keep robot in HOLD mode.
        
        The Bridge will detect no reference and keep robot at current position.
        
        Args:
            state: Current robot state (not used)
            
        Returns:
            None - causes HOLD mode in the controller
        """
        if self._first_call and self.log_on_first_call:
            print("[HOLD PLANNER] Robot in HOLD mode - waiting for external commands...")
            self._first_call = False
        return None
    
    def reset(self) -> None:
        """Reset planner state including first-call flag."""
        super().reset()
        self._first_call = True
