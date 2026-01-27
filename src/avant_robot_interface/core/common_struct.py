"""
Common data structures for robot simulation and control.
"""

from dataclasses import dataclass
import numpy as np
from typing import Dict, Any


@dataclass
class RobotState:
    """Robot state containing joint space and task space quantities."""
    # Joint space
    q: np.ndarray       # Joint positions
    qd: np.ndarray      # Joint velocities
    tau: np.ndarray     # Joint torques

    # Task space (ee)
    ts_pose_pos: np.ndarray   # [x, y, z] – position
    ts_pose_rot: np.ndarray   # 3×3 rotation matrix
    ts_vel: np.ndarray        # [wx, wy, wz, vx, vy, vz]
    ts_acc: np.ndarray        # Task space acceleration
    ts_wre: np.ndarray        # Task space wrench

    qdd: np.ndarray = None # Optional joint acceleration (defaults to zeros)

    def __post_init__(self):
        if self.qdd is None:
            self.qdd = np.zeros_like(self.q)
