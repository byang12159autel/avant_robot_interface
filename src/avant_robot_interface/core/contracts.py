from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional
import numpy as np

'''
Separation of Concerns

- `RobotState` = raw sensor data (joint positions/velocities)
- `TaskSpaceReference` = When and how to track it). A time-stamped reference that wraps CartesianTarget with mode and validity
- `CartesianTarget` = where to go (EE pose). A pure spatial goal (pose, frame, tolerances)
- `JointCommand` = what to send to hardware
- Each has a single, clear responsibility

Planner creats CartesianTarget -> Planner wraps it in TaskSpaceReference 
'''

@dataclass(frozen=True)
class TimeStamp:
    t: float  # seconds, monotonic

class Frame(Enum):
    WORLD = auto()
    BASE = auto()

class RefMode(Enum):
    HOLD = auto()
    TRACK = auto()

@dataclass(frozen=True)
class SE3:
    p: np.ndarray  # (3,)
    q: np.ndarray  # (4,) quaternion (x,y,z,w)

@dataclass(frozen=True)
class RobotState:
    """Complete robot state with joint and task space information.
    
    All fields except stamp, q, and dq are optional for backward compatibility.
    This allows existing code to continue working while enabling comprehensive
    state representation for dynamics-based and force control applications.
    """
    stamp: TimeStamp
    
    # Joint space (required)
    q: np.ndarray   # (n,) Joint positions [rad]
    dq: np.ndarray  # (n,) Joint velocities [rad/s]
    
    # Joint space (optional)
    tau: Optional[np.ndarray] = None   # (n,) Joint torques/efforts [Nm]
    qdd: Optional[np.ndarray] = None   # (n,) Joint accelerations [rad/s²]
    
    # Task space (optional) - end-effector state
    ts_pose: Optional[SE3] = None                  # End-effector pose (position + orientation)
    ts_vel: Optional[np.ndarray] = None            # (6,) Spatial velocity [wx, wy, wz, vx, vy, vz]
    ts_acc: Optional[np.ndarray] = None            # (6,) Task space acceleration
    ts_wrench: Optional[np.ndarray] = None         # (6,) Wrench [fx, fy, fz, tx, ty, tz]

@dataclass(frozen=True)
class CartesianTarget:
    link_name: str
    in_frame: Frame
    T: SE3
    v: Optional[np.ndarray] = None   # (6,) desired twist
    weight: float = 1.0
    pos_tol_m: float = 0.002
    rot_tol_rad: float = 0.02

@dataclass(frozen=True)
class TaskSpaceReference:
    stamp: TimeStamp
    horizon_s: float
    mode: RefMode
    target: Optional[CartesianTarget] = None

@dataclass(frozen=True)
class JointCommand:
    stamp: TimeStamp
    q_des: Optional[np.ndarray] = None
    dq_des: Optional[np.ndarray] = None
    tau_ff: Optional[np.ndarray] = None