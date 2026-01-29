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
    stamp: TimeStamp
    q: np.ndarray   # (n,)
    dq: np.ndarray  # (n,)

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