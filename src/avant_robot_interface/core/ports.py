from __future__ import annotations
from typing import Protocol
from .contracts import RobotState, TaskSpaceReference, JointCommand

class TaskPlanner(Protocol):
    def update(self, state: RobotState) -> TaskSpaceReference: ...

class PositionController(Protocol):
    def set_reference(self, ref: TaskSpaceReference) -> None: ...
    def step(self, state: RobotState) -> JointCommand: ...