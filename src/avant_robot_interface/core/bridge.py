from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Callable
from .contracts import TaskSpaceReference, RefMode, RobotState
from .ports import PositionController

@dataclass
class Bridge:
    controller: PositionController
    now: Callable[[], float]

    _ref: Optional[TaskSpaceReference] = None

    def planner_tick(self, ref: TaskSpaceReference) -> None:
        self._ref = ref
        self.controller.set_reference(ref)

    def control_tick(self, state: RobotState):
        if self._ref is None:
            # no planner ref yet -> HOLD
            self.controller.set_reference(TaskSpaceReference(
                stamp=state.stamp, horizon_s=0.0, mode=RefMode.HOLD, target=None
            ))
        else:
            age = self.now() - self._ref.stamp.t
            if age > self._ref.horizon_s:
                # stale -> HOLD
                self.controller.set_reference(TaskSpaceReference(
                    stamp=state.stamp, horizon_s=0.0, mode=RefMode.HOLD, target=None
                ))
                self._ref = None
        return self.controller.step(state)