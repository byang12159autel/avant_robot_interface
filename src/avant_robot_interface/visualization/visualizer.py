"""
Abstract visualizer interface.
Control loop doesn't depend on visualization!
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Optional

class Visualizer(ABC):
    """Abstract base class for visualizers."""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize visualizer."""
        pass
    
    @abstractmethod
    def update(self, state: 'RobotState') -> None:
        """Update visualization with current state."""
        pass
    
    @abstractmethod
    def is_running(self) -> bool:
        """Check if visualizer is still active."""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """Shutdown visualizer."""
        pass
