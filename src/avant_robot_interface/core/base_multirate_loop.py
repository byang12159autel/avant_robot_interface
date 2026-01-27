"""
base_multirate_loop.py
Base class for multi-rate control loops.

Provides generic infrastructure for:
- Decimation-based task scheduling
- Timing and rate limiting
- Statistics tracking
- Lifecycle management (initialization, run, cleanup)
"""

import time
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from loop_rate_limiters import RateLimiter


class BaseMultiRateControlLoop(ABC):
    """
    Abstract base class for multi-rate control loops.
    
    Handles the generic multi-rate control infrastructure, allowing
    subclasses to focus on domain-specific initialization and task logic.
    
    The main loop runs at the base (fastest) frequency, with slower tasks
    scheduled using decimation factors.
    """

    def __init__(
        self, 
        base_frequency_hz: float,
        task_frequencies_hz: Dict[str, float],
        args: Optional[Any] = None,
        config: Optional[Any] = None
    ):
        """
        Initialize multi-rate control loop.
        
        Args:
            base_frequency_hz: Base loop frequency (fastest rate)
            task_frequencies_hz: Dictionary mapping task names to frequencies
                e.g., {'planner': 20.0, 'controller': 1000.0}
            args: Optional command-line arguments
            config: Optional configuration object
        """
        self.args = args
        self.config = config
        
        # Timing configuration
        self.base_frequency_hz = base_frequency_hz
        self.base_dt = 1.0 / self.base_frequency_hz
        
        # Task configuration
        self.task_frequencies_hz = task_frequencies_hz
        self.task_decimations = {}
        self.task_counters = {}
        
        # Calculate decimation factors for each task
        for task_name, freq_hz in task_frequencies_hz.items():
            decimation = int(self.base_frequency_hz / freq_hz)
            self.task_decimations[task_name] = decimation
            self.task_counters[task_name] = 0
        
        # Loop state
        self.iteration_counter = 0
        self.t_start = None
        self.should_stop = False
        
    def _print_configuration(self):
        """Print loop configuration."""
        print("="*60)
        print("Multi-Rate Control Loop Configuration")
        print("="*60)
        print(f"Base frequency: {self.base_frequency_hz} Hz")
        for task_name, freq_hz in self.task_frequencies_hz.items():
            decimation = self.task_decimations[task_name]
            print(f"  {task_name}: {freq_hz} Hz (every {decimation} iteration(s))")
        print("="*60)
        print()
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize all control components.
        
        Subclasses must implement this to set up:
        - Robot interfaces
        - Controllers
        - Planners
        - Visualizers
        - Any other components
        
        Returns:
            True if initialization successful, False otherwise
        """
        pass
    
    @abstractmethod
    def cleanup(self):
        """
        Cleanup resources before shutdown.
        
        Subclasses must implement this to:
        - Close robot connections
        - Shutdown visualizers
        - Release resources
        """
        pass
    
    def should_continue(self, elapsed: float, duration_s: Optional[float]) -> bool:
        """
        Check if loop should continue running.
        
        Can be overridden to add custom termination conditions
        (e.g., check if visualizer window is closed).
        
        Args:
            elapsed: Time elapsed since start (seconds)
            duration_s: Target duration (None = run forever)
            
        Returns:
            True if loop should continue, False to stop
        """
        if self.should_stop:
            return False
        
        if duration_s is not None and elapsed >= duration_s:
            return False
        
        return True
    
    def execute_task(self, task_name: str, *args, **kwargs):
        """
        Execute a task if it's time based on decimation.
        
        Args:
            task_name: Name of the task (must be in task_frequencies_hz)
            *args, **kwargs: Arguments to pass to task method
            
        Returns:
            Result of task execution, or None if task not executed
        """
        if task_name not in self.task_decimations:
            raise ValueError(f"Unknown task: {task_name}")
        
        decimation = self.task_decimations[task_name]
        
        if self.iteration_counter % decimation == 0:
            # Call the task method (e.g., planner_tick, controller_tick)
            method_name = f"{task_name}_tick"
            if hasattr(self, method_name):
                result = getattr(self, method_name)(*args, **kwargs)
                self.task_counters[task_name] += 1
                return result
            else:
                raise NotImplementedError(
                    f"Task method '{method_name}' not implemented"
                )
        
        return None
    
    def on_iteration_start(self, elapsed: float):
        """
        Hook called at the start of each iteration.
        
        Can be overridden for custom per-iteration logic.
        
        Args:
            elapsed: Time elapsed since start (seconds)
        """
        pass
    
    def on_iteration_end(self, elapsed: float):
        """
        Hook called at the end of each iteration.
        
        Can be overridden for custom per-iteration logic.
        
        Args:
            elapsed: Time elapsed since start (seconds)
        """
        pass
    
    def run(self, duration_s: Optional[float] = None):
        """
        Run the multi-rate control loop.
        
        Args:
            duration_s: Duration to run in seconds (None = run until interrupted)
        """
        # Initialize
        if not self.initialize():
            print("✗ Initialization failed")
            return
        
        self._print_configuration()
        print("Starting control loop...")
        print("Press Ctrl+C to stop\n")
        
        self.t_start = time.time()
        self.iteration_counter = 0
        self.should_stop = False
        
        try:
            while True:
                loop_start = time.time()
                elapsed = loop_start - self.t_start
                
                # Check termination conditions
                if not self.should_continue(elapsed, duration_s):
                    break
                
                # Iteration start hook
                self.on_iteration_start(elapsed)
                
                # Main loop body - subclass implements task execution
                self.loop_iteration(elapsed)
                
                # Iteration end hook
                self.on_iteration_end(elapsed)
                
                # Increment counter
                self.iteration_counter += 1
                
                # Rate limiting
                loop_elapsed = time.time() - loop_start
                sleep_time = self.base_dt - loop_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")
        finally:
            elapsed = time.time() - self.t_start
            self.cleanup()
            self.print_statistics(elapsed)
    
    @abstractmethod
    def loop_iteration(self, elapsed: float):
        """
        Execute one iteration of the control loop.
        
        Subclasses must implement this to:
        - Execute tasks based on decimation using execute_task()
        - Perform any per-iteration logic
        
        Args:
            elapsed: Time elapsed since start (seconds)
        """
        pass
    
    def print_statistics(self, elapsed: float):
        """
        Print execution statistics.
        
        Can be overridden to add custom statistics.
        
        Args:
            elapsed: Total elapsed time (seconds)
        """
        print("\n" + "="*60)
        print("Execution Statistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total iterations: {self.iteration_counter}")
        print(f"  Average frequency: {self.iteration_counter/elapsed:.1f} Hz")
        print()
        
        # Print task-specific statistics
        for task_name, freq_hz in self.task_frequencies_hz.items():
            count = self.task_counters[task_name]
            expected = int(freq_hz * elapsed)
            print(f"  {task_name} calls: {count} (expected: ~{expected})")
        
        print("="*60)
    
    def stop(self):
        """Request the loop to stop at the next iteration."""
        self.should_stop = True
