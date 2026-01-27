"""
simple_control_loop.py
Minimal working example of multi-rate control loop.

Demonstrates:
- Task 1: 1000 Hz (every iteration)
- Task 2: 500 Hz  (every 2 iterations)
- Task 3: 100 Hz  (every 10 iterations)
"""

import time
import numpy as np

class MultiRateControlLoop:
    """
    Simple multi-rate control loop.
    Main loop runs at fastest rate, slower tasks are decimated.
    """

    def __init__(self):
        # Timing configuration
        self.base_frequency_hz = 1000.0  # Fastest rate
        self.base_dt = 1.0 / self.base_frequency_hz  # 1ms

        # Task frequencies
        self.task1_frequency = 1000.0  # Hz
        self.task2_frequency = 500.0   # Hz
        self.task3_frequency = 100.0   # Hz

        # Calculate decimation factors
        self.task1_decimation = int(self.base_frequency_hz / self.task1_frequency)  # = 1
        self.task2_decimation = int(self.base_frequency_hz / self.task2_frequency)  # = 2
        self.task3_decimation = int(self.base_frequency_hz / self.task3_frequency)  # = 10

        # Counters
        self.iteration_counter = 0
        self.task1_counter = 0
        self.task2_counter = 0
        self.task3_counter = 0

        print(f"Control Loop Configuration:")
        print(f"  Base frequency: {self.base_frequency_hz} Hz ({self.base_dt*1000:.2f} ms)")
        print(f"  Task 1: {self.task1_frequency} Hz (every {self.task1_decimation} iteration)")
        print(f"  Task 2: {self.task2_frequency} Hz (every {self.task2_decimation} iterations)")
        print(f"  Task 3: {self.task3_frequency} Hz (every {self.task3_decimation} iterations)")
        print()

    def task1_fast(self):
        """Fast task - runs at 1000 Hz."""
        # Simulate fast control computation
        result = np.random.randn(3) * 0.1
        self.task1_counter += 1
        return result

    def task2_medium(self):
        """Medium task - runs at 500 Hz."""
        # Simulate medium-speed computation
        result = np.random.randn(6) * 0.5
        self.task2_counter += 1
        return result

    def task3_slow(self):
        """Slow task - runs at 100 Hz."""
        # Simulate slow computation (e.g., visualization update)
        result = np.random.randn(10)
        self.task3_counter += 1
        print(f"  [Slow task] Iteration {self.iteration_counter}, "
              f"Task3 calls: {self.task3_counter}")
        return result

    def run(self, duration_s: float = 5.0):
        """
        Run the multi-rate control loop.

        Args:
            duration_s: Duration to run in seconds
        """
        print(f"Starting control loop for {duration_s}s...")
        print("=" * 60)

        start_time = time.time()
        self.iteration_counter = 0

        while True:
            loop_start = time.time()

            # Check if duration exceeded
            elapsed = loop_start - start_time
            if elapsed >= duration_s:
                break

            # ═══════════════════════════════════════════════
            # TASK 1: Fast (1000 Hz) - runs EVERY iteration
            # ═══════════════════════════════════════════════
            if self.iteration_counter % self.task1_decimation == 0:
                result1 = self.task1_fast()
                # Use result1...

            # ═══════════════════════════════════════════════
            # TASK 2: Medium (500 Hz) - runs every 2 iterations
            # ═══════════════════════════════════════════════
            if self.iteration_counter % self.task2_decimation == 0:
                result2 = self.task2_medium()
                # Use result2...

            # ═══════════════════════════════════════════════
            # TASK 3: Slow (100 Hz) - runs every 10 iterations
            # ═══════════════════════════════════════════════
            if self.iteration_counter % self.task3_decimation == 0:
                result3 = self.task3_slow()
                # Use result3...

            # Increment counter
            self.iteration_counter += 1

            # Sleep to maintain timing (simple rate limiting)
            loop_elapsed = time.time() - loop_start
            sleep_time = self.base_dt - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Print statistics
        self.print_statistics(elapsed)

    def print_statistics(self, elapsed: float):
        """Print execution statistics."""
        print("=" * 60)
        print("Execution Statistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total iterations: {self.iteration_counter}")
        print(f"  Average frequency: {self.iteration_counter/elapsed:.1f} Hz")
        print()
        print(f"  Task 1 calls: {self.task1_counter} "
              f"(expected: {int(self.task1_frequency * elapsed)})")
        print(f"  Task 2 calls: {self.task2_counter} "
              f"(expected: {int(self.task2_frequency * elapsed)})")
        print(f"  Task 3 calls: {self.task3_counter} "
              f"(expected: {int(self.task3_frequency * elapsed)})")
        print("=" * 60)


if __name__ == '__main__':
    # Create and run control loop
    control_loop = MultiRateControlLoop()
    control_loop.run(duration_s=5.0)