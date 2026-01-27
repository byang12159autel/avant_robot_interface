"""Mink IK with robot_task_interface and crisp_py for FR3 control.

This demo integrates mink's IK solver with the robot_task_interface architecture:
- MinkPlanner: generates task-space trajectory references
- MinkIKController: solves IK to compute joint positions
- Bridge: handles rate mismatch and staleness detection
- CrispRobotInterface: wraps crisp_py for FR3 hardware/simulation

The architecture demonstrates clean separation between:
- High-level task-space planning (what to do)
- Low-level position control (how to do it)
- Safe behavior under rate mismatch and planner dropouts

Usage:
    1. Start the FR3 stack (simulation):
       cd crisp_controllers_demos
       ROBOT_IP=$ROBOT_IP FRANKA_FAKE_HARDWARE=true RMW=cyclone ROS_NETWORK_INTERFACE=$ROS_NETWORK_INTERFACE docker compose up launch_franka

    2. Run this script:
       cd avant_task_interface
       
       # With visualization showing actual robot
       python examples/demo_mink_fr3_integrated.py --visualize --show-mode actual
       
       # With visualization showing commanded (IK solution)
       python examples/demo_mink_fr3_integrated.py --visualize --show-mode commanded
       
       # With visualization showing both + error metrics
       python examples/demo_mink_fr3_integrated.py --visualize --show-mode both
       
       # No visualization (control only)
       python examples/demo_mink_fr3_integrated.py
       
       # Simulate planner dropout after 3 seconds
       python examples/demo_mink_fr3_integrated.py --visualize --planner-alive 3.0
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation

import mink

from crisp_py.robot import make_robot
from crisp_py.utils import MuJoCoVisualizer

from avant_robot_interface.core.contracts import (
    TimeStamp, RobotState, SE3, CartesianTarget,
    TaskSpaceReference, RefMode, Frame, JointCommand
)
from avant_robot_interface.core.bridge import Bridge
from avant_robot_interface.core.ports import TaskPlanner, PositionController
from avant_robot_interface.core.config import load_config


class MinkPlanner:
    """
    Task-space trajectory planner using circular trajectory.

    Generates CartesianTarget references at planner rate (~20 Hz).
    Simulates planner dropout after alive_s seconds if specified.
    """

    def __init__(
        self,
        initial_position: np.ndarray,
        initial_orientation: np.ndarray,
        horizon_s: float,
        ee_link: str,
        pos_tol_m: float,
        rot_tol_rad: float,
        amplitude: float = 0.10,
        frequency: float = 0.2,
        alive_s: Optional[float] = None,
    ):
        """
        Args:
            initial_position: Starting XYZ position for trajectory center
            initial_orientation: Starting quaternion (x,y,z,w) for orientation
            horizon_s: Reference validity horizon (seconds)
            ee_link: End-effector link name
            pos_tol_m: Position tolerance for Cartesian target
            rot_tol_rad: Rotation tolerance for Cartesian target
            amplitude: Radius of circular trajectory (meters)
            frequency: Frequency of circular motion (Hz)
            alive_s: Planner lifetime before dropout (None = run forever)
        """
        self.initial_position = initial_position
        self.initial_orientation = initial_orientation
        self.horizon_s = horizon_s
        self.ee_link = ee_link
        self.pos_tol_m = pos_tol_m
        self.rot_tol_rad = rot_tol_rad
        self.amplitude = amplitude
        self.frequency = frequency
        self.alive_s = alive_s
        self.t0 = time.monotonic()
        
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]:
        """Generate task-space reference at planner rate."""
        now = time.monotonic()
        elapsed = now - self.t0
        
        # Simulate planner dropout if configured
        if self.alive_s is not None and elapsed > self.alive_s:
            print(f"[PLANNER] Simulating dropout at t={elapsed:.2f}s")
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
            stamp=TimeStamp(now),
            horizon_s=self.horizon_s,
            mode=RefMode.TRACK,
            target=target,
        )


class MinkIKController:
    """
    Position controller using mink IK solver.

    Receives TaskSpaceReference from Bridge and solves IK to compute
    joint positions. Handles HOLD and TRACK modes.
    """

    def __init__(
        self,
        mujoco_model: mujoco.MjModel,
        control_dt: float,
        ee_link: str,
        ik_solver: str,
        pos_threshold: float,
        ori_threshold: float,
        max_iters: int,
        ee_position_cost: float,
        ee_orientation_cost: float,
        ee_lm_damping: float,
        posture_cost: float,
        damping: float,
    ):
        """
        Args:
            mujoco_model: MuJoCo model for kinematics
            control_dt: Controller timestep for IK integration
            ee_link: End-effector link name
            ik_solver: IK solver name
            pos_threshold: Position convergence threshold
            ori_threshold: Orientation convergence threshold
            max_iters: Maximum IK iterations
            ee_position_cost: End-effector position task cost
            ee_orientation_cost: End-effector orientation task cost
            ee_lm_damping: End-effector Levenberg-Marquardt damping
            posture_cost: Posture task cost
            damping: IK damping
        """
        self.model = mujoco_model
        self.dt = control_dt
        self.ik_solver = ik_solver
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.max_iters = max_iters
        self.damping = damping
        self.configuration = mink.Configuration(mujoco_model)

        # Create IK tasks
        self.end_effector_task = mink.FrameTask(
            frame_name=ee_link,
            frame_type="site",
            position_cost=ee_position_cost,
            orientation_cost=ee_orientation_cost,
            lm_damping=ee_lm_damping,
        )
        self.posture_task = mink.PostureTask(model=mujoco_model, cost=np.full(mujoco_model.nq, posture_cost))
        self.tasks = [self.end_effector_task, self.posture_task]

        # Current reference
        self.current_ref: Optional[TaskSpaceReference] = None
        
    def set_reference(self, ref: TaskSpaceReference) -> None:
        """Update the task-space reference from Bridge."""
        self.current_ref = ref
        
    def step(self, state: RobotState) -> JointCommand:
        """
        Compute joint command based on current reference.
        
        Args:
            state: Current robot state (joint positions, velocities)
            
        Returns:
            JointCommand with desired joint positions
        """
        # Update mink configuration from actual robot state
        q_padded = self._pad_joints(state.q, self.model.nq)
        self.configuration.update(q_padded)
        
        if self.current_ref is None or self.current_ref.mode == RefMode.HOLD:
            # HOLD mode: command current position
            q_des = state.q[:7].copy()
        else:
            # TRACK mode: solve IK for target pose
            target = self.current_ref.target
            if target is not None:
                # Convert to mink SE3
                target_SE3 = mink.SE3.from_rotation_and_translation(
                    mink.SO3(np.array([target.T.q[3], target.T.q[0], target.T.q[1], target.T.q[2]])),
                    target.T.p
                )
                
                # Set IK target
                self.end_effector_task.set_target(target_SE3)
                
                # Solve IK
                self._converge_ik()
                
                # Extract joint positions (first 7 DOF for FR3)
                q_des = self.configuration.q[:7].copy()
            else:
                # No target in TRACK mode -> fallback to HOLD
                q_des = state.q[:7].copy()
        
        return JointCommand(
            stamp=state.stamp,
            q_des=q_des,
            dq_des=None,
            tau_ff=None,
        )
    
    def _converge_ik(self):
        """Run IK iterations until convergence or max iterations."""
        for _ in range(self.max_iters):
            vel = mink.solve_ik(
                self.configuration,
                self.tasks,
                self.dt,
                self.ik_solver,
                damping=self.damping
            )
            self.configuration.integrate_inplace(vel, self.dt)

            # Check convergence
            err = self.end_effector_task.compute_error(self.configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= self.pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= self.ori_threshold

            if pos_achieved and ori_achieved:
                break
    
    def _pad_joints(self, joint_array: np.ndarray, target_size: int) -> np.ndarray:
        """Pad joint array to target size with zeros (for gripper DOF)."""
        if len(joint_array) < target_size:
            padded = np.zeros(target_size)
            padded[:len(joint_array)] = joint_array
            return padded
        return joint_array[:target_size]
    
    def initialize_posture_target(self):
        """Set posture task target to current configuration."""
        self.posture_task.set_target_from_configuration(self.configuration)


class CrispRobotInterface:
    """
    Wrapper around crisp_py robot for robot_task_interface compatibility.

    Provides get_state() and send_command() methods that use the
    interface contracts (RobotState, JointCommand).
    """

    def __init__(self, robot_name: str):
        """
        Args:
            robot_name: Robot identifier for crisp_py
        """
        print(f"Connecting to {robot_name} via crisp_py...")
        self.robot = make_robot(robot_name)
        self.robot.wait_until_ready()
        print("Robot ready!")

        # Switch to joint impedance controller
        self.robot.controller_switcher_client.switch_controller("joint_impedance_controller")
        print("Switched to joint_impedance_controller")
        
    def get_state(self) -> RobotState:
        """Get current robot state."""
        return RobotState(
            stamp=TimeStamp(time.monotonic()),
            q=self.robot.joint_values.copy(),
            dq=np.zeros_like(self.robot.joint_values),
        )
    
    def send_command(self, cmd: JointCommand) -> None:
        """Send joint command to robot."""
        if cmd.q_des is not None:
            self.robot.set_target_joint(cmd.q_des)
    
    def get_initial_ee_pose(self):
        """Get initial end-effector pose for trajectory planning."""
        pose = self.robot.end_effector_pose
        position = pose.position
        
        # Convert rotation matrix to quaternion (x,y,z,w)
        rot_matrix = pose.orientation.as_matrix()
        scipy_rot = Rotation.from_matrix(rot_matrix)
        quaternion = scipy_rot.as_quat()  # Returns [x, y, z, w]
        
        return position, quaternion
    
    def shutdown(self):
        """Shutdown robot connection."""
        print("Shutting down robot...")
        self.robot.shutdown()
        print("Robot shutdown complete.")


class FrankaMultiRateControlLoop:
    """
    Multi-rate control loop for Franka robot with Mink IK.
    
    Integrates:
    - Planner task at lower frequency (e.g., 20 Hz)
    - Controller task at higher frequency (e.g., 1000 Hz)
    - Visualization updates at controller frequency
    - Proper decimation-based timing
    """

    def __init__(self, args, config):
        """
        Initialize all control components.
        
        Args:
            args: Command-line arguments
            config: Configuration object
        """
        self.args = args
        self.config = config
        
        # Timing configuration
        self.plan_freq = args.plan_freq
        self.control_freq = args.control_freq
        self.base_frequency_hz = self.control_freq  # Base rate = controller rate
        self.base_dt = 1.0 / self.base_frequency_hz
        self.control_dt = 1.0 / self.control_freq
        
        # Calculate decimation factors
        self.planner_decimation = int(self.control_freq / self.plan_freq)
        
        # Counters
        self.iteration_counter = 0
        self.planner_counter = 0
        self.controller_counter = 0
        
        # Timing
        self.t_start = None
        self.last_mode = None
        
        print("="*60)
        print("Mink IK + robot_task_interface + crisp_py Demo")
        print("="*60)
        print(f"Planner frequency: {self.plan_freq} Hz (every {self.planner_decimation} iterations)")
        print(f"Controller frequency: {self.control_freq} Hz (every iteration)")
        if args.planner_alive:
            print(f"Planner dropout after: {args.planner_alive}s")
        print("="*60)
        print()
        
        # 1. Setup MuJoCo model for mink IK
        xml_path = Path(config.mujoco_xml_path)
        print(f"Loading MuJoCo model from: {xml_path}")
        if not xml_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found at {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        print(f"Model loaded: {self.model.nq} DOF\n")

        # 2. Setup robot interface
        self.robot_interface = CrispRobotInterface(config.robot_name)
        initial_state = self.robot_interface.get_state()
        initial_position, initial_orientation = self.robot_interface.get_initial_ee_pose()

        print(f"Initial joint configuration: {initial_state.q}")
        print(f"Initial EE position: {initial_position}")
        print()

        # 3. Create controller
        self.controller = MinkIKController(
            mujoco_model=self.model,
            control_dt=self.control_dt,
            ee_link=config.ee_link,
            ik_solver=config.ik_solver,
            pos_threshold=config.pos_threshold,
            ori_threshold=config.ori_threshold,
            max_iters=config.max_iters,
            ee_position_cost=config.ee_position_cost,
            ee_orientation_cost=config.ee_orientation_cost,
            ee_lm_damping=config.ee_lm_damping,
            posture_cost=config.posture_cost,
            damping=config.damping,
        )

        # Initialize mink configuration and posture target
        q_padded = np.zeros(self.model.nq)
        q_padded[:len(initial_state.q)] = initial_state.q
        self.controller.configuration.update(q_padded)
        self.controller.initialize_posture_target()

        # 4. Create planner
        self.planner = MinkPlanner(
            initial_position=initial_position,
            initial_orientation=initial_orientation,
            horizon_s=config.horizon_s,
            ee_link=config.ee_link,
            pos_tol_m=config.pos_tol_m,
            rot_tol_rad=config.rot_tol_rad,
            amplitude=0.10,  # 10cm radius
            frequency=0.2,   # 5 second period
            alive_s=args.planner_alive,
        )
        
        # Store initial state for planner
        self.initial_state = initial_state
        
        # 5. Create bridge
        self.bridge = Bridge(controller=self.controller, now=time.monotonic)
        
        # 6. Setup visualization (optional)
        self.visualizer = None
        if args.visualize:
            print(f"Starting visualization (mode: {args.show_mode})...\n")
            self.visualizer = MuJoCoVisualizer(self.model, q_padded, mode=args.show_mode)
            self.visualizer.set_control_frequency(self.control_freq)
            self.visualizer.start()

    def planner_tick(self):
        """Execute planner task - runs at planner frequency."""
        ref = self.planner.update(self.initial_state)  # state not used by this planner
        if ref is not None:
            self.bridge.planner_tick(ref)
        self.planner_counter += 1

    def controller_tick(self, state: RobotState) -> JointCommand:
        """Execute controller task - runs at controller frequency."""
        # Controller tick via Bridge
        cmd = self.bridge.control_tick(state)
        
        # Send command to robot
        self.robot_interface.send_command(cmd)
        
        self.controller_counter += 1
        return cmd

    def update_visualization(self, state: RobotState, cmd: JointCommand):
        """Update visualization with current state and command."""
        if self.visualizer is None:
            return
            
        # Update commanded joints
        if cmd.q_des is not None:
            q_cmd_padded = np.zeros(self.model.nq)
            q_cmd_padded[:len(cmd.q_des)] = cmd.q_des
            self.visualizer.update_commanded(q_cmd_padded)
        
        # Update actual joints
        q_actual_padded = np.zeros(self.model.nq)
        q_actual_padded[:len(state.q)] = state.q
        self.visualizer.update_actual(q_actual_padded)

        # Compute tracking errors for 'both' mode
        if self.args.show_mode == 'both' and cmd.q_des is not None:
            joint_error = np.linalg.norm(cmd.q_des - state.q)

            # Compute task-space error
            if self.controller.current_ref and self.controller.current_ref.mode == RefMode.TRACK:
                ee_err = self.controller.end_effector_task.compute_error(self.controller.configuration)
                pos_error = np.linalg.norm(ee_err[:3])
                ori_error = np.linalg.norm(ee_err[3:])
            else:
                pos_error = 0.0
                ori_error = 0.0

            self.visualizer.set_error_metrics(joint_error, pos_error, ori_error)

    def report_mode_transition(self, elapsed: float):
        """Report controller mode transitions."""
        current_mode = self.controller.current_ref.mode if self.controller.current_ref else RefMode.HOLD
        if current_mode != self.last_mode:
            mode_info = "HOLD (stale/no ref)" if current_mode == RefMode.HOLD else "TRACK"
            print(f"[t={elapsed:5.2f}s] Controller mode -> {mode_info}")
            self.last_mode = current_mode

    def run(self, duration_s: Optional[float] = None):
        """
        Run the multi-rate control loop.

        Args:
            duration_s: Duration to run in seconds (None = run until interrupted)
        """
        print("Starting control loop...")
        print("Press Ctrl+C or close viewer window to stop\n")
        
        self.t_start = time.time()
        self.iteration_counter = 0

        try:
            while True:
                loop_start = time.time()

                # Check if duration exceeded
                elapsed = loop_start - self.t_start
                if duration_s is not None and elapsed >= duration_s:
                    break

                # Check if visualizer is still running
                if self.visualizer is not None and not self.visualizer.is_running():
                    print("Viewer window closed")
                    break

                # ═══════════════════════════════════════════════
                # TASK 1: Planner (low frequency, e.g., 20 Hz)
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.planner_decimation == 0:
                    self.planner_tick()

                # ═══════════════════════════════════════════════
                # TASK 2: Controller (high frequency, e.g., 1000 Hz)
                # ═══════════════════════════════════════════════
                # Get current robot state
                state = self.robot_interface.get_state()
                
                # Execute controller
                cmd = self.controller_tick(state)

                # ═══════════════════════════════════════════════
                # TASK 3: Visualization (controller frequency)
                # ═══════════════════════════════════════════════
                self.update_visualization(state, cmd)

                # Report mode transitions
                self.report_mode_transition(elapsed)

                # Increment counter
                self.iteration_counter += 1

                # Sleep to maintain timing
                loop_elapsed = time.time() - loop_start
                sleep_time = self.base_dt - loop_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")
        finally:
            # Cleanup
            elapsed = time.time() - self.t_start
            self.cleanup()
            self.print_statistics(elapsed)

    def cleanup(self):
        """Cleanup resources."""
        if self.visualizer is not None:
            self.visualizer.stop()
        self.robot_interface.shutdown()
        print("Demo complete.")

    def print_statistics(self, elapsed: float):
        """Print execution statistics."""
        print("\n" + "="*60)
        print("Execution Statistics:")
        print(f"  Total time: {elapsed:.2f}s")
        print(f"  Total iterations: {self.iteration_counter}")
        print(f"  Average frequency: {self.iteration_counter/elapsed:.1f} Hz")
        print()
        print(f"  Planner calls: {self.planner_counter} "
              f"(expected: ~{int(self.plan_freq * elapsed)})")
        print(f"  Controller calls: {self.controller_counter} "
              f"(expected: ~{int(self.control_freq * elapsed)})")
        print("="*60)


def main():
    """Main entry point for Franka robot control with Mink IK."""
    # Load configuration
    config = load_config("../configs/robots/franka_single.yaml")

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Mink IK with robot_task_interface for FR3 control'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Enable MuJoCo visualization'
    )
    parser.add_argument(
        '--show-mode',
        type=str,
        choices=['commanded', 'actual', 'both'],
        default='both',
        help='Visualization mode: commanded (IK solution), actual (robot feedback), both (actual + error)'
    )
    parser.add_argument(
        '--planner-alive',
        type=float,
        default=None,
        help='Planner lifetime in seconds (simulates dropout). None = run forever'
    )
    parser.add_argument(
        '--plan-freq',
        type=float,
        default=config.planner_hz,
        help=f'Planner update frequency in Hz (default: {config.planner_hz})'
    )
    parser.add_argument(
        '--control-freq',
        type=float,
        default=config.controller_hz,
        help=f'Controller update frequency in Hz (default: {config.controller_hz})'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=None,
        help='Duration to run in seconds (default: run until interrupted)'
    )
    args = parser.parse_args()
    
    # Create and run control loop
    control_loop = FrankaMultiRateControlLoop(args, config)
    control_loop.run(duration_s=args.duration)

if __name__ == '__main__':
    main()
