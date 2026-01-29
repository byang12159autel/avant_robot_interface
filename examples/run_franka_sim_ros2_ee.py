"""Mink IK with ROS2 End-Effector Commands for FR3 control.

This demo extends run_franka_sim.py with ROS2 end-effector command support:
- MinkPlanner: generates task-space trajectory references (fallback/default)
- ROS2 EE Commands: override planner with external pose commands
- MinkIKController: solves IK to compute joint positions
- Bridge: handles rate mismatch and staleness detection
- CrispRobotInterface: wraps crisp_py for FR3 hardware/simulation

New Feature: ROS2 End-Effector Control
--------------------------------------
When ROS2 is enabled (--enable-ros2), you can publish end-effector pose
commands that will override the internal circular trajectory planner.

The control flow:
1. Check for ROS2 EE commands (topic: /ee_command)
2. If available: use ROS2 EE pose → Mink IK → joint commands
3. If not available: use internal planner → Mink IK → joint commands

Usage:
    1. Start the FR3 stack (simulation):
       cd crisp_controllers_demos
       ROBOT_IP=$ROBOT_IP FRANKA_FAKE_HARDWARE=true RMW=cyclone ROS_NETWORK_INTERFACE=$ROS_NETWORK_INTERFACE docker compose up launch_franka

    2. Run this script with ROS2 enabled:
       python examples/run_franka_sim_ros2_ee.py --visualize --enable-ros2
       
    3. Publish EE commands from another terminal:
       ros2 topic pub /ee_command geometry_msgs/PoseStamped \
         "header: {frame_id: 'world'}
          pose: {
            position: {x: 0.5, y: 0.1, z: 0.4},
            orientation: {x: 0, y: 0, z: 0, w: 1}
          }" -r 10
       
    Other options:
       # With specific show mode
       python examples/run_franka_sim_ros2_ee.py --visualize --enable-ros2 --show-mode commanded
       
       # Simulate planner dropout (fallback to ROS2 after 3s)
       python examples/run_franka_sim_ros2_ee.py --visualize --enable-ros2 --planner-alive 3.0
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

from avant_robot_interface.core.contracts import (
    TimeStamp, RobotState, SE3, CartesianTarget,
    TaskSpaceReference, RefMode, Frame, JointCommand
)
from avant_robot_interface.core.bridge import Bridge
from avant_robot_interface.core.ports import TaskPlanner, PositionController
from avant_robot_interface.core.config import load_config
from avant_robot_interface.core.simple_control_loop import BaseMultiRateControlLoop
from avant_robot_interface.visualization.mujoco.mujuco_viewer import MuJoCoVisualizer


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


class FrankaMultiRateControlLoop(BaseMultiRateControlLoop):
    """
    Multi-rate control loop for Franka robot with Mink IK.
    
    Extends BaseMultiRateControlLoop with ROS2 integration enabled.
    
    Integrates:
    - Planner task at lower frequency (e.g., 20 Hz)
    - Controller task at higher frequency (e.g., 1000 Hz)
    - Visualization updates at medium frequency (e.g., 100 Hz)
    - ROS2 pub/sub for external commands and state monitoring
    - Proper decimation-based timing via base class
    """

    def __init__(self, args, config):
        """
        Initialize all control components.
        
        Args:
            args: Command-line arguments
            config: Configuration object
        """
        # Joint names for ROS2 integration
        joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]
        
        # Initialize base class with optional inline ROS2 (no jitter!)
        super().__init__(
            base_frequency_hz=args.control_freq,
            task1_frequency_hz=args.plan_freq,       # Planner
            task2_frequency_hz=args.control_freq,    # Controller
            task3_frequency_hz=args.ros2_freq,       # Visualization + ROS2 publishing
            enable_ros2=args.enable_ros2,            # Optional ROS2 integration
            inline_ros2=True,                        # Use inline mode to eliminate jitter
            joint_names=joint_names
        )
        
        self.args = args
        self.config = config
        
        # Timing configuration
        self.plan_freq = args.plan_freq
        self.control_freq = args.control_freq
        self.control_dt = 1.0 / self.control_freq
        
        # Counters (additional to base class)
        self.planner_counter = 0
        self.controller_counter = 0
        
        # Timing
        self.t_start = None
        self.last_mode = None
        
        print("="*60)
        print("Mink IK + robot_task_interface + crisp_py Demo")
        print("="*60)
        print(f"Planner frequency: {self.plan_freq} Hz")
        print(f"Controller frequency: {self.control_freq} Hz")
        print(f"Visualization/ROS2 frequency: {args.ros2_freq} Hz")
        print(f"ROS2 enabled: {self.enable_ros2}")
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
        self.mujoco_data = None
        if args.visualize:
            print(f"Starting visualization (mode: {args.show_mode})...")
            print("  Note: Viewer updates asynchronously at display refresh rate (~60 Hz)")
            print("  Rendering happens in separate thread - no control loop blocking\n")
            
            # Create MuJoCo data structure for visualization
            self.mujoco_data = mujoco.MjData(self.model)
            self.mujoco_data.qpos[:len(q_padded)] = q_padded
            mujoco.mj_forward(self.model, self.mujoco_data)
            
            # Camera configuration for FR3
            camera_config = {
                'distance': 2.0,
                'elevation': -20,
                'azimuth': 135,
                'lookat': [0.3, 0.0, 0.4]
            }
            
            self.visualizer = MuJoCoVisualizer(self.model, self.mujoco_data, camera_config)
            if not self.visualizer.initialize():
                print("Warning: Failed to initialize visualizer, continuing without visualization")
                self.visualizer = None

    # ═══════════════════════════════════════════════════════════
    # BaseMultiRateControlLoop abstract methods
    # ═══════════════════════════════════════════════════════════
    
    def task1_fast(self):
        """
        Task 1: Planner - runs at planner frequency (e.g., 20 Hz).
        
        This task generates task-space references and checks for ROS2 EE commands.
        Priority: ROS2 EE commands > Internal planner
        """
        # ═══════════════════════════════════════════════
        # 1. Check for ROS2 end-effector commands (highest priority)
        # ═══════════════════════════════════════════════
        ros2_ref = None
        if self.enable_ros2:
            ee_cmd = self.get_ros2_ee_command(max_age_s=1.0)
            if ee_cmd is not None:
                # Convert ROS2 EE pose to TaskSpaceReference
                ros2_ref = self.create_task_space_ref_from_ros2(ee_cmd)
                if self.planner_counter % 20 == 0:  # Log every second (at 20 Hz)
                    print(f"[ROS2 EE] Using external EE command: pos={ee_cmd['position'][:2]}")
        
        # ═══════════════════════════════════════════════
        # 2. Generate internal planner reference (fallback)
        # ═══════════════════════════════════════════════
        if ros2_ref is not None:
            # Use ROS2 command
            ref = ros2_ref
        else:
            # Use internal planner as fallback
            ref = self.planner.update(self.initial_state)
        
        # ═══════════════════════════════════════════════
        # 3. Send reference to bridge
        # ═══════════════════════════════════════════════
        if ref is not None:
            self.bridge.planner_tick(ref)
        
        self.planner_counter += 1
        return ref
    
    def task2_medium(self):
        """
        Task 2: Controller - runs at controller frequency (e.g., 1000 Hz).
        
        This task reads robot state, computes control commands, and sends them.
        """
        # ═══════════════════════════════════════════════
        # 1. Get current robot state
        # ═══════════════════════════════════════════════
        state = self.robot_interface.get_state()
        
        # ═══════════════════════════════════════════════
        # 2. Execute controller via Bridge
        # ═══════════════════════════════════════════════
        cmd = self.bridge.control_tick(state)
        
        # ═══════════════════════════════════════════════
        # 3. Send command to robot
        # ═══════════════════════════════════════════════
        self.robot_interface.send_command(cmd)
        
        # ═══════════════════════════════════════════════
        # 4. Report mode transitions (periodically)
        # ═══════════════════════════════════════════════
        if self.controller_counter % 1000 == 0:  # Every second at 1000Hz
            self.report_mode_transition()
        
        self.controller_counter += 1
        
        # Store for visualization
        self.latest_state = state
        self.latest_cmd = cmd
        
        return cmd
    
    def task3_slow(self):
        """
        Task 3: Visualization and ROS2 publishing - runs at 100 Hz.
        
        This task updates visualization and publishes robot state to ROS2.
        In inline ROS2 mode, also processes ROS2 callbacks here.
        """
        # ═══════════════════════════════════════════════
        # 1. Process ROS2 callbacks (inline mode - no thread!)
        # ═══════════════════════════════════════════════
        if self.enable_ros2 and self.inline_ros2:
            self.spin_ros2_once(timeout_sec=0.0)  # Non-blocking
        
        # ═══════════════════════════════════════════════
        # 2. Update visualization
        # ═══════════════════════════════════════════════
        if hasattr(self, 'latest_state') and hasattr(self, 'latest_cmd'):
            self.update_visualization(self.latest_state, self.latest_cmd)
        
        # ═══════════════════════════════════════════════
        # 3. Publish robot state to ROS2
        # ═══════════════════════════════════════════════
        if self.enable_ros2 and hasattr(self, 'latest_state'):
            self.publish_ros2_robot_state(
                positions=self.latest_state.q[:7],
                velocities=self.latest_state.dq[:7] if len(self.latest_state.dq) >= 7 else None,
                efforts=None
            )
        
        return None
    
    # ═══════════════════════════════════════════════════════════
    # Helper methods
    # ═══════════════════════════════════════════════════════════
    
    def create_task_space_ref_from_ros2(self, ee_cmd: dict) -> TaskSpaceReference:
        """
        Convert ROS2 end-effector pose to TaskSpaceReference.
        
        Args:
            ee_cmd: Dictionary with 'position' (xyz) and 'orientation' (xyzw quaternion)
            
        Returns:
            TaskSpaceReference for use with Bridge and Mink IK
        """
        # Extract position and orientation from ROS2 command
        position = ee_cmd['position']
        orientation = ee_cmd['orientation']  # xyzw quaternion
        
        # Create SE3 transform
        target_SE3 = SE3(
            p=position,
            q=orientation,  # Already in (x,y,z,w) format
        )
        
        # Create Cartesian target
        target = CartesianTarget(
            link_name=self.config.ee_link,
            in_frame=Frame.WORLD,
            T=target_SE3,
            v=None,
            weight=1.0,
            pos_tol_m=self.config.pos_tol_m,
            rot_tol_rad=self.config.rot_tol_rad,
        )
        
        # Create TaskSpaceReference
        return TaskSpaceReference(
            stamp=TimeStamp(time.monotonic()),
            horizon_s=self.config.horizon_s,
            mode=RefMode.TRACK,
            target=target,
        )
    
    def update_visualization(self, state: RobotState, cmd: JointCommand):
        """
        Update visualization with current state (non-blocking).
        
        This updates the joint positions in the MuJoCo data and calls viewer.sync(),
        which is very fast (~microseconds). The actual rendering happens asynchronously
        in a separate thread at display refresh rate (~60 Hz).
        """
        if self.visualizer is None or self.mujoco_data is None:
            return
        
        # Decide which configuration to visualize based on show_mode
        if self.args.show_mode == 'commanded' and cmd.q_des is not None:
            # Show commanded/IK solution
            q_viz = cmd.q_des
        else:
            # Show actual robot state (default for 'actual' and 'both' modes)
            q_viz = state.q
        
        # Update MuJoCo data with joint positions
        q_padded = np.zeros(self.model.nq)
        q_padded[:len(q_viz)] = q_viz
        self.mujoco_data.qpos[:] = q_padded
        
        # Forward kinematics
        mujoco.mj_forward(self.model, self.mujoco_data)
        
        # Sync with viewer (non-blocking, ~1 microsecond)
        self.visualizer.update()

    def report_mode_transition(self):
        """Report controller mode transitions."""
        current_mode = self.controller.current_ref.mode if self.controller.current_ref else RefMode.HOLD
        if current_mode != self.last_mode:
            elapsed = time.time() - self.t_start if self.t_start else 0
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
        
        # Store latest state/cmd for visualization
        self.latest_state = None
        self.latest_cmd = None

        try:
            # Use base class run method with custom termination check
            original_duration = duration_s if duration_s else 9999999.0
            
            # Override to add visualizer check
            start_time = time.time()
            
            while True:
                loop_start = time.time()

                # Check if duration exceeded
                elapsed = loop_start - start_time
                if duration_s is not None and elapsed >= duration_s:
                    break

                # Check if visualizer is still running
                if self.visualizer is not None and not self.visualizer.is_running():
                    print("Viewer window closed")
                    break

                # ═══════════════════════════════════════════════
                # TASK 1: Planner (decimated)
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task1_decimation == 0:
                    self.task1_fast()
                    self.task1_counter += 1

                # ═══════════════════════════════════════════════
                # TASK 2: Controller (decimated)
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task2_decimation == 0:
                    self.task2_medium()
                    self.task2_counter += 1

                # ═══════════════════════════════════════════════
                # TASK 3: Visualization + ROS2 (decimated)
                # ═══════════════════════════════════════════════
                if self.iteration_counter % self.task3_decimation == 0:
                    self.task3_slow()
                    self.task3_counter += 1

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
            elapsed = time.time() - start_time
            self.cleanup()
            self.print_final_statistics(elapsed)

    def shutdown(self):
        """Shutdown resources (called by base class)."""
        # Call base class shutdown (handles ROS2 in both modes)
        super().shutdown()
        
        # Clean up Franka-specific resources
        if self.visualizer is not None:
            self.visualizer.shutdown()
        self.robot_interface.shutdown()

    def cleanup(self):
        """Cleanup resources."""
        if self.visualizer is not None:
            self.visualizer.shutdown()
        self.robot_interface.shutdown()
        
        # Base class handles ROS2 shutdown
        if self.enable_ros2:
            super().shutdown()
        
        print("Demo complete.")

    def print_final_statistics(self, elapsed: float):
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
        print(f"  Visualization/ROS2 calls: {self.task3_counter} "
              f"(expected: ~{int(self.task3_frequency * elapsed)})")
        if self.enable_ros2:
            print(f"  ROS2: Enabled - published {self.task3_counter} state messages")
        print("="*60)


def main():
    """Main entry point for Franka robot control with Mink IK."""
    _HOME_PATH = Path(__file__).parent.parent    
    _CONFIG_PATH = _HOME_PATH / "configs" / "robots" / "franka_single.yaml"
    
    # Load configuration
    config = load_config(_CONFIG_PATH)

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
    parser.add_argument(
        '--ros2-freq',
        type=float,
        default=50.0,
        help='ROS2 publishing and visualization frequency in Hz (default: 50.0)'
    )
    parser.add_argument(
        '--enable-ros2',
        action='store_true',
        help='Enable ROS2 integration for pub/sub (default: disabled to avoid timing jitter)'
    )
    args = parser.parse_args()
    
    # Create and run control loop
    control_loop = FrankaMultiRateControlLoop(args, config)
    control_loop.run(duration_s=args.duration)

if __name__ == '__main__':
    main()
