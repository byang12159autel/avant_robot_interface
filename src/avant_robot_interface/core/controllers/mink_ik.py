"""
Mink-based IK controller implementation.

This module provides an IK controller using the mink library for
inverse kinematics solving with MuJoCo models.

Requirements:
    - mink: pip install mink
    - mujoco: pip install mujoco
"""

from typing import Optional
import numpy as np

try:
    import mink
    import mujoco
    MINK_AVAILABLE = True
except ImportError:
    MINK_AVAILABLE = False

from .base import BaseIKController
from ..contracts import RobotState, TaskSpaceReference, JointCommand, RefMode


class MinkIKController(BaseIKController):
    """
    Position controller using mink IK solver.

    Receives TaskSpaceReference from Bridge and solves IK to compute
    joint positions. Handles HOLD and TRACK modes.
    
    Features:
    - End-effector position and orientation tracking
    - Posture task for nullspace control
    - Configurable convergence thresholds
    - Levenberg-Marquardt damping for singularity handling
    
    Example:
        model = mujoco.MjModel.from_xml_path('robot.xml')
        controller = MinkIKController(
            mujoco_model=model,
            control_dt=0.001,
            ee_link='ee_site',
        )
        controller.initialize_posture_target()
        
        # In control loop:
        controller.set_reference(task_space_ref)
        joint_cmd = controller.step(robot_state)
    """

    def __init__(
        self,
        mujoco_model: 'mujoco.MjModel',
        control_dt: float,
        ee_link: str,
        ik_solver: str = 'quadprog',
        pos_threshold: float = 0.001,
        ori_threshold: float = 0.01,
        max_iters: int = 10,
        ee_position_cost: float = 1.0,
        ee_orientation_cost: float = 1.0,
        ee_lm_damping: float = 1e-3,
        posture_cost: float = 1e-3,
        damping: float = 1e-4,
        n_controlled_joints: int = 7,
    ):
        """
        Initialize Mink IK controller.
        
        Args:
            mujoco_model: MuJoCo model for kinematics
            control_dt: Controller timestep for IK integration (seconds)
            ee_link: End-effector site name in MuJoCo model
            ik_solver: IK solver name ('quadprog', 'osqp', etc.)
            pos_threshold: Position convergence threshold (meters)
            ori_threshold: Orientation convergence threshold (radians)
            max_iters: Maximum IK iterations per step
            ee_position_cost: End-effector position task cost weight
            ee_orientation_cost: End-effector orientation task cost weight
            ee_lm_damping: Levenberg-Marquardt damping for EE task
            posture_cost: Posture task cost weight (nullspace control)
            damping: General IK damping factor
            n_controlled_joints: Number of joints to control (excludes gripper)
        """
        if not MINK_AVAILABLE:
            raise ImportError(
                "mink and mujoco are required for MinkIKController. "
                "Install with: pip install mink mujoco"
            )
        
        # Initialize base class
        super().__init__(
            control_dt=control_dt,
            ee_link=ee_link,
            pos_threshold=pos_threshold,
            ori_threshold=ori_threshold,
            max_iters=max_iters,
        )
        
        # Store MuJoCo model and solver settings
        self.model = mujoco_model
        self.ik_solver = ik_solver
        self.damping = damping
        self.n_controlled_joints = n_controlled_joints
        
        # Create mink configuration
        self.configuration = mink.Configuration(mujoco_model)

        # Create IK tasks
        self.end_effector_task = mink.FrameTask(
            frame_name=ee_link,
            frame_type="site",
            position_cost=ee_position_cost,
            orientation_cost=ee_orientation_cost,
            lm_damping=ee_lm_damping,
        )
        
        self.posture_task = mink.PostureTask(
            model=mujoco_model, 
            cost=np.full(mujoco_model.nq, posture_cost)
        )
        
        self.tasks = [self.end_effector_task, self.posture_task]
        
    def set_reference(self, ref: TaskSpaceReference) -> None:
        """
        Update the task-space reference from Bridge.
        
        Args:
            ref: Task-space reference containing target pose and mode
        """
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
            q_des = state.q[:self.n_controlled_joints].copy()
        else:
            # TRACK mode: solve IK for target pose
            target = self.current_ref.target
            if target is not None:
                # Convert to mink SE3
                # Note: mink expects quaternion as (w, x, y, z) but our contracts use (x, y, z, w)
                target_SE3 = mink.SE3.from_rotation_and_translation(
                    mink.SO3(np.array([
                        target.T.q[3],  # w
                        target.T.q[0],  # x
                        target.T.q[1],  # y
                        target.T.q[2],  # z
                    ])),
                    target.T.p
                )
                
                # Set IK target
                self.end_effector_task.set_target(target_SE3)
                
                # Solve IK
                self._converge_ik()
                
                # Extract joint positions (first n_controlled_joints DOF)
                q_des = self.configuration.q[:self.n_controlled_joints].copy()
            else:
                # No target in TRACK mode -> fallback to HOLD
                q_des = state.q[:self.n_controlled_joints].copy()
        
        return JointCommand(
            stamp=state.stamp,
            q_des=q_des,
            dq_des=None,
            tau_ff=None,
        )
    
    def _converge_ik(self) -> bool:
        """
        Run IK iterations until convergence or max iterations.
        
        Returns:
            True if converged, False if max iterations reached
        """
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
                return True
        
        return False
    
    def _pad_joints(self, joint_array: np.ndarray, target_size: int) -> np.ndarray:
        """
        Pad joint array to target size with zeros (for gripper DOF).
        
        Args:
            joint_array: Input joint positions
            target_size: Target array size
            
        Returns:
            Padded array of size target_size
        """
        if len(joint_array) < target_size:
            padded = np.zeros(target_size)
            padded[:len(joint_array)] = joint_array
            return padded
        return joint_array[:target_size]
    
    def initialize_posture_target(self) -> None:
        """Set posture task target to current configuration."""
        self.posture_task.set_target_from_configuration(self.configuration)
    
    def update_configuration(self, q: np.ndarray) -> None:
        """
        Update the IK solver's internal configuration state.
        
        Args:
            q: Joint positions to update configuration with
        """
        q_padded = self._pad_joints(q, self.model.nq)
        self.configuration.update(q_padded)
    
    def get_ee_error(self) -> Optional[np.ndarray]:
        """
        Get current end-effector error (position and orientation).
        
        Returns:
            6D error vector [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z] or None
        """
        if self.current_ref is None or self.current_ref.target is None:
            return None
        return self.end_effector_task.compute_error(self.configuration)
    
    def get_configuration_q(self) -> np.ndarray:
        """
        Get current IK configuration joint positions.
        
        Returns:
            Joint positions from IK solver's internal state
        """
        return self.configuration.q.copy()
