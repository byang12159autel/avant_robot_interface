#!/usr/bin/env python3
"""
Impedance Control Demo for Franka FR3 Robot.

This example demonstrates joint-space impedance control, a fundamental control strategy
for compliant robot manipulation. Impedance control regulates the relationship between
position error and force, enabling safe interaction and smooth motion.

Control Law:
    τ = Kp(q_desired - q) - Kd(dq) + τ_gravity

Where:
    - Kp: Stiffness matrix (diagonal) - resistance to position error
    - Kd: Damping matrix (diagonal) - resistance to velocity
    - τ_gravity: Gravity compensation torque (optional)

The controller drives the robot from an initial configuration to a target configuration
while collecting trajectory data for analysis.

Features:
---------
- Pure simulation-based (no hardware required)
- Configurable stiffness/damping gains
- Real-time MuJoCo visualization
- Data logging (positions, velocities, torques)
- Optional trajectory plotting
- Support for custom target poses

Usage:
------
1. Basic demo (5 second duration):
   python examples/impedance_control_demo.py

2. With visualization (recommended):
   python examples/impedance_control_demo.py --visualize

3. Custom duration and plot results:
   python examples/impedance_control_demo.py --visualize --duration 10 --plot

4. Custom stiffness/damping gains:
   python examples/impedance_control_demo.py --visualize --stiffness 150 --damping 15

5. Custom target pose (7 joint angles in radians):
   python examples/impedance_control_demo.py --visualize --target 0 -0.785 0 -2.356 0 1.571 0.785

Theory:
-------
Impedance control is widely used in robotics for:
- Compliant manipulation tasks
- Human-robot interaction
- Contact-rich operations
- Safe trajectory tracking

Higher Kp → Stiffer behavior (faster convergence, may oscillate)
Higher Kd → More damping (smoother motion, slower convergence)

Typical relationship: Kd ≈ 2*sqrt(Kp) for critical damping
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np
import matplotlib.pyplot as plt

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from avant_robot_interface.visualization.mujoco.mujuco_viewer import MuJoCoVisualizer


class ImpedanceController:
    """
    Joint-space impedance controller for robot manipulators.
    
    Implements the control law: τ = Kp(q_d - q) - Kd(dq)
    
    This creates a virtual spring-damper system between the current and
    desired joint positions, providing compliant motion control.
    """
    
    def __init__(
        self,
        n_joints: int,
        stiffness: float = 100.0,
        damping: float = 10.0,
        gravity_comp: bool = False,
    ):
        """
        Initialize impedance controller.
        
        Args:
            n_joints: Number of controlled joints
            stiffness: Diagonal stiffness coefficient (N⋅m/rad)
            damping: Diagonal damping coefficient (N⋅m⋅s/rad)
            gravity_comp: Whether to include gravity compensation
        """
        self.n_joints = n_joints
        self.Kp = np.diag([stiffness] * n_joints)
        self.Kd = np.diag([damping] * n_joints)
        self.gravity_comp = gravity_comp
        
        print(f"Impedance Controller initialized:")
        print(f"  DOF: {n_joints}")
        print(f"  Kp (stiffness): {stiffness} N⋅m/rad")
        print(f"  Kd (damping): {damping} N⋅m⋅s/rad")
        print(f"  Gravity compensation: {gravity_comp}")
    
    def compute_torque(
        self,
        q_desired: np.ndarray,
        q_current: np.ndarray,
        dq_current: np.ndarray,
        tau_gravity: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Compute impedance control torque.
        
        Args:
            q_desired: Desired joint positions (radians)
            q_current: Current joint positions (radians)
            dq_current: Current joint velocities (rad/s)
            tau_gravity: Gravity compensation torque (optional)
        
        Returns:
            Control torque to apply (N⋅m)
        """
        # Position error
        error = q_desired - q_current
        
        # Impedance control law: spring + damper
        tau = self.Kp @ error - self.Kd @ dq_current
        
        # Add gravity compensation if enabled
        if self.gravity_comp and tau_gravity is not None:
            tau += tau_gravity
        
        return tau


class SingleJointStepTorqueTest:
    """
    Single-joint step torque test for robot manipulators.
    
    Applies a step torque to each joint individually and records the response.
    This is useful for characterizing joint dynamics, friction, and compliance.
    
    Test Protocol:
        1. Start from stable home configuration
        2. For each joint:
           a. Apply constant external torque for specified duration
           b. Record position and velocity response
           c. Allow settling time before next joint
        3. Generate comparison plots
    """
    
    def __init__(
        self,
        xml_path: str,
        ext_torque: float = 1.5,
        step_duration: float = 0.5,
        settling_time: float = 1.0,
        visualize: bool = True,
        control_freq: float = 1000.0,
    ):
        """
        Initialize single-joint step torque test.
        
        Args:
            xml_path: Path to MuJoCo XML model
            ext_torque: External torque to apply (N⋅m)
            step_duration: Duration of torque application (seconds)
            settling_time: Time to wait between joint tests (seconds)
            visualize: Enable MuJoCo visualization
            control_freq: Control loop frequency (Hz)
        """
        self.ext_torque = ext_torque
        self.step_duration = step_duration
        self.settling_time = settling_time
        self.visualize = visualize
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # Load MuJoCo model
        print(f"\nLoading MuJoCo model from: {xml_path}")
        if not Path(xml_path).exists():
            raise FileNotFoundError(f"MuJoCo model not found: {xml_path}")
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Number of joints to test (exclude gripper - last actuator)
        self.n_joints = min(7, self.model.nu)  # FR3 has 7 arm joints
        
        print(f"✓ Model loaded: {self.model.nq} DOF total")
        print(f"✓ Joints to test: {self.n_joints}")
        
        # Set initial configuration (home pose)
        self.q_home = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04])
        
        # Initialize simulation state
        self.data.qpos[:self.model.nu] = self.q_home
        mujoco.mj_forward(self.model, self.data)
        
        # Setup data storage for all joints
        self.test_results = {}
        
        # Setup visualization
        self.visualizer = None
        if visualize:
            camera_config = {
                'distance': 2.0,
                'elevation': -20,
                'azimuth': 135,
                'lookat': [0.3, 0.0, 0.4]
            }
            self.visualizer = MuJoCoVisualizer(self.model, self.data, camera_config)
            if self.visualizer.initialize():
                print("✓ MuJoCo visualizer initialized")
            else:
                print("⚠ Visualization failed to initialize")
                self.visualizer = None
        
        print(f"\nStep Torque Test Configuration:")
        print(f"  External torque: {ext_torque} N⋅m")
        print(f"  Step duration: {step_duration} s")
        print(f"  Settling time: {settling_time} s")
        print(f"  Control frequency: {control_freq} Hz")
    
    def _reset_to_home(self):
        """Reset robot to home configuration with settling."""
        # Apply PD control to return to home
        kp = 100.0
        kd = 20.0
        settle_steps = int(self.settling_time / self.dt)
        
        for _ in range(settle_steps):
            q = self.data.qpos[:self.model.nu].copy()
            dq = self.data.qvel[:self.model.nu].copy()
            
            # PD control to home
            tau = kp * (self.q_home[:self.model.nu] - q) - kd * dq
            self.data.ctrl[:] = tau
            
            mujoco.mj_step(self.model, self.data)
            
            if self.visualizer is not None:
                self.visualizer.update()
                if not self.visualizer.is_running():
                    return False
            
            time.sleep(self.dt * 0.1)  # Slow down for visualization
        
        return True
    
    def _run_single_joint_test(self, joint_idx: int) -> dict:
        """
        Run step torque test on a single joint.
        
        Args:
            joint_idx: Index of joint to test (0-based)
        
        Returns:
            Dictionary with test results for this joint
        """
        num_steps = int(self.step_duration / self.dt)
        
        # Data arrays
        time_data = np.zeros(num_steps)
        q_data = np.zeros((num_steps, self.model.nu))
        dq_data = np.zeros((num_steps, self.model.nu))
        tau_data = np.zeros((num_steps, self.model.nu))
        
        print(f"\n  Applying {self.ext_torque} N⋅m to Joint {joint_idx + 1} for {self.step_duration}s...")
        
        for step in range(num_steps):
            t = step * self.dt
            
            # Get current state
            q = self.data.qpos[:self.model.nu].copy()
            dq = self.data.qvel[:self.model.nu].copy()
            
            # Apply step torque only to the target joint
            tau = np.zeros(self.model.nu)
            tau[joint_idx] = self.ext_torque
            
            # Apply control
            self.data.ctrl[:] = tau
            
            # Step simulation
            mujoco.mj_step(self.model, self.data)
            
            # Log data
            time_data[step] = t
            q_data[step] = q
            dq_data[step] = dq
            tau_data[step] = tau
            
            # Update visualization
            if self.visualizer is not None:
                self.visualizer.update()
                if not self.visualizer.is_running():
                    break
            
            time.sleep(self.dt * 0.1)  # Slow down for visualization
        
        # Calculate response metrics
        q_initial = q_data[0, joint_idx]
        q_final = q_data[-1, joint_idx]
        delta_q = q_final - q_initial
        max_velocity = np.max(np.abs(dq_data[:, joint_idx]))
        
        print(f"    Position change: {np.degrees(delta_q):.2f}° ({delta_q:.4f} rad)")
        print(f"    Max velocity: {max_velocity:.4f} rad/s")
        
        return {
            'joint_idx': joint_idx,
            'time': time_data,
            'q': q_data,
            'dq': dq_data,
            'tau': tau_data,
            'delta_q': delta_q,
            'max_velocity': max_velocity,
        }
    
    def run(self) -> dict:
        """
        Run step torque test on all joints sequentially.
        
        Returns:
            Dictionary containing test results for all joints
        """
        print(f"\n{'='*70}")
        print(f"Single-Joint Step Torque Test")
        print(f"{'='*70}")
        print(f"Testing {self.n_joints} joints with {self.ext_torque} N⋅m step torque")
        print(f"Duration per joint: {self.step_duration}s")
        if self.visualize:
            print("Press Ctrl+C or close viewer window to stop")
        print(f"{'='*70}")
        
        try:
            for joint_idx in range(self.n_joints):
                print(f"\n[Joint {joint_idx + 1}/{self.n_joints}]")
                
                # Reset to home position
                print(f"  Idle ...")
                if not self._reset_to_home():
                    print("  Viewer closed, stopping test")
                    break
                
                # Run test on this joint
                result = self._run_single_joint_test(joint_idx)
                self.test_results[joint_idx] = result
                
                # Check if viewer is still open
                if self.visualizer is not None and not self.visualizer.is_running():
                    print("  Viewer closed, stopping test")
                    break
            
            # Final reset
            print(f"\n  Final reset to home position...")
            self._reset_to_home()
        
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
        
        finally:
            # Summary
            print(f"\n{'='*70}")
            print("Step Torque Test Summary")
            print(f"{'='*70}")
            print(f"{'Joint':<10} {'Δ Position (°)':<18} {'Δ Position (rad)':<18} {'Max Vel (rad/s)':<15}")
            print("-" * 70)
            
            for joint_idx, result in self.test_results.items():
                delta_deg = np.degrees(result['delta_q'])
                delta_rad = result['delta_q']
                max_vel = result['max_velocity']
                print(f"Joint {joint_idx + 1:<4} {delta_deg:<18.3f} {delta_rad:<18.4f} {max_vel:<15.4f}")
            
            print(f"{'='*70}\n")
            
            # Cleanup visualization
            if self.visualizer is not None:
                self.visualizer.shutdown()
        
        return self.test_results
    
    def plot_results(self, save_path: Optional[str] = None):
        """
        Plot step torque test results for all joints.
        
        Args:
            save_path: Optional path to save figure
        """
        if not self.test_results:
            print("No test results to plot")
            return
        
        n_tested = len(self.test_results)
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        fig.suptitle(f'Single-Joint Step Torque Test Results\n'
                     f'(Torque: {self.ext_torque} N⋅m, Duration: {self.step_duration}s)',
                     fontsize=14, fontweight='bold')
        
        colors = plt.cm.tab10(np.linspace(0, 1, n_tested))
        
        # Plot 1: Position change (relative to initial)
        ax1 = axes[0]
        for i, (joint_idx, result) in enumerate(self.test_results.items()):
            time = result['time']
            q = result['q'][:, joint_idx]
            q_rel = q - q[0]  # Relative to initial position
            ax1.plot(time, np.degrees(q_rel), label=f'Joint {joint_idx + 1}', 
                     color=colors[i], linewidth=2)
        ax1.set_ylabel('Position Change (°)', fontsize=12, fontweight='bold')
        ax1.set_title('Joint Position Response to Step Torque', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper left', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Plot 2: Velocity response
        ax2 = axes[1]
        for i, (joint_idx, result) in enumerate(self.test_results.items()):
            time = result['time']
            dq = result['dq'][:, joint_idx]
            ax2.plot(time, dq, label=f'Joint {joint_idx + 1}', 
                     color=colors[i], linewidth=2)
        ax2.set_ylabel('Velocity (rad/s)', fontsize=12, fontweight='bold')
        ax2.set_title('Joint Velocity Response to Step Torque', fontsize=13, fontweight='bold')
        ax2.legend(loc='upper left', fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Plot 3: Applied torque (verification)
        ax3 = axes[2]
        for i, (joint_idx, result) in enumerate(self.test_results.items()):
            time = result['time']
            tau = result['tau'][:, joint_idx]
            ax3.plot(time, tau, label=f'Joint {joint_idx + 1}', 
                     color=colors[i], linewidth=2)
        ax3.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax3.set_ylabel('Applied Torque (N⋅m)', fontsize=12, fontweight='bold')
        ax3.set_title('Applied Step Torque', fontsize=13, fontweight='bold')
        ax3.legend(loc='upper left', fontsize=10)
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Plot saved to: {save_path}")
        
        plt.show()


class ImpedanceControlDemo:
    """
    Demonstration of impedance control on Franka FR3 robot in MuJoCo.
    
    Runs a complete simulation with visualization and data logging.
    """
    
    def __init__(
        self,
        xml_path: str,
        duration: float = 5.0,
        stiffness: float = 100.0,
        damping: float = 10.0,
        target_joints: Optional[np.ndarray] = None,
        visualize: bool = True,
        control_freq: float = 1000.0,
    ):
        """
        Initialize impedance control demo.
        
        Args:
            xml_path: Path to MuJoCo XML model
            duration: Simulation duration (seconds)
            stiffness: Controller stiffness gain
            damping: Controller damping gain
            target_joints: Target joint configuration (None = use default)
            visualize: Enable MuJoCo visualization
            control_freq: Control loop frequency (Hz)
        """
        self.duration = duration
        self.visualize = visualize
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # Load MuJoCo model
        print(f"\nLoading MuJoCo model from: {xml_path}")
        if not Path(xml_path).exists():
            raise FileNotFoundError(f"MuJoCo model not found: {xml_path}")
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        print(f"✓ Model loaded: {self.model.nq} DOF total")
        print(f"✓ Controllable actuators: {self.model.nu}")
        
        # Initialize controller (control first nu DOF)
        self.controller = ImpedanceController(
            n_joints=self.model.nu,
            stiffness=stiffness,
            damping=damping,
            gravity_comp=False,  # MuJoCo handles gravity in simulation
        )
        
        # Set initial configuration (home pose)
        self.q_initial = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04])
        
        # Set target configuration
        if target_joints is not None:
            if len(target_joints) != self.model.nu:
                raise ValueError(f"Target joints must have {self.model.nu} elements")
            self.q_target = np.array(target_joints)
        else:
            # Default target: slight variation from initial
            self.q_target = self.q_initial.copy()
            self.q_target[0] += 1.0  # Move joint 1
            self.q_target[1] = -1.2 # Move joint 2
            self.q_target[3] = -2  # Move joint 4
        
        print(f"\nInitial configuration: {np.round(self.q_initial, 3)}")
        print(f"Target configuration:  {np.round(self.q_target, 3)}")
        
        # Initialize simulation state
        self.data.qpos[:self.model.nu] = self.q_initial
        mujoco.mj_forward(self.model, self.data)
        
        # Setup data logging
        self.num_steps = int(duration / self.dt)
        self.q_history = np.zeros((self.num_steps, self.model.nu))
        self.dq_history = np.zeros((self.num_steps, self.model.nu))
        self.tau_history = np.zeros((self.num_steps, self.model.nu))
        self.time_history = np.zeros(self.num_steps)
        self.step_idx = 0
        
        # Setup visualization
        self.visualizer = None
        if visualize:
            camera_config = {
                'distance': 2.0,
                'elevation': -20,
                'azimuth': 135,
                'lookat': [0.3, 0.0, 0.4]
            }
            self.visualizer = MuJoCoVisualizer(self.model, self.data, camera_config)
            if self.visualizer.initialize():
                print("✓ MuJoCo visualizer initialized")
            else:
                print("⚠ Visualization failed to initialize")
                self.visualizer = None
    
    def run(self) -> dict:
        """
        Run the impedance control simulation.
        
        Returns:
            Dictionary containing logged data
        """
        print(f"\n{'='*70}")
        print(f"Starting Impedance Control Demo")
        print(f"{'='*70}")
        print(f"Duration: {self.duration}s @ {self.control_freq}Hz")
        print(f"Total steps: {self.num_steps}")
        if self.visualize:
            print("Visualization: Enabled")
            print("Press Ctrl+C or close viewer window to stop")
        else:
            print("Visualization: Disabled")
        print(f"{'='*70}\n")
        
        t_start = time.time()
        t_last_print = t_start
        
        try:
            for step in range(self.num_steps):
                t_sim = step * self.dt
                
                # Get current state
                q = self.data.qpos[:self.model.nu].copy()
                dq = self.data.qvel[:self.model.nu].copy()
                
                # Compute control torque
                tau = self.controller.compute_torque(
                    q_desired=self.q_target,
                    q_current=q,
                    dq_current=dq,
                )
                
                # Apply control torque
                self.data.ctrl[:] = tau
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Log data
                self.q_history[step] = q
                self.dq_history[step] = dq
                self.tau_history[step] = tau
                self.time_history[step] = t_sim
                
                # Update visualization (non-blocking)
                if self.visualizer is not None:
                    self.visualizer.update()
                    
                    # Check if window closed
                    if not self.visualizer.is_running():
                        print("\nViewer window closed")
                        self.step_idx = step
                        break
                
                # Progress update every second
                t_now = time.time()
                if t_now - t_last_print >= 1.0:
                    progress = (step + 1) / self.num_steps * 100
                    elapsed = t_now - t_start
                    error_norm = np.linalg.norm(self.q_target - q)
                    print(f"Progress: {progress:5.1f}% | "
                          f"Time: {t_sim:5.2f}s | "
                          f"Error: {error_norm:.4f} rad | "
                          f"Real-time factor: {t_sim/elapsed:.2f}x")
                    t_last_print = t_now
                
                self.step_idx = step
                
                # Sleep to maintain control frequency (if faster than real-time)
                time.sleep(max(0, self.dt - (time.time() - t_now)))
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            # Final statistics
            t_elapsed = time.time() - t_start
            actual_steps = self.step_idx + 1
            
            print(f"\n{'='*70}")
            print("Simulation Complete")
            print(f"{'='*70}")
            print(f"Simulated time: {actual_steps * self.dt:.2f}s")
            print(f"Real time: {t_elapsed:.2f}s")
            print(f"Real-time factor: {(actual_steps * self.dt) / t_elapsed:.2f}x")
            print(f"Steps executed: {actual_steps}/{self.num_steps}")
            
            # Final error
            final_error = np.linalg.norm(self.q_target - self.q_history[actual_steps-1])
            print(f"Final position error: {final_error:.6f} rad")
            print(f"{'='*70}\n")
            
            # Cleanup visualization
            if self.visualizer is not None:
                self.visualizer.shutdown()
            
            # Return logged data
            return {
                'time': self.time_history[:actual_steps],
                'q': self.q_history[:actual_steps],
                'dq': self.dq_history[:actual_steps],
                'tau': self.tau_history[:actual_steps],
                'q_target': self.q_target,
                'dt': self.dt,
            }
    
    def plot_results(self, data: dict, save_path: Optional[str] = None):
        """
        Plot simulation results.
        
        Args:
            data: Dictionary of logged data from run()
            save_path: Optional path to save figure
        """
        time = data['time']
        q = data['q']
        dq = data['dq']
        tau = data['tau']
        q_target = data['q_target']
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Impedance Control - Simulation Results', fontsize=16, fontweight='bold')
        
        joint_names = [f'Joint {i+1}' for i in range(q.shape[1])]
        colors = plt.cm.tab10(np.linspace(0, 1, q.shape[1]))
        
        # Plot 1: Joint Positions
        ax1 = axes[0]
        for j in range(q.shape[1]):
            ax1.plot(time, q[:, j], label=joint_names[j], color=colors[j], linewidth=1.5)
            # Plot target as dashed line
            ax1.axhline(y=q_target[j], color=colors[j], linestyle='--', alpha=0.5, linewidth=1)
        ax1.set_ylabel('Position (rad)', fontsize=12, fontweight='bold')
        ax1.set_title('Joint Positions', fontsize=13, fontweight='bold')
        ax1.legend(ncol=4, loc='upper right', fontsize=9)
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Joint Velocities
        ax2 = axes[1]
        for j in range(dq.shape[1]):
            ax2.plot(time, dq[:, j], label=joint_names[j], color=colors[j], linewidth=1.5)
        ax2.set_ylabel('Velocity (rad/s)', fontsize=12, fontweight='bold')
        ax2.set_title('Joint Velocities', fontsize=13, fontweight='bold')
        ax2.legend(ncol=4, loc='upper right', fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Control Torques
        ax3 = axes[2]
        for j in range(tau.shape[1]):
            ax3.plot(time, tau[:, j], label=joint_names[j], color=colors[j], linewidth=1.5)
        ax3.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax3.set_ylabel('Torque (N⋅m)', fontsize=12, fontweight='bold')
        ax3.set_title('Control Torques', fontsize=13, fontweight='bold')
        ax3.legend(ncol=4, loc='upper right', fontsize=9)
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Plot saved to: {save_path}")
        
        plt.show()


def main():
    """Main entry point for impedance control demo."""
    # Get project root
    project_root = Path(__file__).parent.parent
    default_xml = project_root / "assets" / "fr3" / "scene.xml"
    
    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Impedance Control Demo for Franka FR3',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic demo with visualization
  python examples/impedance_control_demo.py --visualize
  
  # Custom gains and duration
  python examples/impedance_control_demo.py --visualize --stiffness 150 --damping 20 --duration 10
  
  # Generate plots after simulation
  python examples/impedance_control_demo.py --visualize --plot --duration 8
  
  # Custom target configuration (7 joint angles in radians)
  python examples/impedance_control_demo.py --visualize --target 0 -1.0 0 -2.5 0 1.8 0.5
        """
    )
    
    parser.add_argument(
        '--xml',
        type=str,
        default=str(default_xml),
        help=f'Path to MuJoCo XML model (default: {default_xml})'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=5.0,
        help='Simulation duration in seconds (default: 5.0)'
    )
    parser.add_argument(
        '--stiffness',
        type=float,
        default=100.0,
        help='Impedance stiffness Kp (N⋅m/rad, default: 100.0)'
    )
    parser.add_argument(
        '--damping',
        type=float,
        default=10.0,
        help='Impedance damping Kd (N⋅m⋅s/rad, default: 10.0)'
    )
    parser.add_argument(
        '--target',
        type=float,
        nargs=7,
        default=None,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'),
        help='Target joint configuration (7 values in radians)'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Enable MuJoCo visualization'
    )
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Generate plots after simulation'
    )
    parser.add_argument(
        '--save-plot',
        type=str,
        default=None,
        help='Save plot to file (e.g., results.png)'
    )
    parser.add_argument(
        '--control-freq',
        type=float,
        default=1000.0,
        help='Control loop frequency in Hz (default: 1000.0)'
    )
    
    # Step torque test arguments
    parser.add_argument(
        '--step-torque-test',
        action='store_true',
        help='Run single-joint step torque test instead of impedance demo'
    )
    parser.add_argument(
        '--ext-torque',
        type=float,
        default=1.5,
        help='External torque for step test (N⋅m, default: 1.5)'
    )
    parser.add_argument(
        '--step-duration',
        type=float,
        default=0.5,
        help='Duration of torque application per joint (seconds, default: 0.5)'
    )
    parser.add_argument(
        '--settling-time',
        type=float,
        default=1.0,
        help='Settling time between joint tests (seconds, default: 1.0)'
    )
    
    args = parser.parse_args()
    
    # Create and run demo
    try:
        if args.step_torque_test:
            # Run single-joint step torque test
            test = SingleJointStepTorqueTest(
                xml_path=args.xml,
                ext_torque=args.ext_torque,
                step_duration=args.step_duration,
                settling_time=args.settling_time,
                visualize=args.visualize,
                control_freq=args.control_freq,
            )
            
            # Run test
            test.run()
            
            # Plot results if requested
            if args.plot or args.save_plot:
                print("\nGenerating plots...")
                test.plot_results(save_path=args.save_plot)
        else:
            # Run impedance control demo
            demo = ImpedanceControlDemo(
                xml_path=args.xml,
                duration=args.duration,
                stiffness=args.stiffness,
                damping=args.damping,
                target_joints=np.array(args.target) if args.target else None,
                visualize=args.visualize,
                control_freq=args.control_freq,
            )
            
            # Run simulation
            data = demo.run()
            
            # Plot results if requested
            if args.plot or args.save_plot:
                print("\nGenerating plots...")
                demo.plot_results(data, save_path=args.save_plot)
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
