#!/usr/bin/env python3
"""
Keyboard Teleoperation for Robot End-Effector Control.

This script allows you to control the robot's end-effector pose (position + orientation)
using keyboard keys. It publishes geometry_msgs/PoseStamped messages to /ee_command.

Design inspired by RoboVerse teleop_utils.py and teleop_keyboard.py.

Keyboard Controls:
------------------
Movement (Translation):
  ↑     : Move +X (forward)
  ↓     : Move -X (backward)
  →     : Move -Y (right)
  ←     : Move +Y (left)
  E     : Move +Z (up)
  D     : Move -Z (down)

Rotation (Orientation):
  Q     : Roll + (rotate around X axis)
  W     : Roll - (rotate around X axis)
  A     : Pitch + (rotate around Y axis)
  S     : Pitch - (rotate around Y axis)
  Z     : Yaw + (rotate around Z axis)
  X     : Yaw - (rotate around Z axis)

Step Size Control:
  +/=   : Increase step size
  -/_   : Decrease step size

Special Commands:
  R     : Reset to initial position
  P     : Print current status
  H     : Show help
  ESC   : Quit

Features:
---------
- Multi-key combination support (e.g., ↑+→ for diagonal movement)
- 6-DOF control (3 position + 3 orientation)
- Incremental orientation changes using euler angles
- Workspace safety limits
- Real-time status display

Usage:
------
1. Start the robot control:
   python examples/run_franka_sim_ros2_ee.py --visualize --enable-ros2

2. In another terminal, run this script:
   python examples/keyboard_ee_teleop.py

3. Use keyboard to move the end-effector!
"""

import sys
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from pynput import keyboard
    HAS_PYNPUT = True
except ImportError:
    print("ERROR: pynput not installed. Install with: pip install pynput")
    HAS_PYNPUT = False
    sys.exit(1)


class KeyboardEETeleop(Node):
    """
    ROS2 node for keyboard teleoperation of end-effector with 6-DOF control.
    """

    def __init__(
        self,
        initial_position: Optional[np.ndarray] = None,
        initial_orientation: Optional[np.ndarray] = None,
        translation_step: float = 0.008,  # 8 mm per key press
        rotation_step: float = 0.03,      # ~1.7 degrees per key press
        publish_rate: float = 10.0,       # 10 Hz
    ):
        """
        Initialize the keyboard teleop node.

        Args:
            initial_position: Initial XYZ position 
            initial_orientation: Initial quaternion xyzw 
            translation_step: Translation step size in meters
            rotation_step: Rotation step size in radians
            publish_rate: Publishing rate in Hz
        """
        super().__init__('keyboard_ee_teleop')

        # Initialize pose
        self.position = initial_position if initial_position is not None else np.array([0.3, 0.0, 0.48])
        self.orientation = initial_orientation if initial_orientation is not None else np.array([1.0, 0.0, 0.0, 0.0])
        
        # Store initial for reset
        self.initial_position = self.position.copy()
        self.initial_orientation = self.orientation.copy()

        # Movement parameters
        self.translation_step = translation_step
        self.rotation_step = rotation_step
        self.min_translation_step = 0.001  # 1 mm
        self.max_translation_step = 0.05   # 5 cm
        self.min_rotation_step = 0.01      # ~0.6 degrees
        self.max_rotation_step = 0.1       # ~5.7 degrees

        # Workspace limits (safety)
        self.position_limits = {
            'x': (0.2, 0.8),
            'y': (-0.5, 0.5),
            'z': (0.1, 0.8)
        }

        # Create publisher
        self.publisher = self.create_publisher(
            PoseStamped,
            '/ee_command',
            10
        )

        # Publishing timer
        self.publish_rate = publish_rate
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pose)

        # Thread safety
        self.lock = threading.Lock()
        self.running = True

        # Key state tracking (from teleop_keyboard.py pattern)
        self.key_states = {
            # Translation
            "up": False,
            "down": False,
            "left": False,
            "right": False,
            "e": False,
            "d": False,
            # Rotation
            "q": False,
            "w": False,
            "a": False,
            "s": False,
            "z": False,
            "x": False,
            # Special
            "space": False,
        }

        # Statistics
        self.publish_count = 0
        self.last_status_print = time.time()

        self.get_logger().info('Keyboard EE Teleop Node initialized')
        self.get_logger().info(f'Initial position: {self.position}')
        self.get_logger().info(f'Initial orientation (xyzw): {self.orientation}')
        self.get_logger().info(f'Translation step: {self.translation_step*1000:.1f} mm')
        self.get_logger().info(f'Rotation step: {np.degrees(self.rotation_step):.1f} deg')
        self.get_logger().info(f'Publishing to /ee_command at {publish_rate} Hz')

    def process_keys(self):
        """
        Process current key states and compute delta position and rotation.
        Returns delta_position (xyz), delta_rotation_euler (xyz in radians).
        """
        delta_position = np.zeros(3, dtype=np.float32)
        delta_rotation = np.zeros(3, dtype=np.float32)

        with self.lock:
            # Translation controls
            if self.key_states["up"]:
                delta_position[0] += self.translation_step
            if self.key_states["down"]:
                delta_position[0] -= self.translation_step
            if self.key_states["left"]:
                delta_position[1] += self.translation_step
            if self.key_states["right"]:
                delta_position[1] -= self.translation_step
            if self.key_states["e"]:
                delta_position[2] += self.translation_step
            if self.key_states["d"]:
                delta_position[2] -= self.translation_step

            # Rotation controls (euler angles)
            if self.key_states["q"]:
                delta_rotation[0] += self.rotation_step  # Roll +
            if self.key_states["w"]:
                delta_rotation[0] -= self.rotation_step  # Roll -
            if self.key_states["a"]:
                delta_rotation[1] += self.rotation_step  # Pitch +
            if self.key_states["s"]:
                delta_rotation[1] -= self.rotation_step  # Pitch -
            if self.key_states["z"]:
                delta_rotation[2] += self.rotation_step  # Yaw +
            if self.key_states["x"]:
                delta_rotation[2] -= self.rotation_step  # Yaw -

        return delta_position, delta_rotation

    def update_pose(self):
        """Update pose based on current key states."""
        delta_pos, delta_rot = self.process_keys()

        # Check if there's any change
        has_change = np.any(np.abs(delta_pos) > 1e-9) or np.any(np.abs(delta_rot) > 1e-9)
        
        if not has_change:
            return

        with self.lock:
            # Update position
            new_position = self.position + delta_pos

            # Apply workspace limits
            new_position[0] = np.clip(new_position[0], *self.position_limits['x'])
            new_position[1] = np.clip(new_position[1], *self.position_limits['y'])
            new_position[2] = np.clip(new_position[2], *self.position_limits['z'])

            # Check if clamped
            if not np.allclose(new_position, self.position + delta_pos):
                self.get_logger().warn('Movement clamped to workspace limits')

            self.position = new_position

            # Update orientation if there's rotation change
            if np.any(np.abs(delta_rot) > 1e-9):
                # Convert current quaternion to rotation object
                current_rot = R.from_quat(self.orientation)  # xyzw format
                
                # Create delta rotation from euler angles (XYZ order)
                delta_rot_obj = R.from_euler('XYZ', delta_rot)
                
                # Apply delta rotation: new_rot = current_rot * delta_rot
                new_rot = current_rot * delta_rot_obj
                
                # Convert back to quaternion (xyzw format)
                self.orientation = new_rot.as_quat()

    def publish_pose(self):
        """Publish current end-effector pose at fixed rate."""
        # Update pose based on keys (called at publish rate)
        self.update_pose()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        with self.lock:
            # Position
            msg.pose.position.x = float(self.position[0])
            msg.pose.position.y = float(self.position[1])
            msg.pose.position.z = float(self.position[2])

            # Orientation (quaternion xyzw)
            msg.pose.orientation.x = float(self.orientation[0])
            msg.pose.orientation.y = float(self.orientation[1])
            msg.pose.orientation.z = float(self.orientation[2])
            msg.pose.orientation.w = float(self.orientation[3])

        self.publisher.publish(msg)
        self.publish_count += 1

        # Print status periodically (every 5 seconds)
        if time.time() - self.last_status_print > 5.0:
            self.print_status_inline()
            self.last_status_print = time.time()

    def reset_position(self):
        """Reset to initial position and orientation."""
        with self.lock:
            self.position = self.initial_position.copy()
            self.orientation = self.initial_orientation.copy()
        self.get_logger().info('Reset to initial pose')

    def adjust_translation_step(self, factor: float):
        """Adjust translation step size by a factor."""
        new_step = self.translation_step * factor
        new_step = np.clip(new_step, self.min_translation_step, self.max_translation_step)
        
        if new_step != self.translation_step:
            self.translation_step = new_step
            self.get_logger().info(f'Translation step: {self.translation_step*1000:.1f} mm')

    def adjust_rotation_step(self, factor: float):
        """Adjust rotation step size by a factor."""
        new_step = self.rotation_step * factor
        new_step = np.clip(new_step, self.min_rotation_step, self.max_rotation_step)
        
        if new_step != self.rotation_step:
            self.rotation_step = new_step
            self.get_logger().info(f'Rotation step: {np.degrees(self.rotation_step):.1f} deg')

    def print_status(self):
        """Print current status (detailed)."""
        with self.lock:
            pos = self.position.copy()
            quat = self.orientation.copy()
        
        # Convert quaternion to euler for display
        rot = R.from_quat(quat)
        euler = rot.as_euler('XYZ', degrees=True)
        
        print("\n" + "="*70)
        print("Keyboard EE Teleop - Status")
        print("="*70)
        print(f"Position (XYZ):    [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
        print(f"Orientation (RPY): [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}] deg")
        print(f"Quaternion (xyzw): [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
        print(f"Translation step:  {self.translation_step*1000:.1f} mm")
        print(f"Rotation step:     {np.degrees(self.rotation_step):.1f} deg")
        print(f"Messages published: {self.publish_count}")
        print("="*70)

    def print_status_inline(self):
        """Print status inline (compact)."""
        with self.lock:
            pos = self.position.copy()
        
        self.get_logger().info(
            f"Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | "
            f"Step: {self.translation_step*1000:.1f}mm | "
            f"Published: {self.publish_count}"
        )

    def print_help(self):
        """Print help information."""
        print("\n" + "="*70)
        print("Keyboard EE Teleop - Controls")
        print("="*70)
        print("Movement (Translation):")
        print("  ↑     : Move +X (forward)")
        print("  ↓     : Move -X (backward)")
        print("  →     : Move -Y (right)")
        print("  ←     : Move +Y (left)")
        print("  E     : Move +Z (up)")
        print("  D     : Move -Z (down)")
        print()
        print("Rotation (Orientation):")
        print("  Q     : Roll + (rotate around X)")
        print("  W     : Roll - (rotate around X)")
        print("  A     : Pitch + (rotate around Y)")
        print("  S     : Pitch - (rotate around Y)")
        print("  Z     : Yaw + (rotate around Z)")
        print("  X     : Yaw - (rotate around Z)")
        print()
        print("Step Size:")
        print("  +/=   : Increase translation step")
        print("  -/_   : Decrease translation step")
        print("  ]/}   : Increase rotation step")
        print("  [/{   : Decrease rotation step")
        print()
        print("Special:")
        print("  R     : Reset to initial position")
        print("  P     : Print current status")
        print("  H     : Show this help")
        print("  ESC   : Quit")
        print()
        print("Tips:")
        print("  - Hold multiple keys for combined movements (e.g., ↑+→)")
        print("  - Adjust step sizes for fine or coarse control")
        print("="*70)


class KeyboardListener:
    """Keyboard listener using pynput (background thread pattern from teleop_keyboard.py)."""

    def __init__(self, teleop_node: KeyboardEETeleop):
        self.node = teleop_node
        self.listener = None

    def on_press(self, key):
        """Handle key press events."""
        try:
            # Get key name
            if hasattr(key, 'char') and key.char:
                key_name = key.char.lower()
            else:
                key_name = str(key).replace('Key.', '').lower()

            # Update key states
            if key_name in self.node.key_states:
                self.node.key_states[key_name] = True

            # Handle special commands (single press actions)
            if key_name == 'r':
                self.node.reset_position()
            elif key_name == 'p':
                self.node.print_status()
            elif key_name == 'h':
                self.node.print_help()
            elif key_name in ['+', '=']:
                self.node.adjust_translation_step(1.5)
            elif key_name in ['-', '_']:
                self.node.adjust_translation_step(0.67)
            elif key_name in [']', '}']:
                self.node.adjust_rotation_step(1.5)
            elif key_name in ['[', '{']:
                self.node.adjust_rotation_step(0.67)
            elif key_name == 'esc':
                print("\nQuitting...")
                self.node.running = False
                return False  # Stop listener

        except Exception as e:
            self.node.get_logger().error(f"Error handling key press: {e}")

    def on_release(self, key):
        """Handle key release events."""
        try:
            # Get key name
            if hasattr(key, 'char') and key.char:
                key_name = key.char.lower()
            else:
                key_name = str(key).replace('Key.', '').lower()

            # Update key states
            if key_name in self.node.key_states:
                self.node.key_states[key_name] = False

        except Exception as e:
            self.node.get_logger().error(f"Error handling key release: {e}")

    def start(self):
        """Start listening to keyboard."""
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def stop(self):
        """Stop listening to keyboard."""
        if self.listener:
            self.listener.stop()


def main():
    """Main entry point."""
    print("="*70)
    print("Keyboard End-Effector Teleoperation (6-DOF Control)")
    print("="*70)
    print("Initializing ROS2...")

    if not HAS_PYNPUT:
        print("ERROR: pynput is required. Install with: pip install pynput")
        sys.exit(1)

    # Initialize ROS2
    rclpy.init()

    # Create teleop node
    teleop_node = KeyboardEETeleop(
        translation_step=0.008,  # 8 mm default
        rotation_step=0.03,      # ~1.7 degrees default
        publish_rate=10.0        # 10 Hz
    )

    # Print help
    teleop_node.print_help()

    # Setup keyboard listener
    keyboard_listener = KeyboardListener(teleop_node)
    keyboard_listener.start()

    print("\n✓ Keyboard listener started!")
    print("✓ Publishing EE commands to /ee_command")
    print("\nPress 'H' for help, 'ESC' to quit\n")

    try:
        # Spin ROS2 node
        while rclpy.ok() and teleop_node.running:
            rclpy.spin_once(teleop_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        keyboard_listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()
        print("\nKeyboard EE Teleop terminated.")


if __name__ == '__main__':
    main()
