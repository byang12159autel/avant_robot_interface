"""
MuJoCo-specific visualizer.
Wraps MuJoCo viewer as optional component with asynchronous rendering.

The passive viewer runs in a separate thread and renders at display refresh rate (~60 Hz).
Your control loop can call update() every iteration without blocking - the viewer.sync()
call is non-blocking and just updates a pointer to the data.
"""

import mujoco
import mujoco.viewer
import numpy as np
from typing import Optional
from pathlib import Path

from avant_robot_interface.visualization.visualizer import BaseVisualizer


class MuJoCoVisualizer(BaseVisualizer):
    """
    MuJoCo visualization implementation with asynchronous rendering.
    
    Uses MuJoCo's passive viewer which runs rendering in a separate thread.
    The update() method is non-blocking and can be called at high frequency
    without impacting control loop performance.
    """
    
    def __init__(self, model, data, camera_config: Optional[dict] = None):
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            camera_config: Camera position settings (dict with 'distance', 'elevation', 'azimuth', 'lookat')
        """
        self.model = model
        self.data = data
        self.camera_config = camera_config or {}
        self.viewer = None
        self.viewer_context = None
    
    def initialize(self) -> bool:
        """
        Launch MuJoCo passive viewer.
        
        Creates a separate rendering thread that runs at display refresh rate.
        The viewer.sync() call in update() is non-blocking.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.viewer_context = mujoco.viewer.launch_passive(
                self.model, self.data
            )
            self.viewer = self.viewer_context.__enter__()
            self._setup_camera()
            print("✓ MuJoCo viewer initialized (asynchronous rendering enabled)")
            return True
        except Exception as e:
            print(f"Failed to initialize MuJoCo viewer: {e}")
            return False
    
    def update(self, state=None) -> None:
        """
        Update viewer (non-blocking).
        
        This calls viewer.sync() which is very fast (~microseconds) and does not
        wait for rendering. The actual rendering happens asynchronously in the
        viewer's separate thread at display refresh rate (~60 Hz).
        
        Safe to call every control iteration regardless of frequency.
        
        Args:
            state: Optional robot state (not used, data is already in self.data)
        """
        if self.viewer is not None:
            self.viewer.sync()
    
    def update_data(self, q: np.ndarray) -> None:
        """
        Update the visualization with new joint positions.
        
        This is useful when you want to visualize a specific configuration
        without stepping the physics simulation.
        
        Args:
            q: Joint positions (size must match model.nq)
        """
        if len(q) != self.model.nq:
            # Pad with zeros if needed (e.g., for gripper DOF)
            q_padded = np.zeros(self.model.nq)
            q_padded[:len(q)] = q
            self.data.qpos[:] = q_padded
        else:
            self.data.qpos[:] = q
        
        # Forward kinematics to update visual state
        mujoco.mj_forward(self.model, self.data)
        
        # Sync with viewer (non-blocking)
        if self.viewer is not None:
            self.viewer.sync()
    
    def is_running(self) -> bool:
        """
        Check if viewer window is open.
        
        Returns:
            True if window is open, False otherwise
        """
        if self.viewer is None:
            return False
        return self.viewer.is_running()
    
    def shutdown(self) -> None:
        """Close viewer and clean up resources."""
        if self.viewer_context is not None:
            self.viewer_context.__exit__(None, None, None)
            print("✓ MuJoCo viewer closed")
    
    def _setup_camera(self):
        """Configure camera position from camera_config."""
        if self.viewer is not None:
            self.viewer.cam.distance = self.camera_config.get('distance', 1.0)
            self.viewer.cam.elevation = self.camera_config.get('elevation', -20)
            self.viewer.cam.azimuth = self.camera_config.get('azimuth', 270)
            
            # Optional lookat point adjustment
            lookat = self.camera_config.get('lookat', None)
            if lookat is not None:
                if isinstance(lookat, (list, tuple, np.ndarray)) and len(lookat) == 3:
                    self.viewer.cam.lookat[:] = lookat
                elif isinstance(lookat, dict):
                    # Allow relative adjustments like {'x': 0.5, 'y': 0.0, 'z': 1.0}
                    self.viewer.cam.lookat[0] += lookat.get('x', 0.0)
                    self.viewer.cam.lookat[1] += lookat.get('y', 0.0)
                    self.viewer.cam.lookat[2] += lookat.get('z', 0.0)


if __name__ == "__main__":
    # export PYTHONPATH=/home/ben/avant_robot_interface/src:$PYTHONPATH
    import os
    import time

    xml_path = os.path.join(
        os.path.dirname(__file__), 
        "../../../assets/fr3/scene.xml"
    )
    xml_path = os.path.abspath(xml_path)
    
    print(f"Loading MuJoCo model from: {xml_path}")
    
    # Load the MuJoCo model from XML file
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Configure camera for better viewing
    camera_config = {
        'distance': 2.5,
        'elevation': -20,
        'azimuth': 135
    }
    
    # Create and initialize the visualizer
    visualizer = MuJoCoVisualizer(model, data, camera_config)
    
    if visualizer.initialize():
        print("MuJoCo viewer initialized successfully!")
        print("Close the viewer window to exit.")
        
        # Run simulation loop
        try:
            while visualizer.is_running():
                # Step the physics simulation
                mujoco.mj_step(model, data)
                
                # Update the viewer
                visualizer.update()
                
                # Small delay to control simulation speed
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            # Clean up
            visualizer.shutdown()
            print("Viewer closed.")
    else:
        print("Failed to initialize MuJoCo viewer!")
