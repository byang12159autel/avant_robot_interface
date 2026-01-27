"""
MuJoCo-specific visualizer.
Wraps MuJoCo viewer as optional component.
"""

import mujoco
import mujoco.viewer
from typing import Optional
from .visualizer import Visualizer

class MuJoCoVisualizer(Visualizer):
    """MuJoCo visualization implementation."""
    
    def __init__(self, model, data, camera_config: Optional[dict] = None):
        """
        Args:
            model: MuJoCo model
            data: MuJoCo data
            camera_config: Camera position settings
        """
        self.model = model
        self.data = data
        self.camera_config = camera_config or {}
        self.viewer = None
        self.viewer_context = None
    
    def initialize(self) -> bool:
        """Launch MuJoCo viewer."""
        try:
            self.viewer_context = mujoco.viewer.launch_passive(
                self.model, self.data
            )
            self.viewer = self.viewer_context.__enter__()
            self._setup_camera()
            return True
        except Exception as e:
            print(f"Failed to initialize MuJoCo viewer: {e}")
            return False
    
    def update(self, state=None) -> None:
        """Update viewer (sync)."""
        if self.viewer is not None:
            self.viewer.sync()
    
    def is_running(self) -> bool:
        """Check if viewer window is open."""
        if self.viewer is None:
            return False
        return self.viewer.is_running()
    
    def shutdown(self) -> None:
        """Close viewer."""
        if self.viewer_context is not None:
            self.viewer_context.__exit__(None, None, None)
    
    def _setup_camera(self):
        """Configure camera position."""
        if self.viewer is not None:
            self.viewer.cam.distance = self.camera_config.get('distance', 1.0)
            self.viewer.cam.elevation = self.camera_config.get('elevation', -20)
            self.viewer.cam.azimuth = self.camera_config.get('azimuth', 270)


if __name__ == "__main__":
    import os
    import time
    
    # Path to the scene XML file
    xml_path = os.path.join(
        os.path.dirname(__file__), 
        "../../../assets/franka_emika_panda/scene.xml"
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
