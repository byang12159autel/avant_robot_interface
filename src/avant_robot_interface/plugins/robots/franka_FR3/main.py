# minimal_mujoco_sim.py (~30-40 lines)

import mujoco
import mujoco.viewer
from pathlib import Path

_XML = Path("/home/ben/crisp_controllers_demos/crisp_controllers_robot_demos/config/fr3/scene.xml")

def run_minimal_simulation(model_path, duration_s=10.0):
    """
    Minimal MuJoCo simulation with passive viewer.
    
    Args:
        model_path: Path to MuJoCo XML model
        duration_s: Simulation duration in seconds
    """
    # 1. Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # 2. Create passive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Optional: Set camera view
        viewer.cam.distance = 2.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 90
        
        # 3. Simulation loop
        while viewer.is_running() and data.time < duration_s:
            # Optional: Apply control (e.g., zero torque, gravity comp, etc.)
            data.ctrl[:] = 0  # or some simple control law
            
            # Step physics
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()

if __name__ == "__main__":
    # Run with any of your models
    run_minimal_simulation(_XML, duration_s=10.0)
