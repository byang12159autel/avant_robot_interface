from dataclasses import dataclass
from pathlib import Path
import yaml

@dataclass
class DemoConfig:
    # Robot
    robot_name: str
    dof: int
    ee_link: str

    # Model
    mujoco_xml_path: str

    # IK
    ik_solver: str
    pos_threshold: float
    ori_threshold: float
    max_iters: int

    # Controller
    posture_cost: float
    damping: float
    ee_position_cost: float
    ee_orientation_cost: float
    ee_lm_damping: float

    # Rates
    planner_hz: float
    controller_hz: float

    # Task
    horizon_s: float
    pos_tol_m: float
    rot_tol_rad: float

def load_config(path: str) -> DemoConfig:
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    # Resolve ${PROJECT_ROOT} token in paths
    project_root = Path(__file__).parent.parent.parent.parent  # Go to Repository root
    print(project_root)
    mujoco_xml_path = data["model"]["mujoco_xml_path"]
    mujoco_xml_path = mujoco_xml_path.replace("${PROJECT_ROOT}", str(project_root))

    return DemoConfig(
        robot_name=data["robot"]["name"],
        dof=data["robot"]["dof"],
        ee_link=data["robot"]["ee_link"],
        mujoco_xml_path=mujoco_xml_path,
        ik_solver=data["ik"]["solver"],
        pos_threshold=float(data["ik"]["pos_threshold"]),
        ori_threshold=float(data["ik"]["ori_threshold"]),
        max_iters=data["ik"]["max_iters"],
        posture_cost=float(data["controller"]["posture_cost"]),
        damping=float(data["controller"]["damping"]),
        ee_position_cost=data["controller"]["ee_position_cost"],
        ee_orientation_cost=data["controller"]["ee_orientation_cost"],
        ee_lm_damping=data["controller"]["ee_lm_damping"],
        planner_hz=data["rates"]["planner_hz"],
        controller_hz=data["rates"]["controller_hz"],
        horizon_s=data["task"]["horizon_s"],
        pos_tol_m=data["task"]["pos_tol_m"],
        rot_tol_rad=data["task"]["rot_tol_rad"],
    )