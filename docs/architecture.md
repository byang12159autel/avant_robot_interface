# Avant Robot Interface - Architecture Overview

This document describes the software architecture of the `avant_robot_interface` framework, including base classes, current implementations, and planned future extensions.

## Table of Contents

- [Directory Structure](#directory-structure)
- [Core Data Contracts](#core-data-contracts)
- [Protocols (Interfaces)](#protocols-interfaces)
- [Base Classes](#base-classes)
  - [BasePlanner](#baseplanner)
  - [BaseIKController](#baseikcontroller)
  - [BaseVisualizer](#basevisualizer)
  - [BaseTeleopInput](#basetelopinput-future)
- [Current Implementations](#current-implementations)
- [Future Implementations](#future-implementations)
- [Data Flow Diagrams](#data-flow-diagrams)
- [Summary Tables](#summary-tables)

---

## Directory Structure

```
avant_robot_interface/
│
├── 📁 core/                          # Core control infrastructure
│   │
│   ├── 📄 contracts.py               # ═══ DATA CONTRACTS ═══
│   │   ├── TimeStamp                 # Monotonic timestamp
│   │   ├── Frame (Enum)              # WORLD | BASE
│   │   ├── RefMode (Enum)            # HOLD | TRACK
│   │   ├── SE3                       # Position (3,) + Quaternion (4,)
│   │   ├── RobotState                # Joint state + optional task-space state
│   │   ├── CartesianTarget           # Where to go (EE pose + tolerances)
│   │   ├── TaskSpaceReference        # When/how to track (mode + horizon)
│   │   ├── JointCommand              # What to send (q_des, dq_des, tau_ff)
│   │   └── TeleopCommand [FUTURE]    # Raw teleop input (delta pose, buttons, etc.)
│   │
│   ├── 📄 ports.py                   # ═══ PROTOCOLS (Interfaces) ═══
│   │   ├── TaskPlanner               # Protocol: update(state) → TaskSpaceReference?
│   │   ├── PositionController        # Protocol: set_reference() + step()
│   │   ├── RobotInterface            # Protocol: get_state() + send_command() + shutdown()
│   │   └── TeleopInput [FUTURE]      # Protocol: read() → TeleopCommand?
│   │
│   ├── 📄 bridge.py                  # Planner↔Controller coordination + staleness detection
│   │
│   ├── 📁 planners/                  # ═══ TASK PLANNERS ═══
│   │   ├── 📄 base.py                # BasePlanner (ABC)
│   │   ├── 📄 hold.py                # ✓ HoldPlanner
│   │   ├── 📄 circular_trajectory.py # ✓ CircularTrajectoryPlanner
│   │   └── 📄 [FUTURE]               # TeleopPlanner, WaypointPlanner, etc.
│   │
│   ├── 📁 controllers/               # ═══ IK CONTROLLERS ═══
│   │   ├── 📄 base.py                # BaseIKController (ABC)
│   │   ├── 📄 mink_ik.py             # ✓ MinkIKController
│   │   └── 📄 [FUTURE]               # PinocchioIK, TracIK, etc.
│   │
│   ├── 📄 simple_control_loop.py     # Multi-rate control loop with decimation
│   ├── 📄 control_loop.py            # Alternative control loop implementation
│   ├── 📄 multirate.py               # Multi-rate scheduling utilities
│   └── 📄 runtime.py                 # Runtime management
│
├── 📁 teleop/                        # ═══ TELEOPERATION INPUTS ═══
│   ├── 📄 base.py [FUTURE]           # BaseTeleopInput (ABC)
│   ├── 📄 keyboard.py                # ✓ KeyboardTeleopInput
│   └── 📄 [FUTURE]                   # SpaceMouse, GELLO, VR, Gamepad, etc.
│
├── 📁 hardware/                      # ═══ HARDWARE INTERFACES ═══
│   ├── 📄 base.py                    # BaseRobotHardware (ABC) [FUTURE]
│   └── 📄 [FUTURE]                   # franka.py, mujoco_sim.py, ros2_control.py
│
├── 📁 plugins/                       # ═══ ROBOT PLUGINS ═══
│   └── 📁 robots/
│       ├── 📁 franka_fr3/            # ✓ Franka FR3 plugin
│       └── 📁 r1pro/ [FUTURE]                
|       └── 📁 unitree_g1/ [FUTURE]    
|       └── 📁 arx_x5/ [FUTURE]   
│
├── 📁 visualization/                 # ═══ VISUALIZERS ═══
│   ├── 📄 visualizer.py              # BaseVisualizer (ABC)
│   └── 📁 mujoco/
│       └── 📄 mujuco_viewer.py       # ✓ MuJoCoVisualizer
│
├── 📁 ros2/                          # ═══ ROS2 INTEGRATION ═══
│   ├── 📄 node.py                    # ROS2 node management
│   ├── 📄 publishers.py              # State publishers (JointState, etc.)
│   ├── 📄 subscribers.py             # Command subscribers
│   └── 📄 handlers.py                # Thread-safe message handlers
│
└── 📁 assets/                        # ═══ SHARED ASSETS ═══
    └── 📁 fr3/                       # ✓ FR3 URDF, meshes, MuJoCo XML
    └── 📁 r1pro/
    └── 📁 unitree_g1/
    └── 📁 arx_x5/
    
```

---

## Core Data Contracts

Defined in `core/contracts.py`, these are the fundamental data structures that flow through the system:

| Contract | Purpose | Key Fields |
|----------|---------|------------|
| **`TimeStamp`** | Monotonic timestamp | `t: float` |
| **`Frame`** | Coordinate frame enum | `WORLD`, `BASE` |
| **`RefMode`** | Reference mode enum | `HOLD`, `TRACK` |
| **`SE3`** | Rigid body transform | `p: (3,)`, `q: (4,)` quaternion |
| **`RobotState`** | Complete robot state | `q`, `dq`, optional: `tau`, `ts_pose`, `ts_wrench` |
| **`CartesianTarget`** | Spatial goal | `link_name`, `T: SE3`, `pos_tol_m`, `rot_tol_rad` |
| **`TaskSpaceReference`** | Timed reference | `stamp`, `horizon_s`, `mode`, `target` |
| **`JointCommand`** | Hardware command | `q_des`, `dq_des`, `tau_ff` |

**Data Flow:**
```
Planner creates CartesianTarget → Wraps in TaskSpaceReference → Controller produces JointCommand
```

---

## Protocols (Interfaces)

Defined in `core/ports.py`, these protocols define the interfaces that components must implement:

### TaskPlanner Protocol
```python
class TaskPlanner(Protocol):
    def update(self, state: RobotState) -> Optional[TaskSpaceReference]: ...
```

### PositionController Protocol
```python
class PositionController(Protocol):
    def set_reference(self, ref: TaskSpaceReference) -> None: ...
    def step(self, state: RobotState) -> JointCommand: ...
```

### RobotInterface Protocol
```python
class RobotInterface(Protocol):
    def get_state(self) -> RobotState: ...
    def send_command(self, cmd: JointCommand) -> None: ...
    def get_initial_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]: ...
    def shutdown(self) -> None: ...
```

---

## Base Classes

### BasePlanner

**Location:** `core/planners/base.py`

Abstract base class for task-space trajectory planners that generate `TaskSpaceReference` objects.

```
╔═══════════════════════════════════════════════════════════════╗
║ BasePlanner (ABC)                                             ║
╠═══════════════════════════════════════════════════════════════╣
║ Attributes:                                                   ║
║   • ee_link: str           - End-effector link name           ║
║   • horizon_s: float       - Reference validity horizon       ║
║   • pos_tol_m: float       - Position tolerance (meters)      ║
║   • rot_tol_rad: float     - Rotation tolerance (radians)     ║
║   • t0: float              - Start time reference             ║
╠═══════════════════════════════════════════════════════════════╣
║ Abstract Methods:                                             ║
║   • update(state) → TaskSpaceReference?              [*]      ║
╠═══════════════════════════════════════════════════════════════╣
║ Concrete Methods:                                             ║
║   • reset()                - Reset planner state              ║
║   • get_elapsed_time()     - Time since start/reset           ║
╚═══════════════════════════════════════════════════════════════╝
```

**Current Implementations:**
- `HoldPlanner` - Returns `None` to keep robot stationary
- `CircularTrajectoryPlanner` - Circular motion in XY plane

---

### BaseIKController

**Location:** `core/controllers/base.py`

Abstract base class for inverse kinematics position controllers.

```
╔═══════════════════════════════════════════════════════════════╗
║ BaseIKController (ABC)                                        ║
╠═══════════════════════════════════════════════════════════════╣
║ Attributes:                                                   ║
║   • dt: float              - Controller timestep              ║
║   • ee_link: str           - End-effector link name           ║
║   • pos_threshold: float   - Position convergence threshold   ║
║   • ori_threshold: float   - Orientation convergence threshold║
║   • max_iters: int         - Maximum IK iterations            ║
║   • current_ref: TaskSpaceReference? - Current reference      ║
╠═══════════════════════════════════════════════════════════════╣
║ Abstract Methods:                                             ║
║   • set_reference(ref)                               [*]      ║
║   • step(state) → JointCommand                       [*]      ║
║   • initialize_posture_target()                      [*]      ║
║   • update_configuration(q)                          [*]      ║
╠═══════════════════════════════════════════════════════════════╣
║ Concrete Methods:                                             ║
║   • get_current_reference() → TaskSpaceReference?             ║
╚═══════════════════════════════════════════════════════════════╝
```

**Current Implementation:**
- `MinkIKController` - Uses mink library with DAQP solver

---

### BaseVisualizer

**Location:** `visualization/visualizer.py`

Abstract base class for visualization backends.

```
╔═══════════════════════════════════════════════════════════════╗
║ BaseVisualizer (ABC)                                          ║
╠═══════════════════════════════════════════════════════════════╣
║ Abstract Methods:                                             ║
║   • initialize() → bool                              [*]      ║
║   • update(state)                                    [*]      ║
║   • is_running() → bool                              [*]      ║
║   • shutdown()                                       [*]      ║
╚═══════════════════════════════════════════════════════════════╝
```

**Current Implementation:**
- `MuJoCoVisualizer` - Real-time MuJoCo rendering

---

### BaseTeleopInput (Future)

**Proposed Location:** `teleop/base.py`

Abstract base class for teleoperation input devices.

```
╔═══════════════════════════════════════════════════════════════╗
║ BaseTeleopInput (ABC) [FUTURE]                                ║
╠═══════════════════════════════════════════════════════════════╣
║ Proposed Methods:                                             ║
║   • connect() → bool                                 [*]      ║
║   • read() → TeleopCommand?                          [*]      ║
║   • is_connected() → bool                            [*]      ║
║   • disconnect()                                     [*]      ║
║   • get_deadman_state() → bool                                ║
╚═══════════════════════════════════════════════════════════════╝
```

---

## Current Implementations

### Planners

| Class | File | Description |
|-------|------|-------------|
| `HoldPlanner` | `planners/hold.py` | Returns `None` → keeps robot stationary. Used for teleoperation wait state. |
| `CircularTrajectoryPlanner` | `planners/circular_trajectory.py` | Generates circular motion in XY plane. Supports dropout simulation. |

### Controllers

| Class | File | Description |
|-------|------|-------------|
| `MinkIKController` | `controllers/mink_ik.py` | Uses mink library with DAQP solver. Supports position + orientation tracking, nullspace posture control. |

### Visualizers

| Class | File | Description |
|-------|------|-------------|
| `MuJoCoVisualizer` | `visualization/mujoco/mujuco_viewer.py` | Real-time MuJoCo rendering with interactive viewer. |

### Teleop (Partial)

| Class | File | Description |
|-------|------|-------------|
| Keyboard Teleop | `examples/keyboard_ee_teleop.py` | pynput-based keyboard listener. Publishes to ROS2 `/ee_command`. |

---

## Future Implementations

### Planners (Future)

| Class | Description |
|-------|-------------|
| `LinearTrajectoryPlanner` | Point-to-point linear motion with velocity profiles |
| `SplineTrajectoryPlanner` | Smooth spline interpolation through waypoints |
| `JointSpacePlanner` | Joint-space trajectory generation |
| `WaypointPlanner` | Sequence of waypoints with blending |
| `RealTimePlanner` | Dynamic replanning with obstacle avoidance |
| `MoveItPlanner` | MoveIt! integration wrapper |
| `TeleopPlanner` | Converts TeleopCommand → TaskSpaceReference |

### Controllers (Future)

| Class | Description |
|-------|-------------|
| `PinocchioIKController` | Pinocchio-based IK solver |
| `KDLIKController` | KDL (Orocos) IK solver |

### Visualizers (Future)

| Class | Description |
|-------|-------------|
| `ViserVisualizer` | Web-based 3D visualization (nerfstudio-project/viser) |
| `RVizVisualizer` | ROS2 RViz integration |
| `HeadlessRecorder` | Video recording without display |

### Teleoperation Devices (Future)

| Class | Description |
|-------|-------------|
| `KeyboardTeleopInput` | pynput-based keyboard listener |
| `SpaceMouseInput` | 3Dconnexion SpaceMouse/SpaceNavigator - 6-DOF analog input |
| `GelloInput` | GELLO teleoperation system - Low-cost leader arm |
| `VRInput` | VR controller teleoperation (Oculus, Vive, Apple Vision Pro) |
| `GamepadInput` | Xbox/PlayStation controllers - Dual analog sticks |

### Robot Plugins (Future)

#### Arms
| Plugin | Description |
|--------|-------------|
| `franka_re3/` | Franka RE3 |
| `arx_arm/` | ARX robotic arm |


#### Humanoids
| Plugin | Description |
|--------|-------------|
| `unitree_g1/` | Unitree G1 humanoid |
| `r1pro/` | R1 Pro humanoid |

### Hardware Interfaces (Future)

| Class | Description |
|-------|-------------|
| `franka.py` | Franka Emika FR3 via crisp_py |
| `mujoco_sim.py` | MuJoCo simulation backend |
| `ros2_control.py` | Generic ros2_control interface |
| `mock.py` | Testing/development mock |

---

## Data Flow Diagrams

### Main Control Flow

```
    ┌─────────────┐                                           ┌─────────────┐
    │   External  │     ROS2 Topics / Direct API              │   Hardware  │
    │   Commands  │ ◄─────────────────────────────────────────│   Sensors   │
    └──────┬──────┘                                           └──────┬──────┘
           │                                                         │
           ▼                                                         ▼
    ╔═════════════╗                                           ╔═════════════╗
    ║   Planner   ║                                           ║   Robot     ║
    ║ (BasePlanner)                                           ║  Interface  ║
    ╚══════╤══════╝                                           ╚══════╤══════╝
           │                                                         │
           │ TaskSpaceReference                                      │ RobotState
           │ (mode, target, horizon)                                 │ (q, dq, tau)
           ▼                                                         │
    ╔═════════════╗                                                  │
    ║   Bridge    ║ ◄────────────────────────────────────────────────┘
    ║  (staleness)║
    ╚══════╤══════╝
           │
           │ Validated TaskSpaceReference
           ▼
    ╔═════════════╗
    ║ Controller  ║
    ║(BaseIKCtrl) ║
    ╚══════╤══════╝
           │
           │ JointCommand
           │ (q_des, dq_des, tau_ff)
           ▼
    ┌─────────────┐
    │   Robot     │
    │  (HW/Sim)   │
    └─────────────┘
```

### Teleoperation Data Flow

```
    ┌──────────────────┐
    │  Physical Device │  (SpaceMouse, GELLO, VR Controller, Keyboard...)
    └────────┬─────────┘
             │
             │ USB/Bluetooth/Network
             ▼
    ╔═══════════════════╗
    ║  TeleopInput      ║  (BaseTeleopInput implementation)
    ║  (e.g. GelloInput)║
    ╚═════════╤═════════╝
              │
              │ TeleopCommand (delta_pose, buttons, gripper_state)
              ▼
    ╔═══════════════════╗
    ║   TeleopPlanner   ║  (Converts teleop input → task-space reference)
    ║   (BasePlanner)   ║  Handles: smoothing, deadband, workspace limits
    ╚═════════╤═════════╝
              │
              │ TaskSpaceReference
              ▼
    ╔═══════════════════╗
    ║      Bridge       ║
    ╚═════════╤═════════╝
              │
              ▼
    ╔═══════════════════╗
    ║    Controller     ║
    ╚═════════╤═════════╝
              │
              │ JointCommand
              ▼
    ╔═══════════════════╗
    ║  RobotInterface   ║  (Hardware driver for target robot)
    ║  (e.g. R1Pro)     ║
    ╚═══════════════════╝
```

### Multi-Rate Control Loop

```
Base frequency: 100 Hz (10 ms period)

┌────────────────────────────────────────────────────────────────┐
│ Task 1: Planner      │ decimation = 2  │ runs @ 50 Hz         │
├────────────────────────────────────────────────────────────────┤
│ Task 2: Controller   │ decimation = 1  │ runs @ 100 Hz        │
├────────────────────────────────────────────────────────────────┤
│ Task 3: ROS2/Viz     │ decimation = 2  │ runs @ 50 Hz         │
└────────────────────────────────────────────────────────────────┘
```

---

## Summary Tables

### Component Location Summary

| Category | Location | Examples |
|----------|----------|----------|
| **Robot Hardware** | `hardware/` | `franka.py`, `ros2_control.py`, `mujoco_sim.py` |
| **Robot Configs** | `plugins/robots/{name}/` | `unitree_g1/`, `r1pro/`, `arx_arm/` |
| **Teleop Devices** | `teleop/` | `keyboard.py`, `spacemouse.py`, `gello.py`, `vr.py` |
| **Teleop → Planner** | `core/planners/` | `TeleopPlanner` (wraps TeleopInput) |
| **Visualizers** | `visualization/` | `mujoco/`, `viser/`, `rviz/` |
| **Control Core** | `core/` | `contracts.py`, `bridge.py`, `ports.py` |

### Base Class Summary

| Base Class | Location | Current Implementations | Future Implementations |
|------------|----------|------------------------|------------------------|
| **BasePlanner** | `core/planners/base.py` | `HoldPlanner`, `CircularTrajectoryPlanner` | Linear, Spline, Waypoint, MoveIt, DMP, Teleop |
| **BaseIKController** | `core/controllers/base.py` | `MinkIKController` | Pinocchio, KDL, TracIK, IKFast |
| **BaseVisualizer** | `visualization/visualizer.py` | `MuJoCoVisualizer` | Viser, RViz, PlotJuggler, Matplotlib |
| **BaseTeleopInput** | `teleop/base.py` [FUTURE] | (Keyboard via example) | SpaceMouse, GELLO, VR, Gamepad |
| **RobotInterface** | `core/ports.py` (Protocol) | (via crisp_py) | Franka, UR, KUKA, ros2_control, Mock |

---

## Legend

| Symbol | Meaning |
|--------|---------|
| ✓ | Currently implemented |
| ○ | Potential future implementation |
| [*] | Abstract method (must be implemented by subclass) |
| ABC | Abstract Base Class |
| [FUTURE] | Planned but not yet implemented |

---

## See Also

- [Keyboard Teleop Architecture](keyboard_teleop_architecture.md) - Detailed teleop communication architecture
