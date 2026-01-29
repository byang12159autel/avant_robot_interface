# Keyboard Teleop Communication Architecture

This document describes the communication architecture for the keyboard teleoperation system used in `run_franka_sim_ros2_teleop.py`.

## System Overview

The keyboard teleop system consists of two independent processes that communicate via ROS2 topics:

1. **Terminal 1**: `keyboard_ee_teleop.py` - Keyboard input handler and ROS2 publisher
2. **Terminal 2**: `run_franka_sim_ros2_teleop.py` - Multi-rate control loop with robot interface

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          TERMINAL 1                                     │
│                   keyboard_ee_teleop.py                                 │
│                                                                         │
│  ┌──────────────────┐      ┌─────────────────┐      ┌───────────────┐ │
│  │    Keyboard      │      │    Process      │      │     ROS2      │ │
│  │    Listener      │ ───> │   Keys & Update │ ───> │   Publisher   │ │
│  │    (pynput)      │      │    EE Pose      │      │               │ │
│  └──────────────────┘      └─────────────────┘      └───────┬───────┘ │
│         ^                                                    │         │
│         │                                                    │         │
│    User presses keys:                                        │         │
│    ↑↓←→ (translation)                                        │         │
│    QWASZX (rotation)                                         │         │
│    +/- (adjust step)                                         │         │
│    R (reset), P (print), H (help)                           │         │
└──────────────────────────────────────────────────────────────┼─────────┘
                                                               │
                                                               │ Publishes
                                                               │ @ 10 Hz
                                                               ▼
                         ╔═════════════════════════════════════════╗
                         ║     ROS2 Topic: /ee_command             ║
                         ║                                         ║
                         ║   Message Type: geometry_msgs/          ║
                         ║                 PoseStamped             ║
                         ║                                         ║
                         ║   Content:                              ║
                         ║     - header (timestamp, frame_id)      ║
                         ║     - pose.position (x, y, z)           ║
                         ║     - pose.orientation (x, y, z, w)     ║
                         ║                                         ║
                         ║   Publishing Rate: 10 Hz                ║
                         ║   QoS History Depth: 10                 ║
                         ╚═════════════════════════════════════════╝
                                                               │
                                                               │ Subscribes
                                                               ▼
┌──────────────────────────────────────────────────────────────┼─────────┐
│                          TERMINAL 2                          │         │
│              run_franka_sim_ros2_teleop.py                   │         │
│                                                              │         │
│  ╔══════════════════════════════════════════════════════════════════╗ │
│  ║      Multi-Rate Control Loop (Base Frequency: 100 Hz)            ║ │
│  ║                                                                   ║ │
│  ║  Uses decimation to achieve different task rates from 100 Hz base║ │
│  ╠══════════════════════════════════════════════════════════════════╣ │
│  ║                                                                   ║ │
│  ║  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  ║ │
│  ║  ┃ TASK 3: ROS2 Spin + Visualization (50 Hz, decimation=2) ┃  ║ │
│  ║  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  ║ │
│  ║  ┌────────────────────────────────────────────────────────────┐ ║ │
│  ║  │ spin_ros2_once(timeout_sec=0.0)  [Non-blocking]           │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Processes subscribed ROS2 callbacks                       │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ EEPoseSubscriber callback triggered                       │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Stores PoseStamped in EEPoseCommandHandler                │ ║ │
│  ║  │   - Thread-safe storage with lock                         │ ║ │
│  ║  │   - Stores: position, orientation, timestamp              │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Update MuJoCo visualization (if enabled)                  │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Publish robot state to /joint_states                      │ ║ │
│  ║  └────────────────────────────────────────────────────────────┘ ║ │
│  ║           │                                                      ║ │
│  ║           │ Latest EE command stored and ready                   ║ │
│  ║           ▼                                                      ║ │
│  ║  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  ║ │
│  ║  ┃ TASK 1: Planner Task (50 Hz, decimation=2)               ┃  ║ │
│  ║  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  ║ │
│  ║  ┌────────────────────────────────────────────────────────────┐ ║ │
│  ║  │ get_ros2_ee_command(max_age_s=1.0)                        │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Check EEPoseCommandHandler for latest message             │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Verify message age < 1.0 seconds (staleness check)        │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ ┌─────────────────────────┐  ┌───────────────────────┐   │ ║ │
│  ║  │ │ IF ROS2 command valid:  │  │ ELSE (no/stale cmd):  │   │ ║ │
│  ║  │ │                         │  │                       │   │ ║ │
│  ║  │ │ Create TaskSpaceRef:    │  │ HoldPlanner.update()  │   │ ║ │
│  ║  │ │  - Extract pos & quat   │  │   ↓                   │   │ ║ │
│  ║  │ │  - Build SE3 transform  │  │ Returns None          │   │ ║ │
│  ║  │ │  - Create CartesianTgt  │  │   ↓                   │   │ ║ │
│  ║  │ │  - Mode = TRACK         │  │ Robot stays in HOLD   │   │ ║ │
│  ║  │ │  - horizon_s = 0.25     │  │                       │   │ ║ │
│  ║  │ └─────────────────────────┘  └───────────────────────┘   │ ║ │
│  ║  │         │                              │                  │ ║ │
│  ║  │         └──────────────┬───────────────┘                  │ ║ │
│  ║  │                        ▼                                  │ ║ │
│  ║  │         Bridge.planner_tick(reference)                    │ ║ │
│  ║  │           - Validates reference                           │ ║ │
│  ║  │           - Checks staleness (horizon)                    │ ║ │
│  ║  │           - Stores for controller                         │ ║ │
│  ║  └────────────────────────────────────────────────────────────┘ ║ │
│  ║           │                                                      ║ │
│  ║           │ Valid reference stored in Bridge                     ║ │
│  ║           ▼                                                      ║ │
│  ║  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  ║ │
│  ║  ┃ TASK 2: Controller Task (100 Hz, decimation=1)           ┃  ║ │
│  ║  ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  ║ │
│  ║  ┌────────────────────────────────────────────────────────────┐ ║ │
│  ║  │ CrispRobotInterface.get_state()                           │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ RobotState (q, dq, timestamp)                             │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Bridge.control_tick(state)                                │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ Retrieves validated reference from planner                │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ MinkIKController.step(state)                              │ ║ │
│  ║  │   ↓                                                        │ ║ │
│  ║  │ ┌─────────────────────────┐  ┌───────────────────────┐   │ ║ │
│  ║  │ │ IF ref mode = TRACK:    │  │ IF ref mode = HOLD:   │   │ ║ │
│  ║  │ │                         │  │                       │   │ ║ │
│  ║  │ │ Update mink config with │  │ Command current       │   │ ║ │
│  ║  │ │ current joint state     │  │ joint positions       │   │ ║ │
│  ║  │ │   ↓                     │  │   ↓                   │   │ ║ │
│  ║  │ │ Set IK target to EE     │  │ q_des = state.q       │   │ ║ │
│  ║  │ │ pose from reference     │  │                       │   │ ║ │
│  ║  │ │   ↓                     │  │                       │   │ ║ │
│  ║  │ │ Solve IK (Mink solver)  │  │                       │   │ ║ │
│  ║  │ │   - Max 20 iterations   │  │                       │   │ ║ │
│  ║  │ │   - Pos thr: 1e-4 m     │  │                       │   │ ║ │
│  ║  │ │   - Ori thr: 1e-4 rad   │  │                       │   │ ║ │
│  ║  │ │   ↓                     │  │                       │   │ ║ │
│  ║  │ │ Extract q_des (7 DOF)   │  │                       │   │ ║ │
│  ║  │ └─────────────────────────┘  └───────────────────────┘   │ ║ │
│  ║  │         │                              │                  │ ║ │
│  ║  │         └──────────────┬───────────────┘                  │ ║ │
│  ║  │                        ▼                                  │ ║ │
│  ║  │         JointCommand (q_des, dq_des, tau_ff)              │ ║ │
│  ║  │                        ↓                                  │ ║ │
│  ║  │         CrispRobotInterface.send_command(cmd)             │ ║ │
│  ║  └────────────────────────────────────────────────────────────┘ ║ │
│  ║           │                                                      ║ │
│  ╚═══════════╪══════════════════════════════════════════════════════╝ │
│              │                                                        │
│              ▼                                                        │
│  ┌──────────────────────────────────────────────────────────┐       │
│  │           CrispRobotInterface                            │       │
│  │           (crisp_py wrapper)                             │       │
│  │                                                          │       │
│  │  - Connects to robot via ROS2 Action Client             │       │
│  │  - Switches to joint_impedance_controller                │       │
│  │  - Sends joint position commands                         │       │
│  └──────────────────────────────┬───────────────────────────┘       │
│                                 │                                    │
└─────────────────────────────────┼────────────────────────────────────┘
                                  │
                                  │ ROS2 Actions/Services
                                  ▼
             ╔═══════════════════════════════════════════╗
             ║        FR3 Robot Hardware/Simulation      ║
             ║                                           ║
             ║  - ROS2 Control Framework                 ║
             ║  - Joint Impedance Controller             ║
             ║  - Hardware Interface or Fake Hardware    ║
             ╚═══════════════════════════════════════════╝
```

## Data Flow Details

### 1. User Input → ROS2 Message (10 Hz)

**Input Processing:**
- User presses keyboard keys
- `pynput` library captures key events (press/release)
- Multiple keys can be held simultaneously for combined movements

**Pose Update:**
- Translation: Incremental position changes (default: 8 mm/press)
- Rotation: Incremental euler angle changes (default: 1.7°/press)
- Applied in world frame coordinates

**Publishing:**
- Rate: 10 Hz continuous publishing
- Message: `geometry_msgs/PoseStamped`
- Topic: `/ee_command`
- Contains current desired EE pose (absolute, not incremental)

### 2. ROS2 → Control Loop (50 Hz processing)

**Message Reception:**
- Task 3 calls `spin_ros2_once()` at 50 Hz
- Non-blocking call (timeout=0.0)
- Processes all pending ROS2 callbacks

**Thread-Safe Storage:**
- `EEPoseCommandHandler` stores latest message
- Uses threading.Lock for thread safety
- Stores timestamp for staleness detection

### 3. Planner Task (50 Hz)

**Command Retrieval:**
- Checks for new commands from handler
- Validates message age (max_age_s=1.0)
- Filters out stale commands

**Reference Generation:**
```python
if valid_ros2_command:
    # Convert to TaskSpaceReference
    reference = TaskSpaceReference(
        stamp=current_time,
        horizon_s=0.25,
        mode=RefMode.TRACK,
        target=CartesianTarget(...)
    )
else:
    # HoldPlanner returns None → HOLD mode
    reference = None
```

**Bridge Integration:**
- Sends reference to Bridge via `planner_tick()`
- Bridge validates and stores for controller

### 4. Controller Task (100 Hz)

**State Reading:**
- Gets current joint positions from robot
- Frequency: 100 Hz (base rate)

**Reference Processing:**
- Bridge provides validated reference via `control_tick()`
- Checks staleness based on horizon

**IK Solving (if TRACK mode):**
```python
# Mink IK configuration
- Solver: DAQP
- Position threshold: 1e-4 m
- Orientation threshold: 1e-4 rad
- Max iterations: 20
- Posture cost: 0.01
- Damping: 0.001
```

**Command Execution:**
- Computes desired joint positions (q_des)
- Sends to robot via crisp_py interface

### 5. Multi-Rate Synchronization

**Decimation Strategy:**
```python
Base frequency: 100 Hz (10 ms period)

Task 1 (Planner):      decimation = 2  → runs every 2nd iteration  → 50 Hz
Task 2 (Controller):   decimation = 1  → runs every iteration      → 100 Hz
Task 3 (ROS2/Viz):     decimation = 2  → runs every 2nd iteration  → 50 Hz
```

**Timing Guarantees:**
- Deterministic execution order within each iteration
- No thread contention (single-threaded execution)
- Inline ROS2 (no separate thread) eliminates jitter

## Key Design Features

### 1. **Decoupled Processes**
- Keyboard and control loop are independent
- Communication only via ROS2 topics
- Allows independent development/testing

### 2. **Staleness Detection**
- Messages older than 1.0 second are rejected
- Prevents robot from following outdated commands
- Automatic fallback to HOLD mode

### 3. **Thread-Safe Communication**
- All ROS2 handlers use locks
- Safe concurrent access to shared data
- No race conditions

### 4. **Rate Adaptation**
- Different components run at appropriate rates
- Planner: 50 Hz (sufficient for human input)
- Controller: 100 Hz (good for simulation)
- ROS2: 50 Hz (reduces overhead)

### 5. **Mode Transitions**
```
Initial State: HOLD mode (robot stationary)
     ↓
Keyboard starts sending commands
     ↓
TRACK mode (robot follows commands)
     ↓
Keyboard stops or commands become stale
     ↓
HOLD mode (robot maintains last position)
```

## Message Formats

### `/ee_command` (Published by keyboard_ee_teleop.py)

```yaml
header:
  stamp:
    sec: 1738184400
    nanosec: 123456789
  frame_id: "world"
pose:
  position:
    x: 0.5      # meters
    y: 0.0      # meters
    z: 0.4      # meters
  orientation:
    x: 0.0      # quaternion x
    y: 0.0      # quaternion y
    z: 0.0      # quaternion z
    w: 1.0      # quaternion w
```

### Internal: `TaskSpaceReference`

```python
TaskSpaceReference(
    stamp=TimeStamp(monotonic_time),
    horizon_s=0.25,                    # Valid for 250ms
    mode=RefMode.TRACK,                # or RefMode.HOLD
    target=CartesianTarget(
        link_name="attachment_site",
        in_frame=Frame.WORLD,
        T=SE3(p=[x,y,z], q=[x,y,z,w]),
        v=None,
        weight=1.0,
        pos_tol_m=0.002,
        rot_tol_rad=0.02
    )
)
```

### Internal: `JointCommand`

```python
JointCommand(
    stamp=TimeStamp(monotonic_time),
    q_des=[j1, j2, j3, j4, j5, j6, j7],  # 7 joint positions (rad)
    dq_des=None,                           # Joint velocities (not used)
    tau_ff=None                            # Feedforward torques (not used)
)
```

## Performance Characteristics

### Latency Analysis

```
User Key Press → Robot Motion:
  1. Key detection:           ~1-5 ms   (pynput)
  2. ROS2 publish:            ~1 ms     (local)
  3. ROS2 receive:            ~20 ms    (next Task 3 cycle @ 50 Hz)
  4. Planner processing:      ~20 ms    (next Task 1 cycle @ 50 Hz)
  5. Controller processing:   ~10 ms    (next Task 2 cycle @ 100 Hz)
  6. Robot command:           ~10 ms    (crisp_py + ROS2 Action)
  
Total latency: ~60-70 ms (perception to actuation)
```

### CPU Usage

- Keyboard teleop: ~1-2% CPU (mostly idle, event-driven)
- Control loop: ~5-15% CPU (depends on IK complexity)
- Visualization: ~10-20% CPU (MuJoCo rendering)

### Network Bandwidth

- `/ee_command`: ~0.1 KB/msg × 10 Hz = ~1 KB/s
- `/joint_states`: ~0.2 KB/msg × 50 Hz = ~10 KB/s
- Total: Very low bandwidth requirements

## Configuration

### Keyboard Teleop (`keyboard_ee_teleop.py`)

```python
translation_step = 0.008    # 8 mm per key press
rotation_step = 0.03        # ~1.7 degrees per key press
publish_rate = 10.0         # 10 Hz

# Adjustable at runtime with +/- keys
min_translation_step = 0.001  # 1 mm
max_translation_step = 0.05   # 5 cm
```

### Control Loop (`franka_single.yaml`)

```yaml
rates:
  planner_hz: 50.0          # Planner frequency
  controller_hz: 100.0      # Controller frequency

task:
  horizon_s: 0.25           # Reference validity
  pos_tol_m: 0.002          # Position tolerance
  rot_tol_rad: 0.02         # Rotation tolerance

ik:
  solver: "daqp"
  pos_threshold: 1e-4
  ori_threshold: 1e-4
  max_iters: 20
```

### Runtime Overrides

```bash
# Change ROS2/visualization frequency
python examples/run_franka_sim_ros2_teleop.py --enable-ros2 --ros2-freq 100.0

# Change controller frequency
python examples/run_franka_sim_ros2_teleop.py --enable-ros2 --control-freq 200.0

# Change planner frequency
python examples/run_franka_sim_ros2_teleop.py --enable-ros2 --plan-freq 100.0
```

## Usage Example

### Terminal 1: Start Robot Control
```bash
# Start the control loop with visualization and ROS2
python examples/run_franka_sim_ros2_teleop.py --visualize --enable-ros2

# Output:
# ============================================================
# Keyboard Teleop Control with Mink IK
# ============================================================
# Planner frequency: 50.0 Hz (HOLD mode only)
# Controller frequency: 100.0 Hz
# Visualization/ROS2 frequency: 50.0 Hz
# ROS2 enabled: True
# ============================================================
#
# [HOLD PLANNER] Robot in HOLD mode - waiting for ROS2 teleop commands...
# ✓ Ready for keyboard teleop!
```

### Terminal 2: Start Keyboard Control
```bash
# Launch keyboard teleop interface
python examples/keyboard_ee_teleop.py

# Output:
# ======================================================================
# Keyboard End-Effector Teleoperation (6-DOF Control)
# ======================================================================
# ✓ Keyboard listener started!
# ✓ Publishing EE commands to /ee_command
#
# Press 'H' for help, 'ESC' to quit
```

### Expected Behavior

1. Robot starts in HOLD mode at initial position
2. Press arrow keys → robot starts moving
3. Console shows: `[TELEOP] ✓ Receiving keyboard commands from /ee_command`
4. Mode transitions to TRACK
5. Release keys → robot holds last position after 1 second timeout
6. Mode transitions back to HOLD

## Troubleshooting

### Robot Not Responding to Keyboard

**Check 1: ROS2 enabled?**
```bash
# Must use --enable-ros2 flag
python examples/run_franka_sim_ros2_teleop.py --visualize --enable-ros2
```

**Check 2: Topic communication**
```bash
# Verify keyboard is publishing
ros2 topic hz /ee_command

# Verify control loop is subscribed
ros2 topic info /ee_command
```

**Check 3: Message age**
- Commands older than 1.0s are rejected
- Increase timeout in code if needed

### High Latency

**Possible causes:**
1. CPU overload → reduce visualization frequency
2. Network issues → check ROS2 DDS configuration
3. IK not converging → check joint limits, adjust IK parameters

### Jerky Motion

**Solutions:**
1. Reduce translation/rotation step size (use `-` key)
2. Increase keyboard publish rate
3. Increase controller frequency
4. Adjust IK damping parameters

## References

- **Base Control Loop**: `src/avant_robot_interface/core/simple_control_loop.py`
- **Bridge Component**: `src/avant_robot_interface/core/bridge.py`
- **ROS2 Integration**: `src/avant_robot_interface/ros2/`
- **Mink IK Solver**: https://github.com/stephane-caron/mink
- **crisp_py Interface**: https://github.com/cambel/crisp_py
