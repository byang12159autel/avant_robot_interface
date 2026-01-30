# Python MuJoCo Simulator Service Node

This directory contains a Python-based MuJoCo simulator that exposes ROS2 services for integration with the CRISP controllers' System Interface.

## Overview

**Purpose:** Migrate from C++ in-process MuJoCo simulator to Python service-based architecture (Approach 1 from architecture diagrams).

**Key Features:**
- ✅ Process-isolated simulator (separate from controller_manager)
- ✅ Python MuJoCo API (easier to use than C API)
- ✅ ROS2 service interface (Init and Step services)
- ✅ Real-time wall-clock synchronization
- ✅ Optional visualization using MuJoCoVisualizer
- ✅ Compatible with crisp_mujoco_sim_msgs service definitions

## Architecture

```
┌─────────────────────────────────────┐
│   Controller Manager Process        │
│  ┌────────────────────────────────┐ │
│  │  CRISP Controllers             │ │
│  │  (Joint Trajectory, Force, etc)│ │
│  └────────────────────────────────┘ │
│               ↓                      │
│  ┌────────────────────────────────┐ │
│  │  System Interface (C++)        │ │
│  │  - Exports command/state ifaces│ │
│  │  - Calls ROS2 services         │ │
│  └────────────────────────────────┘ │
└─────────────────────────────────────┘
            ↓ ROS2 Services
            ↓ (Init, Step)
┌─────────────────────────────────────┐
│   Python Simulator Process          │
│  ┌────────────────────────────────┐ │
│  │  MuJoCoSimulatorService Node   │ │
│  │  - Init service handler        │ │
│  │  - Step service handler        │ │
│  └────────────────────────────────┘ │
│               ↓                      │
│  ┌────────────────────────────────┐ │
│  │  MuJoCo Python API             │ │
│  │  - mj.MjModel, mj.MjData       │ │
│  │  - mj.mj_step()                │ │
│  └────────────────────────────────┘ │
│               ↓                      │
│  ┌────────────────────────────────┐ │
│  │  MuJoCoVisualizer (Optional)   │ │
│  └────────────────────────────────┘ │
└─────────────────────────────────────┘
```

## Prerequisites

### 1. Build crisp_mujoco_sim_msgs

```bash
cd ~/crisp_controllers_demos
colcon build --packages-select crisp_mujoco_sim_msgs
source install/setup.bash
```

### 2. Install Python Dependencies

```bash
# MuJoCo Python bindings
pip install mujoco

# ROS2 Python packages (should already be installed)
# rclpy, geometry_msgs, sensor_msgs, etc.
```

### 3. Install avant_robot_interface (for visualization)

```bash
cd ~/avant_robot_interface
pip install -e .
```

## Usage

### Standalone Testing (Without System Interface)

#### 1. Basic Usage - Wait for Init Service

Start the simulator and wait for an Init service call:

```bash
cd ~/avant_robot_interface
python examples/mujoco_sim_service_node.py --visualize
```

Then call the Init service from another terminal:

```bash
# Source ROS2 workspace
source ~/crisp_controllers_demos/install/setup.bash

# Call Init service
ros2 service call /mujoco_init crisp_mujoco_sim_msgs/srv/Init \
  "{model_path: '$HOME/crisp_controllers_demos/crisp_controllers_robot_demos/config/fr3/scene.xml'}"
```

Then you can call Step service:

```bash
# Step with zero effort commands (7 DOF for FR3)
ros2 service call /mujoco_step crisp_mujoco_sim_msgs/srv/Step \
  "{effort_commands: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### 2. Auto-Start Mode

Auto-load the model and start simulation immediately:

```bash
cd ~/avant_robot_interface
python examples/mujoco_sim_service_node.py \
  --model ~/crisp_controllers_demos/crisp_controllers_robot_demos/config/fr3/scene.xml \
  --auto-start \
  --visualize
```

#### 3. Custom Simulation Frequency

```bash
python examples/mujoco_sim_service_node.py \
  --model ~/crisp_controllers_demos/crisp_controllers_robot_demos/config/fr3/scene.xml \
  --auto-start \
  --freq 500 \
  --visualize \
  --viz-freq 30
```

### Command-Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--model` | string | None | Path to MuJoCo XML model file |
| `--auto-start` | flag | False | Auto-start simulation with model |
| `--freq` | float | 1000.0 | Simulation frequency in Hz |
| `--visualize` | flag | False | Enable visualization |
| `--viz-freq` | float | 60.0 | Visualization update frequency in Hz |

### Services Provided

#### `/mujoco_init`

**Type:** `crisp_mujoco_sim_msgs/srv/Init`

**Request:**
```yaml
string model_path  # Path to MuJoCo XML model
```

**Response:**
```yaml
float64[] position  # Initial joint positions
float64[] velocity  # Initial joint velocities
float64[] effort    # Initial joint efforts
bool success        # True if initialization succeeded
```

#### `/mujoco_step`

**Type:** `crisp_mujoco_sim_msgs/srv/Step`

**Request:**
```yaml
float64[] effort_commands  # Joint torque commands
```

**Response:**
```yaml
float64[] position  # Updated joint positions
float64[] velocity  # Updated joint velocities
float64[] effort    # Updated joint efforts
```

## Integration with System Interface

### Current Status

The `crisp_mujoco_sim/src/system_interface.cpp` currently uses direct C++ singleton access:

```cpp
// Current implementation (C++ MuJoCo simulator)
MuJoCoSimulator::getInstance().write(m_effort_commands);
MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts);
```

### Required Changes

To use the Python simulator service, modify `system_interface.cpp`:

#### 1. Initialize Service Clients in `on_init()`

```cpp
Simulator::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo & info)
{
  // ... existing code ...
  
  // Create ROS2 node for service clients
  m_node = std::make_shared<rclcpp::Node>("mujoco_sim_client");
  m_init_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Init>("mujoco_init");
  m_step_client = m_node->create_client<crisp_mujoco_sim_msgs::srv::Step>("mujoco_step");
  m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  m_executor->add_node(m_node);
  
  // Wait for Init service
  while (!m_init_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("Simulator"), "Waiting for /mujoco_init service...");
  }
  
  // Call Init service
  auto init_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Init::Request>();
  init_request->model_path = m_mujoco_model;
  
  auto init_future = m_init_client->async_send_request(init_request);
  if (m_executor->spin_until_future_complete(init_future) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = init_future.get();
    if (response->success) {
      // Initialize state from response
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        m_positions[i] = response->position[i];
        m_velocities[i] = response->velocity[i];
        m_efforts[i] = response->effort[i];
      }
      RCLCPP_INFO(rclcpp::get_logger("Simulator"), "MuJoCo simulator initialized via service");
      return CallbackReturn::SUCCESS;
    }
  }
  
  RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Failed to initialize MuJoCo simulator");
  return CallbackReturn::ERROR;
}
```

#### 2. Replace `write()` to Use Step Service

```cpp
Simulator::return_type Simulator::write(const rclcpp::Time & time,
                                        const rclcpp::Duration & period)
{
  // Create Step request
  auto step_request = std::make_shared<crisp_mujoco_sim_msgs::srv::Step::Request>();
  step_request->effort_commands = m_effort_commands;
  
  // Call Step service (synchronous for simplicity)
  auto step_future = m_step_client->async_send_request(step_request);
  
  // Wait for response (with timeout)
  auto status = m_executor->spin_until_future_complete(
    step_future, 
    std::chrono::milliseconds(10)
  );
  
  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = step_future.get();
    
    // Update stored state for next read()
    for (size_t i = 0; i < m_positions.size(); ++i) {
      m_positions[i] = response->position[i];
      m_velocities[i] = response->velocity[i];
      m_efforts[i] = response->effort[i];
    }
    
    return return_type::OK;
  }
  
  RCLCPP_WARN_THROTTLE(
    rclcpp::get_logger("Simulator"), 
    *clock, 
    1000, 
    "Step service call timeout or failed"
  );
  return return_type::ERROR;
}
```

#### 3. Simplify `read()`

```cpp
Simulator::return_type Simulator::read(const rclcpp::Time & time,
                                       const rclcpp::Duration & period)
{
  // State already updated in write() - nothing to do
  return return_type::OK;
}
```

### Testing Integration

#### Terminal 1: Start Python Simulator

```bash
cd ~/avant_robot_interface
source ~/crisp_controllers_demos/install/setup.bash

python examples/mujoco_sim_service_node.py --visualize
```

#### Terminal 2: Launch Controller Manager (After System Interface Modifications)

```bash
cd ~/crisp_controllers_demos
source install/setup.bash

# For single FR3
ros2 launch crisp_controllers_robot_demos franka.launch.py

# For dual FR3
ros2 launch crisp_controllers_robot_demos dual_franka.launch.py
```

## Performance Considerations

### Latency

- **C++ In-Process:** < 1 microsecond
- **Python Service-Based:** ~100-500 microseconds (depends on DDS/message size)

For 1kHz control loops, service call latency is typically acceptable.

### Real-Time Synchronization

The simulator uses **wall-clock synchronization** (Strategy 2):

```python
# In handle_step_request()
target_wall_time = self.start_wall_time + self.sim_time
while time.time() < target_wall_time:
    time.sleep(0.0001)  # 100 microseconds
```

This matches the C++ implementation and provides:
- ✅ No timing drift
- ✅ Self-correcting behavior
- ✅ Good long-term accuracy

## Troubleshooting

### "crisp_mujoco_sim_msgs not found"

```bash
cd ~/crisp_controllers_demos
colcon build --packages-select crisp_mujoco_sim_msgs
source install/setup.bash
```

### "MuJoCoVisualizer not available"

```bash
cd ~/avant_robot_interface
pip install -e .
```

Or run without `--visualize` flag.

### Service Call Timeouts

Check that:
1. Python simulator is running
2. ROS2 environment is sourced in both terminals
3. ROS2 middleware is configured correctly (DDS settings)

List available services:
```bash
ros2 service list
```

You should see:
- `/mujoco_init`
- `/mujoco_step`

### Model Not Found

Use absolute paths or ensure model path is relative to the location where you run the script:

```bash
python examples/mujoco_sim_service_node.py \
  --model "$HOME/crisp_controllers_demos/crisp_controllers_robot_demos/config/fr3/scene.xml" \
  --auto-start
```

## Next Steps

### Phase 1: Standalone Testing ✅ (Complete)
- ✅ Python simulator node implemented
- ✅ Services exposed and tested
- ✅ Visualization working

### Phase 2: System Interface Modification (In Progress)
- [ ] Modify `system_interface.cpp` to use service clients
- [ ] Test service-based communication
- [ ] Verify control loop performance

### Phase 3: Integration Testing
- [ ] Test with joint trajectory controller
- [ ] Test with force controller
- [ ] Test with dual robot setup
- [ ] Compare performance with C++ version

### Phase 4: Production Deployment
- [ ] Create launch files
- [ ] Add error handling and recovery
- [ ] Document deployment procedures
- [ ] Performance benchmarking

## Benefits of This Approach

✅ **Process Isolation** - Simulator crash doesn't kill controller  
✅ **Python Ecosystem** - Easier integration with ML/vision pipelines  
✅ **Easier Debugging** - Can restart simulator independently  
✅ **Cleaner Architecture** - Clear separation of concerns  
✅ **Flexibility** - Can swap simulators without recompiling C++  

## References

- [Architecture Diagrams](../../crisp_controllers_demos/crisp_mujoco_sim/docs/architecture_diagrams.md)
- [Service Definitions](../../crisp_controllers_demos/crisp_mujoco_sim_msgs/srv/)
- [MuJoCo Python Documentation](https://mujoco.readthedocs.io/en/stable/python.html)
- [ROS2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
