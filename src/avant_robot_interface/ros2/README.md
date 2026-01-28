# ROS2 Integration Module

This module provides thread-safe ROS2 integration for the avant_robot_interface control system.

## Architecture

```
┌─────────────────────────────────────────────┐
│   Main Thread (Control Loop)                │
│   Runs at 1000 Hz                           │
│   ┌─────────────────────────────────────┐   │
│   │  • Read sensors                     │   │
│   │  • Get ROS2 commands (via handlers) │◄──┼──┐
│   │  • Control computation              │   │  │
│   │  • Write commands to hardware       │   │  │
│   │  • Update ROS2 state (via handlers) │──►┼──┤
│   └─────────────────────────────────────┘   │  │
└─────────────────────────────────────────────┘  │
                                                 │
         Thread-safe handlers with locks        │
                                                 │
┌─────────────────────────────────────────────┐  │
│   ROS2 Thread                               │  │
│   Runs rclpy.spin()                         │  │
│   ┌─────────────────────────────────────┐   │  │
│   │  • Subscribers receive messages     │───┼──┘
│   │  • Store in handlers (thread-safe)  │   │
│   │  • Publishers read from handlers    │◄──┼──┘
│   │  • Publish messages                 │   │
│   └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

## Key Design Principles

1. **Thread Separation**: ROS2 runs in its own thread, separate from the real-time control loop
2. **Thread-Safe Communication**: All data exchange uses locks to prevent race conditions
3. **Non-Blocking**: Control loop never waits for ROS2 operations
4. **Decoupled**: ROS2 is optional and can be disabled without affecting control loop

## Module Structure

```
ros2/
├── __init__.py          # Package exports
├── handlers.py          # Thread-safe data handlers
├── subscribers.py       # ROS2 subscriber definitions
├── publishers.py        # ROS2 publisher definitions
├── node.py             # Main ROS2 node wrapper
└── README.md           # This file
```

## File Descriptions

### `handlers.py`
Thread-safe classes for exchanging data between ROS2 and control threads:
- `JointCommandHandler`: Receives commands from ROS2 subscribers
- `RobotStatePublisher`: Provides state data to ROS2 publishers

### `subscribers.py`
ROS2 subscriber definitions:
- `JointCommandSubscriber`: Subscribes to joint commands
- `create_subscribers()`: Factory function to create all subscribers

### `publishers.py`
ROS2 publisher definitions:
- `JointStatePublisherNode`: Publishes robot joint states
- `create_publishers()`: Factory function to create all publishers

### `node.py`
Main ROS2 node wrapper:
- `ROS2Node`: Manages ROS2 lifecycle in separate thread
- `create_ros2_node()`: Convenience function for initialization

## Usage

### Basic Usage

```python
from avant_robot_interface.ros2 import create_ros2_node

# Create and start ROS2 node
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
ros2_node = create_ros2_node(
    node_name='my_robot_node',
    joint_names=joint_names
)

# In your control loop (main thread at 1000 Hz):
while running:
    # Get latest command from ROS2 (non-blocking)
    joint_cmd = ros2_node.get_joint_command(max_age_s=1.0)
    if joint_cmd is not None:
        # Use the command
        target_position = joint_cmd
    
    # ... perform control computation ...
    
    # Publish robot state to ROS2
    ros2_node.publish_robot_state(
        positions=current_positions,
        velocities=current_velocities,
        efforts=current_efforts
    )
    
# Shutdown ROS2 when done
ros2_node.shutdown()
```

### Integration with simple_control_loop.py

```python
from avant_robot_interface.core.simple_control_loop import MultiRateControlLoop

# Enable ROS2 integration
control_loop = MultiRateControlLoop(
    enable_ros2=True,
    joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
)

# Run the control loop
control_loop.run(duration_s=10.0)
```

## ROS2 Topics

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/joint_command` | `sensor_msgs/JointState` | Joint position commands |

### Published Topics

| Topic | Message Type | Description | Rate |
|-------|-------------|-------------|------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint states | 100 Hz |

## Testing

### 1. Run the Example

```bash
# Terminal 1: Run the control loop with ROS2
python examples/ros2_control_example.py
```

### 2. Monitor Published Topics

```bash
# Terminal 2: Monitor joint states
ros2 topic echo /joint_states
```

### 3. Send Commands

```bash
# Terminal 3: Send a joint command
ros2 topic pub /joint_command sensor_msgs/msg/JointState \
  '{position: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]}'
```

### 4. List Available Topics

```bash
ros2 topic list
```

### 5. Check Topic Info

```bash
ros2 topic info /joint_states
ros2 topic info /joint_command
```

## Extending the Module

### Adding a New Subscriber

1. **Add handler in `handlers.py`:**
```python
class MyCustomHandler:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
    
    def update(self, msg):
        with self._lock:
            self._data = msg
    
    def get_latest(self):
        with self._lock:
            return self._data
```

2. **Add subscriber in `subscribers.py`:**
```python
class MyCustomSubscriber:
    def __init__(self, node, handler, topic='/my_topic'):
        self.handler = handler
        self.subscriber = node.create_subscription(
            MyMessageType,
            topic,
            self._callback,
            10
        )
    
    def _callback(self, msg):
        self.handler.update(msg)
```

3. **Register in `create_subscribers()`:**
```python
if 'my_custom' in handlers:
    subscribers['my_custom'] = MyCustomSubscriber(
        node=node,
        handler=handlers['my_custom'],
        topic='/my_topic'
    )
```

### Adding a New Publisher

Similar process - add handler, publisher class, and register in `create_publishers()`.

## Thread Safety

All handlers use `threading.Lock()` to ensure thread-safe access:
- Control loop (main thread) reads/writes at 1000 Hz
- ROS2 callbacks write whenever messages arrive
- Publishers read at their publishing rate (e.g., 100 Hz)

Example from `JointCommandHandler`:
```python
def get_latest(self, max_age_s: float = 1.0):
    with self._lock:  # Acquire lock
        if self._latest_msg is None:
            return None
        
        age = time.time() - self._timestamp
        if age > max_age_s:
            return None
        
        return self._latest_msg
    # Lock automatically released
```

## Performance Considerations

1. **Non-Blocking**: Control loop never blocks waiting for ROS2
2. **Message Age**: Old commands are automatically discarded
3. **Minimal Overhead**: Thread-safe handlers use fast locks
4. **Separate Thread**: ROS2 spin runs independently
5. **Decimation**: Publishers can run at lower rates (e.g., 100 Hz vs 1000 Hz)

## Troubleshooting

### ROS2 not available
```
⚠️ ROS2 not available: No module named 'rclpy'
```
**Solution**: Install ROS2 and source the setup file:
```bash
source /opt/ros/humble/setup.bash  # Or your ROS2 distro
```

### Topic not publishing
**Check**:
1. Is ROS2 node running? `ros2 node list`
2. Are topics available? `ros2 topic list`
3. Is data being updated? Check handlers

### Stale commands
If commands are being ignored, check the `max_age_s` parameter:
```python
# Allow older messages (default is 1.0 second)
cmd = ros2_node.get_joint_command(max_age_s=5.0)
```

## Summary

This ROS2 integration module provides:
- ✅ Thread-safe communication between ROS2 and control loop
- ✅ Non-blocking, real-time safe design
- ✅ Easy to use API
- ✅ Optional integration (can be disabled)
- ✅ Extensible architecture for adding new topics
- ✅ Proper cleanup and shutdown handling

For more examples, see `examples/ros2_control_example.py` and `core/simple_control_loop.py`.
