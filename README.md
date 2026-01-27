avant-motion-controller/
├── README.md
├── pyproject.toml
├── requirements.txt
│
├── src/
│   └── avant_controller/
│       │
│       ├── core/                          # Hardware-agnostic control (NO viz)
│       │   ├── __init__.py
│       │   ├── control_loop.py
│       │   ├── runtime.py
│       │   └── config.py
│       │
│       ├── hardware/                      # Hardware abstraction
│       │   ├── __init__.py
│       │   ├── interface.py               # Abstract base
│       │   ├── simulation.py              # MuJoCo (includes viz option)
│       │   └── robot.py                   # Real hardware
│       │
│       ├── visualization/                 # ← VISUALIZATION HERE
│       │   ├── __init__.py
│       │   ├── visualizer.py              # Abstract visualizer
│       │   ├── mujoco_viewer.py           # MuJoCo visualizer
│       │   ├── rviz_publisher.py          # RViz/ROS2 visualizer
│       │   └── plotter.py                 # Data plotting
│       │
│       ├── robots/                        # ← NEW ROBOTS HERE
│       │   ├── __init__.py
│       │   ├── robot_config.py            # Robot configuration class
│       │   ├── raven/
│       │   │   ├── __init__.py
│       │   │   ├── config.py              # Raven-specific config
│       │   │   └── calibration.py         # Calibration data
│       │   ├── ur5/
│       │   │   ├── __init__.py
│       │   │   └── config.py
│       │   └── franka/
│       │       ├── __init__.py
│       │       └── config.py
│       │
│       ├── interfaces/                    # Command handlers
│       ├── controllers/                   # Control algorithms
│       ├── estimation/                    # State estimation
│       ├── planning/                      # Trajectory planning
│       ├── dynamics/                      # Kinodynamics
│       └── ros2/                          # ROS2 integration
│
├── assets/                                # ← ROBOT MODELS HERE
│   ├── raven/
│   │   ├── raven_full_dyn.xml            # MuJoCo model
│   │   ├── raven.urdf                     # URDF model
│   │   ├── presets.yaml                   # Robot presets
│   │   └── meshes/                        # 3D meshes
│   │       ├── L_0.STL
│   │       └── ...
│   ├── ur5/
│   │   ├── ur5.xml
│   │   ├── ur5.urdf
│   │   └── meshes/
│   └── franka/
│       ├── franka.xml
│       ├── franka.urdf
│       └── meshes/
│
├── configs/                               # ← ROBOT CONFIGS HERE
│   ├── robots/
│   │   ├── raven.yaml                    # Raven configuration
│   │   ├── ur5.yaml                      # UR5 configuration
│   │   └── franka.yaml                   # Franka configuration
│   ├── controllers/
│   │   ├── position_default.yaml
│   │   └── impedance_default.yaml
│   └── simulation.yaml                    # General sim settings
│
├── examples/
│   ├── run_raven_sim.py                  # Run Raven in simulation
│   ├── run_ur5_hardware.py               # Run UR5 on hardware
│   └── run_custom_robot.py               # Custom robot example
│
└── tests/
    ├── test_visualization.py
    └── test_robot_configs.py
