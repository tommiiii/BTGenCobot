# BTGenCobot

Generate robot behavior trees from natural language using an on-robot LLM.

## What It Does

BTGenCobot converts natural language commands like "Navigate to the kitchen and say hello" into valid BehaviorTree.CPP XML that executes on your robot. The LLM runs directly as a ROS2 node on the robot, enabling autonomous behavior tree generation.

## Quick Start

```bash
# Build and start the main container (native architecture)
docker-compose up --build

# Optional: Start with CoppeliaSim (x86 only)
docker-compose --profile coppelia up --build

# Optional: Start with Gazebo instead
docker-compose --profile gazebo up --build

# Inside the container, build the workspace
colcon build --symlink-install

# Run the system
ros2 launch bt_bringup robot_bt_bringup.launch.py

# Send commands via ROS2 topic
ros2 topic pub /user_command std_msgs/String "data: 'Go to the kitchen and say hello'"
```

## Architecture

```
User Input (text)
    ↓
ROS2 Topic: /user_command
    ↓
bt_generator_node (ROS2)
    ├─ Llama 3.2-1B (PyTorch)
    ├─ outlines (structured generation)
    └─ BT XML Schema validation
    ↓
Generated BT XML
    ↓
Nav2 bt_navigator
    ├─ Custom BT plugins
    └─ Standard Nav2 plugins
    ↓
CoppeliaSim Robot
```

## Project Structure

```
BTGenCobot/                     # ROS2 Workspace Root
├── src/                        # ROS2 Packages
│   ├── bt_generator/           # LLM BT generation node
│   │   ├── bt_generator/       # Python module
│   │   │   ├── __init__.py
│   │   │   └── bt_generator_node.py
│   │   ├── config/             # Generator parameters (future)
│   │   ├── test/               # Unit tests
│   │   ├── resource/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── bt_bringup/             # System launch and configuration
│   │   ├── launch/             # Launch files (Nav2, SLAM, full system)
│   │   ├── config/             # Nav2, SLAM, AMCL configs
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── bt_description/         # Robot description (URDF, meshes)
│   │   ├── urdf/               # Robot URDF files
│   │   ├── meshes/             # 3D models
│   │   ├── config/             # Robot-specific configs
│   │   ├── launch/             # Description launchers
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── bt_nav2_plugins/        # Custom BT action plugins
│   │   ├── include/bt_nav2_plugins/
│   │   ├── src/                # Plugin implementations (SayText, etc.)
│   │   ├── plugins/            # Plugin XML descriptors
│   │   ├── test/               # Plugin tests
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── bt_interfaces/          # Custom ROS2 messages/services
│       ├── msg/                # Custom message definitions
│       ├── srv/                # Custom service definitions
│       ├── action/             # Custom action definitions
│       ├── CMakeLists.txt
│       └── package.xml
│
├── models/                     # Non-ROS: LLM model data
│   └── llama_bt_generator/
│       ├── models/             # Model weights (*.safetensors)
│       ├── prompts/            # LLM prompt templates
│       ├── schemas/            # JSON schemas for structured output
│       ├── bt_examples/        # Example BT XML files
│       ├── behavior_constraints/  # Safety constraints
│       ├── examples/           # Training examples
│       ├── outputs/            # Generated outputs (gitignored)
│       └── README.md
│
├── Dockerfile                  # Main container (ROS2 + LLM)
├── Dockerfile.coppelia         # CoppeliaSim container (x86 only)
├── Dockerfile.gazebo           # Gazebo container (alternative)
├── docker-compose.yml          # Multi-container orchestration
└── README.md
```

## Components

- **bt_generator**: ROS2 Python node running Llama 3.2-1B with outlines for structured BT XML generation
- **bt_bringup**: System-level launch files and configurations for Nav2, SLAM, and AMCL
- **bt_description**: Robot URDF, meshes, and visualization configurations
- **bt_nav2_plugins**: Custom BehaviorTree.CPP plugins compiled for Nav2
- **bt_interfaces**: Custom ROS2 message and service definitions
- **Nav2**: Standard ROS2 navigation stack with BT execution
- **CoppeliaSim**: Robot simulation environment (x86 container)
- **Gazebo**: Alternative simulation (optional, native architecture)

## Tech Stack

- **ROS2 Jazzy** - Robot Operating System
- **PyTorch** - Llama 3.2-1B inference
- **outlines** - Structured generation with JSON schema
- **Nav2** - Navigation stack with BehaviorTree.CPP
- **CoppeliaSim** - Robot simulator (x86 container)
- **Gazebo** - Alternative simulator (native architecture)
- **Docker** - Multi-container deployment with host networking

## Development Status

🚧 **Under Active Development** - See ROADMAP.md for implementation plan

## License

MIT
