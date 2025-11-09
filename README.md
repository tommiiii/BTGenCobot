# BTGenCobot

Generate robot behavior trees from natural language using an on-robot LLM.

## What It Does

BTGenCobot converts natural language commands like "Navigate to the kitchen and say hello" into valid BehaviorTree.CPP XML that executes on your robot. The LLM runs directly as a ROS2 node on the robot, enabling autonomous behavior tree generation.

## Quick Start

```bash
# Build and start the container
docker-compose up --build

# Inside the container, build the workspace
colcon build --symlink-install
source install/setup.bash

# Launch Gazebo simulation with the mobile manipulator
ros2 launch mobile_manipulator gazebo.launch.py

# In another terminal (or tmux pane), run Nav2
ros2 launch bt_bringup nav2_bringup.launch.py

# Run the BT generator node
ros2 run bt_generator bt_generator_node

# Send commands via ROS2 topic
ros2 topic pub /user_command std_msgs/String "data: 'Go to the kitchen and say hello'"
```

**Access VNC Desktop**: Open http://localhost:6080 in your browser (password: `vncpassword`)

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
Gazebo Simulation (mobile manipulator)
```

## Project Structure

```
BTGenCobot/                     # ROS2 Workspace Root
├── src/                        # ROS2 Packages
│   ├── bt_generator/           # LLM BT generation node
│   │   ├── bt_generator/       # Python module
│   │   │   ├── __init__.py
│   │   │   └── bt_generator_node.py
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
│   └── bt_nav2_plugins/        # Custom BT action plugins
│       ├── include/bt_nav2_plugins/
│       ├── src/                # Plugin implementations (SayText, etc.)
│       ├── plugins/            # Plugin XML descriptors
│       ├── CMakeLists.txt
│       └── package.xml
│
├── robot_description/          # Robot URDF descriptions
│   └── mobile_manipulator/     # Mobile base + OpenManipulator-X arm
│       ├── urdf/               # URDF and xacro files
│       ├── launch/             # Launch files for Gazebo and Nav2
│       ├── config/             # Nav2 and controller configs
│       ├── worlds/             # Gazebo world files
│       ├── behavior_trees/     # Example BT XML files
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
│
├── models/                     # Non-ROS: LLM model data
│   └── btgenbot2/
│       ├── models/             # Model weights and config
│       └── README.md
│
├── Dockerfile                  # All-in-one container (ROS2 + Gazebo + Nav2 + LLM)
├── docker-compose.yml          # Single container setup
├── README.md
├── SETUP.md                    # Detailed setup instructions
└── TESTING_GUIDE.md            # Testing guide for OpenManipulator-X
```

## Components

- **bt_generator**: ROS2 Python node running Llama 3.2-1B with outlines for structured BT XML generation
- **bt_bringup**: System-level launch files and configurations for Nav2, SLAM, and AMCL
- **bt_nav2_plugins**: Custom BehaviorTree.CPP plugins compiled for Nav2
- **mobile_manipulator**: Custom mobile base with OpenManipulator-X arm (4-DOF + gripper)
- **Nav2**: Standard ROS2 navigation stack with BT execution
- **Gazebo Harmonic**: Robot simulation environment

## Tech Stack

- **ROS2 Jazzy** - Robot Operating System
- **PyTorch** - Llama 3.2-1B inference
- **outlines** - Structured generation with JSON schema
- **Nav2** - Navigation stack with BehaviorTree.CPP
- **Gazebo Harmonic** - Robot simulator with ros2_control integration
- **Docker** - Single container with all dependencies

## Development Status

🚧 **Under Active Development** - See ROADMAP.md for implementation plan

## License

MIT
