# BTGenCobot

Natural language to BehaviorTree XML generation for ROS2 mobile manipulators.

## Overview

BTGenCobot takes commands like "pick up the red cup and place it on the table" and generates valid BehaviorTree.CPP XML for execution on Nav2. Uses a fine-tuned Llama 3.2-1B model with grammar-constrained decoding to ensure syntactically correct output.

## Requirements

- Docker & Docker Compose
- NVIDIA GPU (optional, for faster inference)

## Usage

```bash
docker-compose up --build

# In the container
colcon build --symlink-install
source install/setup.bash

# Start simulation
ros2 launch turtlebot3_manipulation_description gazebo.launch.py

# Start Nav2
ros2 launch bt_bringup nav2_bringup.launch.py

# Start inference server (on host or in container)
cd inference_server && uv run serve

# Send commands
ros2 topic pub /btgen_nl_command std_msgs/String "data: 'pick up the red cup'"
```

## Architecture

```
Natural Language Command
         │
         ▼
┌─────────────────────────┐
│   Inference Server      │
│   (FastAPI + Llama 3.2) │
│   + EBNF Grammar CFG    │
└───────────┬─────────────┘
            │
            ▼
      BT XML Output
            │
            ▼
┌─────────────────────────┐
│  bt_text_interface      │
│  (ROS2 Action Server)   │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│  Nav2 bt_navigator      │
│  + Custom BT Plugins    │
└───────────┬─────────────┘
            │
            ▼
      Robot Execution
```

## Project Structure

```
BTGenCobot/
├── inference_server/           # LLM inference (FastAPI)
│   ├── api/                    # REST endpoints
│   ├── core/                   # Model loading, generation
│   ├── validation/             # XML validation, post-processing
│   └── prompts/                # System prompts
│
├── src/
│   ├── bt_text_interface/      # ROS2 action server for BT generation
│   ├── bt_nav2_plugins/        # Custom Nav2 BT nodes (DetectObject, Pick, Place, Spin)
│   ├── bt_bringup/             # Launch files and Nav2 config
│   ├── manipulator_control/    # Arm IK and control service
│   ├── vision_services/        # Florence-2 object detection
│   └── btgencobot_interfaces/  # Custom ROS2 messages/services
│
└── robot_description/          # TurtleBot3 + OpenManipulator-X URDF
```

## Custom BT Nodes

| Node | Description |
|------|-------------|
| `DetectObject` | Open-vocabulary object detection via Florence-2 |
| `PickObject` | Approach and grasp detected object |
| `PlaceObject` | Place held object at detected location |
| `SpinLeft/SpinRight` | Rotate in place |

## License

MIT
