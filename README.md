# BTGenCobot

Natural language to BehaviorTree XML generation for ROS 2 mobile manipulators,
with persistent semantic navigation backed by MIT-SPARK Hydra.

## Overview

BTGenCobot takes commands such as "pick up the red cup and place it on the
table" and generates valid BehaviorTree.CPP XML for execution on Nav2. A
fine-tuned Llama 3.2-1B model uses runtime grammar-constrained decoding.
Generated `NavigateSemantic` nodes contain free-label references such as
`entity_ref="room:kitchen"` or `entity_ref="object:red cup"`. Immediately
before execution, the ROS interface resolves those references through the
persistent scene graph built by the official MIT-SPARK Hydra implementation.

## Requirements

- Docker and Docker Compose
- Apple Silicon or an NVIDIA GPU is optional; all inference paths have CPU
  fallbacks

## Usage

```bash
# One-time official Hydra build (large research dependency stack)
docker compose build hydra

# Start the simulation and Hydra services
docker compose up -d btgencobot hydra

# Start the inference server on the host, not in Docker
cd inference_server && uv run serve
```

### Web supervision frontend

The companion frontend in `../tesi.triennale.CobotV2/frontend` is integrated
with the normal `house_pick_and_place` bringup. Start it on the host with
`npm ci && npm run dev`, open the displayed local URL, and connect the dashboard
to the Foxglove bridge. Commands use the host inference API on port 8080, while
the camera, metric map, Behavior Tree state, execution log, and Hydra semantics
arrive through Foxglove on port 8765.

The Hydra adapter publishes `/hydra/scene_graph_snapshot`, a compact transient
snapshot containing room/object nodes, membership edges, and positions in the
ROS `map` frame. The dashboard overlays those nodes directly on `/map`; raw DSG,
mesh, and visualizer topics remain filtered from Foxglove to avoid burdening the
operator connection.

### Automated mapping and scene-graph construction

Run this in the `btgencobot` container:

```bash
source /workspace/install/setup.bash
ros2 launch bt_bringup robot_bt_mapping.launch.py \
  environment:=house_pick_and_place \
  use_rviz:=true \
  autonomous_exploration:=true \
  exploration_speed_multiplier:=1.0
```

The frontier explorer drives the robot without teleoperation, holds a calibrated
forward/downward head pose, pauses at viewpoints while semantic observations are
captured, and then runs a camera-coverage pass after lidar frontiers are
exhausted. Hydra treats camera extrinsics as rigid, so mapping deliberately gets
additional views by moving and rotating the base rather than actuating TIAGo's
head. RGB-D publication is interlocked until the torso and head reach that
calibrated posture. The coverage selector only chooses viewpoints in the
robot-connected free-space component, preventing repeated navigation timeouts
to disconnected map islands.

The visual taxonomy preserves Hydra's upstream ADE20K indoor grouping. It
retains actionable categories such as tables, chairs, beds, shelves, storage,
couches, lights, and appliances while mapping visually unstable fine classes
into robust structural or generic groups. As in MIT-SPARK's
`semantic_inference`, SegFormer selects the ADE20K class first and that label ID
is then recolored into Hydra's compact indoor space. Hydra's reconstructed 3D
mesh—not a project-specific 2D depth heuristic—determines which semantic
regions become objects. There is no house-specific remap or task object list.
The multiplier is capped at 2.5 and normal limits are restored when exploration
stops.

After exploration reports completion, save both persistent artifacts in one
operation:

```bash
ros2 run semantic_exploration save_mapping_state \
  --map-name /workspace/maps/house_pick_and_place
```

This writes the SLAM map under `maps/` and the DSG under
`hydra_data/house_pick_and_place/`. The Hydra adapter loads the saved DSG on
future runs and republishes one complete, latched DSG snapshot for the Hydra
visualizer. Saving is transactional. A candidate with a systematic floating
furniture geometry error is rejected instead of replacing the last usable DSG;
near-identical same-class Hydra segments are collapsed in the canonical graph.

In Foxglove, add `/hydra_visualizer/graph` as a `MarkerArray` in the 3D panel
and use `map` as the fixed frame. The `dynamic_objects` topic is only for
classes configured as dynamic (currently humans and animals); ordinary
furniture and task objects are rendered by the main `graph` topic.

### Semantic task execution

Start the normal saved-map bringup in the simulation container, then send
commands:

```bash
ros2 launch bt_bringup robot_bt_bringup.launch.py \
  environment:=house_pick_and_place

ros2 topic pub /btgen_nl_command std_msgs/String \
  "data: 'pick up the red cup'"
```

Commands that arrive before the Hydra graph is ready remain queued, publish
`queued` feedback, and resume after the graph becomes resolvable. A remembered
object is approached through Hydra/Nav2, locally reacquired once with vision,
and its detected pose is passed directly into `PickObject`.

## Architecture

```text
Natural-language command
          |
          v
Query Rewriter (OpenRouter API w/ Local Fallback)
          |
          v
Local SLM + runtime constrained grammar
          |
          v
BT XML with free-label semantic references
          |
          v
Foxglove WebSocket Bridge (ros_bridge)
          |
          v
bt_text_interface resolves references  <-- Supervised by Web Frontend
          |
          +------> Official MIT-SPARK Hydra persistent DSG
          |                         |
          v                         v
BehaviorTree.CPP execution <---- Nav2 metric route
          |
          v
Vision-supported manipulation
```

Hydra is deliberately built in a companion image from exact pinned upstream
commits. The image contains the official `Hydra`, `Hydra-ROS`, `Spark-DSG`, and
their upstream dependencies; project code is limited to ROS interfaces and a
thin DSG-to-Nav2 adapter. Both containers use ROS domain 42 and Fast DDS over
UDP, so they appear in one ROS graph while keeping Hydra's dependency stack
isolated from the simulator.

## Project Structure

```text
BTGenCobot/
├── inference_server/              # Query rewriter, local SLM inference, Foxglove bridge
├── src/
│   ├── bt_text_interface/         # BT generation/execution action server
│   ├── bt_nav2_plugins/           # Detect, pick, place, and motion BT nodes
│   ├── bt_bringup/                # Simulation, frontend relays, SLAM, and Nav2 launches
│   ├── hydra_semantic_navigation/ # Thin official-Hydra/Nav2 adapter
│   ├── semantic_exploration/      # Frontier exploration and live-view fallback
│   ├── manipulator_control/       # Arm IK and control service
│   ├── vision_services/           # SegFormer semantics + GroundingDINO
│   └── btgencobot_interfaces/     # Custom ROS 2 messages and services
├── hydra/                         # Pinned official upstream repositories
├── Dockerfile.hydra               # Companion official-Hydra image
└── tiago_harmonic/                 # TIAGo robot dependencies
```

## Custom BT Nodes

| Node | Description |
|------|-------------|
| `DetectObject` | Open-vocabulary object detection via GroundingDINO |
| `PickObject` | Grasp the supplied detection pose, with vision fallback if absent |
| `PlaceObject` | Place a held object at a visually detected location |
| `SpinLeft` / `SpinRight` | Rotate in place |

## License

MIT
