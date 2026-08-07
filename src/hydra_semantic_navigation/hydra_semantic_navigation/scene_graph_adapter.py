"""ROS 2 adapter between Hydra's DSG and semantic BT navigation."""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from hydra_msgs.msg import DsgUpdate
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros
import yaml

import spark_dsg as dsg

from btgencobot_interfaces.msg import SceneGraphStatus, SemanticObjectObservation
from btgencobot_interfaces.srv import ResolveSemanticNavigation

from .core import (
    Place,
    SemanticEntity,
    choose_unambiguous_match,
    infer_room_label,
    merge_nearby_entities,
    nearest_place,
    normalize_label,
    parse_entity_ref,
    rank_entities,
    rank_matches_from_position,
    route_between_nearest_places,
    simplify_route,
    support_surface_height,
)


STATUS_OK = 0
STATUS_GRAPH_NOT_READY = 1
STATUS_UNKNOWN_DESTINATION = 2
STATUS_AMBIGUOUS_DESTINATION = 3
STATUS_STALE_DESTINATION = 4
STATUS_NO_TRAVERSABLE_PLACE = 5
STATUS_UNREACHABLE = 6
STATUS_INTERNAL_ERROR = 7


def _node_id(value) -> str:
    if hasattr(value, "str"):
        return value.str()
    return dsg.NodeSymbol(value).str()


def _position_tuple(attributes) -> tuple[float, float, float]:
    value = attributes.position
    return float(value[0]), float(value[1]), float(value[2])


def _bounds_tuple(attributes):
    """Return Hydra's world-aligned object bounds when available."""
    try:
        bounds = attributes.bounding_box
        if not bounds.is_valid():
            return None, None
        return (
            tuple(float(value) for value in bounds.min),
            tuple(float(value) for value in bounds.max),
        )
    except Exception:
        return None, None


class SceneGraphAdapter(Node):
    def __init__(self) -> None:
        super().__init__("hydra_semantic_navigation")
        self.declare_parameter("dsg_topic", "/hydra/backend/live_dsg")
        self.declare_parameter(
            "published_dsg_topic", "/hydra/backend/dsg"
        )
        self.declare_parameter(
            "persistence_path",
            "/data/house_pick_and_place/backend_dsg.json",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("navigation_map_topic", "/map")
        self.declare_parameter("navigation_goal_clearance_m", 0.30)
        self.declare_parameter("navigation_goal_projection_radius_m", 1.25)
        self.declare_parameter("navigation_occupied_threshold", 65)
        self.declare_parameter("min_place_clearance", 0.32)
        # TIAGo's arm can reach floor objects reliably from roughly half a
        # metre away. Keep semantic object goals outside the base footprint,
        # but do not select the navigation-only 0.8 m places that leave a low
        # grasp outside the arm workspace.
        self.declare_parameter("object_standoff_min", 0.45)
        self.declare_parameter("object_standoff_max", 0.85)
        # Persisted reconstructions can preserve an object's dimensions while
        # carrying a bad absolute Z offset.  A compact support whose entire
        # AABB floats well above the navigation floor is re-anchored by height;
        # genuinely tall/stacked geometry is left untouched and will fail
        # reachability instead of being silently moved.
        self.declare_parameter("support_bounds_max_floor_gap_m", 0.25)
        self.declare_parameter("support_bounds_max_reanchor_height_m", 1.20)
        self.declare_parameter("minimum_semantic_match_score", 0.72)
        self.declare_parameter("object_stale_after_sec", 3600.0)
        self.declare_parameter("emit_intermediate_waypoints", False)
        self.declare_parameter("assume_mapping_complete", False)
        self.declare_parameter("freeze_persisted_graph", True)
        self.declare_parameter("max_object_count", 300)
        self.declare_parameter("max_objects_per_label", 80)
        self.declare_parameter("min_place_count", 20)
        self.declare_parameter("min_largest_place_component_ratio", 0.35)
        self.declare_parameter(
            "forbidden_object_labels",
            ["door", "rail", "window", "decor", "light"],
        )
        self.declare_parameter(
            "room_grounding_config",
            str(
                Path(get_package_share_directory("hydra_semantic_navigation"))
                / "config"
                / "room_grounding.yaml"
            ),
        )
        self.declare_parameter(
            "semantic_label_space",
            str(
                Path(get_package_share_directory("hydra_semantic_navigation"))
                / "config"
                / "tiago_label_space.yaml"
            ),
        )

        self._persistence_path = Path(
            self.get_parameter("persistence_path").value
        )
        self._observation_path = self._persistence_path.with_suffix(
            ".observations.json"
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._lock = threading.RLock()
        self._graph = None
        self._graph_version = 0
        self._loaded_from_disk = False
        self._accept_live_updates = True
        self._mapping_complete = bool(
            self.get_parameter("assume_mapping_complete").value
        )
        self._last_update_monotonic = 0.0
        self._update_times: list[float] = []
        self._last_error = ""
        self._navigation_map: OccupancyGrid | None = None
        self._semantic_observations: list[dict] = []
        self._room_aliases: dict[str, str] = {}
        self._room_rules: dict[str, list[str]] = {}
        self._semantic_label_names: dict[int, str] = {}
        self._load_grounding_config()
        self._load_semantic_label_space()
        self._load_persistent_state()

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3600.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        dsg_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # The official Hydra-ROS DsgSender publishes complete, repeated
            # snapshots with volatile durability.
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._status_pub = self.create_publisher(
            SceneGraphStatus, "/hydra/scene_graph_status", latched
        )
        self._known_entities_pub = self.create_publisher(
            String, "/hydra/known_entities", latched
        )
        self._dsg_pub = self.create_publisher(
            DsgUpdate,
            str(self.get_parameter("published_dsg_topic").value),
            latched,
        )
        self.create_subscription(
            DsgUpdate,
            str(self.get_parameter("dsg_topic").value),
            self._dsg_callback,
            dsg_qos,
        )
        self.create_subscription(
            Bool,
            "/mapping/exploration_complete",
            self._mapping_complete_callback,
            latched,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("navigation_map_topic").value),
            self._navigation_map_callback,
            latched,
        )
        self.create_subscription(
            SemanticObjectObservation,
            "/hydra/semantic_object_observations",
            self._semantic_observation_callback,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._restore_timer = None
        if self._loaded_from_disk:
            # Publish after node construction. Transient-local durability keeps
            # this one snapshot available when the visualizer connects later.
            self._restore_timer = self.create_timer(
                0.5, self._publish_persisted_graph
            )
        self.create_service(
            ResolveSemanticNavigation,
            "/hydra/resolve_semantic_navigation",
            self._resolve_callback,
        )
        self.create_service(
            Trigger,
            "/hydra/save_scene_graph",
            self._save_callback,
        )
        self.create_service(
            Trigger,
            "/hydra/prepare_scene_graph_save",
            self._prepare_save_callback,
        )
        self.create_service(
            Trigger,
            "/hydra/commit_scene_graph_save",
            self._commit_save_callback,
        )
        self.create_service(
            Trigger,
            "/hydra/abort_scene_graph_save",
            self._abort_save_callback,
        )
        self.create_service(
            Trigger,
            "/hydra/validate_scene_graph",
            self._validate_callback,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Hydra semantic adapter ready; persistence={self._persistence_path}"
        )

    def _load_grounding_config(self) -> None:
        config_path = Path(self.get_parameter("room_grounding_config").value)
        try:
            with config_path.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            self._room_aliases = {
                normalize_label(key): normalize_label(value)
                for key, value in (config.get("room_aliases") or {}).items()
            }
            self._room_rules = config.get("room_evidence") or {}
        except Exception as exc:
            self._last_error = f"room grounding config: {exc}"
            self.get_logger().warning(self._last_error)

    def _load_semantic_label_space(self) -> None:
        config_path = Path(self.get_parameter("semantic_label_space").value)
        try:
            with config_path.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            self._semantic_label_names = {
                int(entry["label"]): normalize_label(entry["name"])
                for entry in (config.get("label_names") or [])
                if "label" in entry and normalize_label(entry.get("name", ""))
            }
        except Exception as exc:
            self._last_error = f"semantic label space: {exc}"
            self.get_logger().warning(self._last_error)

    def _load_persistent_state(self) -> None:
        if self._persistence_path.exists():
            try:
                self._graph = dsg.DynamicSceneGraph.load(
                    str(self._persistence_path)
                )
                self._graph_version = 1
                self._loaded_from_disk = True
                self._accept_live_updates = not bool(
                    self.get_parameter("freeze_persisted_graph").value
                )
                self._mapping_complete = True
                self.get_logger().info(
                    f"Loaded persisted DSG from {self._persistence_path}"
                )
            except Exception as exc:
                self._last_error = f"failed to load persisted DSG: {exc}"
                self.get_logger().error(self._last_error)
        if self._observation_path.exists():
            try:
                self._semantic_observations = json.loads(
                    self._observation_path.read_text(encoding="utf-8")
                )
            except Exception as exc:
                self._last_error = f"failed to load semantic observations: {exc}"
                self.get_logger().warning(self._last_error)

    def _publish_persisted_graph(self) -> None:
        if self._restore_timer is not None:
            self._restore_timer.cancel()
        try:
            with self._lock:
                if self._graph is None or not self._loaded_from_disk:
                    return
                contents = self._graph.to_binary()
                sequence_number = self._graph_version
            msg = DsgUpdate()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.layer_contents = contents
            msg.full_update = True
            msg.sequence_number = sequence_number
            self._dsg_pub.publish(msg)
            self.get_logger().info(
                "Published restored DSG snapshot for visualization"
            )
        except Exception as exc:
            self._last_error = f"failed to publish persisted DSG: {exc}"
            self.get_logger().error(self._last_error)

    def _dsg_callback(self, msg: DsgUpdate) -> None:
        if not self._accept_live_updates:
            return
        try:
            contents = msg.layer_contents.tobytes()
            with self._lock:
                if self._graph is None:
                    self._graph = dsg.DynamicSceneGraph.from_binary(contents)
                else:
                    self._graph.update_from_binary(contents)
                for raw_node_id in msg.deleted_nodes:
                    try:
                        self._graph.remove_node(dsg.NodeSymbol(raw_node_id))
                    except Exception:
                        pass
                self._graph_version = max(
                    self._graph_version + 1,
                    int(msg.sequence_number) + 1,
                )
                now = time.monotonic()
                self._last_update_monotonic = now
                self._update_times.append(now)
                self._update_times = self._update_times[-30:]
                self._last_error = ""
            # The adapter owns the canonical stream consumed by the visualizer:
            # accepted mapping updates pass through, while runtime startup uses
            # the restored snapshot published above.
            self._dsg_pub.publish(msg)
        except Exception as exc:
            self._last_error = f"DSG update failed: {exc}"
            self.get_logger().error(self._last_error)

    def _mapping_complete_callback(self, msg: Bool) -> None:
        mapping_complete = bool(msg.data)
        with self._lock:
            if not mapping_complete and not self._accept_live_updates:
                # A new explicit exploration run supersedes persisted memory.
                # Discard it before applying Hydra's fresh graph snapshots so
                # node deletions from the new run cannot partially corrupt the
                # saved graph.
                self.get_logger().info(
                    "Exploration started; switching from persisted DSG to "
                    "live Hydra updates"
                )
                self._graph = None
                self._graph_version = 0
                self._loaded_from_disk = False
                self._accept_live_updates = True
                self._semantic_observations = []
            self._mapping_complete = mapping_complete

    def _semantic_observation_callback(
        self, msg: SemanticObjectObservation
    ) -> None:
        label = normalize_label(msg.label)
        if not label:
            return
        pose = msg.pose.pose.position
        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        record = {
            "label": label,
            "position": [float(pose.x), float(pose.y), float(pose.z)],
            "confidence": float(msg.confidence),
            "observed_at_ns": stamp_ns,
            "observed_wall_time_ns": time.time_ns(),
            "source": msg.source,
            "hydra_node_id": msg.hydra_node_id,
        }
        with self._lock:
            replacement = None
            for index, existing in enumerate(self._semantic_observations):
                dx = existing["position"][0] - record["position"][0]
                dy = existing["position"][1] - record["position"][1]
                if (
                    normalize_label(existing["label"]) == label
                    and math.hypot(dx, dy) <= 0.55
                ):
                    replacement = index
                    break
            if replacement is None:
                self._semantic_observations.append(record)
            elif record["confidence"] >= self._semantic_observations[replacement].get(
                "confidence", 0.0
            ):
                self._semantic_observations[replacement] = record

    def _layer(self, layer_id):
        if self._graph is None:
            return None
        try:
            return self._graph.get_layer(layer_id)
        except Exception:
            return None

    def _label_for_node(self, node, layer_id) -> str:
        attributes = node.attributes
        name = normalize_label(getattr(attributes, "name", ""))
        node_symbol = normalize_label(_node_id(node.id))
        if name and name != node_symbol and not re_symbol(name):
            return name
        try:
            labelspace = self._graph.get_labelspace(layer_id)
            labels = labelspace.labels_to_names
            semantic_label = int(attributes.semantic_label)
            label = labels.get(semantic_label, "") if hasattr(labels, "get") else ""
            normalized = normalize_label(label)
            if normalized:
                return normalized
        except Exception:
            pass
        try:
            semantic_label = int(attributes.semantic_label)
            return self._semantic_label_names.get(semantic_label, "")
        except Exception:
            return ""

    def _object_entities(self) -> list[SemanticEntity]:
        entities: list[SemanticEntity] = []
        objects = self._layer(dsg.DsgLayers.OBJECTS)
        if objects is not None:
            for node in objects.nodes:
                label = self._label_for_node(node, dsg.DsgLayers.OBJECTS)
                if not label or re_symbol(label):
                    continue
                bounds_min, bounds_max = _bounds_tuple(node.attributes)
                entities.append(
                    SemanticEntity(
                        node_id=_node_id(node.id),
                        entity_type="object",
                        label=label,
                        position=_position_tuple(node.attributes),
                        bounds_min=bounds_min,
                        bounds_max=bounds_max,
                        observed_at_ns=int(
                            getattr(node.attributes, "last_update_time_ns", 0)
                        ),
                        source="hydra",
                    )
                )
        for index, observation in enumerate(self._semantic_observations):
            position = observation.get("position", [0.0, 0.0, 0.0])
            entities.append(
                SemanticEntity(
                    node_id=observation.get("hydra_node_id")
                    or f"observation-{index}",
                    entity_type="object",
                    label=observation["label"],
                    position=(
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                    observed_at_ns=int(observation.get("observed_at_ns", 0)),
                    observed_wall_time_ns=int(
                        observation.get("observed_wall_time_ns", 0)
                    ),
                    source=str(observation.get("source", "live_observation")),
                )
            )
        return merge_nearby_entities(entities)

    def _hierarchical_room_objects(self) -> dict[str, list[SemanticEntity]]:
        """Collect objects using Hydra's object -> place -> room edges."""
        evidence: dict[str, list[SemanticEntity]] = {}
        objects = self._layer(dsg.DsgLayers.OBJECTS)
        if objects is None or self._graph is None:
            return evidence

        for object_node in objects.nodes:
            label = self._label_for_node(
                object_node,
                dsg.DsgLayers.OBJECTS,
            )
            if not label or re_symbol(label) or not object_node.has_parent():
                continue
            try:
                place_node = self._graph.get_node(object_node.get_parent())
                if not place_node.has_parent():
                    continue
                room_id = _node_id(place_node.get_parent())
            except Exception:
                continue
            evidence.setdefault(room_id, []).append(
                SemanticEntity(
                    node_id=_node_id(object_node.id),
                    entity_type="object",
                    label=label,
                    position=_position_tuple(object_node.attributes),
                    observed_at_ns=int(
                        getattr(
                            object_node.attributes,
                            "last_update_time_ns",
                            0,
                        )
                    ),
                )
            )
        return evidence

    def _room_entities(
        self, object_entities: list[SemanticEntity]
    ) -> list[SemanticEntity]:
        rooms = self._layer(dsg.DsgLayers.ROOMS)
        if rooms is None:
            return []
        result = []
        room_nodes = list(rooms.nodes)
        hierarchical_objects = self._hierarchical_room_objects()
        generic_room_labels = {
            "",
            "none",
            "room",
            "unknown",
            "unlabeled",
            "unassigned",
        }
        for room_node in room_nodes:
            room_position = _position_tuple(room_node.attributes)
            direct_label = self._label_for_node(room_node, dsg.DsgLayers.ROOMS)
            room_id = _node_id(room_node.id)
            room_objects = hierarchical_objects.get(room_id, [])
            evidence = [entity.label for entity in room_objects]
            if not evidence:
                # Older DSGs may not contain the complete hierarchy. Keep the
                # proximity-based behavior as a compatibility fallback.
                evidence = [
                    obj.label
                    for obj in object_entities
                    if math.hypot(
                        obj.position[0] - room_position[0],
                        obj.position[1] - room_position[1],
                    )
                    <= 4.0
                ]
            if self._room_rules:
                inferred, confidence = infer_room_label(
                    evidence,
                    self._room_rules,
                )
            else:
                inferred, confidence = infer_room_label(evidence)
            label = (
                direct_label
                if direct_label not in generic_room_labels
                and direct_label != room_id.lower()
                and not re_symbol(direct_label)
                else inferred
            )
            expected_evidence = {
                normalize_label(value)
                for value in self._room_rules.get(label, [])
            }
            supporting_objects = [
                entity
                for entity in room_objects
                if normalize_label(entity.label) in expected_evidence
            ]
            navigation_position = room_position
            if supporting_objects:
                navigation_position = tuple(
                    sum(entity.position[index] for entity in supporting_objects)
                    / len(supporting_objects)
                    for index in range(3)
                )
            aliases = tuple(
                alias
                for alias, canonical in self._room_aliases.items()
                if canonical == label
            )
            result.append(
                SemanticEntity(
                    node_id=room_id,
                    entity_type="room",
                    label=label,
                    position=navigation_position,
                    observed_at_ns=int(
                        getattr(room_node.attributes, "last_update_time_ns", 0)
                    ),
                    aliases=aliases,
                    evidence=tuple(evidence),
                )
            )
            if confidence == 0.0:
                self.get_logger().debug(
                    f"No semantic room evidence for {_node_id(room_node.id)}"
                )
        return result

    def _places(self) -> dict[str, Place]:
        layer = self._layer(dsg.DsgLayers.PLACES)
        if layer is None:
            return {}
        nodes = {
            _node_id(node.id): {
                "position": _position_tuple(node.attributes),
                "clearance": float(getattr(node.attributes, "distance", 0.0)),
                "neighbors": {},
            }
            for node in layer.nodes
            if bool(getattr(node.attributes, "real_place", True))
        }
        for edge in layer.edges:
            source = _node_id(edge.source)
            target = _node_id(edge.target)
            if source not in nodes or target not in nodes:
                continue
            a = nodes[source]["position"]
            b = nodes[target]["position"]
            weight = math.dist(a, b)
            nodes[source]["neighbors"][target] = weight
            nodes[target]["neighbors"][source] = weight
        return {
            node_id: Place(
                node_id=node_id,
                position=data["position"],
                clearance=data["clearance"],
                neighbors=data["neighbors"],
            )
            for node_id, data in nodes.items()
        }

    def _robot_position(self) -> tuple[float, float, float]:
        transform = self._tf_buffer.lookup_transform(
            self._map_frame,
            self._robot_frame,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        translation = transform.transform.translation
        return float(translation.x), float(translation.y), float(translation.z)

    def _navigation_map_callback(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._navigation_map = msg

    def _project_navigation_goal(
        self,
        target: tuple[float, float, float],
        robot: tuple[float, float, float],
        semantic_target: tuple[float, float, float] | None = None,
        target_standoff: tuple[float, float] | None = None,
    ) -> tuple[float, float] | None:
        """Project a Hydra place onto robot-clear map space reachable from here."""
        with self._lock:
            msg = self._navigation_map
        if msg is None or not msg.data:
            return None

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None
        grid = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
        threshold = int(
            self.get_parameter("navigation_occupied_threshold").value
        )
        traversable = (grid >= 0) & (grid < threshold)

        clearance_cells = max(
            1,
            int(
                math.ceil(
                    float(
                        self.get_parameter(
                            "navigation_goal_clearance_m"
                        ).value
                    )
                    / resolution
                )
            ),
        )
        blocked = ~traversable
        obstacle_near = np.zeros(grid.shape, dtype=bool)
        for dr in range(-clearance_cells, clearance_cells + 1):
            for dc in range(-clearance_cells, clearance_cells + 1):
                if dr * dr + dc * dc > clearance_cells * clearance_cells:
                    continue
                source_r = slice(max(0, -dr), min(height, height - dr))
                source_c = slice(max(0, -dc), min(width, width - dc))
                target_r = slice(max(0, dr), min(height, height + dr))
                target_c = slice(max(0, dc), min(width, width + dc))
                obstacle_near[target_r, target_c] |= blocked[source_r, source_c]
        clear = traversable & ~obstacle_near

        origin = msg.info.origin
        quaternion = origin.orientation
        yaw = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        def world_to_cell(x: float, y: float) -> tuple[int, int]:
            dx = x - origin.position.x
            dy = y - origin.position.y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            return (
                int(math.floor(local_y / resolution)),
                int(math.floor(local_x / resolution)),
            )

        def cell_to_world(row: int, col: int) -> tuple[float, float]:
            local_x = (col + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            return (
                origin.position.x + cos_yaw * local_x - sin_yaw * local_y,
                origin.position.y + sin_yaw * local_x + cos_yaw * local_y,
            )

        robot_cell = world_to_cell(robot[0], robot[1])
        if not (
            0 <= robot_cell[0] < height
            and 0 <= robot_cell[1] < width
        ):
            return None
        if not clear[robot_cell]:
            rows, cols = np.nonzero(clear)
            if rows.size == 0:
                return None
            distances = np.hypot(rows - robot_cell[0], cols - robot_cell[1])
            nearest = int(np.argmin(distances))
            if distances[nearest] * resolution > 1.0:
                return None
            robot_cell = int(rows[nearest]), int(cols[nearest])

        reachable = np.zeros(grid.shape, dtype=bool)
        reachable[robot_cell] = True
        frontier = deque([robot_cell])
        while frontier:
            row, col = frontier.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                neighbor = row + dr, col + dc
                if (
                    0 <= neighbor[0] < height
                    and 0 <= neighbor[1] < width
                    and clear[neighbor]
                    and not reachable[neighbor]
                ):
                    reachable[neighbor] = True
                    frontier.append(neighbor)

        target_cell = world_to_cell(target[0], target[1])
        projection_cells = max(
            1,
            int(
                math.ceil(
                    float(
                        self.get_parameter(
                            "navigation_goal_projection_radius_m"
                        ).value
                    )
                    / resolution
                )
            ),
        )
        row_min = max(0, target_cell[0] - projection_cells)
        row_max = min(height, target_cell[0] + projection_cells + 1)
        col_min = max(0, target_cell[1] - projection_cells)
        col_max = min(width, target_cell[1] + projection_cells + 1)
        local_rows, local_cols = np.nonzero(
            reachable[row_min:row_max, col_min:col_max]
        )
        if local_rows.size == 0:
            return None
        rows = local_rows + row_min
        cols = local_cols + col_min
        distances = np.hypot(rows - target_cell[0], cols - target_cell[1])
        within_radius = distances <= projection_cells
        if not np.any(within_radius):
            return None
        candidate_indices = np.flatnonzero(within_radius)
        if semantic_target is not None and target_standoff is not None:
            min_standoff, max_standoff = target_standoff
            standoff_ok = np.zeros(candidate_indices.shape, dtype=bool)
            for offset, candidate_index in enumerate(candidate_indices):
                candidate_x, candidate_y = cell_to_world(
                    int(rows[candidate_index]),
                    int(cols[candidate_index]),
                )
                distance = math.hypot(
                    candidate_x - semantic_target[0],
                    candidate_y - semantic_target[1],
                )
                standoff_ok[offset] = (
                    min_standoff <= distance <= max_standoff
                )
            candidate_indices = candidate_indices[standoff_ok]
            if candidate_indices.size == 0:
                return None
        best = int(candidate_indices[np.argmin(distances[candidate_indices])])
        return cell_to_world(int(rows[best]), int(cols[best]))

    def _resolve_callback(self, request, response):
        try:
            entity_type, label = parse_entity_ref(
                request.entity_ref,
                request.entity_type,
                request.label,
            )
        except ValueError as exc:
            response.status = STATUS_INTERNAL_ERROR
            response.error_message = str(exc)
            return response

        with self._lock:
            if not self._mapping_complete:
                response.status = STATUS_GRAPH_NOT_READY
                response.error_message = "mapping/exploration in progress"
                return response
            places = self._places()
            object_entities = self._object_entities()
            room_entities = self._room_entities(object_entities)
            entities = room_entities if entity_type == "room" else object_entities
            graph_version = self._graph_version

        if not places:
            response.status = STATUS_GRAPH_NOT_READY
            response.error_message = "Hydra graph has no traversable places"
            return response

        matches = rank_entities(label, entity_type, entities)
        minimum_match_score = float(
            self.get_parameter("minimum_semantic_match_score").value
        )
        if matches and matches[0].score < minimum_match_score:
            self.get_logger().info(
                f'Best graph match for "{label}" is only '
                f'{matches[0].score:.2f}; requiring live fallback below '
                f'{minimum_match_score:.2f}'
            )
            matches = []
        start_position = None
        if matches and not request.preferred_id:
            try:
                start_position = self._robot_position()
            except Exception as exc:
                response.status = STATUS_INTERNAL_ERROR
                response.error_message = f"robot pose unavailable: {exc}"
                return response
            matches = rank_matches_from_position(matches, start_position)
        match, error = choose_unambiguous_match(
            matches,
            request.preferred_id,
            reference_position=start_position,
        )
        if match is None:
            response.status = (
                STATUS_AMBIGUOUS_DESTINATION
                if error and error.startswith("ambiguous")
                else STATUS_UNKNOWN_DESTINATION
            )
            response.error_message = error or "unknown destination"
            return response

        stale_after = float(self.get_parameter("object_stale_after_sec").value)
        if (
            entity_type == "object"
            and match.entity.source != "hydra"
            and not request.allow_stale
        ):
            if match.entity.observed_wall_time_ns:
                age_sec = max(
                    0.0,
                    (time.time_ns() - match.entity.observed_wall_time_ns) / 1e9,
                )
            elif match.entity.observed_at_ns:
                now_ns = self.get_clock().now().nanoseconds
                age_sec = (
                    math.inf
                    if now_ns < match.entity.observed_at_ns
                    else (now_ns - match.entity.observed_at_ns) / 1e9
                )
            else:
                age_sec = math.inf
            if age_sec > stale_after:
                response.status = STATUS_STALE_DESTINATION
                response.error_message = (
                    f"object observation is stale ({age_sec:.0f}s old)"
                )
                return response

        min_clearance = float(self.get_parameter("min_place_clearance").value)
        target_standoff = None
        # Both persistent Hydra objects and one-shot detector observations need
        # an arm-reachable base standoff.  Without this for live observations,
        # the closest Hydra place can coincide with the object itself, so the
        # base pushes a floor object away before manipulation begins.
        if entity_type == "object":
            target_standoff = (
                float(self.get_parameter("object_standoff_min").value),
                float(self.get_parameter("object_standoff_max").value),
            )
        target_place = nearest_place(
            places,
            match.entity.position,
            min_clearance,
            target_standoff,
        )
        if target_place is None and not request.preferred_id:
            for alternative in matches[1:]:
                candidate_place = nearest_place(
                    places,
                    alternative.entity.position,
                    min_clearance,
                    target_standoff,
                )
                if candidate_place is not None:
                    match = alternative
                    target_place = candidate_place
                    break
        if target_place is None:
            response.status = STATUS_NO_TRAVERSABLE_PLACE
            response.error_message = "no collision-safe Hydra place near destination"
            return response

        if start_position is None:
            try:
                start_position = self._robot_position()
            except Exception as exc:
                response.status = STATUS_INTERNAL_ERROR
                response.error_message = f"robot pose unavailable: {exc}"
                return response
        if nearest_place(places, start_position, min_clearance) is None:
            response.status = STATUS_NO_TRAVERSABLE_PLACE
            response.error_message = "no traversable Hydra place near robot"
            return response

        route = route_between_nearest_places(
            places,
            start_position,
            match.entity.position,
            min_clearance,
            target_standoff,
        )
        if not route and entity_type == "object" and not request.preferred_id:
            for alternative in matches:
                if alternative.entity.node_id == match.entity.node_id:
                    continue
                candidate_place = nearest_place(
                    places,
                    alternative.entity.position,
                    min_clearance,
                    target_standoff,
                )
                if candidate_place is None:
                    continue
                candidate_route = route_between_nearest_places(
                    places,
                    start_position,
                    alternative.entity.position,
                    min_clearance,
                    target_standoff,
                )
                if candidate_route:
                    match = alternative
                    route = candidate_route
                    break
        if not route:
            response.status = STATUS_UNREACHABLE
            response.error_message = (
                "Hydra place graph has no component connecting robot to destination"
            )
            return response
        target_place = places[route[-1]]
        route = simplify_route(route, places)
        with self._lock:
            navigation_map_ready = self._navigation_map is not None
        if not navigation_map_ready:
            response.status = STATUS_GRAPH_NOT_READY
            response.error_message = "waiting for the saved navigation map"
            return response
        projected_target = self._project_navigation_goal(
            target_place.position,
            start_position,
            (
                match.entity.position
                if match.entity.source == "grounding_dino:live_fallback"
                else None
            ),
            (
                target_standoff
                if match.entity.source == "grounding_dino:live_fallback"
                else None
            ),
        )
        if projected_target is None:
            response.status = STATUS_UNREACHABLE
            response.error_message = (
                "no robot-clear saved-map cell connects the robot to the "
                "Hydra destination"
            )
            return response
        projection_distance = math.hypot(
            projected_target[0] - target_place.position[0],
            projected_target[1] - target_place.position[1],
        )
        if projection_distance > 0.05:
            self.get_logger().info(
                f"Projected Hydra place {target_place.node_id} by "
                f"{projection_distance:.2f} m onto reachable saved-map space"
            )

        response.success = True
        response.status = STATUS_OK
        response.destination_id = match.entity.node_id
        response.destination_label = match.entity.label
        response.entity_type = entity_type
        response.graph_version = graph_version
        response.confidence = float(match.score)
        response.observed_at.sec = match.entity.observed_at_ns // 1_000_000_000
        response.observed_at.nanosec = (
            match.entity.observed_at_ns % 1_000_000_000
        )
        response.topological_path = route
        if entity_type == "object":
            # Use the final navigation pose to choose the near side of a large
            # support, while Hydra's AABB supplies the missing surface height
            # and usable XY bounds. This works for both compact boxes and beds.
            target_x, target_y, target_z = match.entity.position
            if match.entity.bounds_min and match.entity.bounds_max:
                lower = match.entity.bounds_min
                upper = match.entity.bounds_max
                inset_x = min(0.10, (upper[0] - lower[0]) * 0.25)
                inset_y = min(0.10, (upper[1] - lower[1]) * 0.25)
                target_x = min(
                    max(projected_target[0], lower[0] + inset_x),
                    upper[0] - inset_x,
                )
                target_y = min(
                    max(projected_target[1], lower[1] + inset_y),
                    upper[1] - inset_y,
                )
                floor_z = start_position[2]
                max_floor_gap = float(
                    self.get_parameter(
                        "support_bounds_max_floor_gap_m"
                    ).value
                )
                max_reanchor_height = float(
                    self.get_parameter(
                        "support_bounds_max_reanchor_height_m"
                    ).value
                )
                support_z, reanchored = support_surface_height(
                    lower,
                    upper,
                    floor_z,
                    max_floor_gap,
                    max_reanchor_height,
                )
                if reanchored:
                    self.get_logger().warning(
                        f"Re-anchored floating support bounds for "
                        f"{match.entity.node_id}: graph top {upper[2]:.2f} m, "
                        f"floor-relative top {support_z:.2f} m"
                    )
                target_z = support_z + 0.05
            response.destination_pose.header.frame_id = self._map_frame
            response.destination_pose.header.stamp = self.get_clock().now().to_msg()
            response.destination_pose.pose.position.x = target_x
            response.destination_pose.pose.position.y = target_y
            response.destination_pose.pose.position.z = target_z
            response.destination_pose.pose.orientation.w = 1.0
        waypoint_place_ids = route[1:] or route
        if not bool(
            self.get_parameter("emit_intermediate_waypoints").value
        ):
            waypoint_place_ids = [route[-1]]
        for index, place_id in enumerate(waypoint_place_ids):
            place = places[place_id]
            waypoint = PoseStamped()
            waypoint.header.frame_id = self._map_frame
            waypoint.header.stamp = self.get_clock().now().to_msg()
            if index == len(waypoint_place_ids) - 1:
                waypoint.pose.position.x = projected_target[0]
                waypoint.pose.position.y = projected_target[1]
            else:
                waypoint.pose.position.x = place.position[0]
                waypoint.pose.position.y = place.position[1]
            if index == len(waypoint_place_ids) - 1 and entity_type == "object":
                dx = match.entity.position[0] - waypoint.pose.position.x
                dy = match.entity.position[1] - waypoint.pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            response.waypoints.append(waypoint)
        return response

    def _save_callback(self, _request, response):
        with self._lock:
            if self._graph is None:
                response.success = False
                response.message = "no scene graph is available"
                return response
            validation_error, summary = self._validate_graph()
            if validation_error:
                self.get_logger().warning(
                    f"Saving scene graph with quality warning: "
                    f"{validation_error}; {summary}"
                )
            try:
                self._persistence_path.parent.mkdir(parents=True, exist_ok=True)
                self._graph.save(str(self._persistence_path), False)
                self._observation_path.write_text(
                    json.dumps(self._semantic_observations, indent=2),
                    encoding="utf-8",
                )
                response.success = True
                response.message = (
                    f"saved DSG to {self._persistence_path} and observations "
                    f"to {self._observation_path}; {summary}"
                    + (
                        f"; quality warning: {validation_error}"
                        if validation_error
                        else ""
                    )
                )
            except Exception as exc:
                response.success = False
                response.message = f"save failed: {exc}"
        return response

    @property
    def _pending_graph_path(self) -> Path:
        return self._persistence_path.with_name(
            f"{self._persistence_path.stem}.pending"
            f"{self._persistence_path.suffix}"
        )

    @property
    def _pending_observation_path(self) -> Path:
        return self._observation_path.with_name(
            f"{self._observation_path.stem}.pending"
            f"{self._observation_path.suffix}"
        )

    def _discard_pending_save(self) -> None:
        for path in (self._pending_graph_path, self._pending_observation_path):
            try:
                path.unlink(missing_ok=True)
            except OSError as exc:
                self.get_logger().warning(
                    f"Could not remove pending save {path}: {exc}"
                )

    def _prepare_save_callback(self, _request, response):
        with self._lock:
            if self._graph is None:
                response.success = False
                response.message = "no scene graph is available"
                return response
            validation_error, summary = self._validate_graph()
            try:
                if validation_error:
                    self.get_logger().warning(
                        f"Preparing scene graph with quality warning: "
                        f"{validation_error}; {summary}"
                    )
                self._persistence_path.parent.mkdir(parents=True, exist_ok=True)
                self._discard_pending_save()
                self._graph.save(str(self._pending_graph_path), False)
                self._pending_observation_path.write_text(
                    json.dumps(self._semantic_observations, indent=2),
                    encoding="utf-8",
                )
                response.success = True
                response.message = (
                    f"prepared scene-graph save; {summary}"
                    + (
                        f"; quality warning: {validation_error}"
                        if validation_error
                        else ""
                    )
                )
            except Exception as exc:
                self._discard_pending_save()
                response.success = False
                response.message = f"prepare save failed: {exc}"
        return response

    def _commit_save_callback(self, _request, response):
        graph_backup = self._persistence_path.with_suffix(
            f"{self._persistence_path.suffix}.transaction-backup"
        )
        observation_backup = self._observation_path.with_suffix(
            f"{self._observation_path.suffix}.transaction-backup"
        )
        with self._lock:
            if not self._pending_graph_path.exists() or not (
                self._pending_observation_path.exists()
            ):
                response.success = False
                response.message = "no prepared scene-graph save exists"
                return response
            graph_existed = self._persistence_path.exists()
            observations_existed = self._observation_path.exists()
            try:
                graph_backup.unlink(missing_ok=True)
                observation_backup.unlink(missing_ok=True)
                if self._persistence_path.exists():
                    os.replace(self._persistence_path, graph_backup)
                if self._observation_path.exists():
                    os.replace(self._observation_path, observation_backup)
                os.replace(self._pending_graph_path, self._persistence_path)
                os.replace(
                    self._pending_observation_path,
                    self._observation_path,
                )
                graph_backup.unlink(missing_ok=True)
                observation_backup.unlink(missing_ok=True)
                response.success = True
                response.message = f"committed DSG to {self._persistence_path}"
            except Exception as exc:
                if not graph_existed:
                    self._persistence_path.unlink(missing_ok=True)
                if not observations_existed:
                    self._observation_path.unlink(missing_ok=True)
                if graph_backup.exists():
                    os.replace(graph_backup, self._persistence_path)
                if observation_backup.exists():
                    os.replace(observation_backup, self._observation_path)
                response.success = False
                response.message = f"commit save failed and was rolled back: {exc}"
            finally:
                self._discard_pending_save()
        return response

    def _abort_save_callback(self, _request, response):
        self._discard_pending_save()
        response.success = True
        response.message = "discarded pending scene-graph save"
        return response

    def _validate_graph(self) -> tuple[str, str]:
        """Summarize graph quality and return at most one advisory warning."""
        places = self._places()
        objects = self._object_entities()
        label_counts: dict[str, int] = {}
        for entity in objects:
            label_counts[entity.label] = label_counts.get(entity.label, 0) + 1

        place_count = len(places)
        object_count = len(objects)
        summary = (
            f"places={place_count}, objects={object_count}, "
            f"labels={dict(sorted(label_counts.items()))}"
        )
        min_places = int(self.get_parameter("min_place_count").value)
        if place_count < min_places:
            return f"only {place_count} traversable places (minimum {min_places})", summary

        unseen = set(places)
        largest_component = 0
        while unseen:
            seed = unseen.pop()
            component_size = 0
            frontier = [seed]
            while frontier:
                current = frontier.pop()
                component_size += 1
                for neighbor in places[current].neighbors:
                    if neighbor in unseen:
                        unseen.remove(neighbor)
                        frontier.append(neighbor)
            largest_component = max(largest_component, component_size)
        component_ratio = largest_component / place_count
        min_ratio = float(
            self.get_parameter("min_largest_place_component_ratio").value
        )
        if component_ratio < min_ratio:
            return (
                f"largest place component is only {component_ratio:.0%} "
                f"(minimum {min_ratio:.0%})"
            ), summary

        max_objects = int(self.get_parameter("max_object_count").value)
        if object_count == 0:
            return "object layer is empty", summary
        if object_count > max_objects:
            return (
                f"object layer is fragmented ({object_count} nodes, "
                f"maximum {max_objects})"
            ), summary

        max_per_label = int(
            self.get_parameter("max_objects_per_label").value
        )
        fragmented = {
            label: count
            for label, count in label_counts.items()
            if count > max_per_label
        }
        if fragmented:
            return (
                f"semantic labels are fragmented beyond {max_per_label}: "
                f"{fragmented}"
            ), summary

        forbidden = {
            normalize_label(value)
            for value in self.get_parameter("forbidden_object_labels").value
            if normalize_label(value)
        }
        unexpected = sorted(forbidden & set(label_counts))
        if unexpected:
            return (
                "structural labels leaked into the object layer: "
                f"{', '.join(unexpected)}"
            ), summary

        return "", (
            f"{summary}, largest_place_component={component_ratio:.0%}"
        )

    def _validate_callback(self, _request, response):
        with self._lock:
            if self._graph is None:
                response.success = False
                response.message = "no scene graph is available"
                return response
            error, summary = self._validate_graph()
        # Graph-quality checks are advisory. A sparse or imperfect graph is
        # still valuable mapping output and must never be blocked from saving.
        response.success = True
        response.message = (
            f"scene graph quality warning: {error}; {summary}"
            if error
            else f"scene graph quality checks passed; {summary}"
        )
        return response

    def _counts(self) -> tuple[int, int, int]:
        def count(layer_id) -> int:
            layer = self._layer(layer_id)
            return int(layer.num_nodes()) if layer is not None else 0

        return (
            count(dsg.DsgLayers.ROOMS),
            count(dsg.DsgLayers.OBJECTS) + len(self._semantic_observations),
            count(dsg.DsgLayers.PLACES),
        )

    def _publish_status(self) -> None:
        with self._lock:
            rooms, objects, places = self._counts()
            now = time.monotonic()
            age = (
                now - self._last_update_monotonic
                if self._last_update_monotonic
                else math.inf
            )
            update_rate = 0.0
            if len(self._update_times) >= 2:
                duration = self._update_times[-1] - self._update_times[0]
                if duration > 0:
                    update_rate = (len(self._update_times) - 1) / duration
            ready = self._mapping_complete and places > 0
            entities = self._room_entities(self._object_entities())
            known = {
                "graph_version": self._graph_version,
                "ready": ready,
                "rooms": sorted(
                    {entity.label for entity in entities if entity.label != "room"}
                ),
                "objects": sorted(
                    {entity.label for entity in self._object_entities()}
                ),
            }

        status = SceneGraphStatus()
        status.header.frame_id = self._map_frame
        status.header.stamp = self.get_clock().now().to_msg()
        status.ready = ready
        status.mapping_complete = self._mapping_complete
        status.loaded_from_disk = self._loaded_from_disk
        status.graph_version = self._graph_version
        status.room_count = rooms
        status.object_count = objects
        status.place_count = places
        status.update_rate_hz = float(update_rate)
        status.last_update_age_sec = float(age if math.isfinite(age) else -1.0)
        status.persistence_path = str(self._persistence_path)
        status.state = (
            "ready"
            if ready
            else "mapping"
            if not self._mapping_complete
            else "waiting_for_graph"
        )
        status.error_message = self._last_error
        self._status_pub.publish(status)
        known_msg = String()
        known_msg.data = json.dumps(known, sort_keys=True)
        self._known_entities_pub.publish(known_msg)


def re_symbol(value: str) -> bool:
    return bool(value) and len(value) <= 4 and value[0].isalpha() and value[1:].isdigit()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneGraphAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
