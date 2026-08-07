"""Pure semantic grounding and graph-routing helpers.

This module intentionally does not import ROS or Spark-DSG so the critical
matching/routing logic can be tested on the host and in CI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from difflib import SequenceMatcher
import heapq
import math
import re
from typing import Iterable, Mapping, Optional


ENTITY_TYPES = {"room", "object"}


def support_surface_height(
    bounds_min: tuple[float, float, float],
    bounds_max: tuple[float, float, float],
    floor_z: float,
    max_floor_gap: float,
    max_reanchor_height: float,
) -> tuple[float, bool]:
    """Return the support top, repairing only compact floating AABBs."""
    height = bounds_max[2] - bounds_min[2]
    should_reanchor = (
        bounds_min[2] - floor_z > max_floor_gap
        and 0.05 < height <= max_reanchor_height
    )
    if should_reanchor:
        return floor_z + height, True
    return bounds_max[2], False


def normalize_label(value: str) -> str:
    value = value.strip().lower().replace("_", " ").replace("-", " ")
    value = re.sub(r"^(?:the|a|an)\s+", "", value)
    return re.sub(r"\s+", " ", value)


def parse_entity_ref(
    entity_ref: str,
    entity_type: str = "",
    label: str = "",
) -> tuple[str, str]:
    """Return a validated ``(entity_type, label)`` semantic reference."""
    ref = entity_ref.strip()
    ref_type = ""
    ref_label = ref
    if ":" in ref:
        ref_type, ref_label = ref.split(":", 1)

    resolved_type = normalize_label(entity_type or ref_type)
    resolved_label = normalize_label(label or ref_label)
    if resolved_type not in ENTITY_TYPES:
        raise ValueError(
            f"entity type must be one of {sorted(ENTITY_TYPES)}, got {resolved_type!r}"
        )
    if not resolved_label:
        raise ValueError("semantic entity label is empty")
    return resolved_type, resolved_label


@dataclass(frozen=True)
class SemanticEntity:
    node_id: str
    entity_type: str
    label: str
    position: tuple[float, float, float]
    bounds_min: Optional[tuple[float, float, float]] = None
    bounds_max: Optional[tuple[float, float, float]] = None
    observed_at_ns: int = 0
    observed_wall_time_ns: int = 0
    source: str = "hydra"
    aliases: tuple[str, ...] = ()
    evidence: tuple[str, ...] = ()

    @property
    def searchable_labels(self) -> tuple[str, ...]:
        return tuple(
            dict.fromkeys(
                normalize_label(value)
                for value in (self.label, *self.aliases)
                if normalize_label(value)
            )
        )


@dataclass(frozen=True)
class Match:
    entity: SemanticEntity
    score: float


def _token_score(query: str, candidate: str) -> float:
    if query == candidate:
        return 1.0
    if query in candidate or candidate in query:
        return 0.86
    query_tokens = set(query.split())
    candidate_tokens = set(candidate.split())
    if not query_tokens or not candidate_tokens:
        return 0.0
    overlap = len(query_tokens & candidate_tokens)
    token_score = overlap / max(len(query_tokens), len(candidate_tokens))
    return max(
        token_score,
        0.85 * SequenceMatcher(None, query, candidate).ratio(),
    )


def rank_entities(
    label: str,
    entity_type: str,
    entities: Iterable[SemanticEntity],
) -> list[Match]:
    query = normalize_label(label)
    matches: list[Match] = []
    for entity in entities:
        if entity.entity_type != entity_type:
            continue
        score = max(
            (_token_score(query, candidate) for candidate in entity.searchable_labels),
            default=0.0,
        )
        if score >= 0.55:
            matches.append(Match(entity, score))
    return sorted(matches, key=lambda match: (-match.score, match.entity.node_id))


def rank_matches_from_position(
    matches: Iterable[Match],
    position: tuple[float, float, float],
) -> list[Match]:
    """Resolve equal semantic matches by nearest physical instance."""
    return sorted(
        matches,
        key=lambda match: (
            -match.score,
            distance_2d(match.entity.position, position),
            match.entity.node_id,
        ),
    )


def merge_nearby_entities(
    entities: Iterable[SemanticEntity],
    radius: float = 0.4,
) -> list[SemanticEntity]:
    """Collapse repeated Hydra observations without merging distinct instances."""
    clusters: list[list[SemanticEntity]] = []
    for entity in entities:
        cluster = next(
            (
                candidate
                for candidate in clusters
                if normalize_label(candidate[0].label) == normalize_label(entity.label)
                and distance_2d(candidate[0].position, entity.position) <= radius
            ),
            None,
        )
        if cluster is None:
            clusters.append([entity])
        else:
            cluster.append(entity)

    merged = []
    for cluster in clusters:
        representative = max(
            cluster,
            key=lambda entity: entity.observed_at_ns,
        )
        count = len(cluster)
        position = tuple(
            sum(entity.position[index] for entity in cluster) / count
            for index in range(3)
        )
        merged.append(
            SemanticEntity(
                node_id=representative.node_id,
                entity_type=representative.entity_type,
                label=representative.label,
                position=position,
                bounds_min=representative.bounds_min,
                bounds_max=representative.bounds_max,
                observed_at_ns=max(
                    entity.observed_at_ns for entity in cluster
                ),
                observed_wall_time_ns=max(
                    entity.observed_wall_time_ns for entity in cluster
                ),
                source=representative.source,
                aliases=tuple(
                    dict.fromkeys(
                        alias for entity in cluster for alias in entity.aliases
                    )
                ),
                evidence=tuple(
                    dict.fromkeys(
                        value for entity in cluster for value in entity.evidence
                    )
                ),
            )
        )
    return merged


def choose_unambiguous_match(
    matches: list[Match],
    preferred_id: str = "",
    ambiguity_margin: float = 0.08,
    reference_position: Optional[tuple[float, float, float]] = None,
    distance_margin: float = 0.75,
) -> tuple[Optional[Match], Optional[str]]:
    if preferred_id:
        for match in matches:
            if match.entity.node_id == preferred_id:
                return match, None
        return None, f"preferred entity {preferred_id!r} is not a matching candidate"

    if not matches:
        return None, "unknown destination"
    if len(matches) > 1:
        first, second = matches[0], matches[1]
        semantically_close = first.score - second.score < ambiguity_margin
        spatially_distinct = distance_2d(
            first.entity.position,
            second.entity.position,
        ) > 0.4
        distance_is_decisive = False
        if reference_position is not None:
            first_distance = distance_2d(first.entity.position, reference_position)
            second_distance = distance_2d(second.entity.position, reference_position)
            distance_is_decisive = (
                second_distance - first_distance >= distance_margin
            )
        if semantically_close and spatially_distinct and not distance_is_decisive:
            candidates = ", ".join(
                f"{match.entity.node_id}:{match.entity.label}"
                for match in matches[:5]
            )
            return None, f"ambiguous destination; candidates: {candidates}"
    return matches[0], None


@dataclass(frozen=True)
class Place:
    node_id: str
    position: tuple[float, float, float]
    clearance: float = 0.0
    neighbors: Mapping[str, float] = field(default_factory=dict)


def distance_2d(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_place(
    places: Mapping[str, Place],
    position: tuple[float, float, float],
    min_clearance: float,
    target_standoff: Optional[tuple[float, float]] = None,
) -> Optional[Place]:
    candidates = [
        place for place in places.values() if place.clearance >= min_clearance
    ]
    if target_standoff is not None:
        min_distance, max_distance = target_standoff
        standoff_candidates = [
            place
            for place in candidates
            if min_distance <= distance_2d(place.position, position) <= max_distance
        ]
        # Object navigation must not silently degrade to an arbitrary nearest
        # place. A missing standoff candidate usually means that the semantic
        # centroid or the saved place graph is wrong; returning that arbitrary
        # place can send the robot to the wrong side of the map.
        candidates = standoff_candidates
    return min(candidates, key=lambda place: distance_2d(place.position, position), default=None)


def astar_route(
    places: Mapping[str, Place],
    start_id: str,
    target_id: str,
) -> list[str]:
    if start_id not in places or target_id not in places:
        return []
    if start_id == target_id:
        return [start_id]

    frontier: list[tuple[float, float, str]] = [(0.0, 0.0, start_id)]
    previous: dict[str, Optional[str]] = {start_id: None}
    cost: dict[str, float] = {start_id: 0.0}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current == target_id:
            break
        if current_cost > cost.get(current, math.inf):
            continue

        for neighbor, edge_cost in places[current].neighbors.items():
            if neighbor not in places:
                continue
            clearance = max(places[neighbor].clearance, 0.05)
            weighted_edge = max(float(edge_cost), 0.01) * (1.0 + 0.15 / clearance)
            next_cost = current_cost + weighted_edge
            if next_cost >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = next_cost
            previous[neighbor] = current
            heuristic = distance_2d(
                places[neighbor].position,
                places[target_id].position,
            )
            heapq.heappush(frontier, (next_cost + heuristic, next_cost, neighbor))

    if target_id not in previous:
        return []
    route = []
    cursor: Optional[str] = target_id
    while cursor is not None:
        route.append(cursor)
        cursor = previous[cursor]
    return list(reversed(route))


def route_between_nearest_places(
    places: Mapping[str, Place],
    start_position: tuple[float, float, float],
    target_position: tuple[float, float, float],
    min_clearance: float,
    target_standoff: Optional[tuple[float, float]] = None,
) -> list[str]:
    """Route between nearby anchors without selecting disconnected fragments."""
    start_candidates = sorted(
        (
            place
            for place in places.values()
            if place.clearance >= min_clearance
        ),
        key=lambda place: distance_2d(place.position, start_position),
    )
    target_candidates = [
        place
        for place in places.values()
        if place.clearance >= min_clearance
    ]
    if target_standoff is not None:
        min_distance, max_distance = target_standoff
        target_candidates = [
            place
            for place in target_candidates
            if min_distance
            <= distance_2d(place.position, target_position)
            <= max_distance
        ]
    target_candidates.sort(
        key=lambda place: distance_2d(place.position, target_position)
    )

    components: dict[str, int] = {}
    for place_id in places:
        if place_id in components:
            continue
        component_id = len(components)
        components[place_id] = component_id
        frontier = [place_id]
        while frontier:
            current = frontier.pop()
            for neighbor in places[current].neighbors:
                if neighbor not in places or neighbor in components:
                    continue
                components[neighbor] = component_id
                frontier.append(neighbor)

    nearest_target_by_component: dict[int, Place] = {}
    for target in target_candidates:
        nearest_target_by_component.setdefault(components[target.node_id], target)

    for start in start_candidates:
        target = nearest_target_by_component.get(components[start.node_id])
        if target is not None:
            return astar_route(places, start.node_id, target.node_id)
    return []


def simplify_route(
    route: list[str],
    places: Mapping[str, Place],
    min_spacing: float = 1.0,
    turn_threshold_rad: float = 0.55,
) -> list[str]:
    if len(route) <= 2:
        return route
    kept = [route[0]]
    for index in range(1, len(route) - 1):
        previous = places[kept[-1]].position
        current = places[route[index]].position
        following = places[route[index + 1]].position
        incoming = math.atan2(current[1] - previous[1], current[0] - previous[0])
        outgoing = math.atan2(following[1] - current[1], following[0] - current[0])
        turn = abs(math.atan2(math.sin(outgoing - incoming), math.cos(outgoing - incoming)))
        if distance_2d(previous, current) >= min_spacing or turn >= turn_threshold_rad:
            kept.append(route[index])
    kept.append(route[-1])
    return kept


DEFAULT_ROOM_EVIDENCE = {
    "kitchen": {
        "appliance",
        "refrigerator",
        "oven",
        "microwave",
        "stove",
        "sink",
        "counter",
        "food",
    },
    "bedroom": {"bed", "pillow", "wardrobe", "clothes"},
    "living room": {"seating", "sofa", "couch", "television", "coffee table"},
    "bathroom": {"toilet", "bathtub", "shower", "sink"},
    "dining room": {"dining table", "table", "chair", "food"},
}


def infer_room_label(
    evidence: Iterable[str],
    rules: Mapping[str, Iterable[str]] = DEFAULT_ROOM_EVIDENCE,
) -> tuple[str, float]:
    observed = {normalize_label(value) for value in evidence}
    scored = []
    for room_label, expected_values in rules.items():
        expected = {normalize_label(value) for value in expected_values}
        hits = len(observed & expected)
        score = hits / max(1.0, math.sqrt(len(expected)))
        scored.append((score, normalize_label(room_label)))
    score, room_label = max(scored, default=(0.0, "room"))
    return (room_label, min(1.0, score)) if score > 0.0 else ("room", 0.0)
