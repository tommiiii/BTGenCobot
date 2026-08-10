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


def parse_node_symbol(value: str) -> tuple[str, int]:
    """Split Spark-DSG's printable node ID into constructor arguments."""
    match = re.fullmatch(r"(.)(\d+)", value.strip())
    if match is None:
        raise ValueError(f"invalid Spark-DSG node symbol: {value!r}")
    return match.group(1), int(match.group(2))


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
class ObjectNodeCandidate:
    """Minimal object-node data needed for deterministic DSG de-duplication."""

    node_id: str
    label: str
    position: tuple[float, float, float]
    mesh_connection_count: int = 0
    observed_at_ns: int = 0
    bounds_min: Optional[tuple[float, float, float]] = None
    bounds_max: Optional[tuple[float, float, float]] = None


def _contains_position(
    candidate: ObjectNodeCandidate,
    position: tuple[float, float, float],
    padding: float = 0.05,
) -> bool:
    if candidate.bounds_min is None or candidate.bounds_max is None:
        return False
    return all(
        candidate.bounds_min[index] - padding
        <= position[index]
        <= candidate.bounds_max[index] + padding
        for index in range(3)
    )


def _has_nondegenerate_bounds(candidate: ObjectNodeCandidate) -> bool:
    if candidate.bounds_min is None or candidate.bounds_max is None:
        return False
    return all(
        candidate.bounds_max[index] - candidate.bounds_min[index] > 1.0e-3
        for index in range(3)
    )


def _same_object_segment(
    lhs: ObjectNodeCandidate,
    rhs: ObjectNodeCandidate,
    fallback_radius: float,
) -> bool:
    if normalize_label(lhs.label) != normalize_label(rhs.label):
        return False
    if (
        lhs.bounds_min is not None
        and lhs.bounds_max is not None
        and rhs.bounds_min is not None
        and rhs.bounds_max is not None
    ):
        # Hydra's own object association uses the same centroid-in-bounds rule.
        return _contains_position(lhs, rhs.position) or _contains_position(
            rhs, lhs.position
        )
    return distance_2d(lhs.position, rhs.position) <= fallback_radius


def redundant_object_node_ids(
    candidates: Iterable[ObjectNodeCandidate],
    radius: float = 0.35,
) -> set[str]:
    """Choose repeated same-class object nodes to remove from the canonical DSG.

    Hydra retains archived segment nodes as its active reconstruction window
    moves. Prefer non-degenerate geometry backed by the most mesh evidence.
    When bounds exist, use Hydra's own centroid-in-bounds association rule;
    distance is only a compatibility fallback for graphs without object bounds.
    """
    ranked = sorted(
        candidates,
        key=lambda candidate: (
            -int(_has_nondegenerate_bounds(candidate)),
            -candidate.mesh_connection_count,
            -candidate.observed_at_ns,
            candidate.node_id,
        ),
    )
    retained: list[ObjectNodeCandidate] = []
    redundant: set[str] = set()
    for candidate in ranked:
        duplicate = any(
            _same_object_segment(candidate, existing, radius)
            for existing in retained
        )
        if duplicate:
            redundant.add(candidate.node_id)
        else:
            retained.append(candidate)
    return redundant


def physically_invalid_object_node_ids(
    candidates: Iterable[ObjectNodeCandidate],
    min_extent_m: float = 0.015,
    max_extent_m: float = 3.0,
) -> set[str]:
    """Reject degenerate mesh patches and room-scale object components.

    This is deliberately class-independent.  Bounds which are absent remain a
    validation error elsewhere; bounds which are present must describe a
    physically useful object rather than a surface sliver or an entire room.
    """
    invalid: set[str] = set()
    for candidate in candidates:
        if candidate.bounds_min is None or candidate.bounds_max is None:
            continue
        extents = tuple(
            candidate.bounds_max[index] - candidate.bounds_min[index]
            for index in range(3)
        )
        if (
            not all(math.isfinite(value) for value in extents)
            or min(extents) < min_extent_m
            or max(extents) > max_extent_m
        ):
            invalid.add(candidate.node_id)
    return invalid


def conflicting_object_node_ids(
    candidates: Iterable[ObjectNodeCandidate],
    exclusive_labels: Iterable[str],
    overlap_fraction: float = 0.80,
) -> set[str]:
    """Suppress mutually exclusive furniture hypotheses on the same volume.

    Pixel semantics can split one physical item into, for example, overlapping
    ``bed`` and ``couch`` components.  Hydra cannot merge those because object
    association is label-specific.  Retain the hypothesis with the strongest
    mesh support when at least ``overlap_fraction`` of the smaller AABB is
    occupied.  The caller controls the exclusive label set so contained small
    objects (a bottle on a table) are never affected.
    """
    exclusive = {normalize_label(value) for value in exclusive_labels}
    bounded = [
        candidate
        for candidate in candidates
        if normalize_label(candidate.label) in exclusive
        and _has_nondegenerate_bounds(candidate)
    ]
    ranked = sorted(
        bounded,
        key=lambda candidate: (
            -candidate.mesh_connection_count,
            -candidate.observed_at_ns,
            candidate.node_id,
        ),
    )
    retained: list[ObjectNodeCandidate] = []
    redundant: set[str] = set()
    for candidate in ranked:
        candidate_extents = tuple(
            candidate.bounds_max[index] - candidate.bounds_min[index]
            for index in range(3)
        )
        candidate_volume = math.prod(candidate_extents)
        conflict = False
        for existing in retained:
            if normalize_label(candidate.label) == normalize_label(existing.label):
                continue
            intersection = math.prod(
                max(
                    0.0,
                    min(candidate.bounds_max[index], existing.bounds_max[index])
                    - max(candidate.bounds_min[index], existing.bounds_min[index]),
                )
                for index in range(3)
            )
            existing_volume = math.prod(
                existing.bounds_max[index] - existing.bounds_min[index]
                for index in range(3)
            )
            smaller_volume = min(candidate_volume, existing_volume)
            if smaller_volume > 0.0 and intersection / smaller_volume >= overlap_fraction:
                conflict = True
                break
        if conflict:
            redundant.add(candidate.node_id)
        else:
            retained.append(candidate)
    return redundant

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
    """Choose a stable candidate; semantic ambiguity is never a hard failure.

    Callers should order candidates by route cost or physical distance before
    invoking this compatibility helper.  ``preferred_id`` remains strict so an
    explicitly grounded object cannot silently change identity.
    """
    if preferred_id:
        for match in matches:
            if match.entity.node_id == preferred_id:
                return match, None
        return None, f"preferred entity {preferred_id!r} is not a matching candidate"

    if not matches:
        return None, "unknown destination"
    return matches[0], None


def split_room_qualified_label(
    label: str,
    rooms: Iterable[SemanticEntity],
) -> tuple[str, tuple[str, ...]]:
    """Split ``kitchen table`` or ``table in the kitchen`` into room context.

    The returned IDs may contain multiple rooms with the same inferred label;
    route cost resolves between them later. If no known room qualifier is
    present, the normalized label and an empty tuple are returned.
    """
    query = normalize_label(label)
    grouped: dict[str, set[str]] = {}
    for room in rooms:
        for room_label in room.searchable_labels:
            grouped.setdefault(room_label, set()).add(room.node_id)

    for room_label in sorted(grouped, key=lambda value: (-len(value), value)):
        prefix = f"{room_label} "
        suffix = f" in {room_label}"
        if query.startswith(prefix) and query[len(prefix):].strip():
            return query[len(prefix):].strip(), tuple(sorted(grouped[room_label]))
        if query.endswith(suffix) and query[:-len(suffix)].strip():
            return query[:-len(suffix)].strip(), tuple(sorted(grouped[room_label]))
        suffix_with_article = f" in the {room_label}"
        if query.endswith(suffix_with_article) and query[:-len(suffix_with_article)].strip():
            return (
                query[:-len(suffix_with_article)].strip(),
                tuple(sorted(grouped[room_label])),
            )
    return query, ()


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


def route_cost(route: Iterable[str], places: Mapping[str, Place]) -> float:
    """Return the metric/topological cost of a place route."""
    route_ids = list(route)
    total = 0.0
    for source, target in zip(route_ids, route_ids[1:]):
        total += float(
            places[source].neighbors.get(
                target,
                distance_2d(places[source].position, places[target].position),
            )
        )
    return total


def choose_best_routable_match(
    matches: Iterable[Match],
    places: Mapping[str, Place],
    start_position: tuple[float, float, float],
    min_clearance: float,
    target_standoff: Optional[tuple[float, float]] = None,
    preferred_id: str = "",
) -> tuple[Optional[Match], list[str], Optional[str]]:
    """Always choose one semantic instance, preferring the cheapest route.

    Semantic score remains the primary key so a nearby fuzzy match cannot beat
    an exact label. Equal semantic matches prefer a reachable Hydra route, then
    route cost, Euclidean distance and finally stable node ID. If every match
    is unreachable, return the nearest deterministic choice with an empty
    route so the caller can report reachability rather than ambiguity.
    """
    candidates = list(matches)
    if preferred_id:
        candidates = [
            match for match in candidates if match.entity.node_id == preferred_id
        ]
        if not candidates:
            return None, [], (
                f"preferred entity {preferred_id!r} is not a matching candidate"
            )
    if not candidates:
        return None, [], "unknown destination"

    evaluated = []
    for match in candidates:
        route = route_between_nearest_places(
            places,
            start_position,
            match.entity.position,
            min_clearance,
            target_standoff,
        )
        evaluated.append(
            (
                match,
                route,
                route_cost(route, places) if route else math.inf,
            )
        )
    evaluated.sort(
        key=lambda value: (
            -value[0].score,
            not bool(value[1]),
            value[2],
            distance_2d(value[0].entity.position, start_position),
            value[0].entity.node_id,
        )
    )
    match, route, _ = evaluated[0]
    return match, route, None


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
        "appliance": 3.0,
        "refrigerator": 5.0,
        "oven": 5.0,
        "microwave": 4.0,
        "stove": 5.0,
        "sink": 2.0,
        "counter": 2.0,
        "food": 0.5,
    },
    "bedroom": {"bed": 6.0, "pillow": 4.0, "wardrobe": 2.0, "clothes": 1.0},
    "living room": {
        "seating": 1.0,
        "sofa": 4.0,
        "couch": 4.0,
        "television": 5.0,
        "coffee table": 2.0,
    },
    "bathroom": {"toilet": 6.0, "bathtub": 6.0, "shower": 5.0, "sink": 1.0},
    "dining room": {"dining table": 6.0, "table": 1.0, "chair": 1.0, "food": 0.5},
}


def infer_room_label(
    evidence: Iterable[str],
    rules: Mapping[
        str,
        Iterable[str] | Mapping[str, float],
    ] = DEFAULT_ROOM_EVIDENCE,
) -> tuple[str, float]:
    observed = {normalize_label(value) for value in evidence}
    scored = []
    for room_label, expected_values in rules.items():
        if isinstance(expected_values, Mapping):
            weights = {
                normalize_label(value): max(0.0, float(weight))
                for value, weight in expected_values.items()
            }
        else:
            weights = {
                normalize_label(value): 1.0 for value in expected_values
            }
        score = sum(weights.get(value, 0.0) for value in observed)
        possible = max(1.0, sum(weights.values()))
        scored.append((score, score / possible, normalize_label(room_label)))
    score, confidence, room_label = min(
        scored,
        key=lambda value: (-value[0], value[2]),
        default=(0.0, 0.0, "room"),
    )
    return (room_label, min(1.0, confidence)) if score > 0.0 else ("room", 0.0)
