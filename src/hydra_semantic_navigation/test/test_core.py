from hydra_semantic_navigation.core import (
    Place,
    SemanticEntity,
    astar_route,
    choose_unambiguous_match,
    infer_room_label,
    merge_nearby_entities,
    nearest_place,
    parse_entity_ref,
    rank_entities,
    rank_matches_from_position,
    route_between_nearest_places,
    support_surface_height,
)


def test_grounded_support_keeps_graph_top():
    height, reanchored = support_surface_height(
        (0.0, 0.0, 0.02), (1.0, 0.5, 0.42), 0.0, 0.25, 1.20
    )
    assert height == 0.42
    assert not reanchored


def test_compact_floating_support_is_reanchored_by_height():
    height, reanchored = support_surface_height(
        (1.0, -2.0, 1.237), (1.8, -1.6, 1.649), 0.0, 0.25, 1.20
    )
    assert abs(height - 0.412) < 1e-9
    assert reanchored


def test_tall_floating_geometry_is_not_silently_relocated():
    height, reanchored = support_surface_height(
        (0.0, 0.0, 0.5), (1.0, 1.0, 2.1), 0.0, 0.25, 1.20
    )
    assert height == 2.1
    assert not reanchored


def test_runtime_labels_remain_free_text():
    assert parse_entity_ref("object:red ceramic cup") == (
        "object",
        "red ceramic cup",
    )
    assert parse_entity_ref("room:the Kitchen") == ("room", "kitchen")


def test_ambiguous_object_requires_a_decisive_location_or_id():
    entities = [
        SemanticEntity("O1", "object", "red cup", (1.0, 0.0, 0.8)),
        SemanticEntity("O2", "object", "red cup", (4.0, 1.0, 0.8)),
    ]
    matches = rank_entities("red cup", "object", entities)

    match, error = choose_unambiguous_match(
        matches,
        reference_position=(2.4, 0.5, 0.0),
    )
    assert match is None
    assert error.startswith("ambiguous destination")

    match, error = choose_unambiguous_match(matches, preferred_id="O2")
    assert error is None
    assert match.entity.node_id == "O2"

    ordered = rank_matches_from_position(matches, (0.0, 0.0, 0.0))
    match, error = choose_unambiguous_match(
        ordered,
        reference_position=(0.0, 0.0, 0.0),
    )
    assert error is None
    assert match.entity.node_id == "O1"


def test_place_graph_route_prefers_connected_path():
    places = {
        "P0": Place("P0", (0.0, 0.0, 0.0), 0.5, {"P1": 1.0}),
        "P1": Place(
            "P1",
            (1.0, 0.0, 0.0),
            0.5,
            {"P0": 1.0, "P2": 1.0},
        ),
        "P2": Place("P2", (2.0, 0.0, 0.0), 0.5, {"P1": 1.0}),
    }
    assert astar_route(places, "P0", "P2") == ["P0", "P1", "P2"]


def test_route_uses_connected_standoff_instead_of_nearest_fragment():
    places = {
        "start": Place("start", (0.0, 0.0, 0.0), 0.5, {"approach": 3.0}),
        "approach": Place(
            "approach", (3.0, 0.0, 0.0), 0.5, {"start": 3.0}
        ),
        "fragment": Place("fragment", (3.4, 0.0, 0.0), 0.5),
    }

    route = route_between_nearest_places(
        places,
        start_position=(0.0, 0.0, 0.0),
        target_position=(4.0, 0.0, 0.0),
        min_clearance=0.32,
        target_standoff=(0.5, 1.5),
    )

    assert route == ["start", "approach"]


def test_route_ignores_isolated_nearest_robot_place():
    places = {
        "fragment": Place("fragment", (0.0, 0.0, 0.0), 0.5),
        "start": Place("start", (0.2, 0.0, 0.0), 0.5, {"approach": 2.8}),
        "approach": Place(
            "approach", (3.0, 0.0, 0.0), 0.5, {"start": 2.8}
        ),
    }

    route = route_between_nearest_places(
        places,
        start_position=(0.0, 0.0, 0.0),
        target_position=(4.0, 0.0, 0.0),
        min_clearance=0.32,
        target_standoff=(0.5, 1.5),
    )

    assert route == ["start", "approach"]


def test_room_label_comes_from_object_evidence():
    label, confidence = infer_room_label(["oven", "sink", "counter"])
    assert label == "kitchen"
    assert confidence > 0.0


def test_fuzzy_label_matching_has_a_threshold():
    entities = [
        SemanticEntity("R1", "room", "bedroom", (0.0, 0.0, 0.0)),
    ]
    assert rank_entities("bedrom", "room", entities)[0].entity.node_id == "R1"
    assert rank_entities("table", "room", entities) == []


def test_specific_object_phrase_matches_graph_category():
    entities = [
        SemanticEntity("O1", "object", "ball", (1.0, 2.0, 0.1)),
        SemanticEntity("O2", "object", "bottle", (2.0, 2.0, 0.4)),
    ]

    assert rank_entities("blue ball", "object", entities)[0].entity.node_id == "O1"
    assert rank_entities("red bottle", "object", entities)[0].entity.node_id == "O2"


def test_equal_semantic_matches_prefer_nearest_instance():
    entities = [
        SemanticEntity("far", "object", "ball", (8.0, 0.0, 0.1)),
        SemanticEntity("near", "object", "ball", (2.0, 0.0, 0.1)),
    ]
    matches = rank_entities("blue ball", "object", entities)

    ordered = rank_matches_from_position(matches, (0.0, 0.0, 0.0))

    assert [match.entity.node_id for match in ordered] == ["near", "far"]


def test_repeated_nearby_objects_are_one_semantic_instance():
    entities = [
        SemanticEntity("O1", "object", "bed", (1.0, 2.0, 0.8)),
        SemanticEntity("O2", "object", "bed", (1.1, 2.0, 0.8)),
        SemanticEntity("O3", "object", "bed", (4.0, 2.0, 0.8)),
    ]
    merged = merge_nearby_entities(entities)
    assert len(merged) == 2


def test_object_place_does_not_ignore_missing_standoff():
    places = {
        "near": Place("near", (0.1, 0.0, 0.0), 0.5),
        "far": Place("far", (2.0, 0.0, 0.0), 0.5),
    }

    assert nearest_place(
        places,
        (0.0, 0.0, 0.0),
        min_clearance=0.32,
        target_standoff=(0.65, 1.15),
    ) is None


def test_object_place_honors_valid_standoff():
    places = {
        "too_near": Place("too_near", (0.1, 0.0, 0.0), 0.5),
        "approach": Place("approach", (0.8, 0.0, 0.0), 0.5),
    }

    assert nearest_place(
        places,
        (0.0, 0.0, 0.0),
        min_clearance=0.32,
        target_standoff=(0.65, 1.15),
    ).node_id == "approach"
