import json

from core.query_rewriter import _local_rewrite


CONTEXT = json.dumps(
    {"ready": True, "rooms": ["living room"], "objects": ["bed"]}
)


def test_navigation_rewrite_uses_runtime_room_type():
    rewritten = _local_rewrite("go to the living room", CONTEXT)
    assert 'entity_ref="room:living room"' in rewritten


def test_navigation_rewrite_uses_runtime_object_type():
    rewritten = _local_rewrite("go to the bed", CONTEXT)
    assert 'entity_ref="object:bed"' in rewritten


def test_unknown_ready_destination_uses_object_discovery():
    rewritten = _local_rewrite("go to the table", CONTEXT)
    assert 'entity_ref="object:table"' in rewritten


def test_pick_and_place_rewrite_uses_graph_for_both_approaches():
    rewritten = _local_rewrite(
        "pick up the blue ball and place it on the bed",
        CONTEXT,
    )
    assert 'entity_ref="object:blue ball"' in rewritten
    assert 'entity_ref="object:bed"' in rewritten
    assert rewritten.count("NavigateSemantic") >= 3  # action list + descriptions


def test_pick_then_place_does_not_leak_sequence_connector_into_object():
    rewritten = _local_rewrite(
        "pick up the blue ball then place it on the bed",
        CONTEXT,
    )
    assert 'entity_ref="object:blue ball"' in rewritten
    assert "blue ball then" not in rewritten
    assert 'entity_ref="object:bed"' in rewritten


def test_place_rewrite_approaches_destination_from_graph():
    rewritten = _local_rewrite("place the held object on the bed", CONTEXT)
    assert "Actions: NavigateSemantic, PlaceObject" in rewritten
    assert 'entity_ref="object:bed"' in rewritten
