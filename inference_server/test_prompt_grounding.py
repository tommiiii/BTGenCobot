from core.inference import (
    generate_restricted_grammar,
    parse_allowed_actions,
    parse_planned_entity_refs,
)
from prompts import build_alpaca_prompt


def test_alpaca_prompt_preserves_original_command_with_planner_notes():
    prompt = build_alpaca_prompt(
        "pick up the blue ball then place it on the bed",
        rewritten_input=(
            "Actions: NavigateSemantic, PickObject, NavigateSemantic, PlaceObject\n"
            "Structure: Sequence\n"
            "Description: approach the object, pick it, approach the destination, place it"
        ),
    )

    assert "Original command: pick up the blue ball then place it on the bed" in prompt
    assert "Planner notes:" in prompt


def test_restricted_grammar_bounds_width_and_types_semantic_ports():
    grammar = generate_restricted_grammar(
        ["NavigateSemantic", "PickObject", "PlaceObject"]
    )

    assert "(WS? node_l2)*" not in grammar
    assert grammar.count("(WS? node_l2)?") == 11
    assert '("room" | "object")' in grammar
    assert '("true" | "false")' in grammar


def test_sequence_grammar_preserves_planned_action_order_and_duplicates():
    grammar = generate_restricted_grammar(
        [
            "NavigateSemantic",
            "PickObject",
            "NavigateSemantic",
            "PlaceObject",
        ],
        "Sequence",
    )

    assert "bt_content: planned_sequence" in grammar
    assert (
        "planned_pick_navigatesemantic_1_action WS? pickobject_action WS? "
        "navigatesemantic_action WS? placeobject_action"
    ) in grammar


def test_nonsequential_structure_is_enforced_at_tree_root():
    grammar = generate_restricted_grammar(
        ["NavigateSemantic", "PickObject"],
        "Fallback",
    )

    assert "bt_content: fallback_l1" in grammar
    assert "bt_content: node_l1" not in grammar


def test_reactive_fallback_is_not_misparsed_as_plain_fallback():
    _, structure = parse_allowed_actions(
        "Actions: NavigateSemantic, PickObject\n"
        "Structure: ReactiveFallback"
    )

    assert structure == "ReactiveFallback"


def test_planner_semantic_references_are_bound_in_order():
    rewritten = (
        "Actions: NavigateSemantic, PickObject, NavigateSemantic, PlaceObject\n"
        "Structure: Sequence\n"
        'SemanticRefs: entity_ref="object:blue ball", '
        'entity_ref="object:box"\n'
        'Description: approach entity_ref="object:blue ball", pick it, then '
        'approach entity_ref="object:box" and place it.'
    )
    actions, structure = parse_allowed_actions(rewritten)
    references = parse_planned_entity_refs(rewritten)
    grammar = generate_restricted_grammar(
        actions,
        structure,
        planned_entity_refs=references,
    )

    assert references == ["object:blue ball", "object:box"]
    assert (
        "planned_navigatesemantic_1_action WS? pickobject_action WS? "
        "planned_navigatesemantic_2_action WS? placeobject_action"
    ) in grammar
    assert 'entity_ref=\\"object:blue ball\\"' in grammar
    assert 'entity_ref=\\"object:box\\"' in grammar
    assert 'reacquire=\\"true\\"' in grammar


def test_incomplete_planner_grounding_does_not_bind_wrong_occurrence():
    grammar = generate_restricted_grammar(
        [
            "NavigateSemantic",
            "PickObject",
            "NavigateSemantic",
            "PlaceObject",
        ],
        "Sequence",
        planned_entity_refs=["object:blue ball"],
    )

    assert "planned_navigatesemantic_1_action" not in grammar
    assert "planned_pick_navigatesemantic_1_action" in grammar
