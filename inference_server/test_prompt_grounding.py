from core.inference import generate_restricted_grammar
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
