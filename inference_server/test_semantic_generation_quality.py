"""Bounded live quality check for constrained semantic BT generation.

Run while the local server is active:
    .venv/bin/python test_semantic_generation_quality.py
"""

from __future__ import annotations

import argparse
import sys
import time
import xml.etree.ElementTree as ET

import requests

from validation.validator import validate_action_space, validate_bt_xml


CASES = [
    {
        "command": "pick up the red coke can",
        "required": {
            "NavigateSemantic",
            "PickObject",
        },
        "forbidden": {"DetectObject", "ComputePathToPose", "FollowPath"},
        "entity_contains": "object:red coke can",
    },
    {
        "command": "pick up the green coke can and place it on the trash bin",
        "required": {
            "NavigateSemantic",
            "PickObject",
            "PlaceObject",
        },
        "forbidden": {"DetectObject", "ComputePathToPose", "FollowPath"},
        "entity_contains": "object:green coke can",
    },
    {
        "command": "go to the kitchen",
        "required": {"NavigateSemantic"},
        "forbidden": {"NavigateToPose"},
        "entity_contains": "room:kitchen",
    },
    {
        # Deliberately absent from scene_graph_context: runtime labels must
        # remain grammar-constrained strings, not a generated enum.
        "command": "go to the pantry",
        "required": {"NavigateSemantic"},
        "forbidden": {"NavigateToPose"},
        "entity_contains": "room:pantry",
    },
    {
        "command": "pick up the blue teapot",
        "required": {
            "NavigateSemantic",
            "PickObject",
        },
        "forbidden": {"DetectObject", "ComputePathToPose", "FollowPath"},
        "entity_contains": "object:blue teapot",
    },
    {
        "command": "go to the bedroom, if that fails go to the kitchen",
        "required": {"NavigateSemantic"},
        "forbidden": {"NavigateToPose"},
    },
    {
        "command": "place the held object on the trash bin",
        "required": {"NavigateSemantic", "PlaceObject"},
        "forbidden": {
            "NavigateToPose",
            "DetectObject",
            "ComputePathToPose",
            "FollowPath",
        },
        "entity_contains": "object:trash bin",
    },
    {
        "command": "move forward 1 meter and wait 2 seconds",
        "required": {"DriveOnHeading", "Wait"},
        "forbidden": {"NavigateSemantic"},
    },
]


def node_id(element: ET.Element) -> str:
    return element.get("ID", "") if element.tag == "Action" else element.tag


def inspect_case(case: dict, xml: str) -> list[str]:
    failures = []
    valid, error = validate_bt_xml(xml, strict=False)
    if not valid:
        return [f"invalid XML: {error}"]
    action_valid, issues = validate_action_space(xml)
    if not action_valid:
        failures.append(f"invalid action space: {issues}")

    root = ET.fromstring(xml)
    nodes = [element for element in root.iter() if node_id(element)]
    ids = {node_id(element) for element in nodes}
    missing = case["required"] - ids
    unexpected = case["forbidden"] & ids
    if missing:
        failures.append(f"missing actions: {sorted(missing)}")
    if unexpected:
        failures.append(f"forbidden actions: {sorted(unexpected)}")

    semantic_nodes = [
        element for element in nodes if node_id(element) == "NavigateSemantic"
    ]
    expected_ref = case.get("entity_contains")
    if expected_ref and not any(
        expected_ref in element.get("entity_ref", "").lower()
        for element in semantic_nodes
    ):
        failures.append(f'missing semantic reference containing "{expected_ref}"')
    for element in semantic_nodes:
        reference_type = element.get("entity_ref", "").split(":", 1)[0]
        if element.get("entity_type") != reference_type:
            failures.append(
                f"semantic entity_type does not match {element.get('entity_ref')}"
            )

    if case.get("pose_flow"):
        detect_nodes = [
            element for element in nodes if node_id(element) == "DetectObject"
        ]
        pick_nodes = [
            element for element in nodes if node_id(element) == "PickObject"
        ]
        if not detect_nodes or detect_nodes[0].get("object_pose") != "{object_pose}":
            failures.append("DetectObject does not export {object_pose}")
        if not pick_nodes or pick_nodes[0].get("object_pose") != "{object_pose}":
            failures.append("PickObject does not consume {object_pose}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--request-timeout", type=float, default=60.0)
    args = parser.parse_args()

    context = (
        "rooms: kitchen, bedroom, living room\n"
        "objects: red coke can, green coke can, cube, trash bin"
    )
    failed = 0
    started = time.monotonic()
    for index, case in enumerate(CASES, start=1):
        command = case["command"]
        try:
            response = requests.post(
                f"{args.url}/generate_bt",
                json={
                    "command": command,
                    "max_tokens": 1024,
                    "temperature": 0.1,
                    "prompt_format": "alpaca",
                    "use_query_rewriting": True,
                    "scene_graph_context": context,
                },
                timeout=args.request_timeout,
            )
            response.raise_for_status()
            payload = response.json()
            xml = payload.get("bt_xml") or ""
            failures = (
                inspect_case(case, xml)
                if payload.get("success")
                else [payload.get("error") or "generation failed"]
            )
        except Exception as exc:
            failures = [f"request failed: {exc}"]
            xml = ""

        status = "PASS" if not failures else "FAIL"
        print(
            f"[{index}/{len(CASES)}] {status} "
            f"{command!r} ({time.monotonic() - started:.1f}s total)"
        )
        for failure in failures:
            print(f"  - {failure}")
        if failures:
            failed += 1
            if xml:
                print(xml)

    print(
        f"{len(CASES) - failed}/{len(CASES)} cases passed in "
        f"{time.monotonic() - started:.1f}s"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
