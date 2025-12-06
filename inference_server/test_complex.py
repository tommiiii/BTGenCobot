#!/usr/bin/env python3
"""Test complex BT generation with multiple control nodes"""
import requests
import json
import xml.etree.ElementTree as ET

BASE_URL = "http://localhost:8080"

def count_nodes(xml_string):
    """Count different node types in the XML"""
    try:
        root = ET.fromstring(xml_string)
        counts = {
            "Action": 0,
            "Condition": 0,
            "Sequence": 0,
            "Fallback": 0,
            "Parallel": 0,
            "ReactiveSequence": 0,
            "ReactiveFallback": 0,
            "Inverter": 0,
            "ForceSuccess": 0,
            "ForceFailure": 0,
            "Repeat": 0,
            "RetryUntilSuccessful": 0,
        }
        
        for elem in root.iter():
            tag = elem.tag
            if tag in counts:
                counts[tag] += 1
                
        return {k: v for k, v in counts.items() if v > 0}
    except:
        return {}

def pretty_xml(xml_string):
    """Format XML with indentation"""
    try:
        root = ET.fromstring(xml_string)
        ET.indent(root)
        return ET.tostring(root, encoding='unicode')
    except:
        return xml_string

def test_generation(command, description=""):
    """Test BT generation for a command"""
    print(f"\n{'='*80}")
    print(f"COMMAND: {command}")
    if description:
        print(f"EXPECTED: {description}")
    print('='*80)
    
    response = requests.post(
        f"{BASE_URL}/generate_bt",
        json={
            "command": command,
            "prompt_format": "alpaca",
            "use_query_rewriting": True,
            "temperature": 0.3,
            "max_tokens": 1024
        },
        timeout=120
    )
    
    data = response.json()
    
    if data.get("success"):
        xml = data.get("bt_xml", "")
        print(f"\n✅ SUCCESS (generation: {data.get('generation_time_ms')}ms, total: {data.get('total_time_ms')}ms)")
        print(f"\nNode counts: {count_nodes(xml)}")
        print(f"\nGenerated XML:\n{pretty_xml(xml)}")
        return True, xml
    else:
        print(f"\n❌ FAILED: {data.get('error')}")
        if data.get("bt_xml"):
            print(f"\nPartial XML:\n{pretty_xml(data.get('bt_xml', ''))}")
        return False, data.get("bt_xml")

# Complex test cases requiring multiple control nodes
test_cases = [
    (
        "find the red ball, navigate to it, pick it up, then place it on the table",
        "Should have: Sequence with DetectObject, ComputePathToPose, FollowPath, PickObject, PlaceObject"
    ),
    (
        "patrol between the kitchen and living room, checking for obstacles",
        "Should have: Sequence/Fallback with multiple NavigateToPose or path planning"
    ),
    (
        "try to pick up the cube, if it fails back up and try again up to 3 times",
        "Should have: RetryUntilSuccessful or Repeat with Fallback containing BackUp"
    ),
    (
        "spin left while checking if the goal is reached, stop when done",
        "Should have: ReactiveSequence or Parallel with SpinLeft and GoalReached condition"
    ),
    (
        "navigate to the charging station, but if battery is not low just wait",
        "Should have: Fallback with IsBatteryLow condition and NavigateToPose vs Wait"
    ),
    (
        "search for any object by spinning around, when found navigate to it and pick it up",
        "Should have: Sequence with SpinLeft/SpinRight, DetectObject, navigation, PickObject"
    ),
]

print("="*80)
print("COMPLEX BEHAVIOR TREE GENERATION TESTS")
print("="*80)

results = []
for command, description in test_cases:
    success, xml = test_generation(command, description)
    results.append((command, success))

print("\n" + "="*80)
print("SUMMARY")
print("="*80)
for command, success in results:
    status = "✅" if success else "❌"
    print(f"{status} {command[:60]}...")

passed = sum(1 for _, s in results if s)
print(f"\nTotal: {passed}/{len(results)} passed")
