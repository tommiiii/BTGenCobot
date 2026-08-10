"""Pure helpers for compiling semantic navigation actions."""

from __future__ import annotations

import xml.etree.ElementTree as ET


def ensure_follow_path_goal_checker(
    xml_string: str,
    default_goal_checker: str = 'general_goal_checker',
) -> str:
    """Give every FollowPath an explicit checker without overriding intent."""
    root = ET.fromstring(xml_string)
    for element in root.iter():
        node_id = element.get('ID') if element.tag == 'Action' else element.tag
        if node_id == 'FollowPath' and not element.get('goal_checker_id', '').strip():
            element.set('goal_checker_id', default_goal_checker)
    return ET.tostring(root, encoding='unicode')


def reacquisition_requested(semantic_node: ET.Element) -> bool:
    """Return whether manipulation should reacquire its target after navigation.

    Reacquisition is the safe default: a navigation-time observation can be
    several metres older and farther away than the final manipulation view.
    """
    value = semantic_node.get('reacquire', 'true').strip().lower()
    return value not in {'false', '0', 'no', 'off'}


def supply_live_pose_to_following_pick(
    root: ET.Element,
    semantic_node: ET.Element,
    live_object_pose: str,
) -> bool:
    """Supply a navigation observation to PickObject only when explicitly kept."""
    if not live_object_pose or reacquisition_requested(semantic_node):
        return False

    document_order = list(root.iter())
    semantic_index = document_order.index(semantic_node)
    for candidate in document_order[semantic_index + 1:]:
        candidate_id = (
            candidate.get('ID')
            if candidate.tag == 'Action'
            else candidate.tag
        )
        if candidate_id == 'NavigateSemantic':
            break
        if candidate_id == 'PickObject':
            candidate.set('object_pose', live_object_pose)
            return True
    return False
