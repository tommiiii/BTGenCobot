"""Pure helpers for compiling semantic navigation actions."""

from __future__ import annotations

import xml.etree.ElementTree as ET


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
