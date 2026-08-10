import xml.etree.ElementTree as ET
import unittest

from bt_text_interface.semantic_compilation import (
    reacquisition_requested,
    supply_live_pose_to_following_pick,
)


def _tree(reacquire=None):
    attribute = '' if reacquire is None else f' reacquire="{reacquire}"'
    return ET.fromstring(
        '<root><BehaviorTree><Sequence>'
        f'<Action ID="NavigateSemantic"{attribute}/>'
        '<Action ID="PickObject" object_description="blue ball"/>'
        '</Sequence></BehaviorTree></root>'
    )


class SemanticCompilationTest(unittest.TestCase):

    def test_reacquire_true_does_not_reuse_navigation_pose(self):
        root = _tree('true')
        navigate, pick = list(root.iter('Action'))

        self.assertTrue(reacquisition_requested(navigate))
        self.assertFalse(
            supply_live_pose_to_following_pick(root, navigate, '[1;2;3]')
        )
        self.assertNotIn('object_pose', pick.attrib)

    def test_reacquire_defaults_to_true(self):
        root = _tree()
        navigate, pick = list(root.iter('Action'))

        self.assertTrue(reacquisition_requested(navigate))
        self.assertFalse(
            supply_live_pose_to_following_pick(root, navigate, '[1;2;3]')
        )
        self.assertNotIn('object_pose', pick.attrib)

    def test_reacquire_false_reuses_navigation_pose(self):
        root = _tree('false')
        navigate, pick = list(root.iter('Action'))

        self.assertFalse(reacquisition_requested(navigate))
        self.assertTrue(
            supply_live_pose_to_following_pick(root, navigate, '[1;2;3]')
        )
        self.assertEqual(pick.get('object_pose'), '[1;2;3]')

    def test_pose_is_not_forwarded_across_another_navigation(self):
        root = ET.fromstring(
            '<root><BehaviorTree><Sequence>'
            '<Action ID="NavigateSemantic" reacquire="false"/>'
            '<Action ID="NavigateSemantic" reacquire="false"/>'
            '<Action ID="PickObject"/>'
            '</Sequence></BehaviorTree></root>'
        )
        first_navigate, _, pick = list(root.iter('Action'))

        self.assertFalse(
            supply_live_pose_to_following_pick(
                root, first_navigate, '[1;2;3]'
            )
        )
        self.assertNotIn('object_pose', pick.attrib)
