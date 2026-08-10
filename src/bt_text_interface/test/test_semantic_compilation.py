import xml.etree.ElementTree as ET
import unittest

from bt_text_interface.semantic_compilation import (
    ensure_follow_path_goal_checker,
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

    def test_missing_follow_path_checker_defaults_to_general(self):
        compiled = ensure_follow_path_goal_checker(
            '<root><BehaviorTree><Sequence>'
            '<FollowPath path="{path}" controller_id="FollowPath"/>'
            '</Sequence></BehaviorTree></root>'
        )
        follow_path = ET.fromstring(compiled).find('.//FollowPath')
        self.assertEqual(
            follow_path.get('goal_checker_id'),
            'general_goal_checker',
        )

    def test_explicit_manipulation_checker_is_preserved(self):
        compiled = ensure_follow_path_goal_checker(
            '<root><BehaviorTree><Sequence>'
            '<FollowPath goal_checker_id="manipulation_goal_checker"/>'
            '</Sequence></BehaviorTree></root>'
        )
        follow_path = ET.fromstring(compiled).find('.//FollowPath')
        self.assertEqual(
            follow_path.get('goal_checker_id'),
            'manipulation_goal_checker',
        )

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
