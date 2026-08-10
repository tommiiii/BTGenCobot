"""Tests for the closed-set label remapping consumed by Hydra."""

from pathlib import Path
import tempfile
import unittest

import numpy as np

from vision_services.semantic_processing import load_label_grouping


def _write_grouping(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "groups.yaml"
    path.write_text(text, encoding="utf-8")
    return path


class SemanticProcessingTest(unittest.TestCase):
    """Exercise failure-sensitive semantic remapping behavior."""

    def test_grouping_builds_source_label_lookup(self):
        """Every source class maps to exactly one output group."""
        with tempfile.TemporaryDirectory() as directory:
            grouping = load_label_grouping(
                _write_grouping(
                    Path(directory),
                    "groups:\n  - [0, 2]\n  - [1, 3]\n",
                ),
                source_label_count=4,
            )

        np.testing.assert_array_equal(grouping.source_to_group, [0, 1, 0, 1])
        self.assertEqual(grouping.group_count, 2)

    def test_grouping_rejects_semantically_dangerous_configs(self):
        """Bad taxonomies fail rather than silently changing label IDs."""
        cases = [
            ("groups:\n  - [0, 1]\n  - [1, 2]\n", "duplicated"),
            ("groups:\n  - [0]\n  - [2]\n", "missing source labels"),
            ("groups:\n  - [0, 1, 4]\n  - [2, 3]\n", "out of range"),
        ]
        for text, message in cases:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as directory:
                with self.assertRaisesRegex(ValueError, message):
                    load_label_grouping(
                        _write_grouping(Path(directory), text),
                        source_label_count=4,
                    )

    def test_official_one_based_grouping_offset_is_applied(self):
        """MIT-SPARK configs use one-based ADE20K IDs and offset -1."""
        with tempfile.TemporaryDirectory() as directory:
            grouping = load_label_grouping(
                _write_grouping(
                    Path(directory),
                    "offset: -1\ngroups:\n  - {labels: [1, 3]}\n  - {labels: [2, 4]}\n",
                ),
                source_label_count=4,
            )

        np.testing.assert_array_equal(grouping.source_to_group, [0, 1, 0, 1])
