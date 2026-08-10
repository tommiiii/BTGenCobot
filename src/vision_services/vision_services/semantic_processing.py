"""Small, testable helpers for Hydra's closed-set label remapping."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class LabelGrouping:
    """A complete source-class to Hydra-class lookup."""

    source_to_group: np.ndarray

    @property
    def group_count(self) -> int:
        return int(self.source_to_group.max()) + 1


def load_label_grouping(
    path: Path,
    source_label_count: int = 150,
) -> LabelGrouping:
    """Load and strictly validate an argmax-then-relabel grouping.

    This mirrors MIT-SPARK semantic_inference: the network first selects one
    ADE20K class per pixel, then that class ID is mapped into Hydra's compact
    label space. Summing probabilities across groups would bias categories
    according to how many source classes they contain.
    """
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    offset = int(config.get("offset", 0))
    raw_groups = config.get("groups")
    if not isinstance(raw_groups, list) or not raw_groups:
        raise ValueError(f"{path}: 'groups' must be a non-empty list")

    groups: list[list[int]] = []
    for entry in raw_groups:
        labels = entry.get("labels") if isinstance(entry, dict) else entry
        if not isinstance(labels, list):
            raise ValueError(f"{path}: every group must contain a label list")
        groups.append([int(value) + offset for value in labels])

    if len(groups) > 256:
        raise ValueError(f"{path}: mono8 output supports at most 256 groups")

    source_to_group = np.full(source_label_count, -1, dtype=np.int16)
    for group, labels in enumerate(groups):
        for source in labels:
            if source < 0 or source >= source_label_count:
                raise ValueError(f"{path}: source label {source} is out of range")
            if source_to_group[source] >= 0:
                raise ValueError(f"{path}: source label {source} is duplicated")
            source_to_group[source] = group

    missing = np.flatnonzero(source_to_group < 0)
    if missing.size:
        raise ValueError(f"{path}: missing source labels {missing.tolist()}")

    return LabelGrouping(source_to_group)
