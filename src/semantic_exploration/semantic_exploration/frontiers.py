"""Pure occupancy-grid frontier extraction helpers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Frontier:
    cells: tuple[tuple[int, int], ...]
    goal_cell: tuple[int, int]
    information_gain: int


def _neighbor_offsets(connectivity: int = 8):
    if connectivity == 4:
        return ((-1, 0), (1, 0), (0, -1), (0, 1))
    return tuple(
        (dr, dc)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if dr or dc
    )


def frontier_mask(
    grid: np.ndarray,
    occupied_threshold: int = 65,
    clearance_cells: int = 3,
) -> np.ndarray:
    """Return free cells bordering unknown space and clear of obstacles."""
    free = (grid >= 0) & (grid < occupied_threshold)
    unknown = grid < 0
    unknown_neighbor = np.zeros(grid.shape, dtype=bool)
    obstacle_near = np.zeros(grid.shape, dtype=bool)

    rows, cols = grid.shape
    for dr, dc in _neighbor_offsets(8):
        source_r = slice(max(0, -dr), min(rows, rows - dr))
        source_c = slice(max(0, -dc), min(cols, cols - dc))
        target_r = slice(max(0, dr), min(rows, rows + dr))
        target_c = slice(max(0, dc), min(cols, cols + dc))
        unknown_neighbor[target_r, target_c] |= unknown[source_r, source_c]

    obstacles = grid >= occupied_threshold
    for dr in range(-clearance_cells, clearance_cells + 1):
        for dc in range(-clearance_cells, clearance_cells + 1):
            if dr * dr + dc * dc > clearance_cells * clearance_cells:
                continue
            source_r = slice(max(0, -dr), min(rows, rows - dr))
            source_c = slice(max(0, -dc), min(cols, cols - dc))
            target_r = slice(max(0, dr), min(rows, rows + dr))
            target_c = slice(max(0, dc), min(cols, cols + dc))
            obstacle_near[target_r, target_c] |= obstacles[source_r, source_c]

    return free & unknown_neighbor & ~obstacle_near


def extract_frontiers(
    grid: np.ndarray,
    occupied_threshold: int = 65,
    clearance_cells: int = 3,
    min_cluster_cells: int = 6,
    gain_radius_cells: int = 10,
) -> list[Frontier]:
    mask = frontier_mask(grid, occupied_threshold, clearance_cells)
    visited = np.zeros(mask.shape, dtype=bool)
    rows, cols = mask.shape
    result = []

    for row, col in zip(*np.nonzero(mask)):
        if visited[row, col]:
            continue
        queue = deque([(int(row), int(col))])
        visited[row, col] = True
        cells = []
        while queue:
            current = queue.popleft()
            cells.append(current)
            for dr, dc in _neighbor_offsets(8):
                nr, nc = current[0] + dr, current[1] + dc
                if (
                    0 <= nr < rows
                    and 0 <= nc < cols
                    and mask[nr, nc]
                    and not visited[nr, nc]
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        if len(cells) < min_cluster_cells:
            continue

        center_row = sum(cell[0] for cell in cells) / len(cells)
        center_col = sum(cell[1] for cell in cells) / len(cells)
        goal = min(
            cells,
            key=lambda cell: math.hypot(
                cell[0] - center_row,
                cell[1] - center_col,
            ),
        )
        r0 = max(0, goal[0] - gain_radius_cells)
        r1 = min(rows, goal[0] + gain_radius_cells + 1)
        c0 = max(0, goal[1] - gain_radius_cells)
        c1 = min(cols, goal[1] + gain_radius_cells + 1)
        gain = int(np.count_nonzero(grid[r0:r1, c0:c1] < 0))
        result.append(Frontier(tuple(cells), goal, gain))

    return result


def cell_to_world(
    cell: tuple[int, int],
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[float, float]:
    row, col = cell
    return (
        origin_x + (col + 0.5) * resolution,
        origin_y + (row + 0.5) * resolution,
    )
