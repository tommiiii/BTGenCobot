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


def select_coverage_goal(
    grid: np.ndarray,
    observed_cells: list[tuple[int, int]],
    robot_cell: tuple[int, int],
    *,
    occupied_threshold: int = 65,
    clearance_cells: int = 3,
    coverage_radius_cells: int = 20,
    candidate_stride_cells: int = 5,
    min_uncovered_cells: int = 20,
    excluded_cells: list[tuple[int, int]] | None = None,
    exclusion_radius_cells: int = 5,
) -> tuple[int, int] | None:
    """Choose a safe viewpoint that adds camera coverage to a known map.

    Lidar can clear occupancy frontiers while the RGB-D camera has only seen
    the same space from far away.  This selector treats successful observation
    viewpoints as discs and sends the robot into remaining known free space.
    It intentionally runs only after normal frontier candidates are exhausted.
    """
    rows, cols = grid.shape
    free = (grid >= 0) & (grid < occupied_threshold)
    blocked = ~free
    blocked_near = np.zeros(grid.shape, dtype=bool)
    for dr in range(-clearance_cells, clearance_cells + 1):
        for dc in range(-clearance_cells, clearance_cells + 1):
            if dr * dr + dc * dc > clearance_cells * clearance_cells:
                continue
            source_r = slice(max(0, -dr), min(rows, rows - dr))
            source_c = slice(max(0, -dc), min(cols, cols - dc))
            target_r = slice(max(0, dr), min(rows, rows + dr))
            target_c = slice(max(0, dc), min(cols, cols + dc))
            blocked_near[target_r, target_c] |= blocked[source_r, source_c]
    safe = free & ~blocked_near

    # A safe-looking cell can still belong to another room-shaped island in
    # the occupancy grid. Restrict coverage goals to the connected component
    # that contains (or is nearest to) the robot, otherwise Nav2 repeatedly
    # spends its full timeout trying to reach geometrically impossible goals.
    robot_row = min(max(int(robot_cell[0]), 0), rows - 1)
    robot_col = min(max(int(robot_cell[1]), 0), cols - 1)
    safe_cells = np.argwhere(safe)
    if safe_cells.size == 0:
        return None
    if safe[robot_row, robot_col]:
        seed = (robot_row, robot_col)
    else:
        distances = (
            (safe_cells[:, 0] - robot_row) ** 2
            + (safe_cells[:, 1] - robot_col) ** 2
        )
        nearest_index = int(np.argmin(distances))
        seed = tuple(int(value) for value in safe_cells[nearest_index])

    reachable = np.zeros(grid.shape, dtype=bool)
    queue = deque([seed])
    reachable[seed] = True
    while queue:
        row, col = queue.popleft()
        for dr, dc in _neighbor_offsets(4):
            nr, nc = row + dr, col + dc
            if (
                0 <= nr < rows
                and 0 <= nc < cols
                and safe[nr, nc]
                and not reachable[nr, nc]
            ):
                reachable[nr, nc] = True
                queue.append((nr, nc))

    covered = np.zeros(grid.shape, dtype=bool)
    radius = max(1, coverage_radius_cells)
    for row, col in observed_cells:
        r0, r1 = max(0, row - radius), min(rows, row + radius + 1)
        c0, c1 = max(0, col - radius), min(cols, col + radius + 1)
        rr, cc = np.ogrid[r0:r1, c0:c1]
        covered[r0:r1, c0:c1] |= (
            (rr - row) ** 2 + (cc - col) ** 2 <= radius * radius
        )
    uncovered = free & ~covered
    if int(np.count_nonzero(uncovered)) < min_uncovered_cells:
        return None

    allowed = safe & reachable & uncovered
    for row, col in excluded_cells or []:
        r0 = max(0, row - exclusion_radius_cells)
        r1 = min(rows, row + exclusion_radius_cells + 1)
        c0 = max(0, col - exclusion_radius_cells)
        c1 = min(cols, col + exclusion_radius_cells + 1)
        allowed[r0:r1, c0:c1] = False

    stride = max(1, candidate_stride_cells)
    best: tuple[float, tuple[int, int]] | None = None
    for row, col in zip(*np.nonzero(allowed)):
        if row % stride or col % stride:
            continue
        r0, r1 = max(0, row - radius), min(rows, row + radius + 1)
        c0, c1 = max(0, col - radius), min(cols, col + radius + 1)
        gain = int(np.count_nonzero(uncovered[r0:r1, c0:c1]))
        if gain < min_uncovered_cells:
            continue
        distance = math.hypot(row - robot_cell[0], col - robot_cell[1])
        score = float(gain) - 0.25 * distance
        candidate = (int(row), int(col))
        if best is None or score > best[0]:
            best = (score, candidate)
    return None if best is None else best[1]


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
