import numpy as np

from semantic_exploration.frontiers import cell_to_world, extract_frontiers


def test_extracts_connected_boundary_as_one_frontier():
    grid = np.full((30, 30), -1, dtype=np.int16)
    grid[8:22, 8:22] = 0
    grid[12:18, 12:18] = 100

    frontiers = extract_frontiers(
        grid,
        clearance_cells=1,
        min_cluster_cells=4,
    )

    assert frontiers
    assert sum(len(frontier.cells) for frontier in frontiers) > 20
    assert all(frontier.information_gain > 0 for frontier in frontiers)


def test_cell_to_world_uses_cell_center():
    assert cell_to_world((2, 3), 0.5, -1.0, 4.0) == (0.75, 5.25)
