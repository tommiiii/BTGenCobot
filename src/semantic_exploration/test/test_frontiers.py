import numpy as np

from semantic_exploration.frontiers import (
    cell_to_world,
    extract_frontiers,
    select_coverage_goal,
)


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


def test_coverage_goal_moves_camera_beyond_lidar_viewpoint():
    grid = np.full((80, 120), 100, dtype=np.int16)
    grid[10:70, 10:110] = 0

    goal = select_coverage_goal(
        grid,
        observed_cells=[(40, 20)],
        robot_cell=(40, 20),
        clearance_cells=2,
        coverage_radius_cells=18,
        candidate_stride_cells=5,
        min_uncovered_cells=40,
    )

    assert goal is not None
    assert np.hypot(goal[0] - 40, goal[1] - 20) > 18
    assert grid[goal] == 0


def test_coverage_goal_finishes_when_free_space_is_observed():
    grid = np.zeros((30, 30), dtype=np.int16)

    goal = select_coverage_goal(
        grid,
        observed_cells=[(15, 15)],
        robot_cell=(15, 15),
        clearance_cells=0,
        coverage_radius_cells=30,
        min_uncovered_cells=10,
    )

    assert goal is None


def test_coverage_goal_respects_obstacles_and_exclusions():
    grid = np.zeros((50, 50), dtype=np.int16)
    grid[:, 25] = 100

    goal = select_coverage_goal(
        grid,
        observed_cells=[(25, 10)],
        robot_cell=(25, 10),
        clearance_cells=3,
        coverage_radius_cells=10,
        candidate_stride_cells=1,
        min_uncovered_cells=20,
        excluded_cells=[(25, 35)],
        exclusion_radius_cells=12,
    )

    assert goal is not None
    assert abs(goal[1] - 25) > 3
    assert np.hypot(goal[0] - 25, goal[1] - 35) > 12


def test_coverage_goal_stays_in_robot_connected_component():
    grid = np.full((40, 60), 100, dtype=np.int16)
    grid[5:35, 5:25] = 0
    grid[5:35, 35:55] = 0

    goal = select_coverage_goal(
        grid,
        observed_cells=[(20, 10)],
        robot_cell=(20, 10),
        clearance_cells=1,
        coverage_radius_cells=8,
        candidate_stride_cells=1,
        min_uncovered_cells=10,
    )

    assert goal is not None
    assert goal[1] < 25
