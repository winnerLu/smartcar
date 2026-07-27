import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'clearance_map.py'
SPEC = importlib.util.spec_from_file_location('clearance_map', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_offsets_encode_35cm_minimum_passage_at_5cm_resolution():
    offsets = MODULE.clearance_offsets(0.175, 0.05)

    assert (0, 0) in offsets
    assert (3, 0) in offsets
    assert (-3, 0) in offsets
    assert (4, 0) not in offsets
    assert (2, 2) in offsets
    assert (3, 2) not in offsets


def test_inflation_closes_a_corridor_narrower_than_twice_radius():
    width = 9
    height = 5
    data = [0] * (width * height)
    data[2 * width + 1] = 100
    data[2 * width + 7] = 100
    offsets = MODULE.clearance_offsets(0.175, 0.05)

    inflated = MODULE.inflate_grid(data, width, height, offsets)

    # Each obstacle expands three 5cm cells.  Their hard regions meet at the
    # centre, so there is no traversable centreline between the two walls.
    assert inflated[2 * width + 4] == 100


def test_inflation_marks_unknown_cells_near_an_obstacle():
    data = [-1] * 25
    data[12] = 100

    inflated = MODULE.inflate_grid(
        data, 5, 5, MODULE.clearance_offsets(0.10, 0.05))

    assert inflated[12] == 100
    assert inflated[11] == 100
    assert inflated[0] == -1


def test_startup_escape_restores_only_original_free_cells():
    width = 7
    height = 7
    source = [0] * (width * height)
    source[3 * width + 2] = 100
    source[2 * width + 3] = -1
    expanded = MODULE.inflate_grid(
        source, width, height, MODULE.clearance_offsets(0.175, 0.05))

    escaped = MODULE.restore_startup_free_cells(
        expanded, source, width, height, centre=(3, 3),
        offsets=MODULE.clearance_offsets(0.10, 0.05))

    assert escaped[3 * width + 3] == 0
    assert escaped[3 * width + 2] == 100
    assert escaped[2 * width + 3] == 100
    assert escaped[3 * width + 0] == 100


def test_invalid_grid_is_rejected():
    with pytest.raises(ValueError):
        MODULE.inflate_grid([0], 2, 2, [(0, 0)])
