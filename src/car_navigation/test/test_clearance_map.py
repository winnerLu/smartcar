import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'clearance_map.py'
SPEC = importlib.util.spec_from_file_location('clearance_map', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_offsets_encode_30cm_minimum_passage_at_5cm_resolution():
    offsets = MODULE.clearance_offsets(0.15, 0.05)

    assert (0, 0) in offsets
    assert (3, 0) in offsets
    assert (-3, 0) in offsets
    assert (4, 0) not in offsets
    assert (2, 2) in offsets
    assert (3, 2) not in offsets


def test_closing_fills_a_corridor_narrower_than_twice_radius():
    width = 9
    height = 15
    data = [0] * (width * height)
    for y in range(height):
        data[y * width + 1] = 100
        data[y * width + 7] = 100
    offsets = MODULE.clearance_offsets(0.15, 0.05)

    filtered = MODULE.close_narrow_passages(
        data, width, height, offsets)

    # The inner wall edges are 30cm apart, so the known-free corridor closes.
    assert filtered[7 * width + 4] == 100


def test_closing_does_not_thicken_an_isolated_obstacle():
    width = 15
    height = 15
    data = [0] * (width * height)
    centre = 7 * width + 7
    data[centre] = 100

    filtered = MODULE.close_narrow_passages(
        data, width, height, MODULE.clearance_offsets(0.15, 0.05))

    assert filtered == data


def test_closing_leaves_a_wide_corridor_open():
    width = 11
    height = 15
    source = [0] * (width * height)
    for y in range(height):
        source[y * width + 1] = 100
        source[y * width + 9] = 100

    filtered = MODULE.close_narrow_passages(
        source, width, height, MODULE.clearance_offsets(0.15, 0.05))

    # The inner wall edges are 40cm apart, above the 30cm threshold.
    assert filtered[7 * width + 5] == 0
    assert filtered == source


def test_closing_never_converts_unknown_cells():
    width = 9
    height = 15
    data = [0] * (width * height)
    for y in range(height):
        data[y * width + 1] = 100
        data[y * width + 7] = 100
    data[7 * width + 4] = -1

    filtered = MODULE.close_narrow_passages(
        data, width, height, MODULE.clearance_offsets(0.15, 0.05))

    assert filtered[7 * width + 4] == -1


def test_invalid_grid_is_rejected():
    with pytest.raises(ValueError):
        MODULE.close_narrow_passages([0], 2, 2, [(0, 0)])
