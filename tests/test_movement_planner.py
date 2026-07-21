"""Unit tests for the pure-math movement planner.

MovementPlanner imports no ROS symbols, so it runs in the plain pytest
environment. Config is injected as a dict fixture matching the real
``board_config.yaml`` numbers, so expected coordinates are hand-computed
against those calibrated values.
"""
import logging

import pytest

from chess_robot.movement.movement_planner import MovementPlanner

LOGGER = logging.getLogger("test.movement_planner")

# Mirrors the calibrated values in config/board_config.yaml.
BOARD_CONFIG = {
    "board": {
        "origin": {"x": 0.15, "y": -0.2, "z": 0.015},
        "square_size": 0.03,
        "piece_height": 0.103,
        "hover_height": 0.10,
    },
    "capture_zone": {
        "origin": {"x": 0.15, "y": 0.1},
        "dimensions": {"width": 0.09, "height": 0.24},
        "grid": {"rows": 8, "cols": 3},
    },
}


@pytest.fixture
def planner():
    return MovementPlanner(LOGGER, config=BOARD_CONFIG)


def approx(value):
    return pytest.approx(value, abs=1e-9)


# --- get_coordinates -------------------------------------------------------

@pytest.mark.parametrize(
    "square, expected",
    [
        # a1: file_idx 0, rank_idx 0 -> x = 0.15 + 7*0.03 + 0.015, y = -0.2 + 0.015
        ("a1", (0.375, -0.185, 0.015)),
        # h8: file_idx 7, rank_idx 7 -> x = 0.15 + 0 + 0.015, y = -0.2 + 7*0.03 + 0.015
        ("h8", (0.165, 0.025, 0.015)),
        # e4: file_idx 4, rank_idx 3 -> x = 0.15 + 4*0.03 + 0.015, y = -0.2 + 4*0.03 + 0.015
        ("e4", (0.285, -0.065, 0.015)),
    ],
)
def test_get_coordinates(planner, square, expected):
    x, y, z = planner.get_coordinates(square)
    assert (x, y, z) == (approx(expected[0]), approx(expected[1]), approx(expected[2]))


def test_get_coordinates_is_case_insensitive(planner):
    assert planner.get_coordinates("A1") == planner.get_coordinates("a1")


# --- validate_square -------------------------------------------------------

@pytest.mark.parametrize("square", ["a1", "h8", "A1", "e4"])
def test_validate_square_accepts(planner, square):
    assert planner.validate_square(square) is True


@pytest.mark.parametrize("square", ["i1", "a9", "a", "a10", "", "11", "aa"])
def test_validate_square_rejects(planner, square):
    assert planner.validate_square(square) is False


# --- capture zone ----------------------------------------------------------

def test_get_next_capture_position_round_trip(planner):
    # 24 slots (8 rows x 3 cols); hand back all of them in order.
    positions = [planner.get_next_capture_position() for _ in range(24)]
    assert positions == list(range(24))


def test_capture_zone_exhaustion_raises(planner):
    for _ in range(24):
        planner.get_next_capture_position()
    with pytest.raises(RuntimeError):
        planner.get_next_capture_position()


def test_get_capture_coordinates_bounds(planner):
    with pytest.raises(ValueError):
        planner.get_capture_coordinates(24)
    with pytest.raises(ValueError):
        planner.get_capture_coordinates(-1)


def test_get_capture_coordinates_first_cell(planner):
    # index 0 -> row 0, col 0. cell_width = 0.09/3 = 0.03, cell_height = 0.24/8 = 0.03
    # x = 0.15 + 0.5*0.03 = 0.165, y = 0.1 + 0.5*0.03 = 0.115, z = board origin z
    x, y, z = planner.get_capture_coordinates(0)
    assert (x, y, z) == (approx(0.165), approx(0.115), approx(0.015))


# --- movement sequences ----------------------------------------------------

def test_create_movement_sequence_shape(planner):
    seq = planner.create_movement_sequence("a1", "h8")
    assert len(seq) == 8
    # Gripper steps grab (idx 2) then release (idx 6).
    assert seq[2]["type"] == "gripper" and seq[2]["action"] is True
    assert seq[6]["type"] == "gripper" and seq[6]["action"] is False
    move_indices = [i for i, s in enumerate(seq) if s["type"] == "move"]
    assert move_indices == [0, 1, 3, 4, 5, 7]


def test_movement_sequence_z_math(planner):
    seq = planner.create_movement_sequence("a1", "h8")
    z = BOARD_CONFIG["board"]["origin"]["z"]
    piece = BOARD_CONFIG["board"]["piece_height"]
    hover = BOARD_CONFIG["board"]["hover_height"]
    # Start hover above, then lower to piece height.
    assert seq[0]["position"][2] == approx(z + piece + hover)
    assert seq[1]["position"][2] == approx(z + piece)
    # Lift back to hover after grab.
    assert seq[3]["position"][2] == approx(z + piece + hover)
    # Final retreat to hover over destination.
    assert seq[7]["position"][2] == approx(z + piece + hover)


def test_create_capture_movement_sequence_shape(planner):
    seq = planner.create_capture_movement_sequence("a1", 0)
    assert len(seq) == 8
    assert seq[2]["type"] == "gripper" and seq[2]["action"] is True
    assert seq[6]["type"] == "gripper" and seq[6]["action"] is False
    # Destination of the capture move is the capture-zone cell 0.
    cap_x, cap_y, _ = planner.get_capture_coordinates(0)
    assert seq[4]["position"][0] == approx(cap_x)
    assert seq[4]["position"][1] == approx(cap_y)
