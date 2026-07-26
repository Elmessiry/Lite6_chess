"""Tests for chess_fritz.move_detector.FritzMoveDetector.

The detector takes its window_handler and pgn_parser via dependency
injection so these tests never import pywinauto (Linux-incompatible).
"""
import logging

from chess_fritz.move_detector import FritzMoveDetector
from chess_fritz.pgn_parser import PGNParser


class FakeWindowHandler:
    """Returns a scripted sequence of clipboard contents, one per call."""

    def __init__(self, contents):
        self._contents = list(contents)

    def copy_game_to_clipboard(self):
        if not self._contents:
            return None
        return self._contents.pop(0)


def make_detector(contents, robot_color="white"):
    logger = logging.getLogger("test")
    handler = FakeWindowHandler(contents)
    parser = PGNParser(logger)
    return FritzMoveDetector(logger, handler, parser, robot_color)


PGN_1_MOVE = "1. e4"
PGN_2_MOVES = "1. e4 e5"
PGN_3_MOVES = "1. e4 e5 2. Nf3"
PGN_DIVERGED_2_MOVES = "1. d4 d5"


def test_longer_move_list_yields_tail_only():
    detector = make_detector([PGN_1_MOVE, PGN_3_MOVES], robot_color="white")
    first = detector.get_new_moves()
    assert first == [("e2e4", "white")]
    second = detector.get_new_moves()
    # Nf3 is white's second move; e5 is black's and filtered out.
    assert second == [("g1f3", "white")]


def test_identical_content_yields_nothing():
    detector = make_detector([PGN_2_MOVES, PGN_2_MOVES], robot_color="white")
    first = detector.get_new_moves()
    assert first == [("e2e4", "white")]
    second = detector.get_new_moves()
    assert second == []


def test_shorter_move_list_resets_and_replays_from_start():
    detector = make_detector([PGN_3_MOVES, PGN_1_MOVE], robot_color="white")
    first = detector.get_new_moves()
    assert first == [("e2e4", "white"), ("g1f3", "white")]
    second = detector.get_new_moves()
    # New (shorter) game detected -- resets tracking and replays from move 1.
    assert second == [("e2e4", "white")]


def test_diverged_same_length_list_resets():
    detector = make_detector([PGN_2_MOVES, PGN_DIVERGED_2_MOVES], robot_color="white")
    first = detector.get_new_moves()
    assert first == [("e2e4", "white")]
    second = detector.get_new_moves()
    # Same length but move 1 differs (e4 vs d4) -- new game, resets.
    assert second == [("d2d4", "white")]


def test_filters_to_robot_color_only():
    detector = make_detector([PGN_2_MOVES], robot_color="black")
    moves = detector.get_new_moves()
    assert moves == [("e7e5", "black")]


def test_no_content_returns_empty_list():
    detector = make_detector([None], robot_color="white")
    assert detector.get_new_moves() == []


def test_first_game_sets_no_reset_pending():
    detector = make_detector([PGN_2_MOVES], robot_color="white")
    detector.get_new_moves()
    assert detector.take_reset_pending() is False


def test_new_game_sets_reset_pending_and_take_clears_it():
    detector = make_detector([PGN_3_MOVES, PGN_1_MOVE], robot_color="white")
    detector.get_new_moves()
    assert detector.take_reset_pending() is False  # first game: no reset
    detector.get_new_moves()                        # shorter list -> new game
    assert detector.take_reset_pending() is True
    assert detector.take_reset_pending() is False   # take() cleared the flag


def test_diverged_game_sets_reset_pending():
    detector = make_detector([PGN_2_MOVES, PGN_DIVERGED_2_MOVES], robot_color="white")
    detector.get_new_moves()
    detector.get_new_moves()
    assert detector.take_reset_pending() is True
