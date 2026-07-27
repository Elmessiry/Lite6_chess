import logging
from typing import List, Tuple

from .pgn_parser import PGNParser


class FritzMoveDetector:
    """Watches Fritz's clipboard PGN and yields new moves for the robot's color.

    ``window_handler`` and ``pgn_parser`` are injected so this module never
    needs to import pywinauto (Windows-only) -- callers (main.py) construct
    the real ``FritzWindowHandler`` and wire it in.
    """

    def __init__(self, logger: logging.Logger, window_handler, pgn_parser: PGNParser,
                 robot_color: str):
        self.logger = logger
        self.window_handler = window_handler
        self.pgn_parser = pgn_parser
        self.robot_color = robot_color
        self.last_content = None
        self.processed_moves: List[Tuple[str, str]] = []
        # Set when a new game is detected; consumed by the caller (main loop)
        # to publish a reset so the robot clears its capture-zone allocator.
        self.reset_pending = False

    def take_reset_pending(self) -> bool:
        """Return whether a new-game reset is pending, clearing the flag."""
        pending = self.reset_pending
        self.reset_pending = False
        return pending

    def get_new_moves(self) -> List[Tuple[str, str]]:
        """Get any new moves since last check"""
        try:
            content = self.window_handler.copy_game_to_clipboard()
            if not content or content == self.last_content:
                return []

            self.last_content = content
            current_moves = self.pgn_parser.parse_moves(content)

            # A new game (shorter move list, or diverged from what we've
            # already processed) resets tracking so we replay from move 1.
            if current_moves[: len(self.processed_moves)] != self.processed_moves:
                self.logger.info("New game detected -- resetting move tracking")
                self.processed_moves = []
                self.reset_pending = True

            new_moves = current_moves[len(self.processed_moves):]
            self.processed_moves = current_moves

            # Only return moves for robot's color
            return [(move, color) for move, color in new_moves
                    if color.lower() == self.robot_color]

        except Exception as e:
            self.logger.error(f"Error getting moves: {e}")
            return []
