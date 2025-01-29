import logging
from typing import List, Tuple, Optional
from .window_handler import FritzWindowHandler
from .pgn_parser import PGNParser

class FritzMoveDetector:
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.window_handler = FritzWindowHandler(logger)
        self.pgn_parser = PGNParser(logger)
        self.last_content = None
        self.processed_moves = []
        self.robot_color = self._get_robot_color()

    def _get_robot_color(self) -> str:
        """Prompt user to specify robot's color"""
        while True:
            color = input("Enter robot's color (white/black): ").lower()
            if color in ['white', 'black']:
                return color
            print("Invalid input. Please enter 'white' or 'black'.")

    def get_new_moves(self) -> List[Tuple[str, str]]:
        """Get any new moves since last check"""
        try:
            content = self.window_handler.copy_game_to_clipboard()
            if not content or content == self.last_content:
                return []

            self.last_content = content
            current_moves = self.pgn_parser.parse_moves(content)

            # Find new moves
            if len(current_moves) > len(self.processed_moves):
                new_moves = current_moves[len(self.processed_moves):]
                self.processed_moves = current_moves
                # Only return moves for robot's color
                return [(move, color) for move, color in new_moves 
                       if color.lower() == self.robot_color]

            return []

        except Exception as e:
            self.logger.error(f"Error getting moves: {e}")
            return []