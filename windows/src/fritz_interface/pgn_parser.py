import chess.pgn
import io
import re
import logging
from typing import List, Tuple

class PGNParser:
    def __init__(self, logger: logging.Logger):
        self.logger = logger

    def _process_capture_move(self, move, from_square: str, to_square: str) -> List[str]:
        """Process a capture move into two parts"""
        return [f"{to_square}xx", f"{from_square}{to_square}"]

    def parse_moves(self, pgn_text: str) -> List[Tuple[str, str]]:
        """Parse all moves from PGN text"""
        try:
            # Remove the result and any annotations
            clean_pgn = re.sub(r'\s+(0-1|1-0|1/2-1/2|\*).*$', '', pgn_text)
            clean_pgn = re.sub(r'\{[^}]*\}', '', clean_pgn)

            # Parse PGN
            game = chess.pgn.read_game(io.StringIO(clean_pgn))
            if not game:
                return []

            moves = []
            board = chess.Board()

            for move in game.mainline_moves():
                from_square = chess.square_name(move.from_square)
                to_square = chess.square_name(move.to_square)
                color = "white" if board.turn else "black"

                if board.is_capture(move):
                    processed_moves = self._process_capture_move(move, from_square, to_square)
                    moves.extend((m, color) for m in processed_moves)
                elif board.is_castling(move):
                    # First add the king's move
                    moves.append((f"{from_square}{to_square}", color))
                    # Then add the rook's move
                    rank = "1" if color == "white" else "8"
                    if to_square == f"c{rank}":  # Queenside castle
                        moves.append((f"a{rank}d{rank}", color))
                    else:  # Kingside castle
                        moves.append((f"h{rank}f{rank}", color))
                else:
                    # Regular move
                    moves.append((f"{from_square}{to_square}", color))

                board.push(move)

            return moves

        except Exception as e:
            self.logger.error(f"Failed to parse moves: {e}")
            return []
