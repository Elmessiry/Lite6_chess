import chess.pgn
import io
import re
import logging
from typing import List, Tuple

class PGNParser:
    def __init__(self, logger: logging.Logger):
        self.logger = logger

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
                    if board.is_en_passant(move):
                        # Captured pawn sits on the to-file at the from-rank,
                        # not on to_square (the en passant target).
                        victim_square = chess.square_name(
                            chess.square(chess.square_file(move.to_square),
                                         chess.square_rank(move.from_square)))
                    else:
                        victim_square = to_square
                    moves.append((f"{victim_square}xx", color))
                    moves.append((f"{from_square}{to_square}", color))
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

                if move.promotion:
                    # Known physical limitation: the robot has no spare
                    # queen (or other piece) to place on the board, so a
                    # promotion move is emitted as a plain move and logged
                    # so the operator is aware manual intervention is needed.
                    self.logger.warning(
                        "Promotion move %s%s (%s): robot cannot place a "
                        "promoted piece -- manual intervention required",
                        from_square, to_square, color)

                board.push(move)

            return moves

        except Exception as e:
            self.logger.error(f"Failed to parse moves: {e}")
            return []
