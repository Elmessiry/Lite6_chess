"""Tests for chess_fritz.pgn_parser.PGNParser."""
import logging

from chess_fritz.pgn_parser import PGNParser


def make_parser():
    return PGNParser(logging.getLogger("test"))


def test_en_passant_removes_actual_victim_square():
    # 1.e4 a6 2.e5 d5 3.exd6 e.p. -- victim pawn is on d5, NOT d6
    pgn = "1. e4 a6 2. e5 d5 3. exd6"
    moves = make_parser().parse_moves(pgn)
    assert ("d5xx", "white") in moves          # clear the real victim
    assert ("e5d6", "white") in moves          # then the capturing move
    assert ("d6xx", "white") not in moves      # old buggy emission


def test_regular_capture_clears_destination():
    pgn = "1. e4 d5 2. exd5"
    moves = make_parser().parse_moves(pgn)
    assert moves[-2:] == [("d5xx", "white"), ("e4d5", "white")]


def test_kingside_castling_emits_king_then_rook():
    pgn = "1. e4 e5 2. Nf3 Nc6 3. Bc4 Bc5 4. O-O"
    moves = make_parser().parse_moves(pgn)
    assert moves[-2:] == [("e1g1", "white"), ("h1f1", "white")]


def test_queenside_castling_emits_king_then_rook():
    pgn = "1. d4 d5 2. Nc3 Nc6 3. Bf4 Bf5 4. Qd2 Qd7 5. O-O-O"
    moves = make_parser().parse_moves(pgn)
    assert moves[-2:] == [("e1c1", "white"), ("a1d1", "white")]


def test_black_kingside_castling_uses_rank_8():
    pgn = "1. e4 e5 2. Nf3 Nf6 3. Bc4 Bc5 4. Nc3 O-O"
    moves = make_parser().parse_moves(pgn)
    assert moves[-2:] == [("e8g8", "black"), ("h8f8", "black")]


def test_promotion_emits_move_and_logs_warning(caplog):
    pgn = "1. g4 h5 2. gxh5 Nf6 3. h6 a6 4. hxg7 a5 5. g8=Q"
    with caplog.at_level(logging.WARNING):
        moves = make_parser().parse_moves(pgn)
    assert ("g7g8", "white") in moves
    assert any("promotion" in r.message.lower() for r in caplog.records)


def test_disambiguation_knight_move():
    # Two knights can reach d2; Nbd2 disambiguates by file
    pgn = "1. Nf3 Nf6 2. Nc3 Nc6 3. d4 d5 4. Nb1 Nb8 5. N1d2"
    moves = make_parser().parse_moves(pgn)
    # Just verify it parses without error and produces the expected squares
    assert ("b1d2", "white") in moves


def test_check_and_mate_suffixes_are_handled():
    # Fool's mate -- final move has '#' suffix
    pgn = "1. f3 e5 2. g4 Qh4#"
    moves = make_parser().parse_moves(pgn)
    assert moves[-1] == ("d8h4", "black")


def test_empty_pgn_returns_empty_list():
    assert make_parser().parse_moves("") == []


def test_garbage_pgn_returns_empty_list():
    assert make_parser().parse_moves("this is not a pgn at all !!!") == []
