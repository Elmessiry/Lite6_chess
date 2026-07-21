import argparse
import sys
import time

from chess_common.config import load_config
from chess_common.logging_setup import setup_logging
from chess_fritz.move_detector import FritzMoveDetector
from chess_fritz.pgn_parser import PGNParser
from chess_fritz.publisher import ChessMovePublisher


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Fritz -> RabbitMQ chess move bridge")
    parser.add_argument("--color", choices=["white", "black"], default=None,
                         help="Robot's color (prompted interactively if omitted)")
    return parser.parse_args(argv)


def _get_robot_color(logger, color_arg: str | None) -> str:
    """Resolve the robot's color from the CLI arg, or prompt interactively."""
    if color_arg in ("white", "black"):
        return color_arg

    if not sys.stdin.isatty():
        raise RuntimeError(
            "Robot color not provided (--color) and no interactive terminal "
            "to prompt on."
        )

    while True:
        color = input("Enter robot's color (white/black): ").lower()
        if color in ("white", "black"):
            return color
        logger.warning("Invalid input %r; please enter 'white' or 'black'.", color)


def main():
    logger = None
    publisher = None
    try:
        # Setup main logger
        logger = setup_logging('fritz_interface')
        logger.info("Starting Fritz interface application")

        args = _parse_args()
        robot_color = _get_robot_color(logger, args.color)

        # Imported lazily so this module (and everything that imports it)
        # stays importable on non-Windows dev/test environments.
        from chess_fritz.window_handler import FritzWindowHandler

        window_handler = FritzWindowHandler(logger)
        pgn_parser = PGNParser(logger)
        detector = FritzMoveDetector(logger, window_handler, pgn_parser, robot_color)
        publisher = ChessMovePublisher(logger)
        poll_interval_sec = load_config('fritz_config')['fritz']['poll_interval_sec']

        logger.info("Starting move monitoring...")
        while True:
            try:
                new_moves = detector.get_new_moves()
                for move, color in new_moves:
                    logger.info(f"New {color} move: {move}")
                    publisher.publish_move(move, color)
            except Exception:
                # One bad poll should never kill the monitoring loop.
                logger.error("Error during poll iteration", exc_info=True)
            time.sleep(poll_interval_sec)

    except KeyboardInterrupt:
        if logger:
            logger.info("Stopping on user request")
        if publisher:
            publisher.cleanup()
    except Exception:
        if logger:
            logger.error("Application error", exc_info=True)
        else:
            print("Application error before logger initialization", file=sys.stderr)


if __name__ == "__main__":
    main()
