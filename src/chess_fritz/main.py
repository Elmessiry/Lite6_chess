import time

from chess_common.logging_setup import setup_logging
from chess_fritz.move_detector import FritzMoveDetector
from chess_fritz.publisher import ChessMovePublisher

def main():
    try:
        # Setup main logger
        logger = setup_logging('fritz_interface')
        logger.info("Starting Fritz interface application")

        detector = FritzMoveDetector(logger)
        publisher = ChessMovePublisher(logger)
        
        logger.info("Starting move monitoring...")
        while True:
            new_moves = detector.get_new_moves()
            for move, color in new_moves:
                logger.info(f"New {color} move: {move}")
                publisher.publish_move(move, color)
            time.sleep(5)
            
    except KeyboardInterrupt:
        logger.info("Stopping on user request")
        if 'publisher' in locals():
            publisher.cleanup()
    except Exception as e:
        if 'logger' in locals():
            logger.error(f"Application error: {e}")
        else:
            print(f"Application error before logger initialization: {e}")

if __name__ == "__main__":
    main()