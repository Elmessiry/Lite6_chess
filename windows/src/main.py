import os
import time
import logging
import logging.config
import yaml
import sys
from fritz_interface.move_detector import FritzMoveDetector
from messaging.publisher import ChessMovePublisher

# Add the robot/src directory to the Python path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
robot_src = os.path.join(project_root, 'robot', 'src')
sys.path.append(robot_src)

# Now import from chess_robot
from chess_robot.logging_utils import setup_logging

def main():
    try:
        # Setup main logger
        logger = setup_logging('fritz_interface')
        logger.info("Starting Fritz interface application")
        
        # Create logs directory if it doesn't exist
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        os.makedirs(os.path.join(project_root, 'logs'), exist_ok=True)
        
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