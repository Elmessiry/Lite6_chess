import os
import time
import logging
import logging.config
import yaml
from fritz_interface.move_detector import FritzMoveDetector
from messaging.publisher import ChessMovePublisher

def setup_logging():
    """Setup logging configuration"""
    try:
        # Basic logging first
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        logger = logging.getLogger('fritz_interface')
        
        # Get absolute path to config
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_dir = os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'config')
        config_path = os.path.join(config_dir, 'logging_config.yaml')
        
        if os.path.exists(config_path):
            # Create logs directory if it doesn't exist
            logs_dir = os.path.join(os.path.dirname(config_dir), 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                logging.config.dictConfig(config)
        else:
            logger.warning(f"Config file not found at {config_path}, using basic logging")
        
        return logger
    except Exception as e:
        print(f"Error setting up logging: {e}")  # Print error since logger might not be configured
        return logging.getLogger('fritz_interface')  # Return basic logger
    
def main():
    try:
        logger = setup_logging()
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