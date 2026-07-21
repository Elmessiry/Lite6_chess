import rclpy
from rclpy.node import Node
import time
import logging
import logging.config
import yaml
import os
from ..visualization.visualizer import ChessboardVisualizer
from ..movement.movement_controller import MovementController
from ..performance_logger import PerformanceLogger
from typing import Optional

class ChessNode(Node):
    """ROS2 node for chess robot operations"""
    
    def __init__(self, perf_logger: Optional[PerformanceLogger] = None):
        super().__init__('chess_robot')
        self.get_logger().info('Initializing Chess Robot Node')
        
        # Setup logging
        self.setup_logging()
        
        # Initialize visualization
        self.visualizer = ChessboardVisualizer(self)
        
        # Initialize movement controller
        self.movement = MovementController(self, perf_logger)
        
        # Wait for subscribers
        self.get_logger().info("Waiting for visualization subscribers...")
        while self.visualizer.marker_pub.get_subscription_count() == 0:
            self.get_logger().info("No subscribers yet...")
            time.sleep(1.0)
        self.get_logger().info("Visualization subscriber connected!")

        self.get_logger().info('Chess Robot Node initialized')

    def setup_logging(self):
        """Setup logging configuration"""
        try:
            # Basic logging first in case config fails
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            logger = logging.getLogger('robot_control')
            
            # In container, configs are at /home/dev_ws/chess/config
            config_path = "/home/dev_ws/chess/config/logging_config.yaml"
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    logging.config.dictConfig(config)
            else:
                self.get_logger().warning(f"Config file not found at {config_path}, using basic logging")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load logging config: {e}")