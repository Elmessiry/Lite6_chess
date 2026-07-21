import time
from typing import Optional

from rclpy.node import Node

from chess_common.config import load_config
from ..visualization.visualizer import ChessboardVisualizer
from ..movement.movement_controller import MovementController
from ..performance_logger import PerformanceLogger


class ChessNode(Node):
    """ROS2 node for chess robot operations"""

    def __init__(self, perf_logger: Optional[PerformanceLogger] = None):
        super().__init__('chess_robot')
        self.get_logger().info('Initializing Chess Robot Node')

        # Initialize visualization
        self.visualizer = ChessboardVisualizer(self)

        # Initialize movement controller
        self.movement = MovementController(self, perf_logger)

        # Wait for RViz subscribers, but only for a bounded time -- the
        # visualization is optional and must never gate startup forever.
        self._wait_for_visualization_subscriber()

        self.get_logger().info('Chess Robot Node initialized')

    def _wait_for_visualization_subscriber(self):
        """Wait (bounded) for an RViz subscriber, then continue regardless."""
        viz_config = load_config('board_config').get('visualization', {})
        timeout = viz_config.get('wait_timeout_sec', 10)
        deadline = time.time() + timeout

        self.get_logger().info("Waiting for visualization subscribers...")
        while self.visualizer.marker_pub.get_subscription_count() == 0:
            if time.time() >= deadline:
                self.get_logger().warning(
                    f"No visualization subscriber after {timeout}s; "
                    "continuing without RViz.")
                return
            self.get_logger().info("No subscribers yet...")
            time.sleep(1.0)
        self.get_logger().info("Visualization subscriber connected!")
