from typing import Dict, Optional
import time

from chess_common.config import load_config
from .robot_hardware import RobotHardware, MoveResult
from .movement_planner import MovementPlanner
from ..performance_logger import PerformanceLogger


class MovementController:
    """Controls and coordinates robot movements for chess operations"""

    def __init__(self, node, perf_logger: Optional[PerformanceLogger] = None,
                 config: Optional[Dict] = None):
        self.node = node
        self.logger = node.get_logger()
        self.robot = RobotHardware(node)
        self.planner = MovementPlanner(self.logger)
        self.perf_logger = perf_logger or PerformanceLogger()
        board_config = config if config is not None else load_config('board_config')
        self.step_settle_sec = board_config['robot']['movement']['step_settle_sec']

    def execute_movement(self, start_square: str, end_square: str) -> bool:
        """Execute a chess piece movement"""
        try:
            start_time = time.time()

            # Validate start square
            if not self.planner.validate_square(start_square):
                self.logger.error(f"Invalid start square: {start_square}")
                return False

            # Handle capture to capture zone
            if end_square.lower() == "xx":
                result = self._handle_capture_movement(start_square)
            else:
                result = self._handle_regular_movement(start_square, end_square)

            # Log performance
            self.perf_logger.log_latency("move_execution", start_time)

            return result

        except Exception as e:
            self.logger.error(f"Error during movement execution: {e}")
            self.perf_logger.log_error("movement_controller", "execution_error", str(e))
            return False

    def _handle_capture_movement(self, start_square: str) -> bool:
        """Handle movement to capture zone"""
        try:
            start_time = time.time()

            capture_index = self.planner.get_next_capture_position()
            self.logger.info(
                f"Moving captured piece from {start_square} to capture zone {capture_index}")

            # Log planning time
            planning_time = (time.time() - start_time) * 1000

            # Execute movement
            movements = self.planner.create_capture_movement_sequence(
                start_square, capture_index)
            execution_start = time.time()
            result = self._execute_movement_sequence(movements)

            # Log execution time
            execution_time = (time.time() - execution_start) * 1000

            # Log overall performance
            self.perf_logger.log_move_execution(
                f"{start_square}-XX",
                result,
                planning_time,
                execution_time
            )

            return result

        except Exception as e:
            self.logger.error(f"Error during capture movement: {e}")
            self.perf_logger.log_error("movement_controller", "capture_error", str(e))
            return False

    def _handle_regular_movement(self, start_square: str, end_square: str) -> bool:
        """Handle regular piece movement"""
        try:
            start_time = time.time()

            if not self.planner.validate_square(end_square):
                self.logger.error(f"Invalid end square: {end_square}")
                return False

            self.logger.info(f"Moving piece from {start_square} to {end_square}")

            # Generate movement sequence
            movements = self.planner.create_movement_sequence(start_square, end_square)

            # Log planning time
            planning_time = (time.time() - start_time) * 1000

            # Execute movement
            execution_start = time.time()
            result = self._execute_movement_sequence(movements)

            # Log execution time
            execution_time = (time.time() - execution_start) * 1000

            # Log overall performance
            self.perf_logger.log_move_execution(
                f"{start_square}-{end_square}",
                result,
                planning_time,
                execution_time
            )

            return result

        except Exception as e:
            self.logger.error(f"Error during regular movement: {e}")
            self.perf_logger.log_error("movement_controller", "movement_error", str(e))
            return False

    def _execute_movement_sequence(self, movements: list) -> bool:
        """Execute a sequence of movements"""
        for i, movement in enumerate(movements):
            step_start = time.time()
            self.logger.info(f"Executing: {movement['description']}")

            if movement['type'] == 'gripper':
                success = self.robot.control_gripper(movement['action'])
                if not success:
                    self.logger.error("Gripper operation failed")
                    return False
            else:  # movement['type'] == 'move'
                result = self.robot.move_to_pose(*movement['position'])
                if result != MoveResult.SUCCESS:
                    self.logger.error(f"Movement failed: {result.value}")
                    return False

            # Log step latency
            self.perf_logger.log_latency(
                f"movement_step_{i+1}_{movement['type']}",
                step_start
            )

            time.sleep(self.step_settle_sec)

        return True
