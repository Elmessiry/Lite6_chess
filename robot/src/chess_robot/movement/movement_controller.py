from typing import Optional
import logging
from .robot_hardware import RobotHardware, MoveResult
from .movement_planner import MovementPlanner
import asyncio

class MovementController:
    """Controls and coordinates robot movements for chess operations"""
    
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        self.robot = RobotHardware(node)
        self.planner = MovementPlanner(self.logger)

    async def execute_movement(self, start_square: str, end_square: str) -> bool:
        """Execute a chess piece movement"""
        try:
            # Validate start square
            if not self.planner.validate_square(start_square):
                self.logger.error(f"Invalid start square: {start_square}")
                return False

            # Handle capture to capture zone
            if end_square.lower() == "xx":
                return await self._handle_capture_movement(start_square)
            else:
                return await self._handle_regular_movement(start_square, end_square)

        except Exception as e:
            self.logger.error(f"Error during movement execution: {e}")
            return False

    async def _handle_capture_movement(self, start_square: str) -> bool:
        """Handle movement to capture zone"""
        try:
            capture_index = self.planner.get_next_capture_position()
            self.logger.info(f"\nMoving captured piece from {start_square} to capture zone {capture_index}")
            
            movements = self.planner.create_capture_movement_sequence(start_square, capture_index)
            return await self._execute_movement_sequence(movements)
            
        except Exception as e:
            self.logger.error(f"Error during capture movement: {e}")
            return False

    async def _handle_regular_movement(self, start_square: str, end_square: str) -> bool:
        """Handle regular piece movement"""
        try:
            if not self.planner.validate_square(end_square):
                self.logger.error(f"Invalid end square: {end_square}")
                return False
            
            self.logger.info(f"\nExecuting movement from {start_square} to {end_square}")
            movements = self.planner.create_movement_sequence(start_square, end_square)
            return await self._execute_movement_sequence(movements)
            
        except Exception as e:
            self.logger.error(f"Error during regular movement: {e}")
            return False

    async def _execute_movement_sequence(self, movements: list) -> bool:
        """Execute a sequence of movements"""
        for movement in movements:
            self.logger.info(f"\nExecuting: {movement['description']}")
            
            if movement['type'] == 'gripper':
                success = await self.robot.control_gripper(movement['action'])
                if not success:
                    self.logger.error("Gripper operation failed")
                    return False
            else:  # movement['type'] == 'move'
                result = await self.robot.move_to_pose(*movement['position'])
                if result != MoveResult.SUCCESS:
                    self.logger.error(f"Movement failed: {result.value}")
                    return False
            
            await asyncio.sleep(1.0)
        
        return True