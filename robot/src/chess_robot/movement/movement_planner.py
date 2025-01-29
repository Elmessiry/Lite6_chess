from typing import Tuple, List, Dict, Optional
import yaml
import os

class MovementPlanner:
    """Handles chess-specific movement planning and validation"""
    
    def __init__(self, logger):
        self.logger = logger
        self.config = self._load_config()
        self.capture_positions = [False] * (self.config['capture_zone']['grid']['rows'] * 
                                          self.config['capture_zone']['grid']['cols'])
        self.next_capture_position = 0

    def _load_config(self) -> Dict:
        """Load board configuration"""
        config_path = "/home/dev_ws/chess/config/board_config.yaml"
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            raise

    def get_coordinates(self, square: str) -> Tuple[float, float, float]:
        """Convert chess notation to coordinates"""
        file_idx = ord(square[0].lower()) - ord('a')
        rank_idx = int(square[1]) - 1
        
        x = (self.config['board']['origin']['x'] + 
             ((7 - rank_idx) * self.config['board']['square_size']) + 
             (self.config['board']['square_size'] / 2))
        y = (self.config['board']['origin']['y'] + 
             (file_idx * self.config['board']['square_size']) + 
             (self.config['board']['square_size'] / 2))
        z = self.config['board']['origin']['z']
        
        return x, y, z

    def validate_square(self, square: str) -> bool:
        """Validate chess square notation"""
        if len(square) != 2:
            return False
        file_valid = 'a' <= square[0].lower() <= 'h'
        rank_valid = '1' <= square[1] <= '8'
        return file_valid and rank_valid

    def get_capture_coordinates(self, index: int) -> Tuple[float, float, float]:
        """Convert capture index to coordinates"""
        if not 0 <= index < (self.config['capture_zone']['grid']['rows'] * 
                           self.config['capture_zone']['grid']['cols']):
            raise ValueError(f"Invalid capture index: {index}")
        
        row = index // self.config['capture_zone']['grid']['cols']
        col = index % self.config['capture_zone']['grid']['cols']
        
        cell_width = self.config['capture_zone']['dimensions']['width'] / self.config['capture_zone']['grid']['cols']
        cell_height = self.config['capture_zone']['dimensions']['height'] / self.config['capture_zone']['grid']['rows']
        
        x = (self.config['capture_zone']['origin']['x'] + 
            (col + 0.5) * cell_width)
        y = (self.config['capture_zone']['origin']['y'] + 
            (row + 0.5) * cell_height)
        z = self.config['board']['origin']['z']
        
        return x, y, z
    
    def get_next_capture_position(self) -> int:
        """Find next available capture zone position"""
        start_pos = self.next_capture_position
        for i in range(len(self.capture_positions)):
            pos = (start_pos + i) % len(self.capture_positions)
            if not self.capture_positions[pos]:
                self.capture_positions[pos] = True
                self.next_capture_position = (pos + 1) % len(self.capture_positions)
                return pos
        raise RuntimeError("No available capture positions")

    def create_movement_sequence(self, start_square: str, end_square: str) -> List[Dict]:
        """Create the sequence of movements for a chess piece"""
        start_x, start_y, start_z = self.get_coordinates(start_square)
        end_x, end_y, end_z = self.get_coordinates(end_square)
        
        hover_height = self.config['board']['hover_height']
        piece_height = self.config['board']['piece_height']

        return [
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height + hover_height),
                'description': "Move above start position"
            },
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height),
                'description': "Lower to piece height"
            },
            {
                'type': 'gripper',
                'action': True,
                'description': "Activate gripper"
            },
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height + hover_height),
                'description': "Lift piece up"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height + hover_height),
                'description': "Move to target position"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height),
                'description': "Lower piece"
            },
            {
                'type': 'gripper',
                'action': False,
                'description': "Release gripper"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height + hover_height),
                'description': "Retreat upward"
            }
        ]

    def create_capture_movement_sequence(self, start_square: str, capture_index: int) -> List[Dict]:
        """Create sequence of movements to move a piece to capture zone"""
        start_x, start_y, start_z = self.get_coordinates(start_square)
        end_x, end_y, end_z = self.get_capture_coordinates(capture_index)
        
        hover_height = self.config['board']['hover_height']
        piece_height = self.config['board']['piece_height']
        
        return [
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height + hover_height),
                'description': "Move above captured piece"
            },
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height),
                'description': "Lower to piece height"
            },
            {
                'type': 'gripper',
                'action': True,
                'description': "Activate gripper"
            },
            {
                'type': 'move',
                'position': (start_x, start_y, start_z + piece_height + hover_height),
                'description': "Lift captured piece"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height + hover_height),
                'description': "Move to capture zone"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height),
                'description': "Lower piece"
            },
            {
                'type': 'gripper',
                'action': False,
                'description': "Release piece"
            },
            {
                'type': 'move',
                'position': (end_x, end_y, end_z + piece_height + hover_height),
                'description': "Retreat upward"
            }
        ]
