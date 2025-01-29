import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
import yaml
import os
from typing import Dict, Any

class ChessboardVisualizer:
    def __init__(self, node):
        self.node = node
        self.config = self._load_config()
        
        # Publisher for visualization
        self.marker_pub = self.node.create_publisher(
            MarkerArray,
            '/chess_board_visualization',
            10
        )
        
        # Create timer for visualization
        self.visualization_timer = self.node.create_timer(0.1, self.publish_visualization)

    def _load_config(self) -> Dict[str, Any]:
        """Load board configuration"""
        config_path = "/home/dev_ws/chess/config/board_config.yaml"  # Use absolute path
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config
        except Exception as e:
            self.node.get_logger().error(f"Failed to load config: {e}")
            raise

    def create_text_marker(self, text: str, x: float, y: float, z: float, 
                          marker_id: int, scale: float = 0.02) -> Marker:
        """Helper function to create text markers"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "chess_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale.z = scale
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        marker.text = text
        return marker

    def _create_board_base(self) -> Marker:
        """Create the chess board base plate"""
        board_base = Marker()
        board_base.header.frame_id = "world"
        board_base.header.stamp = self.node.get_clock().now().to_msg()
        board_base.ns = "chess_board"
        board_base.id = 0
        board_base.type = Marker.CUBE
        board_base.action = Marker.ADD
        
        square_size = self.config['board']['square_size']
        board_base.scale.x = square_size * 8 + 0.02
        board_base.scale.y = square_size * 8 + 0.02
        board_base.scale.z = 0.02
        
        board_base.pose.position.x = (self.config['board']['origin']['x'] + 
                                    square_size * 4)
        board_base.pose.position.y = (self.config['board']['origin']['y'] + 
                                    square_size * 4)
        board_base.pose.position.z = self.config['board']['origin']['z'] - 0.01
        
        board_base.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        board_base.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
        
        return board_base

    def _create_board_squares(self) -> list:
        """Create chess board squares"""
        squares = []
        for i in range(8):
            for j in range(8):
                square = Marker()
                square.header.frame_id = "world"
                square.header.stamp = self.node.get_clock().now().to_msg()
                square.ns = "chess_squares"
                square.id = 1 + i * 8 + j
                square.type = Marker.CUBE
                square.action = Marker.ADD
                
                square_size = self.config['board']['square_size']
                square.scale.x = square_size * 0.98
                square.scale.y = square_size * 0.98
                square.scale.z = 0.01
                
                square.pose.position.x = (self.config['board']['origin']['x'] + 
                                        (7-j + 0.5) * square_size)
                square.pose.position.y = (self.config['board']['origin']['y'] + 
                                        (i + 0.5) * square_size)
                square.pose.position.z = self.config['board']['origin']['z']
                
                square.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                
                if (i + j) % 2 == 0:
                    square.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                else:
                    square.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
                
                squares.append(square)
        return squares

    def _create_board_labels(self) -> list:
        """Create chess board labels"""
        labels = []
        square_size = self.config['board']['square_size']
        
        # File labels (a-h)
        for i in range(8):
            file_label = chr(ord('a') + i)
            # Left side
            labels.append(self.create_text_marker(
                file_label,
                self.config['board']['origin']['x'] - 0.02,
                self.config['board']['origin']['y'] + (i + 0.5) * square_size,
                self.config['board']['origin']['z'],
                100 + i
            ))
            # Right side
            labels.append(self.create_text_marker(
                file_label,
                self.config['board']['origin']['x'] + 8 * square_size + 0.02,
                self.config['board']['origin']['y'] + (i + 0.5) * square_size,
                self.config['board']['origin']['z'],
                120 + i
            ))

        # Rank labels (1-8)
        for i in range(8):
            rank_label = str(8 - i)
            # Bottom
            labels.append(self.create_text_marker(
                rank_label,
                self.config['board']['origin']['x'] + (i + 0.5) * square_size,
                self.config['board']['origin']['y'] - 0.02,
                self.config['board']['origin']['z'],
                140 + i
            ))
            # Top
            labels.append(self.create_text_marker(
                rank_label,
                self.config['board']['origin']['x'] + (i + 0.5) * square_size,
                self.config['board']['origin']['y'] + 8 * square_size + 0.02,
                self.config['board']['origin']['z'],
                160 + i
            ))
        return labels

    def _create_capture_zone(self) -> list:
        """Create capture zone visualization"""
        markers = []
        
        # Create base plate
        capture_base = Marker()
        capture_base.header.frame_id = "world"
        capture_base.header.stamp = self.node.get_clock().now().to_msg()
        capture_base.ns = "capture_zone"
        capture_base.id = 200
        capture_base.type = Marker.CUBE
        capture_base.action = Marker.ADD
        
        capture_base.scale.x = self.config['capture_zone']['dimensions']['width']
        capture_base.scale.y = self.config['capture_zone']['dimensions']['height']
        capture_base.scale.z = 0.01
        
        capture_base.pose.position.x = (self.config['capture_zone']['origin']['x'] + 
                                    self.config['capture_zone']['dimensions']['width'] / 2)
        capture_base.pose.position.y = (self.config['capture_zone']['origin']['y'] + 
                                    self.config['capture_zone']['dimensions']['height'] / 2)
        capture_base.pose.position.z = self.config['board']['origin']['z']
        
        capture_base.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        capture_base.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)
        
        markers.append(capture_base)
        
        # Create grid cells
        cell_width = (self.config['capture_zone']['dimensions']['width'] / 
                     self.config['capture_zone']['grid']['cols'])
        cell_height = (self.config['capture_zone']['dimensions']['height'] / 
                      self.config['capture_zone']['grid']['rows'])
        
        for i in range(self.config['capture_zone']['grid']['rows']):
            for j in range(self.config['capture_zone']['grid']['cols']):
                cell = Marker()
                cell.header.frame_id = "world"
                cell.header.stamp = self.node.get_clock().now().to_msg()
                cell.ns = "capture_grid"
                cell.id = 300 + i * self.config['capture_zone']['grid']['cols'] + j
                cell.type = Marker.CUBE
                cell.action = Marker.ADD
                
                cell.scale.x = cell_width * 0.98
                cell.scale.y = cell_height * 0.98
                cell.scale.z = 0.005
                
                cell.pose.position.x = (self.config['capture_zone']['origin']['x'] + 
                                    (j + 0.5) * cell_width)
                cell.pose.position.y = (self.config['capture_zone']['origin']['y'] + 
                                    (i + 0.5) * cell_height)
                cell.pose.position.z = self.config['board']['origin']['z'] + 0.001
                
                cell.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                cell.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.3)
                
                markers.append(cell)
        
        return markers

    def publish_visualization(self):
        """Publish all visualization markers"""
        marker_array = MarkerArray()
        
        # Add board base
        marker_array.markers.append(self._create_board_base())
        
        # Add squares
        marker_array.markers.extend(self._create_board_squares())
        
        # Add labels
        marker_array.markers.extend(self._create_board_labels())
        
        # Add capture zone
        marker_array.markers.extend(self._create_capture_zone())
        
        self.marker_pub.publish(marker_array)