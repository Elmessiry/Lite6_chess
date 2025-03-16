import rclpy
import asyncio
import pika
import json
import threading
import time
from queue import Queue
import logging
import yaml
import os
from typing import Dict, Any, Optional
from ..nodes.chess_node import ChessNode
from ..performance_logger import PerformanceLogger, PerformanceMetrics

class ChessRobotSubscriber:
    """Handles RabbitMQ message subscription and movement execution"""
    
    def __init__(self, perf_logger: Optional[PerformanceLogger] = None):
        # Setup logging
        self.setup_logging()
        
        # Setup performance logging
        self.perf_logger = perf_logger or PerformanceLogger()
        
        # Initialize ROS node
        self.node = ChessNode(perf_logger=self.perf_logger)
        
        # Initialize move queue
        self.move_queue = Queue()
        
        # Load config and setup RabbitMQ
        self.config = self._load_config()
        self.setup_rabbitmq()
        
        # Start RabbitMQ consumer thread
        self.start_rabbitmq_consumer()

    def _load_config(self) -> Dict[str, Any]:
        """Load messaging configuration"""
        try:
            # Use absolute path in container
            config_path = "/home/dev_ws/chess/config/messaging_config.yaml"
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                return config['rabbitmq']
            else:
                self.logger.error(f"Config file not found at {config_path}")
                raise FileNotFoundError(f"Config file not found at {config_path}")
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            raise

    def setup_logging(self):
        """Setup logging configuration"""
        try:
            # Basic logging first
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            self.logger = logging.getLogger("robot_control")
            
            # In container, configs are at /home/dev_ws/chess/config
            config_path = "/home/dev_ws/chess/config/logging_config.yaml"
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    logging.config.dictConfig(config)
            else:
                self.logger.warning(f"Config file not found at {config_path}, using basic logging")
                
        except Exception as e:
            self.logger.error(f"Error setting up logging: {e}")

    def setup_rabbitmq(self):
        """Setup RabbitMQ connection with retry logic"""
        retry_count = 0
        while retry_count < self.config['connection']['max_retries']:
            try:
                self.connection = pika.BlockingConnection(
                    pika.ConnectionParameters(
                        host=self.config['host'],
                        port=self.config['port'],
                        heartbeat=self.config['connection']['heartbeat']
                    )
                )
                self.channel = self.connection.channel()
                
                self.channel.exchange_declare(
                    exchange=self.config['exchange'],
                    exchange_type=self.config['exchange_type']
                )
                self.channel.queue_declare(queue=self.config['queue'])
                self.channel.queue_bind(
                    exchange=self.config['exchange'],
                    queue=self.config['queue'],
                    routing_key=self.config['routing_key']
                )
                
                self.logger.info("Successfully connected to RabbitMQ")
                break
                
            except Exception as e:
                retry_count += 1
                self.logger.error(f"Connection attempt {retry_count} failed: {e}")
                if retry_count < self.config['connection']['max_retries']:
                    time.sleep(self.config['connection']['retry_delay'])
                else:
                    raise

    def rabbitmq_callback(self, ch, method, properties, body):
        """Handle received messages from RabbitMQ"""
        try:
            start_time = time.time()
            message = json.loads(body)
            self.logger.info(f"Received move: {message}")
            
            # Add move to queue for processing
            self.move_queue.put(message)
            
            # Acknowledge message
            ch.basic_ack(delivery_tag=method.delivery_tag)
            
            # Log message processing latency
            self.perf_logger.log_latency("message_processing", start_time)
            
        except Exception as e:
            self.logger.error(f"Error processing message: {e}")
            ch.basic_nack(delivery_tag=method.delivery_tag)

    def start_rabbitmq_consumer(self):
        """Start RabbitMQ consumer in a separate thread"""
        def consume():
            self.channel.basic_qos(prefetch_count=1)
            self.channel.basic_consume(
                queue=self.config['queue'],
                on_message_callback=self.rabbitmq_callback
            )
            self.logger.info("Started consuming messages from RabbitMQ")
            try:
                self.channel.start_consuming()
            except Exception as e:
                self.logger.error(f"RabbitMQ consuming error: {e}")

        self.consumer_thread = threading.Thread(target=consume, daemon=True)
        self.consumer_thread.start()

    async def process_moves(self):
        """Process moves from the queue"""
        while True:
            try:
                # Process ROS callbacks
                rclpy.spin_once(self.node, timeout_sec=0.1)
                
                # Check for new moves
                if not self.move_queue.empty():
                    move = self.move_queue.get()
                    from_square = move['from_square']
                    to_square = move['to_square']
                    
                    # Validate move format
                    if not from_square or not to_square:
                        self.logger.error("Invalid move format: missing squares")
                        continue
                    
                    # Execute move with robot
                    success = await self.node.movement.execute_movement(
                        from_square,
                        to_square
                    )
                    
                    if success:
                        self.logger.info(f"Successfully executed move from {from_square} to {to_square}")
                    else:
                        self.logger.error(f"Failed to execute move from {from_square} to {to_square}")
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.logger.error(f"Error processing move: {e}")
                self.perf_logger.log_error("move_processor", "execution_error", str(e))
                await asyncio.sleep(1)

    def cleanup(self):
        """Cleanup resources"""
        try:
            if hasattr(self, 'connection') and not self.connection.is_closed:
                self.connection.close()
                self.logger.info("RabbitMQ connection closed")
            self.node.destroy_node()
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")