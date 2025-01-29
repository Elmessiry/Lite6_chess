import os
import pika
import json
import logging
import yaml
import time
from typing import Dict, Any

class ChessMovePublisher:
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.config = self._load_config()
        self._setup_connection()

    def _load_config(self) -> Dict[str, Any]:
        """Load RabbitMQ configuration"""
        try:
            # Get the correct path to config directory
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
            config_path = os.path.join(project_root, 'config', 'messaging_config.yaml')
            
            if not os.path.exists(config_path):
                raise FileNotFoundError(f"Config file not found at: {config_path}")
                
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config['rabbitmq']
                
        except Exception as e:
            self.logger.error(f"Failed to load config: {e}")
            raise

    def _setup_connection(self) -> None:
        """Setup RabbitMQ connection with retry logic"""
        retry_count = 0
        while retry_count < self.config['connection']['max_retries']:
            try:
                # Simplified connection parameters
                self.connection = pika.BlockingConnection(
                    pika.ConnectionParameters(
                        host='localhost'  # Using localhost instead of IP
                    )
                )
                self.channel = self.connection.channel()
                
                # Setup exchange and queue
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

    def publish_move(self, move: str, color: str) -> None:
        """Publish a move to RabbitMQ"""
        message = {
            "from_square": move[:2],
            "to_square": move[2:]
        }

        try:
            self.channel.basic_publish(
                exchange=self.config['exchange'],
                routing_key=self.config['routing_key'],
                body=json.dumps(message),
                properties=pika.BasicProperties(delivery_mode=2)
            )
            self.logger.info(f"Published {color} move: {move}")
        except Exception as e:
            self.logger.error(f"Failed to publish move: {e}")
            self._setup_connection()  # Attempt reconnection
            self.channel.basic_publish(
                exchange=self.config['exchange'],
                routing_key=self.config['routing_key'],
                body=json.dumps(message),
                properties=pika.BasicProperties(delivery_mode=2)
            )

    def cleanup(self) -> None:
        """Close the RabbitMQ connection"""
        if hasattr(self, 'connection') and not self.connection.is_closed:
            self.connection.close()