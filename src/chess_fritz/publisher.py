import json
import logging
import time
from typing import Any, Dict

import pika
import pika.exceptions

from chess_common.config import load_config


class ChessMovePublisher:
    """Publishes robot moves to RabbitMQ.

    ``connection_factory`` is injectable (defaults to
    ``pika.BlockingConnection``) so tests can supply a fake and never touch
    a real broker.
    """

    def __init__(self, logger: logging.Logger, connection_factory=pika.BlockingConnection):
        self.logger = logger
        self.connection_factory = connection_factory
        self.config: Dict[str, Any] = self._load_config()
        self.connection = None
        self.channel = None
        self._setup_connection()

    def _load_config(self) -> Dict[str, Any]:
        """Load RabbitMQ configuration via the shared chess_common loader."""
        try:
            return load_config('messaging_config')['rabbitmq']
        except Exception:
            self.logger.error("Failed to load messaging config", exc_info=True)
            raise

    def _setup_connection(self) -> None:
        """Setup RabbitMQ connection with retry logic"""
        conn_cfg = self.config['connection']
        max_retries = conn_cfg['max_retries']
        retry_delay = conn_cfg['retry_delay']
        retry_count = 0
        while retry_count < max_retries:
            try:
                self.connection = self.connection_factory(
                    pika.ConnectionParameters(
                        host=self.config['host_from_windows'],
                        port=self.config['port'],
                        heartbeat=conn_cfg['heartbeat'],
                    )
                )
                self.channel = self.connection.channel()
                self.channel.confirm_delivery()

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
                return

            except Exception:
                retry_count += 1
                self.logger.error(
                    "Connection attempt %d/%d failed", retry_count, max_retries,
                    exc_info=True,
                )
                if retry_count < max_retries:
                    time.sleep(retry_delay)
                else:
                    raise

    def publish_move(self, move: str, color: str) -> bool:
        """Publish a move to RabbitMQ.

        Bounded retry: on transient failure, reconnects and retries up to
        ``max_retries`` times. Never raises -- returns True on success,
        False if the move could not be published (caller logs and
        continues rather than crashing the poll loop).
        """
        message = {
            "from_square": move[:2],
            "to_square": move[2:]
        }
        conn_cfg = self.config['connection']
        max_retries = conn_cfg['max_retries']
        retry_delay = conn_cfg['retry_delay']

        for attempt in range(1, max_retries + 1):
            try:
                self.channel.basic_publish(
                    exchange=self.config['exchange'],
                    routing_key=self.config['routing_key'],
                    body=json.dumps(message),
                    properties=pika.BasicProperties(delivery_mode=2),
                    mandatory=True,
                )
                self.logger.info(f"Published {color} move: {move}")
                return True

            except pika.exceptions.UnroutableError:
                # Broker explicitly rejected the route -- not transient,
                # retrying won't help.
                self.logger.error(
                    "Move %s was unroutable (nacked by broker)", move, exc_info=True
                )
                return False

            except Exception:
                self.logger.error(
                    "Publish attempt %d/%d failed for move %s",
                    attempt, max_retries, move, exc_info=True,
                )
                if attempt < max_retries:
                    try:
                        self._setup_connection()
                    except Exception:
                        self.logger.error("Reconnection failed", exc_info=True)
                    time.sleep(retry_delay)

        self.logger.error(f"Failed to publish move {move} after {max_retries} attempts")
        return False

    def cleanup(self) -> None:
        """Close the RabbitMQ connection"""
        if self.connection is not None and not self.connection.is_closed:
            self.connection.close()
