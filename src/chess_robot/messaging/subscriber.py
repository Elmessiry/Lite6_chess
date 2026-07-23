import json
import threading
import time
from queue import Queue, Empty
from typing import Optional

import pika
import rclpy

from chess_common.config import load_config
from chess_common.logging_setup import setup_logging
from ..nodes.chess_node import ChessNode
from ..performance_logger import PerformanceLogger


class ChessRobotSubscriber:
    """RabbitMQ subscriber that executes chess moves on the robot.

    Concurrency model (pika is single-writer per connection):

    - The **consumer thread** creates and owns the pika connection and
      channel end to end. Every channel/connection call happens on that
      thread -- either directly inside the consume callback, or scheduled
      onto it via ``connection.add_callback_threadsafe``. No other thread
      ever touches pika objects directly.
    - The **worker loop** runs on the main thread: it spins the ROS node
      and pulls validated moves off a thread-safe ``Queue``, executes them
      synchronously, then schedules the resulting ack/nack back onto the
      consumer thread.

    Delivery semantics: ``prefetch_count=1`` plus ack-after-execute gives
    **at-least-once** delivery with natural backpressure. A move is acked
    only after the robot finishes it, and the broker delivers no new
    message until then; a crash mid-move redelivers that move on
    reconnect. Malformed or empty messages are nacked without requeue
    (poison-message drop) so they never block the queue.
    """

    def __init__(self, perf_logger: Optional[PerformanceLogger] = None):
        self.logger = setup_logging("chess_robot.subscriber")
        self.perf_logger = perf_logger or PerformanceLogger()
        self.node = ChessNode(perf_logger=self.perf_logger)

        self.config = load_config("messaging_config")["rabbitmq"]
        self.move_queue: Queue = Queue()

        # Coordination between the main thread and the consumer thread.
        self.shutdown_event = threading.Event()
        self._consumer_ready = threading.Event()
        self._consumer_error: Optional[BaseException] = None

        # Owned exclusively by the consumer thread once created.
        self.connection = None
        self.channel = None

        self._start_consumer_thread()
        self._await_consumer_ready()

    # --- consumer thread (owns the pika connection) --------------------

    def _start_consumer_thread(self):
        self.consumer_thread = threading.Thread(
            target=self._consume, name="rabbitmq-consumer", daemon=True)
        self.consumer_thread.start()

    def _await_consumer_ready(self):
        """Block until the consumer thread is consuming or has failed setup."""
        self._consumer_ready.wait()
        if self._consumer_error is not None:
            raise RuntimeError(
                "RabbitMQ consumer failed to start") from self._consumer_error

    def _connect(self):
        """Create connection, channel and topology. Runs on consumer thread."""
        conn_config = self.config["connection"]
        retry_count = 0
        while True:
            try:
                self.connection = pika.BlockingConnection(
                    pika.ConnectionParameters(
                        host=self.config["host_from_container"],
                        port=self.config["port"],
                        heartbeat=conn_config["heartbeat"],
                    )
                )
                self.channel = self.connection.channel()
                self.channel.exchange_declare(
                    exchange=self.config["exchange"],
                    exchange_type=self.config["exchange_type"],
                )
                self.channel.queue_declare(queue=self.config["queue"])
                self.channel.queue_bind(
                    exchange=self.config["exchange"],
                    queue=self.config["queue"],
                    routing_key=self.config["routing_key"],
                )
                self.channel.basic_qos(prefetch_count=1)
                self.logger.info("Successfully connected to RabbitMQ")
                return
            except Exception:
                retry_count += 1
                self.logger.error(
                    f"Connection attempt {retry_count} failed", exc_info=True)
                if retry_count >= conn_config["max_retries"]:
                    raise
                time.sleep(conn_config["retry_delay"])

    def _consume(self):
        """Consumer-thread entry point: set up, consume, tear down."""
        try:
            self._connect()
            self.channel.basic_consume(
                queue=self.config["queue"],
                on_message_callback=self._on_message,
            )
        except BaseException as exc:  # setup failed: hand the error to main
            self._consumer_error = exc
            self._consumer_ready.set()
            return

        # Setup succeeded; unblock the main thread before we block on I/O.
        self._consumer_ready.set()
        self.logger.info("Started consuming messages from RabbitMQ")

        try:
            self.channel.start_consuming()
        except Exception:
            if not self.shutdown_event.is_set():
                self.logger.error("RabbitMQ consuming error", exc_info=True)
        finally:
            self._close_connection()

    def _on_message(self, ch, method, properties, body):
        """Consumer-thread callback: validate and enqueue, or poison-drop."""
        start_time = time.time()
        try:
            message = json.loads(body)
            from_square = message["from_square"]
            to_square = message["to_square"]
        except (ValueError, KeyError, TypeError) as exc:
            self.logger.error(f"Dropping malformed message {body!r}: {exc}")
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=False)
            return

        if not from_square or not to_square:
            self.logger.error(f"Dropping message with empty squares: {message}")
            ch.basic_nack(delivery_tag=method.delivery_tag, requeue=False)
            return

        self.logger.info(f"Received move: {message}")
        self.move_queue.put((message, method.delivery_tag))
        self.perf_logger.log_latency("message_processing", start_time)

    # --- worker loop (main thread) -------------------------------------

    def process_moves(self):
        """Main-thread worker: spin ROS, execute moves, ack after execution."""
        while not self.shutdown_event.is_set():
            # Service ROS callbacks even when no move is pending.
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                message, delivery_tag = self.move_queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                ok = self.node.movement.execute_movement(
                    message["from_square"], message["to_square"])
            except Exception as e:
                self.logger.error(
                    f"Error executing move {message}: {e}", exc_info=True)
                self.perf_logger.log_error(
                    "move_processor", "execution_error", str(e))
                ok = False

            if ok:
                self.logger.info(f"Executed move: {message}")
            else:
                self.logger.error(f"Failed to execute move: {message}")

            self._schedule_ack(delivery_tag, ok)

    def _schedule_ack(self, delivery_tag, ok):
        """Schedule the ack/nack on the consumer thread (pika's owner)."""
        def ack():
            if ok:
                self.channel.basic_ack(delivery_tag)
            else:
                self.channel.basic_nack(delivery_tag, requeue=False)

        try:
            self.connection.add_callback_threadsafe(ack)
        except Exception:
            self.logger.error(
                "Failed to schedule ack/nack; connection may be closed",
                exc_info=True)

    # --- shutdown ------------------------------------------------------

    def shutdown(self):
        """Signal shutdown and stop consuming from the owner thread."""
        self.shutdown_event.set()
        try:
            self.connection.add_callback_threadsafe(self.channel.stop_consuming)
        except Exception:
            self.logger.error("Failed to schedule stop_consuming", exc_info=True)

    def cleanup(self):
        """Stop consuming, join the consumer thread, destroy the ROS node."""
        self.shutdown()
        if self.consumer_thread.is_alive():
            self.consumer_thread.join(timeout=10)
        try:
            self.node.destroy_node()
        except Exception:
            self.logger.error("Error destroying ROS node", exc_info=True)

    def _close_connection(self):
        """Close the connection. Runs on the consumer thread only."""
        try:
            if self.connection is not None and self.connection.is_open:
                self.connection.close()
                self.logger.info("RabbitMQ connection closed")
        except Exception:
            self.logger.error("Error closing RabbitMQ connection", exc_info=True)
