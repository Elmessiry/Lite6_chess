import time

import rclpy

from chess_common.logging_setup import setup_logging
from chess_robot.messaging.subscriber import ChessRobotSubscriber
from chess_robot.performance_logger import PerformanceLogger


def main():
    logger = setup_logging("chess_robot")
    logger.info("Starting chess robot application")

    perf_logger = PerformanceLogger()
    # B5: capture start time before the try so the finally can always log
    # runtime, even if subscriber construction fails.
    start_time = time.time()
    subscriber = None

    rclpy.init()
    try:
        subscriber = ChessRobotSubscriber(perf_logger=perf_logger)
        subscriber.process_moves()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception:
        logger.error("Fatal error in chess robot application", exc_info=True)
    finally:
        perf_logger.log_latency("application_runtime", start_time)
        if subscriber is not None:
            subscriber.cleanup()
        perf_logger.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
