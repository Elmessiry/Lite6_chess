import rclpy
import asyncio
import time
from chess_robot.messaging.subscriber import ChessRobotSubscriber
from chess_robot.logging_utils import setup_logging
from chess_robot.performance_logger import PerformanceLogger

# Setup main logger
logger = setup_logging('chess_robot')
logger.info("Starting chess robot application")

# Create performance logger
perf_logger = PerformanceLogger()

async def main():
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create subscriber with performance logger
        subscriber = ChessRobotSubscriber(perf_logger=perf_logger)
        
        # Log startup latency
        start_time = time.time()
        
        # Process moves
        await subscriber.process_moves()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Log shutdown
        perf_logger.log_latency("application_runtime", start_time)
        
        if 'subscriber' in locals():
            subscriber.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())