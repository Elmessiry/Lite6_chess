import rclpy
import asyncio
from chess_robot.messaging.subscriber import ChessRobotSubscriber

async def main():
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create subscriber
        subscriber = ChessRobotSubscriber()
        
        # Process moves
        await subscriber.process_moves()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if 'subscriber' in locals():
            subscriber.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())