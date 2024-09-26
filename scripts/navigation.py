#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RotateBot(Node):

    def __init__(self):
        super().__init__('rotate_bot')
        
        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define a Twist message
        self.rotate_cmd = Twist()
        
        # We only need to set angular velocity (z-axis rotation)
        self.rotate_cmd.linear.x = 0.5  # No forward/backward movement
        self.rotate_cmd.angular.z = 0.0  # Counter-clockwise rotation (adjust the speed as needed)

    def rotate(self, duration):
        # Publish the rotate command for the given duration
        start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Get current time
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.publisher_.publish(self.rotate_cmd)
            self.get_logger().info("Rotating...")
            time.sleep(0.1)

        # Stop the rotation after the duration is complete
        self.rotate_cmd.linear.x = 0.0
        self.rotate_cmd.angular.z = 0.0
        self.publisher_.publish(self.rotate_cmd)
        self.get_logger().info("Rotation complete!")


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    rotate_bot = RotateBot()

    try:
        # Rotate for a certain duration (about 360 degrees)
        duration = 2.0  # Adjust duration based on the robot's speed (7 seconds for a 360-degree rotation at 0.5 rad/s)
        rotate_bot.rotate(duration)

    except KeyboardInterrupt:
        pass

    finally:
        # Shutdown the node
        rotate_bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
