#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time
from orient_bot import OrientBot  # Import the orientation functionality from orient_bot

class Navigation(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to the /farthest_obstacle topic to get the coordinates
        self.obstacle_subscription = self.create_subscription(
            Point,
            '/farthest_obstacle',
            self.obstacle_callback,
            10
        )

        # Variable to store the farthest obstacle coordinates
        self.farthest_obstacle = None

        # Define a Twist message for movement
        self.move_cmd = Twist()

        # Create an instance of OrientBot for handling orientation
        self.orient_bot = OrientBot()

    def obstacle_callback(self, msg):
        # Update the farthest obstacle coordinates when received
        self.farthest_obstacle = (msg.x, msg.y)
        self.get_logger().info(f'Farthest obstacle received: (x: {msg.x}, y: {msg.y})')

        # Orient the bot towards the farthest obstacle once the coordinates are received
        self.orient_towards_obstacle()

    def rotate(self, duration):
        # Rotate the robot
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.move_cmd.angular.z = 0.5
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.publisher_.publish(self.move_cmd)
            self.get_logger().info("Rotating...")
            time.sleep(0.1)

        # Stop the robot
        self.move_cmd.angular.z = 0.0
        self.publisher_.publish(self.move_cmd)
        self.get_logger().info("Rotation complete!")

    def orient_towards_obstacle(self):
        # Check if the farthest obstacle is available
        if self.farthest_obstacle:
            target_x, target_y = self.farthest_obstacle
            self.get_logger().info(f'Oriented towards obstacle at: (x: {target_x}, y: {target_y})')

            # Use the OrientBot to orient the robot towards the obstacle
            self.orient_bot.orient_towards(target_x, target_y)
        else:
            self.get_logger().info('No farthest obstacle available for orientation.')


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()

    try:
        # Step 1: Perform rotation
        navigation.rotate(3)  # Rotate for 3 seconds

        # Step 2: Wait for farthest obstacle and orient towards it
        rclpy.spin(navigation)

    except KeyboardInterrupt:
        pass

    # Shutdown
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
