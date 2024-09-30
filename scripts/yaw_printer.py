#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class YawSubscriber(Node):

    def __init__(self):
        super().__init__('yaw_subscriber')

        # Subscribe to the /odom topic
        self.create_subscription(
            Odometry,
            '/odom',  # Use the correct topic for your robot's odometry
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Get orientation as a quaternion from the Odometry message
        orientation_q = msg.pose.pose.orientation
        # Extract the quaternion components
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w

        # Calculate the yaw (rotation around Z-axis) from the quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        # Print the yaw angle in radians
        self.get_logger().info(f'Current Yaw: {yaw:.3f} radians')

def main(args=None):
    rclpy.init(args=args)

    # Create a node and spin to process incoming messages
    yaw_subscriber = YawSubscriber()

    try:
        rclpy.spin(yaw_subscriber)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    yaw_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
