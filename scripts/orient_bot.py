#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sin, cos

class OrientBot(Node):
    def __init__(self):
        super().__init__('orient_bot')

        # Publisher for the /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Subscriber to get the robot's position and orientation from the /base_link_coordinates topic (PoseStamped)
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/base_link_coordinates',
            self.position_callback,
            10
        )

    def position_callback(self, msg):
        """Callback to update robot's position and orientation from base_link_coordinates topic"""
        # Extract position
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        
        # Extract orientation (quaternion) and convert it to yaw
        orientation_q = msg.pose.orientation
        self.robot_yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        print(f"Received position: x={self.robot_x}, y={self.robot_y}, yaw={self.robot_yaw}")
        self.get_logger().info(f"Position updated: x={self.robot_x}, y={self.robot_y}, yaw={self.robot_yaw}")

    def quaternion_to_euler(self, x, y, z, w):
        """Converts quaternion to Euler yaw (rotation about Z-axis)"""
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return atan2(t3, t4)

    def orient_towards(self, target_x, target_y):
        """Function to rotate the robot towards the target"""
        rclpy.spin_once(self)
        # Calculate the desired yaw angle to face the target coordinates
        target_yaw = atan2(target_y - self.robot_y, target_x - self.robot_x)
        print(self.robot_x,self.robot_y)
        yaw_diff = target_yaw - self.robot_yaw

        # Normalize the yaw difference to be within [-pi, pi]
        yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff))

        # Rotate the bot until it is facing the target
        rotate_cmd = Twist()

        while abs(yaw_diff) > 0.05:  # Keep rotating until yaw difference is sufficiently small
            rotate_cmd.angular.z = 0.2 if yaw_diff > 0 else -0.2
            self.cmd_vel_pub.publish(rotate_cmd)
            print(f"Rotating... yaw_diff: {yaw_diff:.2f}")

            # Allow ROS to process callbacks so that position_callback can update the values
            rclpy.spin_once(self)

            # Recalculate yaw difference as the robot rotates
            yaw_diff = target_yaw - self.robot_yaw
            yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff))

        # Stop rotation when the bot is facing the target
        rotate_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(rotate_cmd)
        self.get_logger().info("Oriented towards the target!")
        return 0

def main(args=None):
    rclpy.init(args=args)
    orient_bot = OrientBot()

    try:
        # Example of orienting towards a point (3, 4) for testing
        orient_bot.orient_towards(3.0, 4.0)

    except KeyboardInterrupt:
        pass

    finally:
        # Cleanup
        orient_bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
