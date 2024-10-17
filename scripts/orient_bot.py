#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String  # For controlling orientation commands
from math import atan2, sin, cos
import time
import json

class OrientBot(Node):
    def __init__(self):
        super().__init__('orient_bot')
        # Publisher for the /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.is_orienting = False  # Flag to check if orientation should run

        # Subscriber to get the robot's position and orientation from the /base_link_coordinates topic (PoseStamped)
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/base_link_coordinates',
            self.position_callback,
            10
        )

        # Subscriber to receive control commands for orienting
        self.command_sub = self.create_subscription(
            String,
            '/orient_command',
            self.command_callback,
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

    def quaternion_to_euler(self, x, y, z, w):
        """Converts quaternion to Euler yaw (rotation about Z-axis)"""
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return atan2(t3, t4)

    def command_callback(self, msg):
        """Callback to handle control commands for orienting."""
        try:
            print("Command received")
            command = json.loads(msg.data)  # Assuming command is sent as a JSON string

            if command['action'] == 'start':
                # Start orienting to given coordinates
                self.target_x = command['target_x']
                self.target_y = command['target_y']
                self.is_orienting = True
                self.get_logger().info(f"Starting orientation towards ({self.target_x}, {self.target_y})")
                self.correct_orientation()

            elif command['action'] == 'stop':
                # Stop orientation
                self.is_orienting = False
                self.stop_orientation()
                self.get_logger().info("Stopping orientation.")

        except (KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid command format: {e}")

    def correct_orientation(self):
        """Checks and corrects the robot's orientation towards the target."""
        while rclpy.ok() and self.is_orienting and self.target_x is not None and self.target_y is not None:
            # Calculate the desired yaw angle to face the target coordinates
            target_yaw = atan2(self.target_y - self.robot_y, self.target_x - self.robot_x)
            yaw_diff = target_yaw - self.robot_yaw

            # Normalize the yaw difference to be within [-pi, pi]
            yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff))

            # Only adjust if the difference is significant (beyond a threshold)
            if abs(yaw_diff) > 0.05:
                rotate_cmd = Twist()
                rotate_cmd.angular.z = 0.5 if yaw_diff > 0 else -0.5
                self.cmd_vel_pub.publish(rotate_cmd)
                self.get_logger().info(f"Reorienting... yaw_diff: {yaw_diff:.2f}")
            else:
                # Stop the robot once it's oriented correctly
                self.stop_orientation()
                self.get_logger().info("Oriented towards the target!")
                break  # Exit the loop after successful orientation

            # Allow ROS to process callbacks so that position_callback can update the values
            rclpy.spin_once(self)

            # Sleep to allow time for position update
            time.sleep(0.2)  # Adjust this value to give enough time for pose updates

    def stop_orientation(self):
        """Stop the robot's angular velocity."""
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(rotate_cmd)

def main(args=None):
    rclpy.init(args=args)
    orient_bot = OrientBot()

    try:
        # Keep the orient_bot node running and listening for new target coordinates
        rclpy.spin(orient_bot)

    except KeyboardInterrupt:
        pass

    finally:
        orient_bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
