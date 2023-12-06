#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist

def move_robot():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('move_robot_node')

    # Create a publisher for sending Twist messages to control the robot's movement
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    

    # Create a Twist message to command the robot's velocity
    twist_msg = Twist()
    twist_msg.linear.x = 0.0  # Set linear velocity in the x-axis (adjust as needed)
    twist_msg.angular.z = 0.2  # Set angular velocity in the z-axis (adjust as needed)
    print("####################################################################################################################################################################################################################################################################################")

    try:
        while rclpy.ok():
            # Publish the Twist message to move the robot
            publisher.publish(twist_msg)
            node.get_logger().info('Moving the robot')

            # Sleep for a short duration
            rclpy.spin_once(node)
            node.sleep(0.1)

    except KeyboardInterrupt:
        pass

    # Stop the robot when the script is interrupted
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    publisher.publish(twist_msg)
    node.get_logger().info('Robot stopped')

    # Clean up and shutdown the ROS node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    move_robot()
