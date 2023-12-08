#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

def move_robot():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('move_robot_node')

    # Create a publisher for sending Twist messages to control the robot's movement
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)


    # Create a subscriber for receiving LaserScan messages from the Lidar
    def scan_callback(msg):

        # Extract distance information from eight sections over a 360-degree horizontal frame
        section_width = len(msg.ranges) // 360
        
        # for i in range(num_sections):
        #     start_index = i * section_width
        #     end_index = (i + 1) * section_width
        #     section_ranges = msg.ranges[start_index:end_index]
            
        #     # Find the minimum distance in each section
        #     min_distance = min(section_ranges)
            
        #     # Update the minimum distance for the section
        #     min_distances[i] = min(min_distances[i], min_distance)

        
    # Create a subscription to the LaserScan topic
    subscriber = node.create_subscription(LaserScan, '/scan', scan_callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop the robot when the script is interrupted
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    publisher.publish(twist_msg)
    node.get_logger().info('Robot stopped')

    # Clean up and shutdown the ROS node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    move_robot()
