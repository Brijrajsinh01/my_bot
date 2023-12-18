#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
import signal
from sensor_msgs.msg import LaserScan

def move_robot():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('keyboard_control_node')

    # Create a publisher for sending Twist messages to control the robot's movement
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    rate = node.create_rate(10)

    def on_key_press(key):
        nonlocal publisher

        twist_msg = Twist()

        if key == keyboard.Key.left:
            twist_msg.angular.z = 2.0  # Positive angular velocity for counterclockwise turn
        elif key == keyboard.Key.right:
            twist_msg.angular.z = -2.0  # Negative angular velocity for clockwise turn
        elif key == keyboard.Key.up:
            twist_msg.linear.x = 0.2  # Positive linear velocity for forward movement
        elif key == keyboard.Key.down:
            twist_msg.linear.x = -0.2  # Negative linear velocity for backward movement
        elif key == keyboard.KeyCode.from_char('q'):
            node.get_logger().info('Pressed "q". Exiting...')
            node.destroy_node()
            rclpy.shutdown()

        publisher.publish(twist_msg)

    def on_key_release(key):
        nonlocal publisher

        twist_msg = Twist()
        publisher.publish(twist_msg)

    def signal_handler(sig, frame):
        nonlocal node
        node.get_logger().info('Received Ctrl+C. Stopping the robot...')
        node.destroy_node()
        rclpy.shutdown()

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Use a separate thread or process for the keyboard listener to avoid blocking
    listener_thread = keyboard.Listener(on_press=on_key_press, on_release=on_key_release, suppress=False)
    listener_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            rate.sleep()
    except KeyboardInterrupt:
        pass

    # Stop the robot when the script is interrupted
    twist_msg = Twist()
    publisher.publish(twist_msg)
    node.get_logger().info('Robot stopped')

    # Clean up and shutdown the ROS node
    node.destroy_node()
    rclpy.shutdown()

    # Stop the keyboard listener thread
    listener_thread.stop()

def lidar():
    rclpy.init()
    
    node = rclpy.create_node('self_controlled')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(msg):

        # Extract distance information from eight sections over a 360-degree horizontal frame
        section_width = len(msg.ranges) // 360

        dis={
            "one": min( msg.ranges[0:44]),
            "two": min(msg.ranges[45:89]),
            "three": min(msg.ranges[90:134]),
            "four": min(msg.ranges[135:170]),
            "five": min(msg.ranges[180:224]),
            "six": min(msg.ranges[225:269]),
            "seven": min(msg.ranges[270:314]),
            "eight": min(msg.ranges[315:360]),

        }
        # for i in range(num_sections):
        #     start_index = i * section_width
        #     end_index = (i + 1) * section_width
        #     section_ranges = msg.ranges[start_index:end_index]

        #     # Find the minimum distance in each section
        #     min_distance = min(section_ranges)

        #     # Update the minimum distance for the section
        #     min_distances[f'section_{i}'] = min(min_distances[f'section_{i}'], min_distance)

        # Print or use the min_distances dictionary as needed
        print(dis)

    subscriber = node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    
    # Set the publishing rate (adjust as needed)
    rate = node.create_rate(10)

    while rclpy.ok():
        rclpy.spin(node)
        rate.sleep()

    # Clean up and shutdown the ROS node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # Uncomment one of the following functions based on your desired mode
    move_robot()  # Manual control using the keyboard
    # lidar()  # Autonomous control using Lidar data
