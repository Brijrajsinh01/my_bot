#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
import signal
from turtlesim.msg import Pose

def move_robot():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('keyboard_control_node')

    # Create a publisher for sending Twist messages to control the robot's movement
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    rate = node.create_rate(10)

    def on_key_press(key):
        nonlocal publisher

        twist_msg = Twist()

        if key == keyboard.Key.left:
            twist_msg.angular.z = 2.0  # Positive angular velocity for counterclockwise turn
        elif key == keyboard.Key.right:
            twist_msg.angular.z = -2.0  # Negative angular velocity for clockwise turn
        elif key == keyboard.Key.up:
            twist_msg.linear.x = 2.0  # Positive linear velocity for forward movement
        elif key == keyboard.Key.down:
            twist_msg.linear.x = -2.0  # Negative linear velocity for backward movement
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

def swarm():
    node= rclpy.create_node('self_arrange')
    sub=node.create_subscription(
        Pose,
        '/turtle2/pose',
        callback(),
        10

    )
    def callback(msg):
        print("x : ", msg.position.x , "y :", msg.position.y)

if __name__ == '__main__':
    # Uncomment one of the following functions based on your desired mode
    move_robot()  # Manual control using the keyboard