#!/usr/bin/env python3
import cv2
import rclpy
from turtlesim.msg import Pose
import numpy as np
import math

# Global variables for turtle poses
turtle_poses = {}

# Callback function for turtle pose
def turtle_pose_callback(msg, turtle_number):
    global turtle_poses
    turtle_poses[turtle_number] = msg

# Function to draw turtle on OpenCV canvas
def draw_turtle(image, pose, turtle_number):
    x, y = int(pose.x * 50), 500 - int(pose.y * 50)  # Scale turtle position for the canvas

    # Calculate the endpoint of the arrow based on the turtle's orientation
    length = 55
    endpoint = (
        int(x + length * np.cos(pose.theta)),
        int(y + length * np.sin(pose.theta + math.pi))
    )

    # Draw the arrow
    cv2.arrowedLine(image, (x, y), endpoint, (0, 255, 0), 2)

    radius = int(1.1 * 50)
    cv2.circle(image, (x, y), radius, (0, 0, 255), 2)

    # Add label text
    label_position = (endpoint[0] + 5, endpoint[1] - 5)  # Adjust label position
    label_text = f'{turtle_number}'
    cv2.putText(image, label_text, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

# Function to display the image
def show_image(image):
    cv2.imshow('TurtleSim Mimic', image)
    cv2.waitKey(1)  # Adjust the waitKey delay as needed

# Main function
def mimic_turtle_motion():
    global turtle_poses

    rclpy.init()
    node = rclpy.create_node('mimic_turtle_motion')

    # Create subscribers for turtle poses
    for i in range(1, 7):
        turtle_poses[f'turtle{i}'] = None
        node.create_subscription(Pose, f'/turtle{i}/pose', lambda msg, i=i: turtle_pose_callback(msg, f'turtle{i}'), 10)

    # Create OpenCV window with the same dimensions as TurtleSim
    image_size = (500, 500, 3)
    image = np.ones(image_size, dtype=np.uint8) * 255

    # Main loop
    while rclpy.ok():
        image.fill(255)

        # Draw turtles on the OpenCV canvas
        for turtle_number, pose in turtle_poses.items():
            if pose:
                draw_turtle(image, pose, turtle_number)

        # Show the image
        show_image(image)

        # Handle ROS events
        rclpy.spin_once(node)

    # Cleanup
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    mimic_turtle_motion()
