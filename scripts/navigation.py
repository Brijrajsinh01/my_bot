#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, Point, PoseStamped
from orient_bot import OrientBot  # Make sure orient_bot.py exists and is correct
import time
import math

# Global variables to store robot's position and the farthest obstacle
robot_x = 0.0
robot_y = 0.0
farthest_obstacle = None
oriented = False
decrement= 0.0

# Publisher for /cmd_vel
cmd_vel_publisher = None

def position_callback(msg):
    """Callback for receiving robot's position from /base_link_coordinates."""
    global robot_x, robot_y
    robot_x = msg.pose.position.x
    robot_y = msg.pose.position.y

def obstacle_callback(msg):
    """Callback for receiving farthest obstacle coordinates."""
    global farthest_obstacle
    farthest_obstacle = (msg.x, msg.y)
    print(f'Farthest obstacle received: (x: {msg.x}, y: {msg.y})')

def orient_towards_obstacle(orient_bot):
    """Orients the robot towards the farthest obstacle using the OrientBot."""
    global farthest_obstacle, oriented
    if farthest_obstacle:
        target_x, target_y = farthest_obstacle
        print(f'Oriented towards obstacle at: (x: {target_x}, y: {target_y})')

        # Use OrientBot to orient towards the obstacle
        orient_bot.orient_towards(target_x, target_y)
        oriented = True
    else:
        print('No farthest obstacle available for orientation.')
        oriented = False

def move(target_distance):
    """Moves the robot forward by a specific target distance."""
    global robot_x, robot_y

    # Get the initial position of the robot
    initial_x = robot_x
    initial_y = robot_y

    # Continuously calculate the distance traveled
    traveled_distance = 0.0

    while traveled_distance < target_distance:
        # Use spin_once to update the robot's position
        rclpy.spin_once(node, timeout_sec=0.1)

        # Calculate the distance traveled using Euclidean distance
        traveled_distance = math.sqrt((robot_x - initial_x) ** 2 + (robot_y - initial_y) ** 2)
        print(f"Traveled distance: {traveled_distance:.2f} meters")

        # Start moving the robot
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Set forward linear speed
        cmd_vel_publisher.publish(move_cmd)

        time.sleep(0.1)  # Sleep to avoid spamming the loop

    # Stop the robot once the distance is covered
    move_cmd.linear.x = 0.0
    cmd_vel_publisher.publish(move_cmd)
    print(f"Target distance of {target_distance} meters reached!")

def distance_to_obstacle():
    """Calculates the distance between the robot and the farthest obstacle."""
    global farthest_obstacle, robot_x, robot_y
    if farthest_obstacle:
        obstacle_x, obstacle_y = farthest_obstacle
        distance = math.sqrt((obstacle_x - robot_x) ** 2 + (obstacle_y - robot_y) ** 2)
        print(f"Distance to farthest obstacle: {distance:.2f} meters")
        return distance
    else:
        print("No farthest obstacle to calculate distance.")
        return None

def rotate(rotation_duration, speed=0.5):
    """Rotates the robot clockwise for a specific duration."""
    rotate_cmd = Twist()
    rotate_cmd.angular.z = abs(speed)  # Set negative angular velocity for clockwise rotation

    start_time = time.time()

    while (time.time() - start_time) < rotation_duration:
        cmd_vel_publisher.publish(rotate_cmd)
        print(f"Rotating clockwise for {rotation_duration:.2f} seconds...")
        time.sleep(0.1)

    # Stop the rotation
    rotate_cmd.angular.z = 0.0
    cmd_vel_publisher.publish(rotate_cmd)
    print("Rotation complete!")

def main(args=None):
    global cmd_vel_publisher, node, farthest_obstacle, oriented, decrement # Declare global variables

    rclpy.init(args=args)
    node = rclpy.create_node('navigation_node')

    # Publisher for /cmd_vel
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # Subscribers for /farthest_obstacle and /base_link_coordinates
    node.create_subscription(Point, '/farthest_obstacle', obstacle_callback, 10)
    node.create_subscription(PoseStamped, '/base_link_coordinates', position_callback, 10)

    rotate((math.pi)*8)
    # Create an instance of OrientBot for handling orientation
    orient_bot = OrientBot()

    try:
        print("trying")
        while rclpy.ok():
            # Wait for the farthest obstacle data
            while farthest_obstacle is None:
                print("Waiting for farthest obstacle data...")
                rclpy.spin_once(node, timeout_sec=0.1)

            # Rotate the robot for a fixed duration (example: 2 seconds)
            rotate(2.0)

            # Orient the robot towards the farthest obstacle
            orient_towards_obstacle(orient_bot)

            # Once oriented, move towards the obstacle
            if oriented:
                distance_left = distance_to_obstacle() - decrement  # Example, subtracting a margin
                print(f"Distance left to the obstacle: {distance_left:.2f} meters")
                move(distance_left)
                decrement=decrement+0.0

            # Reset for next cycle
            farthest_obstacle = None
            oriented = False
            print("Starting new cycle...\n")

    except KeyboardInterrupt:
        pass

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
