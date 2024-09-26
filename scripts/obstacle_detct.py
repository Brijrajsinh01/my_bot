#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker
import numpy as np
import math

class MapSubscriber(Node):

    def __init__(self):
        super().__init__('map_subscriber')
        
        # Initialize robot position variables
        self.robot_x = None
        self.robot_y = None
        self.grouping_threshold = 0.1  # Threshold to group obstacles by proximity (in meters)

        # Subscribe to the robot's position (from /odom)
        self.create_subscription(
            Odometry,
            '/odom',  # Subscribing to /odom for robot position updates
            self.position_callback,
            10
        )

        # Subscribe to the /map topic to receive the occupancy grid
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create a publisher to publish markers for visualization in RViz
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

    def position_callback(self, msg):
        # Update the robot's position from the Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        if self.robot_x is None or self.robot_y is None:
            self.get_logger().info('Waiting for robot position...')
            return
        
        # Get map metadata
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position  # Bottom-left corner of the map
        
        # Convert the occupancy grid data into a numpy array for easier processing
        map_data = np.array(msg.data).reshape((height, width))

        # Log some map details
        self.get_logger().info(f'Map received: {width}x{height}, resolution: {resolution}')

        # Detect obstacles (value of 100 represents obstacles)
        obstacles = np.where(map_data == 100)
        obstacle_positions = []

        if len(obstacles[0]) > 0:
            self.get_logger().info(f'Obstacles detected at: {len(obstacles[0])} points')

            # Store obstacle positions in the map frame
            for i in range(len(obstacles[0])):
                obstacle_x = obstacles[1][i] * resolution + origin.x
                obstacle_y = obstacles[0][i] * resolution + origin.y
                obstacle_positions.append((obstacle_x, obstacle_y))

            # Now group the obstacles and find the closest one in each group
            self.find_closest_in_groups(obstacle_positions)
        else:
            self.get_logger().info('No obstacles detected.')

    def find_closest_in_groups(self, obstacle_positions):
        groups = []
        while obstacle_positions:
            # Start a new group with the first obstacle
            group = [obstacle_positions.pop(0)]
            
            # Compare the rest of the obstacles to the current group
            i = 0
            while i < len(obstacle_positions):
                obstacle_x, obstacle_y = obstacle_positions[i]
                in_group = False
                
                # Check if the current obstacle is close to any in the group
                for gx, gy in group:
                    if math.sqrt((gx - obstacle_x) ** 2 + (gy - obstacle_y) ** 2) <= self.grouping_threshold:
                        group.append(obstacle_positions.pop(i))
                        in_group = True
                        break
                
                if not in_group:
                    i += 1

            # Find the closest obstacle in this group to the robot
            self.find_closest_obstacle(group)

    def find_closest_obstacle(self, group):
        closest_obstacle = None
        min_distance = float('inf')
        
        # Calculate the distance to each obstacle in the group
        for obstacle_x, obstacle_y in group:
            distance = math.sqrt((obstacle_x - self.robot_x) ** 2 + (obstacle_y - self.robot_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = (obstacle_x, obstacle_y)
        
        if closest_obstacle:
            self.get_logger().info(f'Closest obstacle in group at (x: {closest_obstacle[0]}, y: {closest_obstacle[1]}), Distance: {min_distance} meters')
            self.publish_marker(closest_obstacle)

    def publish_marker(self, obstacle_position):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "odom"  # Ensure the marker frame matches your robot's frame (usually "map" or "odom")
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "obstacle_markers"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = obstacle_position[0]
        marker.pose.position.y = obstacle_position[1]
        marker.pose.position.z = 0.0  # Keep it on the ground plane

        # Marker orientation and scale
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Increased size of the marker
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Marker color (White with full opacity)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set a longer lifetime for the marker to remain visible in RViz
        marker.lifetime.sec = 0  # Marker will be visible for 60 seconds

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    # Create a node and spin to process incoming map and position messages
    map_subscriber = MapSubscriber()

    try:
        rclpy.spin(map_subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    map_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
