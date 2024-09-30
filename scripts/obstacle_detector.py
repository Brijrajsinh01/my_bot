#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import math

class SimpleObstacleDetector(Node):

    def __init__(self, node_name='simple_obstacle_detector'):
        super().__init__(node_name)
        
        # Initialize the robot position and farthest obstacle coordinates
        self.farthest_obstacle = None
        self.max_distance = float('-inf')

        # Subscribe to the /map topic to receive the occupancy grid
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create a publisher to publish markers for visualization in RViz
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # Create a publisher to publish the farthest obstacle coordinates
        self.farthest_obstacle_publisher = self.create_publisher(Point, '/farthest_obstacle', 10)

    def map_callback(self, msg):
        # Get map metadata
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position  # Bottom-left corner of the map

        # Convert the occupancy grid data into a numpy array for easier processing
        map_data = np.array(msg.data).reshape((height, width))

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

            # Find the farthest obstacle
            self.find_farthest_obstacle(obstacle_positions)
        else:
            self.get_logger().info('No obstacles detected.')

    def find_farthest_obstacle(self, obstacle_positions):
        # Assume robot is at the origin (0, 0)
        robot_x, robot_y = 0.0, 0.0
        max_distance = float('-inf')
        farthest_obstacle = None

        # Calculate the distance to each obstacle
        for obstacle_x, obstacle_y in obstacle_positions:
            distance = math.sqrt((obstacle_x - robot_x) ** 2 + (obstacle_y - robot_y) ** 2)
            if distance > max_distance:
                max_distance = distance
                farthest_obstacle = (obstacle_x, obstacle_y)

        # Update and log the farthest obstacle
        self.farthest_obstacle = farthest_obstacle
        self.max_distance = max_distance
        self.get_logger().info(f'Farthest obstacle at (x: {farthest_obstacle[0]}, y: {farthest_obstacle[1]}), Distance: {max_distance} meters')

        # Publish the marker to visualize the farthest obstacle in RViz
        self.publish_marker(farthest_obstacle)

        # Publish the coordinates of the farthest obstacle as a Point message
        point_msg = Point()
        point_msg.x = farthest_obstacle[0]
        point_msg.y = farthest_obstacle[1]
        point_msg.z = 0.0  # Assuming flat plane
        self.farthest_obstacle_publisher.publish(point_msg)

    def publish_marker(self, obstacle_position):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "odom"  # Make sure the marker frame matches the map frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "farthest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker to the farthest obstacle
        marker.pose.position.x = obstacle_position[0]
        marker.pose.position.y = obstacle_position[1]
        marker.pose.position.z = 0.0  # Keep it on the ground plane

        # Marker orientation and scale
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Size of the marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Marker color (Green with full opacity)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    # Create a node and spin to process incoming map messages
    obstacle_detector = SimpleObstacleDetector()

    try:
        rclpy.spin(obstacle_detector)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
