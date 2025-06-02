#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import numpy as np
import cv2
import tf2_ros
import math

class ContourFrontierExplorer(Node):
    def __init__(self):
        super().__init__('contour_frontier_explorer')

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/exploration_goal', 10)
        self.marker_publisher = self.create_publisher(Marker, '/exploration_goal_marker', 10)
        self.done_subscription = self.create_subscription(Bool, '/goal_done', self.done_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_goal_sent = False
        self.goal_accepted = False
        self.last_published_goal = None
        self.goal_in_progress = False

        self.timer = self.create_timer(1.0, self.publish_goal_continuously)

    def done_callback(self, msg):
        """
        Handle goal completion. This callback updates the status of the goal.
        """
        if msg.data:
            self.get_logger().info("Goal completed. Ready to publish the next goal.")
            self.goal_accepted = True
            self.goal_in_progress = False
            self.last_goal_sent = False

    def map_callback(self, msg):
        """
        Compute and publish a new goal after checking the current goal status.
        """
        if self.goal_in_progress or self.last_goal_sent:
            return  # Skip map callback if the goal is in progress

        # Get the robot's position
        robot_x, robot_y = self.get_robot_position()
        
        # Use your map processing and frontier detection logic
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        grid = np.array(msg.data).reshape((height, width))

        # Build a binary image for frontier detection
        frontier_mask = np.zeros_like(grid, dtype=np.uint8)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid[y, x] == 0:
                    neighborhood = grid[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood == -1):
                        frontier_mask[y, x] = 255

        # Find contours in the frontier mask
        contours, _ = cv2.findContours(frontier_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info("No frontier contours found.")
            return

        best_score = -float('inf')
        best_midpoint = None

        for contour in contours:
            contour_length = len(contour)
            if contour_length < 10:
                continue
            mid_idx = contour_length // 2
            pixel_x, pixel_y = contour[mid_idx][0]
            map_x = origin.x + pixel_x * resolution
            map_y = origin.y + pixel_y * resolution
            distance = math.hypot(map_x - robot_x, map_y - robot_y)
            score = contour_length / (distance + 1e-5)  # Trade-off: longer contour and closer
            if score > best_score:
                best_score = score
                best_midpoint = (map_x, map_y)

        if best_midpoint is None:
            self.get_logger().info("No suitable contour found.")
            return

        map_x, map_y = best_midpoint

        # Create the goal message
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = map_x
        goal.pose.position.y = map_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        # Store the last goal sent to avoid publishing the same goal continuously
        self.last_published_goal = goal
        self.last_goal_sent = True
        self.goal_in_progress = True  # Mark that the goal is being processed

        # Publish goal
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"Published best-scoring contour goal at ({map_x:.2f}, {map_y:.2f})")

        # Publish marker for visualization
        self.publish_goal_marker(goal)

    def publish_goal_continuously(self):
        """
        Periodically re-publish the last goal if not acknowledged.
        """
        if self.last_goal_sent and not self.goal_accepted:
            self.get_logger().info("Re-publishing the same goal.")
            self.goal_publisher.publish(self.last_published_goal)

        elif self.goal_accepted:
            self.get_logger().info("Goal accepted. Waiting for new goal.")
            self.last_goal_sent = False
            self.goal_in_progress = False

    def publish_goal_marker(self, goal):
        """
        Publish a marker for visualization in RViz.
        """
        marker = Marker()
        marker.header = goal.header
        marker.ns = "contour_frontier"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 5
        self.marker_publisher.publish(marker)

    def get_robot_position(self):
        """
        Get the robot's current position using TF.
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}. Using default (0,0)")
            return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ContourFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
