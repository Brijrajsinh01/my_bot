#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from visualization_msgs.msg import Marker

class ExplorationGoalPublisher(Node):

    def __init__(self):
        super().__init__('exploration_goal_publisher')  # ✅ Node must be initialized first

        # Publishers
        self.publisher = self.create_publisher(PoseStamped, '/exploration_goal', 10)
        self.marker_pub = self.create_publisher(Marker, '/exploration_goal_marker', 10)
        self.last_goal = None
        self.goal_distance_threshold = 0.2  # meters; ignore goals too close to the last one

        # Subscriptions
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Parameters
        self.frontier_radius = 5
        self.info_gain_threshold = 5
        self.robot_pos = (0.0, 0.0)


    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Convert flat map to 2D array
        map_data = np.array(msg.data).reshape((height, width))

        # Find frontiers (free cell next to unknown)
        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if map_data[y, x] == 0:  # Free
                    if self.is_frontier_cell(map_data, x, y, width, height):
                        frontiers.append((x, y))

        if not frontiers:
            self.get_logger().info('No frontiers detected.')
            return

        best_score = -1
        best_frontier = None

        for fx, fy in frontiers:
            info_gain = self.compute_information_gain(map_data, fx, fy, width, height, self.frontier_radius)
            if info_gain < self.info_gain_threshold:
                continue

            # Convert to world coords
            wx = origin.x + fx * resolution
            wy = origin.y + fy * resolution

            # Distance to robot (approx; can improve with TF later)
            distance = math.sqrt((wx - self.robot_pos[0])**2 + (wy - self.robot_pos[1])**2)
            score = info_gain / (distance + 1e-5)

            if score > best_score:
                best_score = score
                best_frontier = (wx, wy)

        if best_frontier:
            self.publish_goal(best_frontier[0], best_frontier[1])
            self.get_logger().info(f"Published exploration goal at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")
        else:
            self.get_logger().info("No suitable exploration goal found.")

    def is_frontier_cell(self, map_data, x, y, width, height):
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if map_data[ny, nx] == -1:  # Unknown
                        return True
        return False

    def compute_information_gain(self, map_data, x, y, width, height, radius):
        gain = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if dx**2 + dy**2 <= radius**2:
                        if map_data[ny, nx] == -1:
                            gain += 1
        return gain



    def publish_goal(self, x, y):
    # 🛑 Skip if goal is too close to last goal
        if self.last_goal is not None:
            dx = x - self.last_goal[0]
            dy = y - self.last_goal[1]
            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.goal_distance_threshold:
                self.get_logger().info(f"Goal ({x:.2f}, {y:.2f}) is too close to last goal, skipping.")
                return

        # ✅ Proceed to publish
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        print("publishing goal")

        # 🟢 Marker
        marker = Marker()
        marker.header = goal.header
        marker.ns = "exploration_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal.pose
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

        print("marker testing")
        self.get_logger().info(f"Published exploration goal at ({x:.2f}, {y:.2f})")

        # 🧠 Update last goal
        self.last_goal = (x, y)




def main(args=None):
    rclpy.init(args=args)
    node = ExplorationGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
