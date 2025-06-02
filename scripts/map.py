#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import cv2


class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.marker_publisher = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )
        self.farthest_obstacle_publisher = self.create_publisher(
            Point,
            '/farthest_obstacle',
            10
        )

        self.map_data = None
        self.scale_factor = 4  # Scale up the canvas
        self.grid_size = 2  # Size of the grid (4x4 chunks)
        self.unexplored_threshold = 0.9  # 50% unexplored cells to mark the block as unexplored
        self.area_threshold = 600  # Minimum area of a contour in pixels (scaled)

    def map_callback(self, msg):
        """Process the received map and find the farthest unexplored region."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        map_array = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Convert to an image representation
        map_image = np.zeros((height, width, 3), dtype=np.uint8)
        map_image[map_array == -1] = (128, 128, 128)  # Unknown = Gray
        map_image[map_array == 0] = (255, 255, 255)  # Free = White
        map_image[map_array == 100] = (0, 0, 0)      # Occupied = Black

        # Scale up the map for better visualization
        larger_map_image = cv2.resize(
            map_image,
            (width * self.scale_factor, height * self.scale_factor),
            interpolation=cv2.INTER_NEAREST
        )

        # Flip the map vertically (ROS coordinate alignment)
        transform_matrix = np.array([
            [1, 0, 0],  
            [0, -1, height * self.scale_factor],  
        ], dtype=np.float32)
        flipped_map_image = cv2.warpAffine(
            larger_map_image,
            transform_matrix,
            (width * self.scale_factor, height * self.scale_factor)
        )

        # Create a mask for unexplored regions
        unexplored_mask = np.zeros(
            (height * self.scale_factor, width * self.scale_factor), dtype=np.uint8
        )

        # Identify unexplored grid areas
        for y in range(0, height, self.grid_size):
            for x in range(0, width, self.grid_size):
                grid_chunk = map_array[y:y + self.grid_size, x:x + self.grid_size]
                unexplored_cells = np.sum(grid_chunk == -1)
                if unexplored_cells / (self.grid_size ** 2) > self.unexplored_threshold:
                    cv2.rectangle(
                        unexplored_mask,
                        (x * self.scale_factor, y * self.scale_factor),
                        ((x + self.grid_size) * self.scale_factor, (y + self.grid_size) * self.scale_factor),
                        255,
                        -1
                    )

        # Apply the transformation to align unexplored regions
        unexplored_mask = cv2.warpAffine(
            unexplored_mask,
            transform_matrix,
            (width * self.scale_factor, height * self.scale_factor)
        )

        # Find contours for unexplored regions
        contours, _ = cv2.findContours(unexplored_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        farthest_centroid = None
        max_distance = 0  # Track the farthest distance

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.area_threshold:
                # Draw the contour for visualization
                cv2.drawContours(flipped_map_image, [contour], -1, (0, 255, 0), 2)  # Green contour

                # Compute centroid of the unexplored region
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx_pixel = int(M["m10"] / M["m00"])
                    cy_pixel = int(M["m01"] / M["m00"])

                    # Convert pixel to map coordinates
                    cx_map = origin.x + (cx_pixel / self.scale_factor) * resolution
                    cy_map = origin.y + ((height * self.scale_factor - cy_pixel) / self.scale_factor) * resolution

                    # Compute distance from the robot's starting position (0,0)
                    distance = np.sqrt(cx_map**2 + cy_map**2)
                    if distance > max_distance:  # Find the farthest unexplored region
                        max_distance = distance
                        farthest_centroid = (cx_map, cy_map, cx_pixel, cy_pixel)

        # If a farthest unexplored region is found, visualize and publish it
        if farthest_centroid:
            cx_map, cy_map, cx_pixel, cy_pixel = farthest_centroid

            # Draw the farthest centroid on the map
            cv2.circle(flipped_map_image, (cx_pixel, cy_pixel), 5, (0, 0, 255), -1)  # Red circle
            cv2.putText(
                flipped_map_image,
                f"({cx_map:.2f}, {cy_map:.2f})",
                (cx_pixel + 10, cy_pixel),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1,
                cv2.LINE_AA
            )

            # Publish a marker for visualization in RViz
            marker = Marker()
            marker.header.frame_id = "map"  # Ensure it matches the global frame in RViz
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "farthest_centroid"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx_map
            marker.pose.position.y = cy_map
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3  # Increase the size for visibility
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0  # Ensure visibility
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime.sec = 10  # Marker stays for 10 seconds

            self.marker_publisher.publish(marker)

            self.get_logger().info(f"Published marker at: ({cx_map:.2f}, {cy_map:.2f})")

        # Display the flipped map with unexplored areas and centroids
        self.map_data = flipped_map_image
        cv2.imshow('Map with Unexplored Contours', self.map_data)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()

    try:
        rclpy.spin(map_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        map_visualizer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
