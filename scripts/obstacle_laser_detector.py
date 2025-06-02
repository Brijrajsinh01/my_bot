#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs  # Import for transforming geometry_msgs
from geometry_msgs.msg import PointStamped  # Import PointStamped

from geometry_msgs.msg import PointStamped  # Import PointStamped

class LaserScanVisualizer(Node):
    def __init__(self):
        super().__init__('laser_scan_visualizer')

        # Create a tf2 buffer and listener to get transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to the /scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create a publisher for the /farthest_obstacle topic
        self.farthest_obstacle_publisher = self.create_publisher(PointStamped, '/farthest_obstacle', 10)

        # OpenCV Canvas Properties
        self.canvas_size = 800  # Canvas width and height in pixels
        self.max_distance = 6.0  # Maximum distance to consider in meters
        self.meters_to_pixels = self.canvas_size / (2 * self.max_distance)  # Scale for plotting

        self.get_logger().info("LaserScanVisualizer node initialized.")

    def scan_callback(self, msg):
        # Create a blank canvas
        canvas = np.zeros((self.canvas_size, self.canvas_size, 3), dtype=np.uint8)

        # Center of the canvas (robot's position)
        center_x = self.canvas_size // 2
        center_y = self.canvas_size // 2

        # Variables to track the farthest obstacle
        max_distance = 0.0
        farthest_point = None

        # Iterate over laser scan data
        angle = msg.angle_min
        for distance in msg.ranges:
            if 0.0 < distance <= self.max_distance:  # Cap the distance at max_distance
                # Convert polar coordinates to Cartesian coordinates
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)

                # Scale to pixels
                pixel_x = int(center_x + x * self.meters_to_pixels)
                pixel_y = int(center_y - y * self.meters_to_pixels)  # Invert y-axis for OpenCV

                # Check if this is the farthest point
                if distance > max_distance:
                    max_distance = distance
                    farthest_point = (pixel_x, pixel_y)

                # Draw the point on the canvas (default green)
                cv2.circle(canvas, (pixel_x, pixel_y), 2, (0, 255, 0), -1)  # Green dots

            angle += msg.angle_increment

        # Highlight the farthest obstacle in blue
        if farthest_point:
            cv2.circle(canvas, farthest_point, 5, (255, 0, 0), -1)  # Blue dot for the farthest point

            # Create a PointStamped message to send the farthest obstacle coordinates
            farthest_msg = PointStamped()
            farthest_msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
            farthest_msg.header.frame_id = "laser_frame"  # Set the frame_id to laser_frame

            farthest_msg.point.x = farthest_point[0] / self.meters_to_pixels  # Convert to meters
            farthest_msg.point.y = farthest_point[1] / self.meters_to_pixels  # Convert to meters
            farthest_msg.point.z = 0.0  # Assuming a 2D scan, so z is 0

            # Transform the farthest point from laser frame to map frame
            transformed_point = self.transform_to_map_frame(farthest_msg)

            # Publish the farthest obstacle coordinates in the map frame
            if transformed_point:
                self.farthest_obstacle_publisher.publish(transformed_point)

        # Draw an arrow for the robot's position and direction
        arrow_length = 50  # Length of the arrow in pixels
        end_x = center_x + arrow_length
        end_y = center_y   # Default pointing upward in the canvas
        cv2.arrowedLine(canvas, (center_x, center_y), (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)

        # Rotate the canvas 90 degrees counterclockwise
        rotated_canvas = cv2.rotate(canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Display the rotated canvas
        cv2.imshow("Laser Scan Visualization", rotated_canvas)
        cv2.waitKey(1)  # Necessary for OpenCV rendering

    def transform_to_map_frame(self, farthest_point):
        # Try to get the transform from the laser frame to the map frame
        try:
            transform = self.tf_buffer.lookup_transform('map', 'laser_frame', rclpy.time.Time())
            # Apply the transform to the farthest_point
            transformed_point = tf2_geometry_msgs.do_transform_point(farthest_point, transform)
            return transformed_point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to get transform from laser_frame to map")
            return None

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    visualizer = LaserScanVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        visualizer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
