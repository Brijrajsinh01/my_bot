#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point  # Import the transformation function for PointStamped

class CoordinateTransformer(Node):

    def __init__(self):
        super().__init__('coordinate_transformer')

        # Create a tf2 buffer and
        #  listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set a timer to repeatedly call the transform function
        self.timer = self.create_timer(1.0, self.transform_coordinates)

    def transform_coordinates(self):
        try:
            # Define the point in the odom frame
            point_in_odom = PointStamped()
            point_in_odom.header.stamp = self.get_clock().now().to_msg()
            point_in_odom.header.frame_id = 'odom'  # Frame we're transforming from
            point_in_odom.point.x = 0.0
            point_in_odom.point.y = 0.0
            point_in_odom.point.z = 0.0  # Example point, adjust if needed

            # Look up the transform from odom to base_link
            transform = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())

            # Transform the point to the base_link frame
            transformed_point = do_transform_point(point_in_odom, transform)

            # Log the transformed coordinates
            self.get_logger().info(f'Transformed coordinates in base_link: '
                                   f'({transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z})')

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform coordinates: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
