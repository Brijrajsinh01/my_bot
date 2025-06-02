#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import tf2_ros
import time  # Used for timeout checking

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')

        # Initialize action client for NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to /farthest_obstacle topic (which publishes Point)
        self.subscription = self.create_subscription(
            Point,  
            '/farthest_obstacle',
            self.obstacle_callback,
            10  # QoS depth
        )
        
        # Transform Listener to get the robot's current position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_active = False  # Track if a goal is currently being executed
        self.goal_handle = None   # Store the active goal handle
        self.goal_start_time = None  # Store goal start time for timeout check

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected! Listening to /farthest_obstacle')

    def obstacle_callback(self, msg):
        """Callback function to receive coordinates from /farthest_obstacle topic"""
        
        # Get robot's current position
        robot_x, robot_y = self.get_robot_position()

        # Calculate distance between robot's position and new goal
        distance = self.calculate_distance((robot_x, robot_y), (msg.x, msg.y))

        # Set a threshold to prevent redundant goals
        if distance < 0.1:  # If the new goal is very close to the robot's position
            self.get_logger().info(f"Goal is too close to current position ({distance:.2f}m). Skipping.")
            return

        # If a goal is already active, check if it's taking too long
        if self.goal_active:
            time_elapsed = time.time() - self.goal_start_time
            if time_elapsed > 30:  # Timeout after 30 seconds
                self.get_logger().warn(f"Goal is stuck! Canceling after {time_elapsed:.2f} seconds.")
                self.cancel_goal()
            else:
                self.get_logger().info("Another goal is in progress. Waiting for completion before sending a new goal.")
                return

        self.get_logger().info(f'New goal received: x={msg.x}, y={msg.y}')
        self.send_goal(msg.x, msg.y)

    def send_goal(self, x, y):
        """Send a goal to the Nav2 action server"""
        if self.goal_active:
            self.get_logger().info("Another goal is in progress. Waiting for completion before sending a new goal.")
            return

        self.goal_active = True  # Mark goal as active
        self.goal_start_time = time.time()  # Record goal start time

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Neutral orientation

        self.get_logger().info(f'Sending goal to (x: {x}, y: {y}) 🚀')
        self._goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle response from the action server after sending the goal"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected ❌')
            self.goal_active = False  # Reset goal state
            return

        self.get_logger().info('Goal accepted ✅')
        self._result_future = self.goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Receive feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

    def _result_callback(self, future):
        """Handle the final result of the goal"""
        result = future.result().result
        if result:
            self.get_logger().info(f'Goal reached! 🎯')
        else:
            self.get_logger().info('Failed to reach goal ❌')

        self.goal_active = False  # Allow new goals

    def cancel_goal(self):
        """Cancel the current goal if it's stuck"""
        if self.goal_handle is not None:
            self.get_logger().warn("Canceling goal due to timeout.")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)
            self.goal_active = False

    def _cancel_callback(self, future):
        """Callback after goal is canceled"""
        self.get_logger().info("Goal canceled successfully. Ready for a new goal.")

    def calculate_distance(self, p1, p2):
        """Calculate Euclidean distance between two points (x1, y1) and (x2, y2)"""
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    def get_robot_position(self):
        """Retrieve the robot's current position from the transform tree"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except tf2_ros.LookupException:
            self.get_logger().warn("Could not get robot position from TF. Using default (0,0).")
            return 0.0, 0.0  # Default if TF lookup fails
        except tf2_ros.ConnectivityException:
            self.get_logger().warn("TF Connectivity error. Using default (0,0).")
            return 0.0, 0.0
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("TF Extrapolation error. Using default (0,0).")
            return 0.0, 0.0

def main(args=None):
    rclpy.init(args=args)
    nav2_goal_sender = Nav2GoalSender()

    try:
        rclpy.spin(nav2_goal_sender)
    except KeyboardInterrupt:
        pass

    nav2_goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
