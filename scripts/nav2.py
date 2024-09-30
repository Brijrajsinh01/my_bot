#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')

        # Initialize action client for NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for the action server to be ready
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send goal to specified coordinates (change these to the desired coordinates)
        goal_msg = NavigateToPose.Goal()

        # Set target pose (change this to your goal coordinates)
        goal_msg.pose.header.frame_id = 'map'  # Usually 'map' frame for Nav2
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set the desired goal coordinates (change this for your specific use case)
        goal_msg.pose.pose.position.x = -0.2  # X coordinate
        goal_msg.pose.pose.position.y = 0.2  # Y coordinate
        goal_msg.pose.pose.orientation.z = 0.0  # Yaw (you can set an orientation)
        goal_msg.pose.pose.orientation.w = 1.0  # Must normalize quaternion for valid orientation

        # Send the goal and monitor the result
        self.get_logger().info(f'Sending goal to (x: {goal_msg.pose.pose.position.x}, y: {goal_msg.pose.pose.position.y})')
        self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        """Send the goal to the Nav2 action server."""
        self._goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle the response from the action server after sending the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """Receive feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance remaining: {feedback.distance_remaining:.2f} meters')

    def _result_callback(self, future):
        """Handle the final result of the goal."""
        result = future.result().result
        if result:
            self.get_logger().info(f'Goal reached! Total time: {result.total_elapsed_time.sec} seconds')
        else:
            self.get_logger().info('Failed to reach goal')

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
