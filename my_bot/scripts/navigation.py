import rclpy
from nav2_msgs.action import NavigateToPose

def main():
    rclpy.init()
    node = rclpy.create_node('navigate_to_pose_example')

    action_client = node.create_client(NavigateToPose, 'navigate_to_pose')

    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Waiting for "navigate_to_pose" action server...')

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.pose.position.x = 2.0
    goal_msg.pose.pose.position.y = 2.0
    goal_msg.pose.pose.orientation.w = 1.0

    send_goal_future = action_client.send_goal_async(goal_msg)

    rclpy.spin_until_future_complete(node, send_goal_future)

    if send_goal_future.result() is not None:
        node.get_logger().info(f'Goal sent successfully')
    else:
        node.get_logger().error('Failed to send goal')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
