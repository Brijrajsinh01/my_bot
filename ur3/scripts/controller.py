import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class UR3Controller(Node):
    def __init__(self):
        super().__init__('ur3_controller')

        # Publisher to publish joint positions
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)

    def publish_joint_positions(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1','joint_2','joint_3','joint_4','gripper_joint_1','gripper_joint_2']
        msg.position = joint_positions
        self.joint_publisher.publish(msg)
        self.get_logger().info(f'Published joint positions: {joint_positions}')

def main():
    rclpy.init()
    node=rclpy.create_node('controller')
    rate = node.create_rate(2)

    ur3_controller = UR3Controller()

    try:
        while rclpy.ok():
            # Define joint positions (replace with your desired positions)
            joint_positions1 = [0.0, 1.0, 0.0, -1.0, 0.0, 0.0]
            ur3_controller.publish_joint_positions(joint_positions1)
            # time.sleep(1.0)

            joint_positions2 = [1.0, 1.0, 1.0, -1.0, 0.0, 0.0]
            # ur3_controller.publish_joint_positions(joint_positions2)
            # time.sleep(1.0)

    except KeyboardInterrupt:
        pass

    ur3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
