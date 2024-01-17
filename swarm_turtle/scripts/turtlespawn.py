#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_turtle_service = self.create_client(Spawn, '/spawn')
        while not self.spawn_turtle_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "/spawn" not available, waiting again...')
        self.request = Spawn.Request()

    def spawn_turtle(self, turtle_name, x, y, theta):
        self.request.name = turtle_name
        self.request.x = x
        self.request.y = y
        self.request.theta = theta
        future = self.spawn_turtle_service.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    print("entering turtlespawn---------------------------------------------------------------------------------------------------------------------------------------------")
    rclpy.init(args=args)
    spawner = TurtleSpawner()

    # Spawn the first turtle
    spawner.spawn_turtle('turtle1', 2.0, 2.0, 0.0)

    # Spawn the second turtle
    spawner.spawn_turtle('turtle2', 0.5, 1.0, 0.0)

    # Spawn the third turtle
    spawner.spawn_turtle('turtle3', 2.5, 1.0, 0.0)

    # Spawn the fourth turtle
    spawner.spawn_turtle('turtle4', 4.5, 1.0, 0.0)

    # Spawn the fifth turtle
    spawner.spawn_turtle('turtle5', 6.5, 1.0, 0.0)

    # Spawn the six turtle
    spawner.spawn_turtle('turtle6', 8.5, 1.0, 0.0)


if __name__ == '__main__':
    main()
