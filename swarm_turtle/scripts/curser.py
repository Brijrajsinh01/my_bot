#!/usr/bin/env python3
import pyautogui
import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# Get the current mouse cursor position
def curser():
    C_x, C_y = pyautogui.position()

    # Print the coordinates
    # print(f"Mouse cursor coordinates: X={C_x}, Y={C_y}")
    return C_x,C_y

def swarm():
    global theta1  # Declare theta1 as a global variable
    rclpy.init()
    node = rclpy.create_node('self_arrange')
    pub1 = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    rate = node.create_rate(10)

    def callback1(msg, turtle_name):
        global theta1  # Declare theta1 as a global variable
        global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        theta2 = msg.theta
        x2=(msg.x)*(500/11)
        y2=(msg.y)*(500/11)
        twist_msg = Twist()
        x1 , y1 =curser()
        y1=1079-y1
        # print(f"Mouse cursor coordinates: X={x1}, Y={y1}")
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-15 <= int(Y) <= 15):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            print("no-expection")
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        distance=math.dist([x1,y1],[x2,y2])
        if distance>50:
            twist_msg.linear.x = abs(0.01*(x1-x2))
            if (-30 <= int(x1-x2) <= 30):
                twist_msg.linear.x = abs(0.01*(y1-y2))
                # print("totototototototo")
        
        pub1.publish(twist_msg)

    node.create_subscription(
        Pose,
        '/turtle1/pose',
        lambda msg: callback1(msg, 'turtle1'),
        10
    )

    rclpy.spin(node)

if __name__ == '__main__':
    swarm()
()