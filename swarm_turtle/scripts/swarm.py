#!/usr/bin/env python3
import rclpy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math, copy
from std_msgs.msg import String
import json


def swarm():
    global theta1,received_dict # Declare theta1 as a global variable
    
    rclpy.init()
    node = rclpy.create_node('self_arrange')
    pub1 = node.create_publisher(Twist, '/turtle2/cmd_vel', 10)
    pub2 = node.create_publisher(Twist, '/turtle3/cmd_vel', 10)
    pub3 = node.create_publisher(Twist, '/turtle4/cmd_vel', 10)
    pub4 = node.create_publisher(Twist, '/turtle5/cmd_vel', 10)
    pub5 = node.create_publisher(Twist, '/turtle6/cmd_vel', 10)
    rate = node.create_rate(10)

    def unfold(msg):
        global received_dict
        received_dict = json.loads(msg.data)
        # print(received_dict)

        

    def callback(msg, turtle_name):
        global theta1,x1,y1 # Use the nonlocal keyword to modify the global variable
        # dist_sub(msg, turtle_name)
        theta1 = msg.theta
        x1=msg.x
        y1=msg.y

        # print(theta1)

    def callback1(msg, turtle_name):
        global theta1,received_dict  # Declare theta1 as a global variable
        global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        # print(msg.theta)
        # matrix=dist_sub(msg, turtle_name)
        theta2 = msg.theta
        x2=msg.x
        y2=msg.y
        twist_msg = Twist()
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-0.22 <= int(Y) <= 0.22):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            # print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            # print(angle_radians)
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        # distance=math.dist([x1,y1],[x2,y2])
        # print(Y)
        # print(received_dict[turtle_name])
        minimum=min(received_dict[turtle_name])
        if minimum>1.1:
            twist_msg.linear.x = abs(0.3*(x1-x2))
            if (-0.66 <= int(x1-x2) <= 0.66):
                twist_msg.linear.x = abs(0.01*(y1-y2))
        # twist_msg.linear.y = abs(0.3*(y1-y2))
        # print(twist_msg.angular.z)
        
        pub1.publish(twist_msg)

    def callback2(msg, turtle_name):
        global theta1  # Declare theta1 as a global variable
        # global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        # print(msg.theta)
        # matrix=dist_sub(msg, turtle_name)
        theta2 = msg.theta
        x2=msg.x
        y2=msg.y
        twist_msg = Twist()
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-0.22 <= int(Y) <= 0.22):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            # print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            # print(angle_radians)
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        # distance=math.dist([x1,y1],[x2,y2])
        # print(Y)
        minimum=min(received_dict[turtle_name])
        if minimum>1.1:
            twist_msg.linear.x = abs(0.3*(x1-x2))
            if (-0.66 <= int(x1-x2) <= 0.66):
                twist_msg.linear.x = abs(0.01*(y1-y2))
        # twist_msg.linear.y = abs(0.3*(y1-y2))
        # print(twist_msg.angular.z)
        
        pub2.publish(twist_msg)


    def callback3(msg, turtle_name):
        global theta1  # Declare theta1 as a global variable
        global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        # print(msg.theta)
        # matrix=dist_sub(msg, turtle_name)
        theta2 = msg.theta
        x2=msg.x
        y2=msg.y
        twist_msg = Twist()
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-0.22 <= int(Y) <= 0.22):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            # print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            # print(angle_radians)
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        # distance=math.dist([x1,y1],[x2,y2])
        # print(Y)
        minimum=min(received_dict[turtle_name])
        if minimum>1.1:
            twist_msg.linear.x = abs(0.3*(x1-x2))
            if (-0.66 <= int(x1-x2) <= 0.66):
                twist_msg.linear.x = abs(0.01*(y1-y2))
        # twist_msg.linear.y = abs(0.3*(y1-y2))
        # print(twist_msg.angular.z)
        
        pub3.publish(twist_msg)

    def callback4(msg, turtle_name):
        global theta1  # Declare theta1 as a global variable
        global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        # print(msg.theta)
        # matrix=dist_sub(msg, turtle_name)
        theta2 = msg.theta
        x2=msg.x
        y2=msg.y
        twist_msg = Twist()
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-0.22 <= int(Y) <= 0.22):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            # print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            # print(angle_radians)
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        # distance=math.dist([x1,y1],[x2,y2])
        # print(Y)
        minimum=min(received_dict[turtle_name])
        if minimum>1.1:
            twist_msg.linear.x = abs(0.3*(x1-x2))
            if (-0.66 <= int(x1-x2) <= 0.66):
                twist_msg.linear.x = abs(0.01*(y1-y2))
        # twist_msg.linear.y = abs(0.3*(y1-y2))
        # print(twist_msg.angular.z)
        
        pub4.publish(twist_msg)
    
    def callback5(msg, turtle_name):
        global theta1  # Declare theta1 as a global variable
        global theta2,x2,y2  # Declare theta2 as a nonlocal variable
        # print(msg.theta)
        # matrix=dist_sub(msg, turtle_name)
        theta2 = msg.theta
        x2=msg.x
        y2=msg.y
        twist_msg = Twist()
        Y=y1-y2
        X=x1-x2
        if X < 0 and (-0.22 <= int(Y) <= 0.22):
            X=abs(X)
            angle_radians = math.atan2(X,Y)
            # print("exception")
            if theta2>0:
                angle_radians=math.pi
            else:
                angle_radians=-math.pi
            twist_msg.angular.z =6.0*(angle_radians-theta2)
        else:
            angle_radians = math.atan2(Y,X)
            # print(angle_radians)
            twist_msg.angular.z =(angle_radians-theta2)  # Set the angular velocity
        # distance=math.dist([x1,y1],[x2,y2])
        # print(Y)
        minimum=min(received_dict[turtle_name])
        if minimum>1.1:
            twist_msg.linear.x = abs(0.3*(x1-x2))
            if (-0.66 <= int(x1-x2) <= 0.66):
                twist_msg.linear.x = abs(0.01*(y1-y2))
        # twist_msg.linear.y = abs(0.3*(y1-y2))
        # print(twist_msg.angular.z)
        
        pub5.publish(twist_msg)

    node.create_subscription(
        String,
        '/distances', 
        unfold, 
        10
    )
    node.create_subscription(
        Pose,
        '/turtle1/pose',
        lambda msg: callback(msg, 'turtle1'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle2/pose',
        lambda msg: callback1(msg, 'turtle2'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle3/pose',
        lambda msg: callback2(msg, 'turtle3'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle4/pose',
        lambda msg: callback3(msg, 'turtle4'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle5/pose',
        lambda msg: callback4(msg, 'turtle5'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle6/pose',
        lambda msg: callback5(msg, 'turtle6'),
        10
    )
    
    rclpy.spin(node)

if __name__ == '__main__':
    swarm()