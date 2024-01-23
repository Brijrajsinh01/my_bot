#!/usr/bin/env python3
import rclpy
from turtlesim.msg import Pose
from std_msgs.msg import String
import json
import math, copy

def brain():
    rclpy.init()
    node = rclpy.create_node('lookup_table')
    dictionary_publisher = node.create_publisher(String, 'distances', 10)
    dictionary_pos_publisher = node.create_publisher(String, 'position', 10)
    dist={}
    
    def dist_sub(msg, turtle_name):
        if not dist:
            dist.update({turtle_name:[msg.x,msg.y]})
        elif turtle_name in dist:
            dist[turtle_name]=[msg.x,msg.y]
        else:
            dist.update({turtle_name:[msg.x,msg.y]})
        print(dist)
        dist_matrix(dist)
        # return(dist)

    def distance(lst1,lst2):
        dist_ecl=math.dist(lst1,lst2)
        return dist_ecl


    def dist_matrix(dist):
        dist_mat=copy.deepcopy(dist)
        for i in dist_mat:
            dist_mat[i].clear()
            for j in dist_mat:
                if i==j:
                    # dist_mat[i].append(0)
                    continue
                else:
                    dist_mat[i].append(distance(dist[i],dist[j]))
        # print(dist_mat)
        json_string = json.dumps(dist_mat)
        msg = String()
        msg.data = json_string
        dictionary_publisher.publish(msg)
        json_pos_string = json.dumps(dist)
        msg_pos = String()
        msg_pos.data = json_pos_string
        dictionary_pos_publisher.publish(msg_pos)
        return dist_mat
    
    node.create_subscription(
        Pose,
        '/turtle1/pose',
        lambda msg: dist_sub(msg, 'turtle1'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle2/pose',
        lambda msg: dist_sub(msg, 'turtle2'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle3/pose',
        lambda msg: dist_sub(msg, 'turtle3'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle4/pose',
        lambda msg: dist_sub(msg, 'turtle4'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle5/pose',
        lambda msg: dist_sub(msg, 'turtle5'),
        10
    )
    node.create_subscription(
        Pose,
        '/turtle6/pose',
        lambda msg: dist_sub(msg, 'turtle6'),
        10
    )
    rclpy.spin(node)
    

if __name__ == '__main__':
    brain()