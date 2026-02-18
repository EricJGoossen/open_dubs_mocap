#!/usr/bin/env python

# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration.
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room.
from copy import deepcopy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import *
import numpy as np

class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_publisher")

        self.declare_parameter('car_name', 'open_dubs')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value

        self.declare_parameter('topic_name', 'default_topic')
        pub_topic = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher = self.create_publisher(
            PoseStamped, 
            "output_pose", 
            qos_profile=1
        )

        self.subscriber = self.create_subscription(
            PoseStamped,
            'input_pose',
            self.publish_car_pose,
            qos_profile=1
        )

    def publish_car_pose(self, msg):
        # The point on the top surface in the center of those 4 points has the following offset
        # w.r.t the base_link: (-0.058325, 0.0, 0.08125)

        # to matrix
        orientation = msg.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation = quaternion_matrix(quat)

        # rotate axes
        R = np.array([[0,  1,  0],
                      [0,  0,  1],
                      [1,  0,  0]], dtype=np.float32)

        R2 = np.array([[0,  0,  1],
                       [1,  0,  0],
                       [0,  1,  0]], dtype=np.float32)

        R3 = np.array([[0, -1,  0],
                       [1,  0,  0],
                       [0,  0,  1]], dtype=np.float32)
                

        # rotation[:3,:3] = np.dot(np.dot(R2, np.dot(rotation[:3,:3], R)), R3)
        rotation[:3,:3] = np.dot(R3, np.dot(R2, np.dot(rotation[:3,:3], R)))
        
        q = quaternion_from_matrix(rotation)

        p = deepcopy(msg)
        p.pose.position.x = msg.pose.position.z + 0.058325
        p.pose.position.y = msg.pose.position.x
        p.pose.position.z = msg.pose.position.y - 0.081250

        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        self.publisher.publish(p)


def main(args=None):
    rclpy.init()
    node = PosePublisher()

    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        node.destroy_node() 
        try:
            rclpy.shutdown()
        except:
            pass