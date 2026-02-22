#!/usr/bin/env python3 
# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration. 
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room. 

import rclpy 
from rclpy.node import Node
from copy import deepcopy 
from geometry_msgs.msg import PoseStamped 
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
import numpy as np 
from ament_index_python.packages import get_package_share_directory

class RelayMocapNode(Node): 
    def __init__(self): 
        super().__init__("pose_publisher") 

        DEG2RAD = np.pi / 180.0 
        
        # Declare parameters        
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)

        self.declare_parameter('offset_roll', 0.0)
        self.declare_parameter('offset_pitch', 0.0)
        self.declare_parameter('offset_yaw', 0.0)

        # Get parameters
        self.offset_x = self.get_parameter('offset_x').get_parameter_value().double_value 
        self.offset_y = self.get_parameter('offset_y').get_parameter_value().double_value 
        self.offset_z = self.get_parameter('offset_z').get_parameter_value().double_value 

        self.offset_roll = self.get_parameter('offset_roll').get_parameter_value().double_value * DEG2RAD 
        self.offset_pitch = self.get_parameter('offset_pitch').get_parameter_value().double_value * DEG2RAD 
        self.offset_yaw = self.get_parameter('offset_yaw').get_parameter_value().double_value * DEG2RAD 
            
        self.publisher = self.create_publisher( 
            PoseStamped, 
            'output_pose', 
            qos_profile=1 
        ) 
        
        self.subscriber = self.create_subscription( 
            PoseStamped, 
            'input_pose', 
            self.publish_car_pose, 
            qos_profile=1 
        ) 

        self.get_logger().info( 
            f"Pose relay node started\n" 
            f"Position offsets: [{self.offset_x}, {self.offset_y}, {self.offset_z}]\n" 
            f"Orientation offsets: [{self.offset_roll}, {self.offset_pitch}, {self.offset_yaw}]" 
        ) 
            
    def publish_car_pose(self, msg): 
        orientation = msg.pose.orientation 
        quat = [orientation.x, orientation.y, orientation.z, orientation.w] 
        roll, pitch, yaw = euler_from_quaternion(quat) 
        
        # hacky orientation offset (ideally you should have quaternion offsets, this sort of offset only works if you just want to rotate around 1 axis at a time) 
        roll += self.offset_roll 
        pitch += self.offset_pitch 
        yaw += self.offset_yaw 
        
        quat = quaternion_from_euler(roll, pitch, yaw) 

        p = deepcopy(msg) 
        p.pose.position.x = msg.pose.position.x + self.offset_x 
        p.pose.position.y = msg.pose.position.y + self.offset_y 
        p.pose.position.z = msg.pose.position.z + self.offset_z 
        
        p.pose.orientation.x = quat[0] 
        p.pose.orientation.y = quat[1] 
        p.pose.orientation.z = quat[2] 
        p.pose.orientation.w = quat[3] 
        self.publisher.publish(p) 
        
def main(args=None): 
    rclpy.init() 
    node = RelayMocapNode() 
    
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