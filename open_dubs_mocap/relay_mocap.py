#!/usr/bin/env python3 
# This publishes "absolute" pose of the car with respect to the "ground" reference set during calibration. 
# Since mocap publishes y-up we swap the axis to be z-up and x to be the horizontal axis of the room. 

import rclpy 
from rclpy.node import Node
from copy import deepcopy 
from geometry_msgs.msg import PoseStamped 
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
import numpy as np 
import yaml 
import sys 
from ament_index_python.packages import get_package_share_directory
import os 

class RelayMocapNode(Node): 
    def __init__(self): 
        super().__init__("pose_publisher") 
        
        pkg_share = get_package_share_directory('open_dubs_mocap')
        config_path = os.path.join(pkg_share, 'config', 'mocap_tf_offset.yaml')
        with open(config_path) as f: 
            config = yaml.safe_load(f) 
            DEG2RAD = np.pi / 180.0 
            
            self.off_x = config["x"] 
            self.off_y = config["y"] 
            self.off_z = config["z"] 
            
            self.off_roll = config["roll"] * DEG2RAD 
            self.off_pitch = config["pitch"] * DEG2RAD 
            self.off_yaw = config["yaw"] * DEG2RAD 
            
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
            
    def publish_car_pose(self, msg): 
        orientation = msg.pose.orientation 
        quat = [orientation.x, orientation.y, orientation.z, orientation.w] 
        roll, pitch, yaw = euler_from_quaternion(quat) 
        
        # hacky orientation offset (ideally you should have quaternion offsets, this sort of offset only works if you just want to rotate around 1 axis at a time) 
        roll += self.off_roll 
        pitch += self.off_pitch 
        yaw += self.off_yaw 
        
        quat = quaternion_from_euler(roll, pitch, yaw) 

        p = deepcopy(msg) 
        p.pose.position.x = msg.pose.position.x + self.off_x 
        p.pose.position.y = msg.pose.position.y + self.off_y 
        p.pose.position.z = msg.pose.position.z + self.off_z 
        
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