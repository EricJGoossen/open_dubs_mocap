#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node
import yaml 
import numpy as np 
from geometry_msgs.msg import PoseStamped 
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
from functools import partial 
from ament_index_python.packages import get_package_share_directory
import os 

class PoseOffsetNode(Node): 
    def __init__(self): 
        super().__init__('pose_offset_node') 
        
        # Load parameters 
        self.declare_parameter('tracked_objects', 
                              ['car', 'ramp1', 'ramp2', 
                               'block1', 'block2', 'block3', 'block4'])
        tracked_objects = self.get_parameter('tracked_objects').get_parameter_value().string_array_value 
        
        pkg_share = get_package_share_directory('open_dubs_mocap')
        config_path = os.path.join(pkg_share, 'config', 'mocap_tf_offset.yaml')
        with open(config_path) as f: 
            config = yaml.safe_load(f) 
            DEG2RAD = np.pi / 180.0 
            
            self.offset_x = config["x"] 
            self.offset_y = config["y"] 
            self.offset_z = config["z"] 

            self.offset_roll = config["roll"] * DEG2RAD 
            self.offset_pitch = config["pitch"] * DEG2RAD 
            self.offset_yaw = config["yaw"] * DEG2RAD 
            
            self.active_publishers = {} 
            self.active_subscribers = {} 
            
            # Create subscribers and publishers for each object 
            for obj in tracked_objects: 
                # Determine input topic based on whether it's the car 
                input_topic = f"/vrpn_client_node/{obj}/pose" 
                output_topic = f"/mocap/{obj}/pose" 
                
                # Create publisher 
                self.active_publishers[obj] = self.create_publisher( 
                    PoseStamped, 
                    output_topic, 
                    qos_profile=10
                ) 
                
                # Create subscriber with callback 
                self.active_subscribers[obj] = self.create_subscription(
                    PoseStamped, 
                    input_topic, 
                    partial(self.pose_callback, obj), 
                    qos_profile=10
                ) 
            
            self.get_logger().info( 
                f"Pose offset node started\n" 
                f"Position offsets: [{self.offset_x}, {self.offset_y}, {self.offset_z}]\n" 
                f"Orientation offsets: [{self.offset_roll}, {self.offset_pitch}, {self.offset_yaw}]" 
            ) 
            
    def pose_callback(self, obj_name, msg): 
        """Process incoming pose message and apply offsets""" 
        try: 
            # Create new pose message 
            new_pose = PoseStamped() 
            new_pose.header = msg.header # Maintain original header 
            
            # Apply position offsets 
            new_pose.pose.position.x = msg.pose.position.x + self.offset_x 
            new_pose.pose.position.y = msg.pose.position.y + self.offset_y 
            new_pose.pose.position.z = msg.pose.position.z + self.offset_z 
            
            # Apply orientation offsets 
            q = [ 
                msg.pose.orientation.x, 
                msg.pose.orientation.y, 
                msg.pose.orientation.z, 
                msg.pose.orientation.w 
            ] 
            
            # Convert to Euler angles (roll, pitch, yaw) 
            roll, pitch, yaw = euler_from_quaternion(q) 
            
            # Apply orientation offsets 
            roll += self.offset_roll 
            pitch += self.offset_pitch 
            yaw += self.offset_yaw 
            
            # Convert back to quaternion 
            q_new = quaternion_from_euler(roll, pitch, yaw) 
            
            new_pose.pose.orientation.x = q_new[0] 
            new_pose.pose.orientation.y = q_new[1] 
            new_pose.pose.orientation.z = q_new[2] 
            new_pose.pose.orientation.w = q_new[3] 
            
            # Publish transformed pose 
            self.active_publishers[obj_name].publish(new_pose) 
            
        except Exception as e: 
            self.get_logger().error(f"Error processing pose for {obj_name}: {str(e)}") 
            
def main(args=None): 
    rclpy.init() 
    node = PoseOffsetNode() 
    
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
