#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class FakeMocapPublisher(Node):
    def __init__(self):
        super().__init__('fake_mocap')

        # Publishers for car and some blocks/ramps
        self.pub_car = self.create_publisher(PoseStamped, '/vrpn_client_node/open_dubs/pose', 10)
        self.pub_ramp1 = self.create_publisher(PoseStamped, '/vrpn_client_node/ramp1/pose', 10)
        self.pub_ramp2 = self.create_publisher(PoseStamped, '/vrpn_client_node/ramp2/pose', 10)
        self.pub_block1 = self.create_publisher(PoseStamped, '/vrpn_client_node/block1/pose', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.t = 0.0

    def timer_callback(self):
        self.t += 0.02

        # Car moves in a circle
        car_msg = PoseStamped()
        car_msg.header.stamp = self.get_clock().now().to_msg()
        car_msg.header.frame_id = 'map'
        car_msg.pose.position.x = np.cos(self.t)
        car_msg.pose.position.y = np.sin(self.t)
        car_msg.pose.position.z = 0.0
        car_msg.pose.orientation.w = 1.0
        self.pub_car.publish(car_msg)

        # Static ramps and blocks 
        for pub, pos in [(self.pub_ramp1, (1.0, 0.0, 0.0)),
                         (self.pub_ramp2, (-1.0, 0.0, 0.0)),
                         (self.pub_block1, (0.5, 0.5, 0.0))]:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            msg.pose.orientation.w = 1.0
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeMocapPublisher()
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
