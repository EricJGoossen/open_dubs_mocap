#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_matrix

class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        self.last_publish = self.get_clock().now()
        self.last_pose = None
        self.last_time = None

        # Low-pass filter coefficient (0 < alpha <= 1)
        self.declare_parameter('alpha', 0.2)
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        self.filtered_velocity = np.array([0.0, 0.0, 0.0])  # Initial filtered velocity

        self.imu_ori = [0.0,0.0,0.0]
        self.odom_msg = Odometry()
        self.pose_msg = PoseStamped()
        self.update_odom = False
        self.max_accel = 20.0
        self.raw_vel = np.array([0.0,0.0,0.0])

        self.odom_pub = self.create_publisher(Odometry, "car_odom", qos_profile=1)
        self.pose_pub = self.create_publisher(PoseStamped, "car_pose", qos_profile=1)
        
        self.subscriber = self.create_subscription(
            PoseStamped, 
            "mocap_pose", 
            self.pose_callback, 
            qos_profile=1
        )

        # rospy.Subscriber("imu/data", Imu, self.imu_callback) ???

        pub_period = 1 / 100.0
        self.timer = self.create_timer(pub_period, self.main_loop_callback)

    def main_loop_callback(self):
        if self.update_odom:
            self.odom_pub.publish(self.odom_msg)
            self.pose_pub.publish(self.pose_msg)
            self.update_odom = False

    # ???
    # def imu_callback(self, msg):
    #     quaternion = (
    #         msg.orientation.x,
    #         msg.orientation.y,
    #         msg.orientation.z,
    #         msg.orientation.w
    #     )
    #     self.imu_ori = euler_from_quaternion(quaternion)
    
    def pose_callback(self, msg):
        # If we have a previous pose, calculate velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_publish).nanoseconds * 1e-9

        if self.last_pose is None or self.last_time is None or dt <= 0:
            self.last_pose = msg
            self.last_time = current_time
            return

        # Global frame velocity
        dx = msg.pose.position.x - self.last_pose.pose.position.x
        dy = msg.pose.position.y - self.last_pose.pose.position.y
        dz = msg.pose.position.z - self.last_pose.pose.position.z

        velocity_global = np.array([dx / dt, dy / dt, dz / dt])

        # Convert velocity to body frame
        orientation = msg.pose.orientation
        quat = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        #use imu for roll and pitch ???
        # rpy = euler_from_quaternion(quat)
        # ori = [0,0,0]
        # ori[0] = self.imu_ori[0]
        # ori[1] = self.imu_ori[1]
        # ori[2] = rpy[2]

        # quat = quaternion_from_euler(*ori, 'sxyz')
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        rotation_matrix = quaternion_matrix(quat)[:3, :3]
        velocity_body = np.dot(rotation_matrix.T, velocity_global)

        vfilter = np.abs(self.raw_vel-velocity_body) / dt < self.max_accel
        self.raw_vel = velocity_body
        
        velocity_body = velocity_body*vfilter + self.filtered_velocity * (vfilter == 0)

        # Apply low-pass filter
        self.filtered_velocity = (
            self.alpha * velocity_body + (1 - self.alpha) * self.filtered_velocity
        )
        # Publish the filtered velocity
        twist_msg = Twist()
        twist_msg.linear.x = self.filtered_velocity[0]
        twist_msg.linear.y = self.filtered_velocity[1]
        twist_msg.linear.z = self.filtered_velocity[2]

        timestamp = msg.header.stamp

        # Update the timestamp for each message
        self.odom_msg.header.stamp = timestamp
        self.odom_msg.header.frame_id = "map"  # or any relevant frame_id
        self.odom_msg.child_frame_id = "base_link"
        self.pose_msg.header.stamp = timestamp
        self.pose_msg.header.frame_id = "map"  # or any relevant frame_id
        self.pose_msg.pose = msg.pose

        twist_covar_msg = TwistWithCovariance()
        twist_covar_msg.twist = twist_msg
        pos_covar_msg = PoseWithCovariance()
        pos_covar_msg.pose = msg.pose

        self.odom_msg.pose = pos_covar_msg
        self.odom_msg.twist = twist_covar_msg
        self.update_odom = True

        # Update the last pose and time
        self.last_pose = msg
        self.last_time = current_time


def main(args=None):
    rclpy.init()
    node = OdomPublisher()

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