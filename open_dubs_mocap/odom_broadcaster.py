"""Broadcast `odom` to `base_link` TF from pose-like odometry input."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class odomBraodcaster(Node):
    """Create TF transforms from incoming odometry pose messages."""

    def __init__(self):
        """Initialize TF broadcaster and odometry subscription."""
        super().__init__('odom_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # car_odom topic is remapped to actual topic name in launch file
        self.subscriber = self.create_subscription(
            PoseStamped,
            'car_odom',
            self.odom_callback,
            qos_profile=qos_profile
        )

    def odom_callback(self, msg):
        """Publish a TF transform from an odometry pose message.

        Args:
            msg (PoseStamped): Pose message used to populate transform fields.
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    """Run the odom TF broadcaster node.

    Args:
        args (list[str] | None): Optional ROS CLI arguments.
    """
    rclpy.init(args=args)
    node = odomBraodcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()