import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class odomBraodcaster(Node):
    def __init__(self):
        super().__init__('odom_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscriber = self.create_subscription(
            PoseStamped,
            'car_odom',
            self.odom_callback,
            qos_profile=qos_profile
        )

    def odom_callback(self, msg):
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
    rclpy.init(args=args)
    node = odomBraodcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()