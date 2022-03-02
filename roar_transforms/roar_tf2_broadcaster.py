import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import tf_transformations


class TransformPublisher(Node):

    def __init__(self):
        super().__init__('roar_tf2_publisher')

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Initialize the subscriber to the odometry message
        self.subscription = self.create_subscription(
            Odometry,
            "iPhone_odom",
            self.handle_odom,
            1)
        self.subscription

    def handle_odom(self, msg):
        # Debug Only
        self.get_logger().info('I heard: "%s"' % msg)

        # Init the tf msf
        tf_msg = TransformStamped()

        # Time stamp for the transform should be the same as the
        # timestamp for the odom message
        tf_msg.header.stamp = msg.header.stamp

        # Assign values from the odom message to the tf message
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'iphone_link'

        # The odom -> iphone_link transform is just the pose in the odom frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation      = msg.pose.pose.orientation

        # Publishing the transform
        self.broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    roar_tf2_publisher = TransformPublisher()
    try:
        rclpy.spin(roar_tf2_publisher)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()