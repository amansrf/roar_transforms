from cmath import pi
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import tf_transformations
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from math import pi


class TransformPublisher(Node):

    def __init__(self):
        super().__init__('roar_tf2_publisher')

        # odom_correction static broadcaster to fix coordinate issues

        self.odom_correction_publisher = StaticTransformBroadcaster(self)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Initialize the subscriber to the odometry message
        self.subscription = self.create_subscription(
            Odometry,
            "iPhone_odom",
            self.handle_odom,
            1)

        # Publish static correction transform once at startup
        self.make_transforms()

        # To avoid unused variable warning
        self.subscription

    def make_transforms(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'odom'
        static_transformStamped.child_frame_id = 'odom_iphone'
        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(
            float(pi), float(0), -float(pi/2))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.odom_correction_publisher.sendTransform(static_transformStamped)

    def handle_odom(self, msg):
        # Debug Only
        self.get_logger().info('I heard: "%s"' % msg)

        # Init the tf msf
        tf_msg = TransformStamped()

        # Time stamp for the transform should be the same as the
        # timestamp for the odom message
        tf_msg.header.stamp = msg.header.stamp

        # Assign values from the odom message to the tf message
        tf_msg.header.frame_id = msg.header.frame_id
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