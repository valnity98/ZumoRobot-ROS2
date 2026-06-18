#!/usr/bin/env python3

"""
TF2 Transform Broadcaster Node for ROS 2
Author: Mutasem Bader

Subscribes to robot_position (Pose) and broadcasts the map → base_link
transform so all RViz visualisations share a common reference frame.

Subscribes:
    robot_position  (Pose)  current pose from path_mapping_node
"""

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
import tf2_ros


class TF2BroadcasterNode(Node):
    """Broadcasts the map → base_link transform from odometry pose."""

    def __init__(self) -> None:
        super().__init__('tf2_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Pose, 'robot_position', self._pose_callback, 10)

    def _pose_callback(self, msg: Pose) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w
        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TF2BroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
