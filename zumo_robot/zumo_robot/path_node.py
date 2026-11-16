#!/usr/bin/env python3

"""
Path Mapping Node for ROS 2
Author: Mutasem Bader

Implements dead-reckoning odometry from wheel encoder ticks to track the
robot's 2-D pose on an OccupancyGrid and visualise it in RViz.

Subscribes:
    encoder_data  (Int32MultiArray)  [left_ticks, right_ticks]
    robot_command (Int8)             2 = reset pose, map and path

Publishes:
    robot_position      (Pose)           current pose in map frame
    path_map            (OccupancyGrid)  100 = visited, -1 = unknown
    visualization_marker (Marker)        LINE_STRIP path overlay
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
from visualization_msgs.msg import Marker

from zumo_robot import odometry
from zumo_robot.log_node import LogPublisher

# The robot constants (WHEEL_RADIUS, CPR, WHEELTRACK) are in odometry.py.

MAP_SIZE = 1000          # cells
RESOLUTION = 0.01        # metres per cell

CMD_RESET = 2            # robot_command value "reset" (see motors_node)


class PathMappingNode(Node):
    """Dead-reckoning path mapper publishing OccupancyGrid and RViz markers."""

    def __init__(self, log_publisher):
        super().__init__('path_mapping_node')
        self.log_publisher = log_publisher

        self.create_subscription(
            Int32MultiArray, 'encoder_data', self.encoder_callback, 10)
        self.create_subscription(
            Int8, 'robot_command', self.command_callback, 10)

        self.position_pub = self.create_publisher(Pose, 'robot_position', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'path_map', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'visualization_marker', 10)

        # OccupancyGrid requires int8; -1 = unknown, 0 = free, 100 = occupied
        self.map_data = np.full(
            (MAP_SIZE, MAP_SIZE), -1, dtype=np.int8)

        # Robot pose: [x_cell, y_cell, theta_rad] — starts at map centre
        self.current_position = [MAP_SIZE / 2.0, MAP_SIZE / 2.0, 0.0]

        self.prev_left = 0
        self.prev_right = 0
        self._rebase = False  # see command_callback

        # Ordered list of visited cells — avoids O(N²) argwhere on every tick
        self._path_points: list[tuple[int, int]] = []

    def command_callback(self, msg):
        """Reset pose, map and path when the GUI sends the reset command."""
        if msg.data != CMD_RESET:
            return

        # motors_node makes the Arduino zero its tick counters, but that goes
        # over serial, so the next encoder packet may still carry the old
        # counts (or the Arduino may not have zeroed at all). Assuming
        # prev_left/prev_right = 0 would turn such a packet into a jump by the
        # whole old tick count. Instead the next packet only becomes the new
        # baseline (_rebase); no distance is integrated from it.
        # Limitation: if that packet is a stale one and the Arduino zeroes
        # afterwards, the packet after it shows one step back by the old count.
        # Stop -> Reset -> Start avoids this: the Arduino only replies to motor
        # packets, so no stale reply is in flight when the robot is stopped.
        self.prev_left = 0
        self.prev_right = 0
        self._rebase = True

        # Same start state as in __init__
        self.current_position = [MAP_SIZE / 2.0, MAP_SIZE / 2.0, 0.0]
        self.map_data.fill(-1)
        self._path_points.clear()

        self._publish_position()
        self._publish_map()
        self._visualize_path()

    def encoder_callback(self, msg):
        if len(msg.data) != 2:
            self.log_publisher.log(
                f"Unexpected encoder data length: {len(msg.data)}", level="warn")
            return

        left_enc, right_enc = msg.data

        if self._rebase:
            # First packet after a reset: new zero point, no distance from it.
            self.prev_left = left_enc
            self.prev_right = right_enc
            self._rebase = False

        ticks_left = left_enc - self.prev_left
        ticks_right = right_enc - self.prev_right
        self.prev_left = left_enc
        self.prev_right = right_enc

        # The pose is kept in map cells, the odometry works in metres.
        x, y, theta = odometry.integrate(
            self.current_position[0] * RESOLUTION,
            self.current_position[1] * RESOLUTION,
            self.current_position[2],
            ticks_left, ticks_right)

        # Clamp to map boundaries
        self.current_position[0] = max(
            0.0, min(MAP_SIZE - 1, x / RESOLUTION))
        self.current_position[1] = max(
            0.0, min(MAP_SIZE - 1, y / RESOLUTION))
        self.current_position[2] = theta

        self.get_logger().debug(
            f"Pose: x={self.current_position[0]:.1f} "
            f"y={self.current_position[1]:.1f} "
            f"θ={math.degrees(self.current_position[2]):.1f}°")

        self._update_map(self.current_position[0], self.current_position[1])
        self._publish_position()
        self._publish_map()
        self._visualize_path()

    def _update_map(self, x: float, y: float) -> None:
        xi = max(0, min(MAP_SIZE - 1, int(x)))
        yi = max(0, min(MAP_SIZE - 1, int(y)))
        if self.map_data[yi, xi] != 100:
            self.map_data[yi, xi] = 100
            self._path_points.append((xi, yi))

    def _publish_position(self):
        pose = Pose()
        origin_offset = -MAP_SIZE * RESOLUTION / 2.0
        pose.position.x = origin_offset + self.current_position[0] * RESOLUTION
        pose.position.y = origin_offset + self.current_position[1] * RESOLUTION
        half_theta = self.current_position[2] / 2.0
        pose.orientation.z = math.sin(half_theta)
        pose.orientation.w = math.cos(half_theta)
        self.position_pub.publish(pose)

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_SIZE
        msg.info.height = MAP_SIZE
        msg.info.origin.position.x = -MAP_SIZE * RESOLUTION / 2.0
        msg.info.origin.position.y = -MAP_SIZE * RESOLUTION / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(msg)

    def _visualize_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        origin_offset = -MAP_SIZE * RESOLUTION / 2.0
        for xi, yi in self._path_points:
            pt = Point()
            pt.x = xi * RESOLUTION + origin_offset
            pt.y = yi * RESOLUTION + origin_offset
            pt.z = 0.0
            marker.points.append(pt)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = PathMappingNode(log_publisher)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    except ExternalShutdownException:
        pass
    finally:
        # Ctrl+C has already shut the context down; a plain rclpy.shutdown()
        # would then raise.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
