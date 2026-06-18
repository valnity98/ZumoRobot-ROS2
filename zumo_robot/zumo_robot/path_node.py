#!/usr/bin/env python3

"""
Path Mapping Node for ROS 2
Author: Mutasem Bader

Implements dead-reckoning odometry from wheel encoder ticks to track the
robot's 2-D pose on an OccupancyGrid and visualise it in RViz.

Subscribes:
    encoder_data  (Int32MultiArray)  [left_ticks, right_ticks]

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
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker

from zumo_robot.log_node import LogPublisher

# Robot physical constants
WHEEL_RADIUS = 0.019     # metres
GEAR_RATIO = 75.81
CPR = GEAR_RATIO * 12  # counts per wheel rev: 12 CPR encoder × gear ratio
                       # A-only CHANGE interrupt gives 2 edges per encoder cycle,
                       # but 12 CPR already counts edges not cycles — no /2 needed.
WHEELTRACK = 0.09        # metres, axle width

MAP_SIZE = 1000          # cells
RESOLUTION = 0.01        # metres per cell


class PathMappingNode(Node):
    """Dead-reckoning path mapper publishing OccupancyGrid and RViz markers."""

    def __init__(self, log_publisher):
        super().__init__('path_mapping_node')
        self.log_publisher = log_publisher

        self.create_subscription(
            Int32MultiArray, 'encoder_data', self.encoder_callback, 10)

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

        # Ordered list of visited cells — avoids O(N²) argwhere on every tick
        self._path_points: list[tuple[int, int]] = []

    def encoder_callback(self, msg):
        if len(msg.data) != 2:
            self.log_publisher.log(
                f"Unexpected encoder data length: {len(msg.data)}", level="warn")
            return

        left_enc, right_enc = msg.data

        ticks_left = left_enc - self.prev_left
        ticks_right = right_enc - self.prev_right
        self.prev_left = left_enc
        self.prev_right = right_enc

        # Convert ticks → metres
        delta_left = (ticks_left / CPR) * (2 * math.pi * WHEEL_RADIUS)
        delta_right = (ticks_right / CPR) * (2 * math.pi * WHEEL_RADIUS)

        delta_dist = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / WHEELTRACK

        # Update pose using midpoint heading to reduce systematic drift on curves.
        theta = self.current_position[2]
        theta_mid = theta + delta_theta / 2.0
        self.current_position[0] += delta_dist * math.cos(theta_mid) / RESOLUTION
        self.current_position[1] += delta_dist * math.sin(theta_mid) / RESOLUTION
        self.current_position[2] = (
            (theta + delta_theta + math.pi) % (2 * math.pi) - math.pi)

        # Clamp to map boundaries
        self.current_position[0] = max(
            0.0, min(MAP_SIZE - 1, self.current_position[0]))
        self.current_position[1] = max(
            0.0, min(MAP_SIZE - 1, self.current_position[1]))

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
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
