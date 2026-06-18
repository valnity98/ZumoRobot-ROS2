#!/usr/bin/env python3

"""
Camera Node for ROS 2
Authors: Mutasem Bader, Felix Biermann

Captures frames from a webcam, detects a dark line via thresholding/contour
analysis across five horizontal image sections, and publishes:
  - Float32MultiArray on 'line_coordinates'  (weighted centroid x, mean y)
  - Image              on 'camera_topic'      (annotated frame)

A 1-D Kalman filter smooths the lateral centroid estimate.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

from zumo_robot.log_node import LogPublisher

NUM_SECTIONS = 5
MIN_CONTOUR_AREA = 100
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TIMER_PERIOD = 0.1  # seconds


class CameraNode(Node):
    """ROS 2 node: captures frames, detects a line, publishes coordinates."""

    def __init__(self, log_publisher):
        super().__init__('camera_node')
        self.log_publisher = log_publisher

        self.publisher_coordinates = self.create_publisher(
            Float32MultiArray, 'line_coordinates', 10)
        self.publisher_image = self.create_publisher(Image, 'camera_topic', 10)

        self.bridge = CvBridge()
        self._init_kalman()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not self.cap.isOpened():
            self.cap.release()
            # Raise instead of calling rclpy.shutdown() inside __init__ —
            # shutting down the context here would cause spin() in main() to
            # fail on an already-dead context.
            raise RuntimeError(
                "Failed to open camera. Please check the connection.")

        log_publisher.log("Camera node is on.")
        self.create_timer(TIMER_PERIOD, self.process_frame)

    def _init_kalman(self):
        """1-D Kalman filter: state = [position, velocity], measurement = position."""
        self.kalman = cv2.KalmanFilter(2, 1)
        self.kalman.measurementMatrix = np.array([[1, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(2, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.array([[10]], np.float32)
        self.kalman.errorCovPost = np.eye(2, dtype=np.float32)

    def process_frame(self):
        """Capture one frame, detect the line, publish coordinates and image."""
        ret, frame = self.cap.read()
        if not ret:
            self.log_publisher.log("Failed to capture frame.", level="warn")
            return

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, edges = cv2.threshold(gray, 60, 200, cv2.THRESH_BINARY_INV)

            height, width = edges.shape
            section_height = height // NUM_SECTIONS

            centroids = []
            for i in range(NUM_SECTIONS):
                y_start = i * section_height
                y_end = (i + 1) * section_height if i < NUM_SECTIONS - 1 else height
                section = edges[y_start:y_end, :]

                contours, _ = cv2.findContours(
                    section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = [c for c in contours
                            if cv2.contourArea(c) > MIN_CONTOUR_AREA]
                if not contours:
                    continue

                largest = max(contours, key=cv2.contourArea)
                shifted = largest + np.array([[[0, y_start]]])
                cv2.drawContours(frame, [shifted], -1, (0, 255, 0), 3)

                M = cv2.moments(shifted)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroids.append((cx, cy))
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            if centroids:
                avg_cx = self._weighted_average_by_position(centroids)
                avg_cy = int(np.mean([c[1] for c in centroids]))

                self.kalman.predict()
                self.kalman.correct(np.array([[np.float32(avg_cx)]]))
                corrected_x = int(self.kalman.statePost[0][0])

                msg = Float32MultiArray(data=[float(avg_cx), float(avg_cy)])
                self.publisher_coordinates.publish(msg)
                self.get_logger().info(
                    f"Published coordinates: cx={avg_cx}, cy={avg_cy}")

                cv2.circle(frame, (avg_cx, avg_cy), 10, (255, 0, 0), -1)
                cv2.circle(frame, (corrected_x, avg_cy), 10, (0, 255, 0), -1)

            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_image.publish(ros_image)

        except Exception as e:
            self.log_publisher.log(
                f"Error during frame processing: {e}", level="error")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

    def _weighted_average_by_position(self, centroids, power=2):
        """Weighted mean of x-coords; earlier sections (lower index) weighted higher."""
        x_values = np.array([c[0] for c in centroids], dtype=float)
        positions = np.arange(1, len(x_values) + 1)
        weights = (1.0 / positions) ** power
        return int(np.sum(x_values * weights) / np.sum(weights))


def main(args=None):
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = None
    try:
        node = CameraNode(log_publisher)
        rclpy.spin(node)
    except RuntimeError as e:
        log_publisher.log(str(e), level="error")
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
