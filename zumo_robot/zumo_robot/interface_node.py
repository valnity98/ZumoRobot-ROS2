#!/usr/bin/env python3

"""Qt window of the robot: camera image, encoder values, motor speeds, pose and log.

Author: Mutasem Bader

The ROS 2 node is spun in a background thread, so the callbacks run outside the
GUI thread. They must not touch widgets; they emit Qt signals instead, and the
connected slots update the widgets in the GUI thread.

The window sends the start, stop and reset commands on `robot_command`.
"""

from collections import defaultdict
import math
import os
import sys
from time import monotonic

from ament_index_python.packages import (
    get_package_share_directory, PackageNotFoundError)
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from PyQt5 import QtCore, uic
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import (
    Float32MultiArray, Int16MultiArray, Int32MultiArray, Int8, String)

from zumo_robot.log_node import LogPublisher

CMD_STOP = 0
CMD_START = 1
CMD_RESET = 2

# Minimum time between two updates of the same text field, in seconds
TEXT_UPDATE_INTERVAL = 0.5


def find_ui_file():
    """
    Return the path of zumorobot.ui.

    Prefers the installed copy (share/zumo_robot/gui, installed via setup.py)
    and falls back to the source tree if the package is not found in the
    ament index or the file is missing there (e.g. a plain `python` run).
    The window icon (gui/ICON/...) is referenced relative to the .ui file by
    uic, so it is found next to whichever copy is returned.
    """
    try:
        share_ui = os.path.join(
            get_package_share_directory('zumo_robot'), 'gui', 'zumorobot.ui')
        if os.path.isfile(share_ui):
            return share_ui
    except PackageNotFoundError:
        pass
    return os.path.join(os.path.dirname(__file__), '..', 'gui', 'zumorobot.ui')


class ROS2Thread(QtCore.QThread):
    """Spins the ROS 2 node in a background thread."""

    def __init__(self, node, log_publisher):
        super().__init__()
        self.node = node
        self.log_publisher = log_publisher

    def run(self):
        try:
            rclpy.spin(self.node)
        except ExternalShutdownException:
            pass  # normal end: the window was closed
        except Exception as e:
            self.log_publisher.log(
                f"Unexpected error occurred in ROS2 thread: {e}", level="error")


class MainWindow(QMainWindow):
    """Main window: shows the robot data and sends the robot commands."""

    # Emitted by the ROS callbacks (ROS thread), handled in the GUI thread.
    encoder_received = QtCore.pyqtSignal(int, int)
    motor_received = QtCore.pyqtSignal(int, int)
    pose_received = QtCore.pyqtSignal(float, float, float)
    line_received = QtCore.pyqtSignal(float, float)
    frame_received = QtCore.pyqtSignal(QImage)
    log_received = QtCore.pyqtSignal(str)

    def __init__(self, node, log_publisher):
        super().__init__()
        self.node = node
        self.log_publisher = log_publisher

        # Load the UI file (installed to share/zumo_robot/gui/zumorobot.ui,
        # source copy: zumo_robot/gui/zumorobot.ui)
        ui_path = find_ui_file()
        try:
            uic.loadUi(ui_path, self)
        except Exception as e:
            self.log_publisher.log(f"Error loading UI file: {e}", level="error")
            raise

        self.bridge = CvBridge()

        # Time of the last update per text field, used by the ROS callbacks
        self.last_update_times = defaultdict(float)

        # The emitter lives in the ROS thread and the window in the GUI
        # thread, so Qt queues these connections automatically.
        self.encoder_received.connect(self.update_encoder_data)
        self.motor_received.connect(self.update_motor_data)
        self.pose_received.connect(self.update_pose_data)
        self.line_received.connect(self.update_line_coordinates)
        self.frame_received.connect(self.update_video)
        self.log_received.connect(self.update_log)

        self.Start_PushButton.clicked.connect(self.send_start_command)
        self.Stop_PushButton.clicked.connect(self.send_stop_command)
        self.Reset_PushButton.clicked.connect(self.send_reset_command)

        self.publisher = self.node.create_publisher(Int8, 'robot_command', 10)

        self.node.create_subscription(
            Float32MultiArray, 'line_coordinates', self.on_line_coordinates, 10)
        self.node.create_subscription(
            Image, 'camera_topic', self.on_camera_image, 10)
        self.node.create_subscription(
            Int32MultiArray, 'encoder_data', self.on_encoder_data, 10)
        self.node.create_subscription(
            Int16MultiArray, 'motor_speeds', self.on_motor_speeds, 10)
        self.node.create_subscription(
            Pose, 'robot_position', self.on_robot_position, 10)
        self.node.create_subscription(
            String, 'log_messages', self.on_log_message, 10)

        self.ros_thread = ROS2Thread(self.node, self.log_publisher)
        # Close the window when ROS shuts down from outside (e.g. Ctrl+C in
        # the terminal that runs the launch file).
        self.ros_thread.finished.connect(self.close)
        self.ros_thread.start()

    def rate_limit(self, key, interval=TEXT_UPDATE_INTERVAL):
        """Return True if `interval` seconds have passed since the last True for `key`."""
        current_time = monotonic()
        if current_time - self.last_update_times[key] > interval:
            self.last_update_times[key] = current_time
            return True
        return False

    # ROS callbacks. They run in the ROS thread and must not touch widgets.

    def on_encoder_data(self, msg):
        if len(msg.data) == 2 and self.rate_limit("encoder_data"):
            self.encoder_received.emit(msg.data[0], msg.data[1])

    def on_motor_speeds(self, msg):
        if len(msg.data) == 2 and self.rate_limit("motor_data"):
            self.motor_received.emit(msg.data[0], msg.data[1])

    def on_robot_position(self, msg):
        if self.rate_limit("pose_data"):
            # Yaw from a quaternion that only rotates about z
            theta = 2 * math.atan2(msg.orientation.z, msg.orientation.w)
            self.pose_received.emit(
                msg.position.x, msg.position.y, math.degrees(theta))

    def on_line_coordinates(self, msg):
        if len(msg.data) == 2 and self.rate_limit("line_coordinates"):
            self.line_received.emit(msg.data[0], msg.data[1])

    def on_camera_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width, _channels = frame.shape
            bytes_per_line = 3 * width  # 3 bytes per pixel (RGB)
            # A QImage built on a buffer does not own the pixel data.
            # copy() gives the image its own data, so it stays valid after
            # `frame` is gone and can be passed to the GUI thread.
            q_img = QImage(frame.data, width, height, bytes_per_line,
                           QImage.Format_RGB888).copy()
            self.frame_received.emit(q_img)
        except CvBridgeError as e:
            self.log_publisher.log(
                f"Error converting ROS image message to OpenCV: {e}",
                level="error")
        except Exception as e:
            self.log_publisher.log(
                f"Unexpected error displaying video: {e}", level="error")

    def on_log_message(self, msg):
        self.log_received.emit(msg.data)

    # Slots. They run in the GUI thread.

    @QtCore.pyqtSlot(int, int)
    def update_encoder_data(self, left_encoder, right_encoder):
        self.Encoder_Left.setText(f"L: {left_encoder}")
        self.Encoder_Right.setText(f"R: {right_encoder}")

    @QtCore.pyqtSlot(int, int)
    def update_motor_data(self, speed_left, speed_right):
        self.Speed_Left.setText(f"L: {speed_left}")
        self.Speed_Right.setText(f"R: {speed_right}")

    @QtCore.pyqtSlot(float, float, float)
    def update_pose_data(self, x, y, theta_degrees):
        self.x_position.setText(f"x: {x:.2f}")
        self.y_position.setText(f"y: {y:.2f}")
        self.Theta.setText(f"Theta: {theta_degrees:.2f}°")

    @QtCore.pyqtSlot(float, float)
    def update_line_coordinates(self, cx, cy):
        self.X_Camera.setText(f"cx: {cx:.2f}")
        self.Y_Camera.setText(f"cy: {cy:.2f}")

    @QtCore.pyqtSlot(QImage)
    def update_video(self, q_img):
        self.Video_Label.setPixmap(QPixmap.fromImage(q_img))

    @QtCore.pyqtSlot(str)
    def update_log(self, message):
        """Append a message to the status log and scroll to the end."""
        self.plainTextEdit.appendPlainText(message)
        scroll_bar = self.plainTextEdit.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def send_start_command(self):
        self._send_robot_command(CMD_START)
        self.log_publisher.log("Robot started", level="info")

    def send_stop_command(self):
        self._send_robot_command(CMD_STOP)
        self.log_publisher.log("Robot stopped", level="info")

    def send_reset_command(self):
        """Clear the displayed values and send the reset command."""
        self.Encoder_Left.setText('L: 0')
        self.Encoder_Right.setText('R: 0')
        self.Speed_Left.setText('L: 0')
        self.Speed_Right.setText('R: 0')
        self.x_position.setText('x: 0.00')
        self.y_position.setText('y: 0.00')
        self.Theta.setText('Theta: 0.00°')
        self.Y_Camera.setText('cy: 0.00')
        self.X_Camera.setText('cx: 0.00')
        self.plainTextEdit.setPlainText('')

        self._send_robot_command(CMD_RESET)
        self.log_publisher.log('Data reset', level='info')

    def _send_robot_command(self, command: int):
        """Publish a command (CMD_STOP, CMD_START or CMD_RESET) on robot_command."""
        try:
            self.publisher.publish(Int8(data=command))
        except Exception as e:
            self.log_publisher.log(
                f"Failed to send robot command: {e}", level="error")

    def closeEvent(self, event):
        """Stop the ROS thread before the window and the node go away."""
        # Shutting the context down ends rclpy.spin() in the ROS thread.
        rclpy.try_shutdown()
        self.ros_thread.wait(2000)  # milliseconds
        self.node.destroy_node()
        event.accept()


def main():
    rclpy.init()
    try:
        node = Node("interface_node")
        log_publisher = LogPublisher('interface_log')
        app = QApplication(sys.argv)
        window = MainWindow(node, log_publisher)
        window.show()
        exit_code = app.exec_()
    finally:
        # closeEvent() has already shut the context down when the window was
        # closed normally; try_shutdown() does nothing in that case.
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
