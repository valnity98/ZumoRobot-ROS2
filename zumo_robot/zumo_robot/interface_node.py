#!/usr/bin/env python3
'''
Qt App Node for ROS 2
Author: Mutasem Bader
Description:
    - This Qt application communicates with a robot using ROS 2.
    - The app allows sending start/stop commands to control the robot and displays the camera feed from the robot.
    - The ROS 2 node handles video stream reception, robot control, and communication via serial.
    - OpenCV and cv_bridge are used to display the video feed in the GUI.
    - ROS2 communication runs in a separate thread to avoid blocking the GUI event loop.
Requirements:
    - ROS 2 installation
    - serial libraries (`pyserial`)
    - OpenCV (`cv2`)
    - PyQt5
    - cv_bridge (for ROS2 <-> OpenCV image conversion)
'''

import math
import os
import sys
from collections import defaultdict
from time import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int16MultiArray, String, Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError
import cv2

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import uic, QtCore
from PyQt5.QtCore import QVariant


from zumo_robot.log_node import LogPublisher



class ROS2Thread(QtCore.QThread):
    """
    ROS2 Thread class to run the ROS2 node in a background thread
    to prevent blocking the GUI.
    """
    def __init__(self, node, log_publisher):
        super().__init__()
        self.node = node
        self.log_publisher = log_publisher

    def run(self):
        """Starts the ROS2 node to process messages in the background."""
        try:
            rclpy.spin(self.node)
        except rclpy.executors.ExternalShutdownException:
            self.log_publisher.log("ROS2 Node stopped successfully.")
        except Exception as e:
            self.log_publisher.log(f"Unexpected error occurred in ROS2 thread: {e}", level="error")

class MainWindow(QMainWindow):
    """
    Main class for the Qt application interface.
    Handles communication with ROS2, robot control, and video display.
    """
    def __init__(self,node, log_publisher):
        super(MainWindow, self).__init__()
        self.node = node
        self.log_publisher = log_publisher
    
        # Load the UI file (located at zumo_robot/gui/zumorobot.ui)
        ui_path = os.path.join(os.path.dirname(__file__), '..', 'gui', 'zumorobot.ui')
        try:
            uic.loadUi(ui_path, self)
        except Exception as e:
            self.log_publisher.log(f"Error loading UI file: {e}", level="error")
            raise e
       

        self.publisher = self.node.create_publisher(Int8, 'robot_command', 10)

        self.line_coordinates = self.node.create_subscription(Float32MultiArray,'line_coordinates',self.show_line_coordinates,10)
        self.video_subscriber = self.node.create_subscription(Image, 'camera_topic', self.show_display_video, 10)
        self.encoder_subscriber = self.node.create_subscription(Int32MultiArray, 'encoder_data', self.show_encoder_data, 10)
        self.motor_subscriber = self.node.create_subscription(Int16MultiArray,'motor_speeds',self.show_motor_data,10)
        self.robot_position_subscriber = self.node.create_subscription(Pose, 'robot_position', self.show_pose_data, 10)
        self.log_subscriber = self.node.create_subscription(String,'log_messages', self.show_log_message, 10)

        self.bridge = CvBridge()  # Initialize CvBridge for converting ROS2 image messages to OpenCV images

        # Connect GUI buttons to their corresponding functions
        self.Start_PushButton.clicked.connect(self.send_start_command)
        self.Stop_PushButton.clicked.connect(self.send_stop_command)
        self.Reset_PushButton.clicked.connect(self.send_reset_command)

        # Initialize rate limiting for GUI updates
        self.last_update_times = defaultdict(lambda: 0)

        # Start the ROS2 communication in a background thread to prevent blocking the GUI
        self.ros_thread = ROS2Thread(self.node, self.log_publisher)
        self.ros_thread.start()

    def rate_limit(self, key, interval=0.5):
        """Checks if the specified interval has passed for the given key."""
        current_time = time()
        if current_time - self.last_update_times[key] > interval:
            self.last_update_times[key] = current_time
            return True
        return False
    
    def show_encoder_data(self, msg):
        """Updates the GUI with the latest encoder values."""
        if self.rate_limit("encoder_data", interval=0.5):
            try:
                left_encoder, right_encoder = msg.data  # Extract encoder values
                self.Encoder_Left.setText(f"L: {left_encoder}")
                self.Encoder_Right.setText(f"R: {right_encoder}")
            except Exception as e:
                self.log_publisher.log(f"Error updating encoder data: {e}", level="error")
                
    def show_motor_data(self, msg):
        """Updates the GUI with the latest motor values."""
        if self.rate_limit("motor_data", interval=0.5):
            try:
                speed_left, speed_right = msg.data  # Extract motor value
                self.Speed_Left.setText(f"L: {speed_left}") 
                self.Speed_Right.setText(f"R: {speed_right}")  
            except Exception as e:
                self.log_publisher.log(f"Error updating speed data: {e}", level="error")

    def show_pose_data(self, msg): 
        """Updates the GUI with the latest robot position."""
        if self.rate_limit("pose_data", interval=0.5):
            try:
                x = msg.position.x
                y = msg.position.y
                z = msg.orientation.z
                w = msg.orientation.w
                theta = 2 * math.atan2(z, w)
                theta_degrees = math.degrees(theta)
                self.x_position.setText(f"x: {x:.2f}")
                self.y_position.setText(f"y: {y:.2f}")
                self.Theta.setText(f"Theta: {theta_degrees:.2f}°")
            except Exception as e:
                self.log_publisher.log(f"Error updating pose data: {e}", level="error")

    def show_line_coordinates(self, msg):
        """Updates the GUI with the latest line coordinates."""
        if self.rate_limit("line_coordinates", interval=0.5):
            try: 
                cx, cy = msg.data
                self.X_Camera.setText(f"cx: {cx:.2f}")
                self.Y_Camera.setText(f"cy: {cy:.2f}")
            except Exception as e:
                self.log_publisher.log(f"Error updating camera coordinates: {e}", level="error")

    def show_display_video(self, msg):
            """Handles video stream data from the robot and updates the GUI display."""
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert the ROS image to OpenCV format
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB format for displaying in the GUI

                # Get image dimensions for creating QImage object
                height, width, channel = frame.shape
                bytes_per_line = 3 * width  # 3 bytes per pixel (RGB)
                # Use bytes() to copy frame data — QImage with raw buffer does
                # not own the data, so frame must stay alive until fromImage() copies it.
                q_img = QImage(bytes(frame.tobytes()), width, height,
                               bytes_per_line, QImage.Format_RGB888)
                self.Video_Label.setPixmap(QPixmap.fromImage(q_img))
            except CvBridgeError as e:
                self.log_publisher.log(f"Error converting ROS image message to OpenCV: {e}", level="error")
            except Exception as e:
                self.log_publisher.log(f"Unexpected error displaying video: {e}", level="error")

    def send_start_command(self):
        """Sends the start command to the robot system."""
        self._send_robot_command(1)
        self.log_publisher.log("Robot started", level="info")

    def send_stop_command(self):
        """Sends the stop command to the robot system."""
        self._send_robot_command(0)
        self.log_publisher.log("Robot stopped", level="info")

    def send_reset_command (self):
        """Resets all robot data and GUI values to their default state."""
   
        # Beispiel für Reset-Daten
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

        self._send_robot_command(2)
        self.log_publisher.log('Data reset', level='info')

    def _send_robot_command(self, command: Int8):
        """Helper function to send commands (start/stop/reset) to the robot via ROS2."""
        try:
            msg = Int8(data=command)
            self.publisher.publish(msg)
        except rclpy.exceptions.RCLException as e:
            self.log_publisher.log(f"Failed to send robot command due to ROS2 exception: {e}", level="error")
        except Exception as e:
            self.log_publisher.log(f"Unexpected error in sending robot command: {e}", level="error")

    def show_log_message(self, msg: String) -> None:
        """Appends a new message to the status log."""
        QtCore.QMetaObject.invokeMethod(
            self, 'update_log', QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(QVariant, QVariant(msg.data)))

    @QtCore.pyqtSlot(QVariant)
    def update_log(self, message):
        current_text = self.plainTextEdit.toPlainText()
        new_text = f"{current_text}\n{message}" if current_text else message
        self.plainTextEdit.setPlainText(str(new_text))
        self.plainTextEdit.verticalScrollBar().setValue(self.plainTextEdit.verticalScrollBar().maximum())



    def closeEvent(self, event):
        """Handles cleanup and ROS2 node shutdown when the window is closed."""
        try:
            self.node.destroy_node()  # Ensure the node is destroyed properly
            rclpy.shutdown()  # Shutdown ROS2 gracefully
            event.accept()
        except Exception as e:
            self.log_publisher.log(f"Error during shutdown: {e}", level="error")
            event.accept()

def main():
    """Main function to run the Qt application."""
    try:
        rclpy.init()  # Initialize ROS2 before creating the app and GUI

        node = Node("interface_node")  # ROS2 Node erstellen
        log_publisher = LogPublisher()  # Single instance of LogPublisher
        app = QApplication(sys.argv)
        window = MainWindow(node, log_publisher)  # Node an die MainWindow-Klasse übergeben
        window.show()

        sys.exit(app.exec_())  # Start the Qt event loop
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    finally:
        # Guard against double shutdown: closeEvent() already calls rclpy.shutdown()
        # when the window is closed normally; calling it again raises RuntimeError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
