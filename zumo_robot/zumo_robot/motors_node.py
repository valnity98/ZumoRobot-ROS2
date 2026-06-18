#!/usr/bin/env python3

"""
Motors Node for ROS 2
Author: Mutasem Bader

Subscribes to 'line_coordinates' and drives the Zumo motors via a PID
controller.  Commands are forwarded to the Arduino over serial using a
simple framed binary protocol (STX / data / control-byte / ETX).

Subscribes:
    line_coordinates  (Float32MultiArray)  centroid (x, y) from camera node
    robot_command     (Int8)               0=stop, 1=start, 2=reset encoders

Publishes:
    motor_speeds      (Int16MultiArray)    [left_speed, right_speed]

ROS Parameters:
    serial_port      (str,   default '/dev/ttyUSB0')
    target_position  (int,   default 320)   image centre column
    kp               (float, default 0.35)
    ki               (float, default 0.10)
    kd               (float, default 0.10)
    max_speed        (float, default 125.0)
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int8
import serial

from Zumo_Library.PIDController import Zumo328PPID
from zumo_robot.log_node import LogPublisher

CTRL_NORMAL = b'\x11'
CTRL_RESET_ENC = b'\x12'
START_BYTE = b'\x02'
END_BYTE = b'\x03'

CMD_STOP = 0
CMD_START = 1
CMD_RESET = 2


class Motors(Node):
    """ROS 2 node: PID line-following motor controller."""

    def __init__(self, log_publisher):
        super().__init__('motors_node')
        self.log_publisher = log_publisher

        # ROS parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('target_position', 320)
        self.declare_parameter('kp', 0.35)
        self.declare_parameter('ki', 0.10)
        self.declare_parameter('kd', 0.10)
        self.declare_parameter('max_speed', 125.0)

        port = self.get_parameter('serial_port').value
        self.target_position = self.get_parameter('target_position').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_speed = self.get_parameter('max_speed').value

        self.create_subscription(
            Float32MultiArray, 'line_coordinates',
            self.line_coordinates_callback, 10)
        self.create_subscription(
            Int8, 'robot_command', self.command_callback, 10)

        self.publisher_motor_speeds = self.create_publisher(
            Int16MultiArray, 'motor_speeds', 10)

        self.serial_port = self._setup_serial(port, 115200)
        self.pid = Zumo328PPID(kp=kp, ki=ki, kd=kd,
                                max_speed=max_speed, active=True)
        self._running = False
        self.log_publisher.log("Motors node is on.")

    def _setup_serial(self, port, baudrate):
        try:
            sp = serial.Serial(port, baudrate, timeout=1)
            self.log_publisher.log(f"Serial port {port} opened successfully.")
            return sp
        except serial.SerialException as e:
            self.log_publisher.log(
                f"Failed to open serial port: {e}", level="error")
            return None

    def line_coordinates_callback(self, msg):
        if not self._running:
            self.get_logger().debug("PID inactive — ignoring coordinates.")
            return
        try:
            c_x, _c_y = msg.data
            self.pid.control_speed(c_x, target_position=self.target_position)

            left_speed = np.int16(self.pid.get_left_speed())
            right_speed = np.int16(self.pid.get_right_speed())

            speed_msg = Int16MultiArray()
            speed_msg.data = [int(left_speed), int(right_speed)]
            self.publisher_motor_speeds.publish(speed_msg)

            self._send_to_arduino(left_speed, right_speed, CTRL_NORMAL)
        except ValueError as e:
            self.log_publisher.log(
                f"Invalid line coordinates: {e}", level="error")
        except Exception as e:
            self.log_publisher.log(
                f"Unexpected error in line_coordinates_callback: {e}",
                level="error")

    def _send_to_arduino(self, left_speed, right_speed, control_byte):
        if not self.serial_port or not self.serial_port.is_open:
            self.log_publisher.log(
                "Serial port unavailable — skipping send.", level="error")
            return
        try:
            payload = struct.pack('<hh', left_speed, right_speed)
            self.serial_port.write(
                START_BYTE + payload + control_byte + END_BYTE)
            self.get_logger().debug(
                f"Sent: left={left_speed}, right={right_speed}")
        except serial.SerialException as e:
            self.log_publisher.log(f"Serial write error: {e}", level="error")
        except Exception as e:
            self.log_publisher.log(
                f"Unexpected error in _send_to_arduino: {e}", level="error")

    def command_callback(self, msg: Int8):
        cmd = msg.data
        if cmd == CMD_STOP:
            self._running = False
            self.pid.set_left_speed(0)
            self.pid.set_right_speed(0)
            self._send_to_arduino(0, 0, CTRL_NORMAL)
        elif cmd == CMD_START:
            self._running = True
            self.pid.reset()
        elif cmd == CMD_RESET:
            left = np.int16(self.pid.get_left_speed())
            right = np.int16(self.pid.get_right_speed())
            self._send_to_arduino(left, right, CTRL_RESET_ENC)

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_publisher.log("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = Motors(log_publisher)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send_to_arduino(0, 0, CTRL_NORMAL)
        log_publisher.log("Node interrupted by user.")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
