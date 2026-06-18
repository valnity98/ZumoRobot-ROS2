#!/usr/bin/env python3

"""
Encoder Node for ROS 2
Author: Mutasem Bader

Reads framed encoder packets from an Arduino over serial and publishes them.

Packet format (10 bytes):
    0x02 | right_enc(4B LE) | left_enc(4B LE) | 0x03

Publishes:
    encoder_data  (Int32MultiArray)  [left_ticks, right_ticks]

ROS Parameters:
    serial_port  (str, default '/dev/ttyUSB0')
    baudrate     (int, default 115200)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

from zumo_robot.log_node import LogPublisher

PACKET_SIZE = 10
STX = 0x02
ETX = 0x03
TIMER_PERIOD = 0.1  # seconds


class Encoder(Node):
    """Reads encoder data from Arduino via serial and publishes it."""

    def __init__(self, log_publisher):
        super().__init__('encoder_node')
        self.log_publisher = log_publisher

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value

        self.publisher_ = self.create_publisher(
            Int32MultiArray, 'encoder_data', 10)
        self.serial_port = self._setup_serial(port, baudrate)

        self.log_publisher.log("Encoder node is on.")
        self.create_timer(TIMER_PERIOD, self.read_encoder_data)

    def _setup_serial(self, port, baudrate):
        try:
            sp = serial.Serial(port, baudrate, timeout=1)
            self.log_publisher.log(
                f"Serial port {port} opened successfully.")
            return sp
        except serial.SerialException as e:
            self.log_publisher.log(
                f"Failed to open serial port: {e}", level="error")
            return None

    def read_encoder_data(self):
        if not self.serial_port:
            return
        try:
            if self.serial_port.in_waiting < PACKET_SIZE:
                return

            raw = self.serial_port.read(PACKET_SIZE)
            if len(raw) != PACKET_SIZE or raw[0] != STX or raw[9] != ETX:
                self.log_publisher.log(
                    "Received invalid or incomplete packet.", level="warn")
                # Flush buffer to re-sync framing; without this every subsequent
                # read stays misaligned and warnings loop forever.
                self.serial_port.reset_input_buffer()
                return

            right_enc = int.from_bytes(raw[1:5], byteorder='little', signed=True)
            left_enc = int.from_bytes(raw[5:9], byteorder='little', signed=True)

            msg = Int32MultiArray()
            msg.data = [left_enc, right_enc]
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Encoders — left: {left_enc}, right: {right_enc}")

        except serial.SerialException as e:
            self.log_publisher.log(f"Serial read error: {e}", level="error")
        except Exception as e:
            self.log_publisher.log(
                f"Unexpected error reading encoder data: {e}", level="error")

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_publisher.log("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    log_publisher = LogPublisher()
    node = None
    try:
        node = Encoder(log_publisher)
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_publisher.log("Node interrupted by user.")
    finally:
        if node is not None:
            node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
