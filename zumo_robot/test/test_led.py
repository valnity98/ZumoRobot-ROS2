#!/usr/bin/env python3

"""
Manual serial check for the Arduino (not an automated test).

Author: Mutasem Bader

Opens the serial port and sends the character '1' at 100 Hz, e.g. to switch
an LED with a matching test sketch on the Arduino. Start it by hand:

    python3 test/test_led.py

pytest finds no test in this file; it is only kept as a wiring check.

Requirements:
    - ROS 2 installation
    - pyserial
"""

import rclpy
from rclpy.node import Node
import serial
from serial.serialutil import SerialException


class LedTestNode(Node):
    """Sends '1' to the Arduino periodically."""

    def __init__(self):
        super().__init__('test_led')

        # Try to establish the serial connection
        self.serial_port = None
        self.init_serial_port()

        # Create a timer that triggers every 0.01 seconds (100 Hz)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.send_to_arduino)

    def init_serial_port(self):
        """Initialize the serial port connection with error handling."""
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Serial connection established to Arduino.')
        except SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            self.serial_port = None

    def send_to_arduino(self):
        """Send a command to the Arduino periodically."""
        if self.serial_port is not None and self.serial_port.is_open:
            msg = '1'
            try:
                self.serial_port.write(msg.encode())
                self.get_logger().info(f'Sent command: {msg} to Arduino')
            except SerialException as e:
                self.get_logger().error(f"Error sending data: {str(e)}")
        else:
            self.get_logger().warn("Serial port not open, skipping command.")

    def close_serial(self):
        """Ensure that the serial port is properly closed when shutting down."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = LedTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
