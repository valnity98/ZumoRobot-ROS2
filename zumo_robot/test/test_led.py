#!/usr/bin/env python3
'''
ROS2 Serial Communication Node with Arduino
Author: Mutasem Bader
Description:
    - This ROS2 node opens a serial connection to an Arduino
    - Sends commands periodically over serial to control the Arduino (e.g., LED control)
Requirements:
    - ROS2 installation
    - pyserial library installed (pip install pyserial)
'''

import rclpy
from rclpy.node import Node
import serial
from serial.serialutil import SerialException

class test_led(Node):
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
    node = test_led()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
