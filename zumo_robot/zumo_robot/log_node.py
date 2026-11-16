#!/usr/bin/env python3

"""
Log Publisher Node for ROS 2
Author: Mutasem Bader

Publishes one-shot log messages to 'log_messages' (String) so the GUI
can display them without being flooded by recurring sensor messages.
"""

import rclpy
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class LogPublisher(Node):
    """ROS 2 node that forwards important log messages to a GUI topic."""

    def __init__(self, node_name: str = 'log_node') -> None:
        # Every process creates its own LogPublisher. A distinct name per
        # process keeps the names in `ros2 node list` unique.
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, 'log_messages', 10)

    def log(self, message: str, level: str = 'info') -> None:
        if not rclpy.ok():
            print(f'ROS 2 context invalid — skipping log: {message}')
            return

        formatted = f'{level.upper()}: {message}'
        try:
            self.publisher.publish(String(data=formatted))
            lvl = level.upper()
            if lvl == 'ERROR':
                self.get_logger().error(formatted)
            elif lvl == 'DEBUG':
                self.get_logger().debug(formatted)
            elif lvl == 'WARN':
                self.get_logger().warn(formatted)
            else:
                self.get_logger().info(formatted)
        except InvalidHandle:
            print(f'Failed to publish log (invalid context): {formatted}')
        except Exception as exc:
            print(f'Unexpected error while logging: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LogPublisher()
    node.log('Log node started.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        # Ctrl+C has already shut the context down; a plain rclpy.shutdown()
        # would then raise.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
