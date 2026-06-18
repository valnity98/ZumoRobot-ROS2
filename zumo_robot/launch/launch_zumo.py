from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Arduino (e.g. /dev/ttyUSB0 or /dev/ttyACM0)',
    )
    target_pos_arg = DeclareLaunchArgument(
        'target_position',
        default_value='320',
        description='Target centroid column for PID line-following (pixels)',
    )

    serial_port = LaunchConfiguration('serial_port')
    target_position = LaunchConfiguration('target_position')

    return LaunchDescription([
        serial_port_arg,
        target_pos_arg,

        Node(
            package='zumo_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='zumo_robot',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
            parameters=[{'serial_port': serial_port}],
        ),
        Node(
            package='zumo_robot',
            executable='motors_node',
            name='motors_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'target_position': target_position,
            }],
        ),
        Node(
            package='zumo_robot',
            executable='path_mapping_node',
            name='path_mapping_node',
            output='screen',
        ),
        Node(
            package='zumo_robot',
            executable='tf2_node',
            name='tf2_node',
            output='screen',
        ),
        Node(
            package='zumo_robot',
            executable='interface_node',
            name='interface_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
