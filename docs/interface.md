# ROS 2 interface and serial protocol

Back to the [README](../README.md).

## ROS 2 interface

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `line_coordinates` | `Float32MultiArray` | camera → motors | Centroid `[cx, cy]` (pixels) |
| `camera_topic` | `sensor_msgs/Image` | camera → GUI | Annotated BGR frame |
| `encoder_data` | `Int32MultiArray` | encoder → path/GUI | `[left_ticks, right_ticks]` |
| `motor_speeds` | `Int16MultiArray` | motors → GUI | `[left_speed, right_speed]` |
| `robot_position` | `geometry_msgs/Pose` | path → tf2/GUI | Dead-reckoning pose |
| `path_map` | `nav_msgs/OccupancyGrid` | path → RViz | Visited-cell map |
| `visualization_marker` | `visualization_msgs/Marker` | path → RViz | Path LINE_STRIP |
| `robot_command` | `Int8` | GUI → motors, path | 0=stop, 1=start, 2=reset (Arduino encoders and odometry) |
| `log_messages` | `std_msgs/String` | all → GUI | Status log |

### Parameters (motors_node)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyUSB0` | Arduino port |
| `target_position` | int | `320` | Line-centre column |
| `kp` | float | `0.35` | PID proportional gain |
| `ki` | float | `0.10` | PID integral gain |
| `kd` | float | `0.10` | PID derivative gain |
| `max_speed` | float | `125.0` | Base motor speed |

`motors_node` always opens the serial port at 115200 baud (not configurable).

### Parameters (encoder_node)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyUSB0` | Arduino port |
| `baudrate` | int | `115200` | Serial baud rate (must match the sketch, which uses 115200) |

## Serial protocol (PC ↔ Arduino)

PC → Arduino (7 bytes):
```
0x02 | left_speed[0] | left_speed[1] | right_speed[0] | right_speed[1] | ctrl | 0x03
```
`ctrl`: `0x11` = normal run (request encoder reply), `0x12` = reset encoders.

Arduino → PC (10 bytes):
```
0x02 | right_enc[0..3] | left_enc[0..3] | 0x03
```
All multi-byte values are little-endian signed integers.
