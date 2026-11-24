# ZumoRobot-ROS2

A Zumo robot that follows a black line with a USB webcam and draws a map of the path it has driven. Everything on the PC side runs on ROS 2 (Python, ament_python).

![Demo](zumo_robot/media/demo.gif)

The full video is in [`zumo_robot/media/demo.mp4`](zumo_robot/media/demo.mp4).

The camera node finds the line with OpenCV, a PID controller turns the line position into motor speeds, and the wheel encoders give the position for the map (dead reckoning).

Team project in the master's course *Autonomous Intelligent Systems* at Frankfurt UAS (winter semester 2024/25). The project is finished. The encoder library on the Arduino side is in a separate repository: [Zumo328P-Library](https://github.com/valnity98/Zumo328P-Library).

## How it works

Six nodes talk over ROS 2 topics. All of them also publish status messages on `log_messages` through the small `LogPublisher` class in `log_node.py`, and the window shows them in its log.

- `camera_node` finds the line in the webcam image and publishes its position (`line_coordinates`) and an annotated image (`camera_topic`).
- `motors_node` runs the PID controller on the line position and sends the motor speeds to the Arduino over serial.
- `encoder_node` reads the encoder counts that the Arduino sends back and publishes them (`encoder_data`).
- `path_mapping_node` integrates the encoder counts to a pose (dead reckoning) and publishes the pose, an occupancy grid of the driven path and a path marker for RViz.
- `tf2_node` publishes the `map` to `base_link` transform for RViz.
- `interface_node` is the Qt window: live video, encoder values, motor speeds, pose and log, with Start, Stop and Reset buttons (`robot_command`).

The line detection thresholds the grayscale image and looks for contours in 5 horizontal sections of the image. A Kalman-filtered centroid is drawn in the debug overlay, but the PID controller gets the unfiltered one. The ROS graph:

![ROS graph](zumo_robot/media/rosgraph.png)

## Hardware requirements

| Component | Details |
|---|---|
| Zumo Shield for Arduino v1.2 | Base platform with DC motors |
| Arduino Leonardo (ATmega32U4) | Motor control and encoder reading |
| Magnetic encoder pair | Pololu #3081, 12 CPR, 2.7–18 V (external, mounted manually) |
| USB webcam | Any Linux-compatible UVC camera |
| Linux PC | Runs ROS 2 nodes |
| Track | Black line on white background, ≥ 2 cm width |

## Software requirements

- ROS 2: Humble or Jazzy (Ubuntu 22.04 / 24.04)
- Python 3.10+
- Arduino IDE ≥ 2.0

## Installation

### 1. System dependencies

```bash
sudo apt update
sudo apt install python3-pip python3-opencv python3-pyqt5 python3-serial \
                 ros-$ROS_DISTRO-cv-bridge \
                 ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-rviz2
sudo apt remove brltty   # conflicts with /dev/ttyUSB0 on Ubuntu 22.04
```

### 2. Clone and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/valnity98/ZumoRobot-ROS2.git
rosdep install --from-paths . --ignore-src -r -y   # optional: install the dependencies declared in package.xml
cd ~/ros2_ws
colcon build --packages-select zumo_robot
source install/setup.bash
```

### 3. Flash the Arduino firmware

1. Install [Zumo328P-Library](https://github.com/valnity98/Zumo328P-Library) as an Arduino library (ZIP download, or clone it into your Arduino `libraries/` folder as `Zumo328P`).
2. Install the library dependencies via Library Manager: FastGPIO and ZumoShield (the sketch uses `ZumoMotors` from it).
3. Open `Zumo328P-Library/examples/ZumoRos2/ZumoRos2.ino` in the Arduino IDE.
4. Flash it to the Arduino Leonardo connected to the Zumo Shield.

## Usage

### Launch everything

```bash
ros2 launch zumo_robot launch_zumo.py
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Arduino serial port |
| `target_position` | `320` | Image column for line centre (pixels) |

Example with custom port:

```bash
ros2 launch zumo_robot launch_zumo.py serial_port:=/dev/ttyACM0
```

An Arduino Leonardo (ATmega32U4) shows up as `/dev/ttyACM0`, a USB-serial adapter as `/dev/ttyUSB0` (the default). Check which one your system created (e.g. with `ls /dev/ttyACM* /dev/ttyUSB*`) and set `serial_port` accordingly.

RViz starts without a saved configuration. Set the fixed frame to `map` and add the displays *TF*, *Map* (topic `path_map`) and *Marker* (topic `visualization_marker`).

### Start the robot

The robot only drives after it has received the start command: press Start in the GUI, or send it from a terminal:

```bash
ros2 topic pub --once /robot_command std_msgs/msg/Int8 "{data: 1}"
```

`0` stops the robot, `2` resets the encoders and the odometry (see the `robot_command` topic in [docs/interface.md](docs/interface.md)).

### Run individual nodes

```bash
ros2 run zumo_robot camera_node
ros2 run zumo_robot encoder_node --ros-args -p serial_port:=/dev/ttyUSB0
ros2 run zumo_robot motors_node  --ros-args -p target_position:=320
ros2 run zumo_robot path_mapping_node
ros2 run zumo_robot interface_node
```

### Calibration

The odometry constants at the top of `zumo_robot/zumo_robot/odometry.py` are not measured for every robot. Check `CPR` and `WHEEL_RADIUS` on yours and rebuild after changing them:

- `CPR`: (encoder counts per wheel revolution, currently `75.81 × 6` ≈ 455): Pololu's 12 CPR applies when both edges of both encoder channels are counted, but the Arduino library only counts the edges of channel A, which gives 6 counts per motor revolution. Send reset (`{data: 2}`), turn one wheel by hand exactly one revolution, send stop once (`{data: 0}`, the Arduino only reports encoder values in reply to motor packets) and read the tick count with `ros2 topic echo /encoder_data`. That count is the value for `CPR`.
- `WHEEL_RADIUS`: (currently `0.019` m, an estimate from the 39 mm tread height of the Zumo chassis): with `CPR` calibrated, reset, drive a straight 1 m and compare the distance reported in `robot_position` (metres from the start) with 1 m. Adjust `WHEEL_RADIUS` until they match.

### Tests

The PID controller and the odometry maths have unit tests that run without ROS and without the robot:

```bash
cd zumo_robot
python3 -m pytest test/test_pid.py test/test_path_node.py
```

## Interface

The topics, the node parameters and the serial protocol between the PC and the Arduino are described in [docs/interface.md](docs/interface.md).

## Limitations

- The only safety stop is a timeout. The Arduino stops the motors when no valid packet arrives for 500 ms, for example when the camera loses the line, a node crashes or the USB cable is pulled. It cannot tell a lost line from a dead PC, and the robot starts again as soon as commands arrive.
- The camera index is fixed. The camera node opens video device 0.
- The odometry constants are nominal. `CPR` and `WHEEL_RADIUS` have not been calibrated on the robot (see [Calibration](#calibration)), so distances in the map are approximate.
- Encoder data follows the motor commands. The Arduino only replies with encoder values to motor packets, so the odometry rate follows the camera and pauses while no line is detected.
- The serial protocol has no checksum. Frames are recognised by their fixed length and the `0x02`/`0x03` markers only.

## Team

| Name | Role |
|---|---|
| Mutasem Bader | ROS 2 nodes, PID control, path mapping, Qt GUI, system integration |
| Felix Fritz Biermann | Image processing (camera node) |

## License

Copyright (c) 2026 Mutasem Bader. All rights reserved.  
Viewing is permitted. Copying, modifying, or submitting as own work is strictly prohibited.  
See [LICENSE](LICENSE) for details.
