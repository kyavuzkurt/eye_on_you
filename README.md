# Eye on You

Face tracking with Arduino-controlled servos, powered by ROS2

## Overview

Eye on You is an innovative ROS2 package that brings face tracking capabilities to life using Arduino-controlled servos. This project combines computer vision, robotics, and real-time control to create a dynamic system that can follow faces in its field of view.

Whether you're interested in robotics, computer vision, or interactive installations, Eye on You provides a fascinating platform for exploration and experimentation in the realm of human-robot interaction.

## Key Features

- **Face Detection and Tracking**: Utilizes advanced computer vision algorithms for accurate face detection and smooth tracking.
- **Real-Time Servo Control**: Implements precise control of two servos (X and Y axis) for responsive movement.
- **Arduino Integration**: Seamlessly integrates with Arduino for reliable hardware control.
- **ROS2 Architecture**: Built on a modular and scalable ROS2 design for easy expansion and modification.
- **Simulation Environment**: Includes a simulation setup for testing and development without physical hardware.

## Requirements 

- ROS2 Humble
- Arduino Uno
- Servo Motor x2 (SG90 recommended)

**Note**: While I use SG90 servos, any Arduino-compatible servo should work with the same serial communication. Since I don't have a servo driver for Arduino, I can't reliably test and support servo drives other than SG90.

## Installation

### Arduino Setup

1. Upload `arduinosetup.ino` to your Arduino board
2. Install the Servo library for Arduino if not already present
3. Connect the servos:
   - X-axis servo to pin 3
   - Y-axis servo to pin 5

### ROS2 Package Setup

```bash
# Clone the repository into your ROS2 workspace
git clone https://github.com/kyavuzkurt/eye_on_you.git

# Build the package
colcon build --packages-select eye_on_you

# Source the workspace
source install/setup.bash
```

## Usage

### Full Launch
To launch the entire package:
```bash
ros2 launch eye_on_you twoaxis.py
```

### Individual Nodes
Alternatively, run the nodes separately in different terminals:
```bash
ros2 run eye_on_you camera_node
ros2 run eye_on_you face_detection_node
ros2 run eye_on_you servo_controller
```
**Note**: Face detection node is currently using the camera node's video stream. So you need to run the camera node before running the face detection node. If your webcam is not working try configuring camera node to your usb camera index.


## Simulation

Launch the simulation environment with:
```bash
ros2 launch eye_on_you simulation.py
```

**Note**: Simulation uses another face detection node that publishes joint commands to the simulation instead of the physical robot. So you cannot control the real robot and simulation at the same time.



## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

Created and maintained by [kyavuzkurt](https://github.com/kyavuzkurt)
