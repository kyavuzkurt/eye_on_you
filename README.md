# Eye on You

Eye on You is an innovative ROS2 package that brings face tracking capabilities to life using Arduino-controlled servos. This project combines computer vision, robotics, and real-time control to create a dynamic system that can follow faces in its field of view.

## Key Features

- **Face Detection and Tracking**: Utilizes advanced computer vision algorithms.
- **Real-Time Servo Control**: Smooth tracking motion with two servos (X and Y axis).
- **Arduino Integration**: Hardware control via Arduino.
- **ROS2 Architecture**: Modular and scalable design.
- **Simulation Environment**: Included for testing and development.

Whether you're interested in robotics, computer vision, or interactive installations, Eye on You provides a fascinating platform for exploration and experimentation in the realm of human-robot interaction.

## Installation

1. **Arduino Setup**:
    - Upload `arduinosetup.ino` to your Arduino.
    - Install the Servo library for Arduino.
    - Connect the X-axis servo to pin 3 and the Y-axis servo to pin 5.

2. **Clone and Build**:
    - Clone the repository into your ROS2 workspace:
      ```bash
      git clone https://github.com/kyavuzkurt/eye_on_you.git
      ```
    - Build the package:
      ```bash
      colcon build --packages-select eye_on_you
      ```

## Launch

### Full Launch
Launch the entire package with:
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

## Simulation

Launch the simulation with:
```bash
ros2 launch eye_on_you simulation.py
```
Note: The simulation is a work in progress. It can publish simulation controller messages, but the simulation itself has some issues. You can still test the controller part with the published messages.

To view the robot model in RViz2:
```bash
ros2 run rviz2 rviz2
```
The joints won't move due to simulation issues, but the model will be displayed correctly.

## To Do

- [ ] Fix simulation
- [ ] Make it work in Gazebo

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
