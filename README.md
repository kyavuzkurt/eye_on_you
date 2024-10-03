# EyeOnYou
ROS2 Package for Arduino controlled 2 servos to track faces.

## Installation

Set up your arduino with arduinosetup.ino and install the Servo library for arduino. Connect the x axis servo to pin 3 and the y axis servo to pin 5.

Clone the repository into your ROS2 workspace and build the package.

```bash
git clone https://github.com/kayeka/EyeOnYou.git
colcon build --packages-select EyeOnYou
```

## Launch

You can launch the package with the following command.

```bash
ros2 launch EyeOnYou twoaxis.py
``` 
Or if you want to run the nodes separately, you can run the following commands on different terminals.

```bash
ros2 run EyeOnYou camera_node
ros2 run EyeOnYou face_detection_node
ros2 run EyeOnYou servo_controller
```




## TO DO

- [ ] Gazebo controllers
- [ ] 3D Model

## License

This project is licensed under the MIT License. See the LICENSE file for more details.
