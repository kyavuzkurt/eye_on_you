#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ServoController(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        super().__init__('servo_controller')
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)  
        
        if self.arduino.in_waiting > 0:
            setup_message = self.arduino.readline().decode().strip()
            self.get_logger().info(f"Arduino setup message: {setup_message}")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/camera/face_detection',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        normalised_x = msg.data[0]
        normalised_y = msg.data[1]
        position_x = int(normalised_x * 180)
        position_y = int(45 + (1 - normalised_y) * 135)
        self.send_position('x', position_x)
        self.send_position('y', position_y)

    def send_position(self, axis, position):
        if 0 <= position <= 180:
            command = f"{axis}{position}\n"
            self.arduino.write(command.encode())
            self.get_logger().info(f"Sent position: {command.strip()}")
            while self.arduino.in_waiting == 0:
                pass
            response = self.arduino.readline().decode().strip()
            self.get_logger().info(f"Arduino response: {response}")
        else:
            self.get_logger().error(f"Error: Position for {axis} must be between 0 and 180.")

    def close(self):
        self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    controller = ServoController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("\nExiting...")
    finally:
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

