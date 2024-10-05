#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState  
from cv_bridge import CvBridge, CvBridgeError
import cv2

class FaceDetectionSimulation(Node):
    def __init__(self):
        super().__init__('face_detection_simulation')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.publisher_ = self.create_publisher(JointState, '/robot/joint_commands_simulation', 10)

    def listener_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        frame = cv2.flip(frame, 1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
            self.get_logger().info(f'Face detected at x:{x}, y:{y}, width:{w}, height:{h}')
            self.control_servo(x + w//2, y + h//2, frame.shape[1], frame.shape[0])
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        cv2.imshow("Face Detection", frame)
        cv2.waitKey(1)

    def control_servo(self, face_x, face_y, frame_width, frame_height):
        normalized_x = face_x / frame_width
        normalized_y = face_y / frame_height

        servo_angle_x = -0.7853 + normalized_x * (0.7853 - (-0.7853))
        servo_angle_y = -1.57 + normalized_y * (1 - (-1.57))

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['base_home_joint', 'home_arm_joint']
        joint_state.position = [servo_angle_x, servo_angle_y]
        joint_state.velocity = [0.0, 0.0]  
        joint_state.effort = [0.0, 0.0]    

        self.publisher_.publish(joint_state)
        self.get_logger().info(
            f'Published joint commands:\n'
            f'Names: {joint_state.name}\n'
            f'Positions: {joint_state.position}\n'
            f'Velocities: {joint_state.velocity}\n'
            f'Efforts: {joint_state.effort}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
