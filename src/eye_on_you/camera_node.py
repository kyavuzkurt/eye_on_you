#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0) # Change to your usb camera index if your webcam is not working
        self.br = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera")
            self.destroy_node()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().error("Failed to capture image")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
