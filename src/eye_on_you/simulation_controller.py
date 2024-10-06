#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/camera/face_detection',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(JointState, '/robot/joint_commands_simulation', 10)
        self.get_logger().info('SimulationController initialized and subscribed to /camera/face_detection')

    def listener_callback(self, msg):
        try:
            if len(msg.data) < 2:
                self.get_logger().error(f'Received Float64MultiArray with insufficient data: {msg.data}')
                return

            normalized_x = msg.data[0]
            normalized_y = msg.data[1]

            servo_angle_x = -0.7853 + normalized_x * (0.7853 - (-0.7853))
            servo_angle_y = -1.57 + normalized_y * (1 - (-1.57))

            self.get_logger().debug(f'Normalized X: {normalized_x}, Normalized Y: {normalized_y}')
            self.get_logger().debug(f'Calculated servo_angle_x: {servo_angle_x}, servo_angle_y: {servo_angle_y}')

            self.control_simulation(servo_angle_x, servo_angle_y)
        except Exception as e:
            self.get_logger().exception(f'Exception in listener_callback: {e}')

    def control_simulation(self, servo_angle_x, servo_angle_y):
        try:
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
        except Exception as e:
            self.get_logger().exception(f'Exception in control_simulation: {e}')

def main(args=None):
    rclpy.init(args=args)
    simulation_controller = SimulationController()
    try:
        rclpy.spin(simulation_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        simulation_controller.get_logger().exception(f'Exception in main: {e}')
    finally:
        simulation_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
