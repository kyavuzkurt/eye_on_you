import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')
        self.get_logger().info('SimulationController node has been started.')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mearm/joint_commands',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Subscription and publisher have been created.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint commands: {msg.data}')
        joint_state = JointState()
        joint_state.name = ['base_home_joint', 'home_arm_joint']
        joint_state.position = [msg.data[0], msg.data[1]]
        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Published joint states: {joint_state.position}')

def main(args=None):
    rclpy.init(args=args)
    simulation_controller = SimulationController()
    rclpy.spin(simulation_controller)
    simulation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
