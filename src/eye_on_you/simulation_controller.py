import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchService
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')
        self.get_logger().info('SimulationController node has been started.')
        
        # Create subscription to receive joint commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mearm/joint_commands',
            self.listener_callback,
            10
        )
        
        # Create publisher to send joint states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Launch the robot state publisher and joint state publisher
        self.launch_robot_description()
        
        self.get_logger().info('Subscription and publisher have been created.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint commands: {msg.data}')
        joint_state = JointState()
        joint_state.name = ['base_home_joint', 'home_arm_joint']
        joint_state.position = [msg.data[0], msg.data[1]]
        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Published joint states: {joint_state.position}')

    def launch_robot_description(self):
        package_share_directory = get_package_share_directory('eye_on_you')
        urdf_file = os.path.join(package_share_directory, 'urdf', 'robot.urdf')
        
        with open(urdf_file, 'r') as infp:
            robot_description_content = infp.read()

        launch_description = LaunchDescription([
            LaunchNode(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description_content}]
            ),
            LaunchNode(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'
            )
        ])

        ls = LaunchService()
        ls.include_launch_description(launch_description)
        ls.run()

def main(args=None):
    rclpy.init(args=args)
    simulation_controller = SimulationController()
    rclpy.spin(simulation_controller)
    simulation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
