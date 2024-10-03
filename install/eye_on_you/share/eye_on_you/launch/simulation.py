import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('eye_on_you'),
        'urdf',
        'robot.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=['--ros-args', '--remap', 'joint_states:=/joint_states']
        ),
        Node(
            package='eye_on_you',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='eye_on_you',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen'
        ),
        Node(
            package='eye_on_you',
            executable='simulation_controller',
            name='simulation_controller',
            output='screen'
        ),
    ])
