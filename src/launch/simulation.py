import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_directory = get_package_share_directory('eye_on_you')
    urdf_file = os.path.join(package_share_directory, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(package_share_directory, 'rviz', 'robot_view.rviz')

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
            package='eye_on_you',
            executable='simulation_controller',
            name='simulation_controller',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
