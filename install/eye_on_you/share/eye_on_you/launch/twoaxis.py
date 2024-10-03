import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()