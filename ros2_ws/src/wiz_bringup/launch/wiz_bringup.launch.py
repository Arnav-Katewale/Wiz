from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wiz_status',
            executable='status_publisher',
            name='status_publisher',
            output='screen',
        ),
        Node(
            package='wiz_status',
            executable='status_subscriber',
            name='status_subscriber',
            output='screen',
        ),
    ])
