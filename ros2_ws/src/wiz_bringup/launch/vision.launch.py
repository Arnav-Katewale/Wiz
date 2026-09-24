from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_type = LaunchConfiguration('source_type')
    source_path = LaunchConfiguration('source_path')
    frame_rate = LaunchConfiguration('frame_rate')
    model_dir = LaunchConfiguration('model_dir')

    return LaunchDescription([
        DeclareLaunchArgument('source_type', default_value='synthetic'),
        DeclareLaunchArgument('source_path', default_value=''),
        DeclareLaunchArgument('frame_rate', default_value='15.0'),
        DeclareLaunchArgument('model_dir', default_value=''),

        Node(
            package='wiz_vision',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'source_type': source_type,
                'source_path': source_path,
                'frame_rate': frame_rate,
            }],
        ),
        Node(
            package='wiz_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{'model_dir': model_dir}],
        ),
    ])
