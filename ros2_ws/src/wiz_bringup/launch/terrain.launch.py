from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_type = LaunchConfiguration('source_type')
    source_path = LaunchConfiguration('source_path')
    frame_rate = LaunchConfiguration('frame_rate')
    grid_rows = LaunchConfiguration('grid_rows')
    grid_cols = LaunchConfiguration('grid_cols')
    assumed_altitude_m = LaunchConfiguration('assumed_altitude_m')
    camera_hfov_deg = LaunchConfiguration('camera_hfov_deg')

    return LaunchDescription([
        DeclareLaunchArgument('source_type', default_value='synthetic'),
        DeclareLaunchArgument('source_path', default_value=''),
        DeclareLaunchArgument('frame_rate', default_value='15.0'),
        DeclareLaunchArgument('grid_rows', default_value='24'),
        DeclareLaunchArgument('grid_cols', default_value='32'),
        # Ground scale is an ASSUMPTION in this sprint, not a measurement:
        # there is no altitude telemetry yet.
        DeclareLaunchArgument('assumed_altitude_m', default_value='3.0'),
        DeclareLaunchArgument('camera_hfov_deg', default_value='70.0'),

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
            package='wiz_terrain',
            executable='terrain_node',
            name='terrain_node',
            output='screen',
            parameters=[{
                'grid_rows': grid_rows,
                'grid_cols': grid_cols,
                'assumed_altitude_m': assumed_altitude_m,
                'camera_hfov_deg': camera_hfov_deg,
            }],
        ),
    ])
