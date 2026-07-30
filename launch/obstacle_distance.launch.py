import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for generic robot obstacle_distance_node."""
    pkg_share = get_package_share_directory('field_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_obstacle_distance.yaml')

    return LaunchDescription([
        Node(
            package='field_control',
            executable='obstacle_distance_node',
            name='obstacle_distance_node',
            output='screen',
            parameters=[config_file]
        )
    ])
