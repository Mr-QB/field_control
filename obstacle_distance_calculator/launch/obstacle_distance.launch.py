import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_distance_calculator')
    config_file = os.path.join(pkg_share, 'config', 'obstacle_distance.yaml')

    return LaunchDescription([
        Node(
            package='obstacle_distance_calculator',
            executable='obstacle_distance_node',
            name='obstacle_distance_node',
            output='screen',
            parameters=[config_file]
        )
    ])
