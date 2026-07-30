from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for obstacle_distance_node."""
    return LaunchDescription([
        Node(
            package='field_control',
            executable='obstacle_distance_node',
            name='obstacle_distance_node',
            output='screen',
            parameters=[{
                'control_rate': 10.0,
                'use_default_table_obstacle': True
            }]
        )
    ])
