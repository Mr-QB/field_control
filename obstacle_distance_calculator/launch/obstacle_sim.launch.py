"""UR kinematic simulation in RViz with random spherical collision objects."""
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('obstacle_distance_calculator'))
    ur_share = Path(get_package_share_directory('ur_description'))
    moveit_share = Path(get_package_share_directory('ur_moveit_config'))
    ur_type = LaunchConfiguration('ur_type').perform(context)
    urdf = xacro.process_file(str(ur_share / 'urdf/ur.urdf.xacro'), mappings={
        'name': 'ur', 'ur_type': ur_type, 'prefix': '',
        'robot_ip': '0.0.0.0', 'use_fake_hardware': 'true',
    }).toxml()
    srdf = xacro.process_file(str(moveit_share / 'srdf/ur.srdf.xacro'), mappings={
        'name': 'ur', 'prefix': '',
    }).toxml()
    model = {
        'robot_description': urdf, 'robot_description_semantic': srdf,
        'robot_description_kinematics.ur_manipulator.kinematics_solver':
            'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'robot_description_kinematics.ur_manipulator.kinematics_solver_timeout': 0.05,
    }
    obstacle_params = {}
    obstacle_params['randomize'] = LaunchConfiguration('randomize').perform(context).lower() == 'true'
    for name in ('count', 'seed'):
        obstacle_params[name] = int(LaunchConfiguration(name).perform(context))
    for name in ('radius_min', 'radius_max', 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'):
        obstacle_params[name] = float(LaunchConfiguration(name).perform(context))
    joints = {
        'robot_description': urdf, 'rate': 50,
        'zeros.shoulder_pan_joint': 0.0,
        'zeros.shoulder_lift_joint': -1.57,
        'zeros.elbow_joint': 1.57,
        'zeros.wrist_1_joint': -1.57,
        'zeros.wrist_2_joint': -1.57,
        'zeros.wrist_3_joint': 0.0,
    }
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[joints], condition=IfCondition(LaunchConfiguration('joint_gui'))),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[joints], condition=UnlessCondition(LaunchConfiguration('joint_gui'))),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[model, {
                 'allow_trajectory_execution': False,
                 'move_group.allow_trajectory_execution': False,
                 'publish_robot_description': True,
                 'publish_robot_description_semantic': True,
                 'publish_planning_scene': True,
                 'publish_geometry_updates': True,
                 'publish_state_updates': True,
                 'publish_transforms_updates': True,
                 'planning_pipelines': ['ompl'],
                 'default_planning_pipeline': 'ompl',
                 'ompl.planning_plugin': 'ompl_interface/OMPLPlanner',
             }]),
        Node(package='obstacle_distance_calculator', executable='obstacle_distance_node',
             name='obstacle_distance_node', output='screen',
             parameters=[str(share / 'config/obstacle_distance.yaml'), model]),
        Node(package='obstacle_distance_calculator', executable='random_obstacles.py',
             parameters=[str(share / 'config/fixed_obstacles.yaml'), obstacle_params], output='screen'),
        Node(package='rviz2', executable='rviz2', parameters=[model],
             arguments=['-d', str(share / 'config/obstacle_sim.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    defaults = {
        'ur_type': 'ur3', 'rviz': 'true', 'joint_gui': 'true',
        'randomize': 'false', 'count': '5', 'seed': '-1', 'radius_min': '0.04', 'radius_max': '0.10',
        # Bounds used only with randomize:=true.  Keep samples near the arm's
        # working volume so generated scenes are challenging as well.
        'x_min': '-0.35', 'x_max': '0.45', 'y_min': '-0.35', 'y_max': '0.35',
        'z_min': '0.22', 'z_max': '0.65',
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()
    ] + [OpaqueFunction(function=setup)])
