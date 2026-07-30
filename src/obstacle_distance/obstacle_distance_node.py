import json
import os
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from sensor_msgs.msg import JointState
import shape_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .obstacle_distance import SolidObstacleDistanceCalculator


class ObstacleDistanceNode(Node):
    """
    Generic, Robot-Independent ROS 2 Node for calculating real-time minimum distance
    from 3D solid obstacles to CAD collision mesh surfaces of ANY robot arm.
    """

    def __init__(self):
        super().__init__('obstacle_distance_node')

        # Declare ROS 2 parameters
        self.declare_parameter('control_rate', 10.0)  # Hz
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('mesh_dir', '')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'tool0')
        self.declare_parameter('use_default_table_obstacle', True)

        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        robot_desc_param = self.get_parameter('robot_description').get_parameter_value().string_value
        mesh_dir = self.get_parameter('mesh_dir').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.tip_link = self.get_parameter('tip_link').get_parameter_value().string_value
        self.use_default_table_obstacle = self.get_parameter('use_default_table_obstacle').get_parameter_value().bool_value

        # Initialize distance calculator dynamically
        self.calc = None
        if robot_desc_param:
            self.calc = SolidObstacleDistanceCalculator(urdf_content=robot_desc_param, mesh_dir=mesh_dir if mesh_dir else None)
        elif urdf_path:
            self.calc = SolidObstacleDistanceCalculator(urdf_path=urdf_path, mesh_dir=mesh_dir if mesh_dir else None)

        # Internal state
        self.latest_q = None
        self.obstacles_map = {}

        if self.use_default_table_obstacle:
            self.init_default_static_obstacles()

        # Dynamic /robot_description Topic Subscriber
        self.robot_desc_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            10
        )

        # Topic Subscribers & Publishers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.planning_scene_sub = self.create_subscription(PlanningScene, '/planning_scene', self.planning_scene_callback, 10)
        self.monitored_scene_sub = self.create_subscription(PlanningScene, '/monitored_planning_scene', self.planning_scene_callback, 10)
        self.collision_object_sub = self.create_subscription(CollisionObject, '/collision_object', self.collision_object_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_distance/markers', 10)
        self.dist_pub = self.create_publisher(String, '/obstacle_distance/per_link_distances', 10)

        self.timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(self.timer_period, self.control_loop_callback)

        self.get_logger().info(f'Obstacle Distance Node initialized for kinematic chain [{self.base_link} -> {self.tip_link}]. Waiting for URDF...')

    def robot_description_callback(self, msg: String):
        """Update robot URDF dynamically when published on /robot_description topic."""
        if not msg.data:
            return
        try:
            self.calc = SolidObstacleDistanceCalculator(urdf_content=msg.data)
            self.get_logger().info('Updated robot model dynamically from /robot_description topic.')
        except Exception as e:
            self.get_logger().error(f'Failed to parse URDF from /robot_description: {e}')

    def init_default_static_obstacles(self):
        """Create default solid table & wall primitives matching MoveIt workspace environment."""
        table_prim = shape_msgs.msg.SolidPrimitive()
        table_prim.type = shape_msgs.msg.SolidPrimitive.BOX
        table_prim.dimensions = [1.2, 0.8, 0.02]

        table_pose = np.eye(4)
        table_pose[2, 3] = -0.01

        back_wall_prim = shape_msgs.msg.SolidPrimitive()
        back_wall_prim.type = shape_msgs.msg.SolidPrimitive.BOX
        back_wall_prim.dimensions = [0.02, 1.0, 0.8]

        back_wall_pose = np.eye(4)
        back_wall_pose[0, 3] = -0.2
        back_wall_pose[2, 3] = 0.4

        self.obstacles_map['default_table'] = {
            'id': 'default_table',
            'primitives': [table_prim, back_wall_prim],
            'primitive_poses': [table_pose, back_wall_pose]
        }

    def joint_states_callback(self, msg: JointState):
        """Extract active joint positions dynamically from kinematic chain."""
        if not self.calc or not self.calc.kin:
            return

        try:
            _, path_joints = self.calc.kin.get_chain(self.base_link, self.tip_link)
            active_joint_names = [j.name for j in path_joints if j.type in ['revolute', 'continuous', 'prismatic']]

            q_temp = []
            for name in active_joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    q_temp.append(msg.position[idx])
            if len(q_temp) == len(active_joint_names):
                self.latest_q = q_temp
        except Exception:
            pass

    def planning_scene_callback(self, msg: PlanningScene):
        for col_obj in msg.world.collision_objects:
            self.process_collision_object(col_obj)

    def collision_object_callback(self, msg: CollisionObject):
        self.process_collision_object(msg)

    def process_collision_object(self, msg: CollisionObject):
        if msg.operation == CollisionObject.REMOVE:
            if msg.id in self.obstacles_map:
                del self.obstacles_map[msg.id]
        else:
            if msg.primitives:
                self.obstacles_map[msg.id] = {
                    'id': msg.id,
                    'primitives': list(msg.primitives),
                    'primitive_poses': list(msg.primitive_poses)
                }

    def control_loop_callback(self):
        if not rclpy.ok() or not self.calc or self.latest_q is None:
            return

        obstacles_list = list(self.obstacles_map.values())
        if not obstacles_list:
            return

        try:
            result = self.calc.compute_per_link_distances(
                self.latest_q, obstacles_list, base_link=self.base_link, tip_link=self.tip_link
            )
        except Exception as e:
            self.get_logger().error(f'Distance calculation error: {e}', throttle_duration_sec=5.0)
            return

        if not rclpy.ok():
            return

        json_msg = String()
        json_msg.data = json.dumps(result)
        try:
            self.dist_pub.publish(json_msg)
        except Exception:
            return

        overall_dist = result['overall_min_distance']
        closest_link = result['overall_closest_link']
        closest_obs = result['overall_closest_obstacle']

        self.get_logger().info(
            f"[CAD Mesh Monitor] Min Dist: {overall_dist:.4f}m | Link: '{closest_link}' | Obstacle: '{closest_obs}'",
            throttle_duration_sec=2.0
        )
        self.publish_rviz_markers(result)

    def publish_rviz_markers(self, result):
        if not rclpy.ok():
            return
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        m_id = 0

        for link_name, data in result['per_link'].items():
            dist = data['distance']
            pt_link, pt_obs = data['closest_point_on_link'], data['closest_point_on_obstacle']
            if dist < 0 or pt_link is None or pt_obs is None:
                continue

            r, g, b = (1.0, 0.0, 0.0) if dist < 0.05 else ((1.0, 1.0, 0.0) if dist < 0.15 else (0.0, 1.0, 0.0))

            line_marker = Marker()
            line_marker.header.frame_id, line_marker.header.stamp = self.base_link, now
            line_marker.ns, line_marker.id = 'mesh_distance_lines', m_id
            m_id += 1
            line_marker.type, line_marker.action = Marker.LINE_STRIP, Marker.ADD
            line_marker.scale.x = 0.008
            line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a = r, g, b, 0.9
            line_marker.points = [Point(x=pt_link[0], y=pt_link[1], z=pt_link[2]), Point(x=pt_obs[0], y=pt_obs[1], z=pt_obs[2])]
            marker_array.markers.append(line_marker)

            text_marker = Marker()
            text_marker.header.frame_id, text_marker.header.stamp = self.base_link, now
            text_marker.ns, text_marker.id = 'mesh_distance_labels', m_id
            m_id += 1
            text_marker.type, text_marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            text_marker.pose.position.x = (pt_link[0] + pt_obs[0]) / 2.0
            text_marker.pose.position.y = (pt_link[1] + pt_obs[1]) / 2.0
            text_marker.pose.position.z = (pt_link[2] + pt_obs[2]) / 2.0 + 0.03
            text_marker.scale.z = 0.035
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = r, g, b, 1.0
            text_marker.text = f"{link_name}: {dist:.3f}m"
            marker_array.markers.append(text_marker)

        try:
            self.marker_pub.publish(marker_array)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
