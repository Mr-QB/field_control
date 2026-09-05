#!/usr/bin/env python3
"""Insert fixed or random static spheres into MoveIt's planning scene."""
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class RandomObstacles(Node):
    def __init__(self):
        super().__init__('random_obstacles')
        defaults = {
            'randomize': False,
            'count': 5, 'seed': -1, 'radius_min': 0.04, 'radius_max': 0.10,
            'x_min': -0.55, 'x_max': 0.55, 'y_min': -0.55, 'y_max': 0.55,
            'z_min': 0.15, 'z_max': 0.75, 'frame_id': 'base_link',
            # Non-empty defaults preserve ROS 2's floating-point array type.
            'fixed_positions': [0.35, -0.30, 0.30], 'fixed_radii': [0.06],
        }
        p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        if p['randomize']:
            spheres = self.random_spheres(p)
        else:
            spheres = self.fixed_spheres(p)

        self.request = ApplyPlanningScene.Request()
        self.request.scene.is_diff = True
        self.request.scene.robot_state.is_diff = True
        for i, (xyz, radius) in enumerate(spheres):
            self.add_sphere(i, p['frame_id'], xyz, radius)
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.future = None
        self.timer = self.create_timer(1.0, self.apply)
        self.get_logger().info('Waiting for /apply_planning_scene...')

    def fixed_spheres(self, p):
        if len(p['fixed_positions']) % 3 or len(p['fixed_positions']) // 3 != len(p['fixed_radii']):
            raise ValueError('fixed_positions must have [x, y, z] for every fixed_radii entry')
        return [
            (tuple(p['fixed_positions'][i:i + 3]), radius)
            for i, radius in zip(range(0, len(p['fixed_positions']), 3), p['fixed_radii'])
        ]

    def random_spheres(self, p):
        if not 1 <= p['count'] <= 100:
            raise ValueError('count must be between 1 and 100')
        rng = random.Random(None if p['seed'] < 0 else p['seed'])
        spheres = []
        for _ in range(p['count']):
            for attempt in range(10000):
                radius = rng.uniform(p['radius_min'], p['radius_max'])
                xyz = tuple(rng.uniform(p[a + '_min'], p[a + '_max']) for a in ('x', 'y', 'z'))
                if math.hypot(xyz[0], xyz[1]) < radius + 0.18:
                    continue
                if any(math.dist(xyz, pos) < radius + r + 0.02 for pos, r in spheres):
                    continue
                break
            else:
                raise ValueError('Cannot fit spheres in these bounds; reduce count/radius or expand bounds')
            spheres.append((xyz, radius))
        return spheres

    def add_sphere(self, index, frame_id, xyz, radius):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = f'obstacle_{index}'
        obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = xyz
        obj.primitive_poses = [pose]
        self.request.scene.world.collision_objects.append(obj)
        color = ObjectColor(id=obj.id)
        color.color.r, color.color.g, color.color.b, color.color.a = 0.95, 0.35, 0.1, 0.85
        self.request.scene.object_colors.append(color)
        self.get_logger().info(f'{obj.id}: radius={radius:.3f} m, position={xyz}')

    def apply(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                if self.future.result().success:
                    self.get_logger().info('Obstacles added to the planning scene.')
                    self.timer.cancel()
                    return
            except Exception as exc:
                self.get_logger().warning(f'Scene request failed: {exc}')
            self.future = None
            self.get_logger().warning('Retrying obstacle insertion...')
        if self.client.service_is_ready():
            self.future = self.client.call_async(self.request)


def main():
    rclpy.init()
    node = None
    try:
        node = RandomObstacles()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
