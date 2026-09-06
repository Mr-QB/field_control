import os
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from obstacle_distance_msgs.msg import LinkObstacleDistanceArray
from ..core.model import GoalTimeField
from ..core.nominal import compute_nominal_velocity


class GoalTimeFieldNode(Node):
    """Publish a nominal joint velocity from a trained Goal Time Field."""
    def __init__(self):
        super().__init__('goal_time_field_node')
        checkpoint_path = self.declare_parameter('checkpoint_path', '').value
        if not checkpoint_path or not os.path.isfile(checkpoint_path):
            raise ValueError('checkpoint_path must point to a trained checkpoint')

        # A non-empty default keeps ROS 2's parameter type as DOUBLE_ARRAY.
        self.q_goal = self.declare_parameter('q_goal', [0.0]).value
        self.qdot_max = self.declare_parameter('qdot_max', 1.0).value
        self.clearance_cap = self.declare_parameter('clearance_cap', 1.0).value

        checkpoint = torch.load(checkpoint_path, map_location='cpu')
        self.names = checkpoint['joint_names']
        self.model = GoalTimeField(
            checkpoint['joint_lower_bounds'],
            checkpoint['joint_upper_bounds'],
            **checkpoint['model_parameters'],
        )
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        self.speed_profile = checkpoint['speed_profile']

        if len(self.q_goal) != len(self.names):
            raise ValueError('q_goal length must match checkpoint joint_names')

        # Use the cap until a distance message arrives.
        self.clearance = self.clearance_cap
        self.publisher = self.create_publisher(
            JointState, '/goal_time_field/nominal_velocity', 10
        )
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(
            LinkObstacleDistanceArray, '/obstacle_distances', self.distance_callback, 10
        )

    def distance_callback(self, message):
        """Remember the latest finite clearance, capped to the useful range."""
        measured_clearance = message.overall_min_distance
        if torch.isfinite(torch.tensor(measured_clearance)):
            self.clearance = min(measured_clearance, self.clearance_cap)
        else:
            self.clearance = self.clearance_cap

    def joint_callback(self, message):
        """Compute and publish a velocity whenever all trained joints are known."""
        positions_by_name = dict(zip(message.name, message.position))
        if not all(name in positions_by_name for name in self.names):
            return

        current_joints = [positions_by_name[name] for name in self.names]
        result = compute_nominal_velocity(
            self.model,
            current_joints,
            self.q_goal,
            self.clearance,
            self.speed_profile,
            self.qdot_max,
        )

        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = self.names
        output.position = current_joints
        output.velocity = result['qdot_nom'].tolist()
        self.publisher.publish(output)

def main():
    rclpy.init()
    node = GoalTimeFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
