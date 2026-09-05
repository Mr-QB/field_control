import os
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from obstacle_distance_msgs.msg import LinkObstacleDistanceArray
from .model import GoalTimeField
from .nominal import compute_nominal_velocity


class GoalTimeFieldNode(Node):
    def __init__(self):
        super().__init__('goal_time_field_node')
        path=self.declare_parameter('checkpoint_path','').value
        if not path or not os.path.isfile(path): raise ValueError('checkpoint_path must point to a trained checkpoint')
        # A non-empty default keeps ROS 2's parameter type as DOUBLE_ARRAY.
        self.q_goal=self.declare_parameter('q_goal',[0.0]).value; self.qdot_max=self.declare_parameter('qdot_max',1.0).value; self.clearance_cap=self.declare_parameter('clearance_cap',1.0).value
        ckpt=torch.load(path,map_location='cpu'); self.names=ckpt['joint_names']; self.model=GoalTimeField(ckpt['joint_lower_bounds'],ckpt['joint_upper_bounds'],**ckpt['model_parameters']); self.model.load_state_dict(ckpt['model_state_dict']); self.model.eval(); self.speed_profile=ckpt['speed_profile']
        if len(self.q_goal) != len(self.names): raise ValueError('q_goal length must match checkpoint joint_names')
        self.clearance=self.clearance_cap; self.pub=self.create_publisher(JointState,'/goal_time_field/nominal_velocity',10)
        self.create_subscription(JointState,'/joint_states',self.joint_callback,10); self.create_subscription(LinkObstacleDistanceArray,'/obstacle_distances',self.distance_callback,10)

    def distance_callback(self,msg):
        self.clearance=min(msg.overall_min_distance,self.clearance_cap) if torch.isfinite(torch.tensor(msg.overall_min_distance)) else self.clearance_cap

    def joint_callback(self,msg):
        lookup=dict(zip(msg.name,msg.position))
        if not all(name in lookup for name in self.names): return
        result=compute_nominal_velocity(self.model,[lookup[name] for name in self.names],self.q_goal,self.clearance,self.speed_profile,self.qdot_max)
        out=JointState(); out.header.stamp=self.get_clock().now().to_msg(); out.name=self.names; out.position=[lookup[name] for name in self.names]; out.velocity=result['qdot_nom'].tolist(); self.pub.publish(out)

def main():
    rclpy.init(); node=GoalTimeFieldNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
