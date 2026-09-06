import torch
from .speed_profile import speed


def compute_nominal_velocity(model, q, q_goal, clearance, speed_profile, qdot_max):
    """Calculate a bounded velocity that descends the learned time field.

    The returned velocity is *nominal*: a separate safety controller may still
    modify it before it is sent to a robot.
    """
    device = model.lower.device
    current_joints = torch.as_tensor(q, dtype=torch.float32, device=device)
    current_joints = current_joints.reshape(1, -1).detach().requires_grad_(True)
    goal_joints = torch.as_tensor(q_goal, dtype=torch.float32, device=device).reshape(1, -1)
    obstacle_clearance = torch.as_tensor(clearance, dtype=torch.float32, device=device).reshape(1)

    time_to_goal = model(current_joints, goal_joints)
    gradient = torch.autograd.grad(time_to_goal.sum(), current_joints)[0]
    gradient_norm = torch.linalg.vector_norm(gradient, dim=-1, keepdim=True)

    requested_speed = speed(obstacle_clearance, speed_profile).unsqueeze(-1)
    # Negative gradient points toward lower estimated time-to-goal.
    nominal_velocity = -requested_speed * gradient / (gradient_norm + 1e-8)

    # Scale the entire vector so no joint exceeds its configured speed limit.
    largest_joint_speed = torch.max(torch.abs(nominal_velocity), dim=-1, keepdim=True).values
    limit_scale = torch.clamp(qdot_max / (largest_joint_speed + 1e-8), max=1.0)
    nominal_velocity = nominal_velocity * limit_scale

    return {
        'T': time_to_goal.detach()[0, 0],
        'gradient': gradient.detach()[0],
        'gradient_norm': gradient_norm.detach()[0, 0],
        'speed': requested_speed.detach()[0, 0],
        'qdot_nom': nominal_velocity.detach()[0],
    }
