import torch
from .speed_profile import speed


def compute_nominal_velocity(model, q, q_goal, clearance, speed_profile, qdot_max):
    q = torch.as_tensor(q, dtype=torch.float32, device=model.lower.device).reshape(1, -1).detach().requires_grad_(True)
    goal = torch.as_tensor(q_goal, dtype=torch.float32, device=q.device).reshape(1, -1)
    d = torch.as_tensor(clearance, dtype=torch.float32, device=q.device).reshape(1)
    value = model(q, goal)
    grad = torch.autograd.grad(value.sum(), q)[0]
    norm = torch.linalg.vector_norm(grad, dim=-1, keepdim=True)
    qdot = -speed(d, speed_profile).unsqueeze(-1) * grad / (norm + 1e-8)
    scale = torch.clamp(qdot_max / (torch.max(torch.abs(qdot), dim=-1, keepdim=True).values + 1e-8), max=1.0)
    qdot = qdot * scale
    return {'T': value.detach()[0, 0], 'gradient': grad.detach()[0], 'gradient_norm': norm.detach()[0, 0],
            'speed': speed(d, speed_profile).detach()[0], 'qdot_nom': qdot.detach()[0]}
