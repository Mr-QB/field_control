import torch
from torch import nn
from torch.nn import functional as F


class GoalTimeField(nn.Module):
    """T(q, q_goal) for a fixed obstacle scene; raw joint inputs remain differentiable."""
    def __init__(self, lower, upper, hidden_dim=256, hidden_layers=4):
        super().__init__()
        self.register_buffer('lower', torch.as_tensor(lower, dtype=torch.float32))
        self.register_buffer('upper', torch.as_tensor(upper, dtype=torch.float32))
        layers, width = [], self.lower.numel() * 2
        for _ in range(hidden_layers):
            layers += [nn.Linear(width, hidden_dim), nn.SiLU()]
            width = hidden_dim
        self.net = nn.Sequential(*layers, nn.Linear(width, 1))

    def normalize(self, q):
        return 2.0 * (q - self.lower) / (self.upper - self.lower) - 1.0

    def forward(self, q, q_goal):
        qn, gn = self.normalize(q), self.normalize(q_goal)
        # Structural boundary: base is exactly zero when q == q_goal.
        base = torch.linalg.vector_norm(qn - gn, dim=-1, keepdim=True)
        return base * F.softplus(self.net(torch.cat((qn, gn), dim=-1)))
