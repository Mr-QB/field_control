import torch
from torch import nn
from torch.nn import functional as F


class GoalTimeField(nn.Module):
    """Estimate time-to-goal for a fixed obstacle scene.

    Inputs are current joint angles ``q`` and goal joint angles ``q_goal``.
    The output stays differentiable with respect to ``q`` because inference
    follows the negative gradient of this value.
    """
    def __init__(self, lower, upper, hidden_dim=256, hidden_layers=4):
        super().__init__()
        self.register_buffer('lower', torch.as_tensor(lower, dtype=torch.float32))
        self.register_buffer('upper', torch.as_tensor(upper, dtype=torch.float32))

        layers = []
        input_width = self.lower.numel() * 2  # Current joints + goal joints.
        width = input_width
        for _ in range(hidden_layers):
            layers.extend([nn.Linear(width, hidden_dim), nn.SiLU()])
            width = hidden_dim
        self.net = nn.Sequential(*layers, nn.Linear(width, 1))

    def normalize(self, q):
        """Map every joint from its physical range into [-1, 1]."""
        return 2.0 * (q - self.lower) / (self.upper - self.lower) - 1.0

    def forward(self, q, q_goal):
        current_normalized = self.normalize(q)
        goal_normalized = self.normalize(q_goal)

        # This factor makes T(q_goal, q_goal) exactly zero by construction.
        distance_to_goal = torch.linalg.vector_norm(
            current_normalized - goal_normalized,
            dim=-1,
            keepdim=True,
        )
        network_input = torch.cat((current_normalized, goal_normalized), dim=-1)
        positive_scale = F.softplus(self.net(network_input))
        return distance_to_goal * positive_scale
