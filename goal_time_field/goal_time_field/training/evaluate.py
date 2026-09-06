import argparse
import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader
from .dataset import ClearanceDataset
from ..core.model import GoalTimeField
from ..core.speed_profile import speed

def main():
    parser = argparse.ArgumentParser(description='Evaluate a Goal Time Field checkpoint.')
    parser.add_argument('--csv', required=True)
    parser.add_argument('--metadata', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--samples', type=int, default=256)
    args = parser.parse_args()

    checkpoint = torch.load(args.checkpoint, map_location='cpu')
    dataset = ClearanceDataset(args.csv, args.metadata)
    model = GoalTimeField(
        checkpoint['joint_lower_bounds'], checkpoint['joint_upper_bounds'],
        **checkpoint['model_parameters'],
    )
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    current_joints, clearance = next(iter(DataLoader(
        dataset, batch_size=min(args.samples, len(dataset))
    )))
    current_joints = current_joints.requires_grad_(True)
    goal_joints = dataset.q[0].expand_as(current_joints)
    predicted_time = model(current_joints, goal_joints)
    time_gradient = torch.autograd.grad(predicted_time.sum(), current_joints)[0]

    gradient_length = torch.linalg.vector_norm(time_gradient, dim=-1)
    pde_residual = torch.abs(
        speed(clearance, checkpoint['speed_profile']) * gradient_length - 1
    )
    finite_values = torch.isfinite(predicted_time).all() and torch.isfinite(time_gradient).all()
    boundary_error = model(goal_joints[:1], goal_joints[:1]).item()

    print(
        f'finite={finite_values} pde_mean={pde_residual.mean():.6f} '
        f'pde_median={pde_residual.median():.6f} '
        f'pde_p95={torch.quantile(pde_residual, .95):.6f} '
        f'boundary_error={boundary_error:.6e}'
    )
    print(
        f'gradient_norm mean={gradient_length.mean():.6f} '
        f'min={gradient_length.min():.6f} max={gradient_length.max():.6f}'
    )
    # Central finite difference check of dT/dq for a few configurations.
    step_size = 1e-4
    finite_difference_errors = []
    for row_index in range(min(5, len(current_joints))):
        for joint_index in range(current_joints.shape[1]):
            plus = current_joints[row_index:row_index + 1].detach().clone()
            minus = plus.clone()
            plus[0, joint_index] += step_size
            minus[0, joint_index] -= step_size
            finite_difference = (
                model(plus, goal_joints[row_index:row_index + 1])
                - model(minus, goal_joints[row_index:row_index + 1])
            ).item() / (2 * step_size)
            finite_difference_errors.append(
                abs(finite_difference - time_gradient[row_index, joint_index].item())
            )
    print(
        f'finite_difference_abs_error mean={np.mean(finite_difference_errors):.6e} '
        f'max={np.max(finite_difference_errors):.6e}'
    )


if __name__ == '__main__':
    main()
