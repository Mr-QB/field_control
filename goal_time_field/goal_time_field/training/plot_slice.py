import argparse
import matplotlib.pyplot as plt
import numpy as np
import torch
from .dataset import load_metadata
from ..core.model import GoalTimeField

def main():
    parser = argparse.ArgumentParser(description='Plot a two-joint slice of a Goal Time Field.')
    parser.add_argument('--metadata', required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--joint-x', default='shoulder_pan_joint')
    parser.add_argument('--joint-y', default='shoulder_lift_joint')
    parser.add_argument('--output', default='field_slice.png')
    parser.add_argument('--resolution', type=int, default=50)
    args = parser.parse_args()

    metadata = load_metadata(args.metadata)
    checkpoint = torch.load(args.checkpoint, map_location='cpu')
    model = GoalTimeField(
        checkpoint['joint_lower_bounds'], checkpoint['joint_upper_bounds'],
        **checkpoint['model_parameters'],
    )
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    joint_x_index = metadata['joint_names'].index(args.joint_x)
    joint_y_index = metadata['joint_names'].index(args.joint_y)
    lower_bounds = np.array(metadata['lower_bounds'])
    upper_bounds = np.array(metadata['upper_bounds'])
    goal = (lower_bounds + upper_bounds) / 2

    x_values = np.linspace(lower_bounds[joint_x_index], upper_bounds[joint_x_index], args.resolution)
    y_values = np.linspace(lower_bounds[joint_y_index], upper_bounds[joint_y_index], args.resolution)
    x_grid, y_grid = np.meshgrid(x_values, y_values)

    # Start every point at the middle of its range, then sweep two joints.
    joint_configurations = np.tile(goal, (x_grid.size, 1))
    joint_configurations[:, joint_x_index] = x_grid.ravel()
    joint_configurations[:, joint_y_index] = y_grid.ravel()
    current_joints = torch.tensor(joint_configurations, dtype=torch.float32, requires_grad=True)
    goal_joints = torch.tensor(goal, dtype=torch.float32).expand_as(current_joints)

    predicted_time = model(current_joints, goal_joints)
    time_gradient = torch.autograd.grad(predicted_time.sum(), current_joints)[0].detach().numpy()
    time_grid = predicted_time.detach().numpy().reshape(x_grid.shape)

    plt.contourf(x_grid, y_grid, time_grid, 30, cmap='viridis')
    plt.colorbar(label='T [s]')
    arrow_step = 3
    plt.quiver(
        x_grid[::arrow_step, ::arrow_step],
        y_grid[::arrow_step, ::arrow_step],
        -time_gradient[:, joint_x_index].reshape(x_grid.shape)[::arrow_step, ::arrow_step],
        -time_gradient[:, joint_y_index].reshape(y_grid.shape)[::arrow_step, ::arrow_step],
        color='white',
    )
    plt.plot(goal[joint_x_index], goal[joint_y_index], 'r*', ms=14, label='goal')
    plt.xlabel(args.joint_x)
    plt.ylabel(args.joint_y)
    plt.legend()
    plt.savefig(args.output, dpi=180, bbox_inches='tight')


if __name__ == '__main__':
    main()
