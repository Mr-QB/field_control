import argparse
import random
from pathlib import Path
import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader
from .dataset import ClearanceDataset
from ..core.model import GoalTimeField
from ..core.speed_profile import speed


def load_config(path):
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def main():
    parser = argparse.ArgumentParser(description='Train a Goal Time Field model.')
    parser.add_argument('--csv', required=True, help='Dataset generated for the fixed obstacle scene.')
    parser.add_argument('--metadata', required=True, help='Joint names and limits for the dataset.')
    parser.add_argument('--config', required=True, help='Training YAML configuration.')
    parser.add_argument('--checkpoint', required=True, help='Where to save the trained model.')
    args = parser.parse_args()

    config = load_config(args.config)
    training_config = config['training']
    random.seed(training_config['seed'])
    np.random.seed(training_config['seed'])
    torch.manual_seed(training_config['seed'])

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    dataset = ClearanceDataset(args.csv, args.metadata)
    metadata = dataset.metadata
    goal_indices = torch.where(
        dataset.clearance >= config['goal_sampling']['goal_clearance_min']
    )[0]
    if len(goal_indices) == 0:
        raise ValueError('No goal meets goal_clearance_min')

    loader = DataLoader(
        dataset,
        batch_size=training_config['batch_size'],
        sampler=dataset.sampler(training_config['near_lambda'], training_config['near_sigma']),
    )
    model = GoalTimeField(
        metadata['lower_bounds'], metadata['upper_bounds'], **config['model']
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=training_config['learning_rate'])

    for epoch in range(1, training_config['epochs'] + 1):
        model.train()
        total_loss = 0.0
        for current_joints, clearance in loader:
            current_joints = current_joints.to(device).requires_grad_(True)
            clearance = clearance.to(device)

            # Pick one safe goal for each input configuration in this batch.
            random_goal_rows = goal_indices[torch.randint(len(goal_indices), (len(current_joints),))]
            goal_joints = dataset.q[random_goal_rows].to(device)

            predicted_time = model(current_joints, goal_joints)
            time_gradient = torch.autograd.grad(
                predicted_time.sum(), current_joints, create_graph=True
            )[0]

            # Eikonal PDE: speed(clearance) * ||dT/dq|| should equal one.
            gradient_length = torch.linalg.vector_norm(time_gradient, dim=-1)
            pde_loss = (
                speed(clearance, config['speed_profile']) * gradient_length - 1.0
            ).square().mean()

            # The model architecture already enforces this, but keep the loss
            # explicit so the training objective remains easy to understand.
            boundary_loss = model(goal_joints, goal_joints).square().mean()
            loss = pde_loss + training_config['lambda_bc'] * boundary_loss

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), training_config['grad_clip'])
            optimizer.step()
            total_loss += loss.item()

        average_loss = total_loss / len(loader)
        print(f'epoch {epoch:04d} loss={average_loss:.6f} device={device}')

    Path(args.checkpoint).parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        'model_state_dict': model.state_dict(),
        'joint_names': metadata['joint_names'],
        'joint_lower_bounds': metadata['lower_bounds'],
        'joint_upper_bounds': metadata['upper_bounds'],
        'speed_profile': config['speed_profile'],
        'model_parameters': config['model'],
        'training_epoch': training_config['epochs'],
        'training_loss': average_loss,
    }, args.checkpoint)

if __name__ == '__main__':
    main()
