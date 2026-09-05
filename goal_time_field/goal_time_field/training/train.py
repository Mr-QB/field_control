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
    with open(path, encoding='utf-8') as file: return yaml.safe_load(file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', required=True); parser.add_argument('--metadata', required=True)
    parser.add_argument('--config', required=True); parser.add_argument('--checkpoint', required=True)
    args = parser.parse_args(); cfg = load_config(args.config); tr = cfg['training']
    random.seed(tr['seed']); np.random.seed(tr['seed']); torch.manual_seed(tr['seed'])
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    data = ClearanceDataset(args.csv, args.metadata); meta = data.metadata
    goals = torch.where(data.clearance >= cfg['goal_sampling']['goal_clearance_min'])[0]
    if len(goals) == 0: raise ValueError('No goal meets goal_clearance_min')
    loader = DataLoader(data, batch_size=tr['batch_size'], sampler=data.sampler(tr['near_lambda'], tr['near_sigma']))
    model = GoalTimeField(meta['lower_bounds'], meta['upper_bounds'], **cfg['model']).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=tr['learning_rate'])
    for epoch in range(1, tr['epochs'] + 1):
        model.train(); total = 0.0
        for q, clearance in loader:
            q, clearance = q.to(device).requires_grad_(True), clearance.to(device)
            indices = goals[torch.randint(len(goals), (len(q),))]
            q_goal = data.q[indices].to(device)
            value = model(q, q_goal)
            grad = torch.autograd.grad(value.sum(), q, create_graph=True)[0]
            pde = ((speed(clearance, cfg['speed_profile']) * torch.linalg.vector_norm(grad, dim=-1) - 1.0) ** 2).mean()
            bc = model(q_goal, q_goal).square().mean()
            loss = pde + tr['lambda_bc'] * bc
            optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), tr['grad_clip']); optimizer.step()
            total += loss.item()
        print(f'epoch {epoch:04d} loss={total / len(loader):.6f} device={device}')
    Path(args.checkpoint).parent.mkdir(parents=True, exist_ok=True)
    torch.save({'model_state_dict': model.state_dict(), 'joint_names': meta['joint_names'],
                'joint_lower_bounds': meta['lower_bounds'], 'joint_upper_bounds': meta['upper_bounds'],
                'speed_profile': cfg['speed_profile'], 'model_parameters': cfg['model'],
                'training_epoch': tr['epochs'], 'training_loss': total / len(loader)}, args.checkpoint)

if __name__ == '__main__': main()
