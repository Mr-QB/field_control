import argparse
import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader
from .dataset import ClearanceDataset
from ..core.model import GoalTimeField
from ..core.speed_profile import speed

def main():
    p = argparse.ArgumentParser(); p.add_argument('--csv', required=True); p.add_argument('--metadata', required=True); p.add_argument('--checkpoint', required=True); p.add_argument('--samples', type=int, default=256); args=p.parse_args()
    ckpt=torch.load(args.checkpoint, map_location='cpu'); data=ClearanceDataset(args.csv,args.metadata); model=GoalTimeField(ckpt['joint_lower_bounds'],ckpt['joint_upper_bounds'],**ckpt['model_parameters']); model.load_state_dict(ckpt['model_state_dict']); model.eval()
    q,d=next(iter(DataLoader(data,batch_size=min(args.samples,len(data))))); q=q.requires_grad_(True); goal=data.q[0].expand_as(q); value=model(q,goal); grad=torch.autograd.grad(value.sum(),q)[0]; residual=torch.abs(speed(d,ckpt['speed_profile'])*torch.linalg.vector_norm(grad,dim=-1)-1)
    finite=torch.isfinite(value).all() and torch.isfinite(grad).all(); boundary=model(goal[:1],goal[:1]).item()
    print(f'finite={finite} pde_mean={residual.mean():.6f} pde_median={residual.median():.6f} pde_p95={torch.quantile(residual,.95):.6f} boundary_error={boundary:.6e}')
    print(f'gradient_norm mean={torch.linalg.vector_norm(grad,dim=-1).mean():.6f} min={torch.linalg.vector_norm(grad,dim=-1).min():.6f} max={torch.linalg.vector_norm(grad,dim=-1).max():.6f}')
    # Central finite difference check of dT/dq for a few configurations.
    h=1e-4; errors=[]
    for i in range(min(5,len(q))):
        for j in range(q.shape[1]):
            plus=q[i:i+1].detach().clone(); minus=plus.clone(); plus[0,j]+=h; minus[0,j]-=h
            fd=(model(plus,goal[i:i+1])-model(minus,goal[i:i+1])).item()/(2*h); errors.append(abs(fd-grad[i,j].item()))
    print(f'finite_difference_abs_error mean={np.mean(errors):.6e} max={np.max(errors):.6e}')
if __name__ == '__main__': main()
