import argparse
import matplotlib.pyplot as plt
import numpy as np
import torch
from .dataset import load_metadata
from ..core.model import GoalTimeField

def main():
    p=argparse.ArgumentParser(); p.add_argument('--metadata',required=True); p.add_argument('--checkpoint',required=True); p.add_argument('--joint-x',default='shoulder_pan_joint'); p.add_argument('--joint-y',default='shoulder_lift_joint'); p.add_argument('--output',default='field_slice.png'); p.add_argument('--resolution',type=int,default=50); args=p.parse_args()
    meta=load_metadata(args.metadata); ckpt=torch.load(args.checkpoint,map_location='cpu'); model=GoalTimeField(ckpt['joint_lower_bounds'],ckpt['joint_upper_bounds'],**ckpt['model_parameters']); model.load_state_dict(ckpt['model_state_dict']); model.eval()
    ix,iy=meta['joint_names'].index(args.joint_x),meta['joint_names'].index(args.joint_y); lo=np.array(meta['lower_bounds']); hi=np.array(meta['upper_bounds']); goal=(lo+hi)/2
    x,y=np.linspace(lo[ix],hi[ix],args.resolution),np.linspace(lo[iy],hi[iy],args.resolution); xx,yy=np.meshgrid(x,y); q=np.tile(goal,(xx.size,1)); q[:,ix],q[:,iy]=xx.ravel(),yy.ravel(); qt=torch.tensor(q,dtype=torch.float32,requires_grad=True); gt=torch.tensor(goal,dtype=torch.float32).expand_as(qt); value=model(qt,gt); grad=torch.autograd.grad(value.sum(),qt)[0].detach().numpy()
    z=value.detach().numpy().reshape(xx.shape); plt.contourf(xx,yy,z,30,cmap='viridis'); plt.colorbar(label='T [s]'); plt.quiver(xx[::3,::3],yy[::3,::3],-grad[:,ix].reshape(xx.shape)[::3,::3],-grad[:,iy].reshape(xx.shape)[::3,::3],color='white'); plt.plot(goal[ix],goal[iy],'r*',ms=14,label='goal'); plt.xlabel(args.joint_x); plt.ylabel(args.joint_y); plt.legend(); plt.savefig(args.output,dpi=180,bbox_inches='tight')
if __name__ == '__main__': main()
