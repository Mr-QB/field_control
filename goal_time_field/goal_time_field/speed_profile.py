import torch


def speed(clearance, config):
    """Heuristic mapping workspace clearance [m] to joint-space speed [rad/s]."""
    x = torch.clamp((clearance - config['d_near']) / (config['d_free'] - config['d_near']), 0.0, 1.0)
    smooth = x * x * (3.0 - 2.0 * x)
    return config['s_min'] + (config['s_max'] - config['s_min']) * smooth
