import torch


def speed(clearance, config):
    """Convert obstacle clearance (m) into the desired joint speed (rad/s).

    ``clearance`` can be one number or a PyTorch tensor.  Speed changes smoothly
    from ``s_min`` near an obstacle to ``s_max`` in free space.
    """
    near_distance = config['d_near']
    free_distance = config['d_free']

    # Convert clearance to a value in [0, 1].
    progress = (clearance - near_distance) / (free_distance - near_distance)
    progress = torch.clamp(progress, min=0.0, max=1.0)

    # smoothstep prevents an abrupt speed change at either threshold.
    smooth_progress = progress * progress * (3.0 - 2.0 * progress)
    return config['s_min'] + (config['s_max'] - config['s_min']) * smooth_progress
