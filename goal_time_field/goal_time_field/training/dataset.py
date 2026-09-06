import json
import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset, WeightedRandomSampler


def load_metadata(path):
    with open(path, encoding='utf-8') as file:
        return json.load(file)


class ClearanceDataset(Dataset):
    """Collision-free joint configurations and their obstacle clearance."""
    def __init__(self, csv_path, metadata_path):
        self.metadata = load_metadata(metadata_path)
        frame = pd.read_csv(csv_path)
        self.joint_names = self.metadata['joint_names']

        collision_free_rows = frame[frame['clearance'] > 0.0].reset_index(drop=True)
        if collision_free_rows.empty:
            raise ValueError('Dataset contains no collision-free configurations')
        self.q = torch.tensor(collision_free_rows[self.joint_names].to_numpy(np.float32))
        self.clearance = torch.tensor(collision_free_rows['clearance'].to_numpy(np.float32))

    def __len__(self):
        return len(self.q)

    def __getitem__(self, index):
        return self.q[index], self.clearance[index]

    def sampler(self, near_lambda, sigma):
        """Sample configurations near obstacles more often during training."""
        weights = 1.0 + near_lambda * torch.exp(-self.clearance / sigma)
        return WeightedRandomSampler(weights, len(weights), replacement=True)
