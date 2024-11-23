import torch
import torch.nn as nn

class PolicyModel(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(PolicyModel, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 128),  # Adjusted input dimension
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_dim)
        )

    def forward(self, state):
        return self.network(state)
