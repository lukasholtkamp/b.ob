import torch
import torch.nn as nn


class PolicyModel(nn.Module):
    def __init__(self, obstacle_count=1):
        super(PolicyModel, self).__init__()
        input_dim = 5 + 3 * obstacle_count  # Path-following inputs + obstacle inputs
        output_dim = 3  # Outputs: [v, ω, ṡ]

        self.network = nn.Sequential(
            nn.Linear(input_dim, 128),  # First hidden layer
            nn.ReLU(),
            nn.Linear(128, 128),  # Second hidden layer
            nn.ReLU(),
            nn.Linear(128, 64),  # Third hidden layer
            nn.ReLU(),
            nn.Linear(64, 32),  # Fourth hidden layer
            nn.ReLU(),
            nn.Linear(32, output_dim),  # Output layer
        )

    def forward(self, state):
        return self.network(state)
