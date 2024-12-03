import torch
import torch.nn as nn

class PolicyModel(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(PolicyModel, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 128),  # Input to first hidden layer
            nn.ReLU(),
            nn.Linear(128, 64),        # First to second hidden layer
            nn.ReLU(),
            nn.Linear(64, 64),         # Second to third hidden layer
            nn.ReLU(),
            nn.Linear(64, 32),         # Third to fourth hidden layer
            nn.ReLU(),
            nn.Linear(32, output_dim)  # Output layer
        )

    def forward(self, state):
        return self.network(state)
